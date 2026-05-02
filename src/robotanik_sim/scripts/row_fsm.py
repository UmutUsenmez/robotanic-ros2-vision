#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


class MissionState(Enum):
    STARTUP_WAIT    = 0
    SEND_GOAL       = 1
    WAIT_GOAL_RESPONSE = 2
    WAIT_RESULT     = 3
    WAIT_CANCEL     = 4
    BACKUP          = 5
    NEXT_WAYPOINT   = 6
    DONE            = 7
    EVACUATE        = 8
    ROW_FOLLOWING   = 9


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class RobotanikRowFSM(Node):
    def __init__(self):
        super().__init__("robotanik_row_fsm")
        self.get_logger().info("### ROBOTANIK FSM V7 BAŞLADI ###")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub    = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_x   = 0.0
        self.current_y   = 0.0

        self.front_center = None
        self.front_left   = None
        self.front_right  = None
        self.back         = None

        self.create_subscription(Range, "/sonar/front_center", self.front_center_cb, 10)
        self.create_subscription(Range, "/sonar/front_left",   self.front_left_cb,   10)
        self.create_subscription(Range, "/sonar/front_right",  self.front_right_cb,  10)
        self.create_subscription(Range, "/sonar/back",         self.back_cb,         10)

        self.corridor_centers  = [10.45, 8.50, 6.50, 4.50, 2.50, 0.55]
        self.y_approach_bottom = -1.0
        self.y_approach_top    = 50.0
        self.y_bottom_turn     = 1.20
        self.y_top_turn        = 48.80

        # Waypoint listesi — dinamik olarak büyüyebilir
        self.waypoints  = self.generate_serpentine_waypoints()
        self.current_wp = 0
        self.state      = MissionState.STARTUP_WAIT

        self.goal_handle      = None
        self.send_goal_future = None
        self.result_future    = None
        self.cancel_future    = None

        self.startup_time  = time.time()
        self.startup_delay = 6.0

        self.obstacle_counter       = 0
        self.obstacle_confirm_count = 3
        self.front_obstacle_threshold = 0.65
        self.back_obstacle_threshold  = 0.30

        self.backup_start_time   = None
        self.backup_duration     = 2.5
        self.backup_speed        = -0.10
        self.evacuate_speed      = -0.20
        self.evacuate_start_time = 0.0

        # Engelli koridorları takip et
        self.skipped_corridors = []

        self.timer = self.create_timer(0.1, self.loop)
        self.nav_client.wait_for_server()

    # ── Waypoint Üretici ─────────────────────────────────────────
    def generate_serpentine_waypoints(self):
        waypoints = []
        for i, x in enumerate(self.corridor_centers):
            going_up = (i % 2 == 0)
            if going_up:
                waypoints.append((x, self.y_approach_bottom,  1.5708, "NAV2"))
                waypoints.append((x, self.y_bottom_turn,      1.5708, "NAV2"))
                waypoints.append((x, self.y_top_turn,         1.5708, "ROW_FOLLOW"))
                waypoints.append((x, self.y_approach_top,     1.5708, "NAV2"))
                if i < len(self.corridor_centers) - 1:
                    nx = self.corridor_centers[i + 1]
                    waypoints.append((nx, self.y_approach_top, -1.5708, "NAV2"))
            else:
                waypoints.append((x, self.y_approach_top,    -1.5708, "NAV2"))
                waypoints.append((x, self.y_top_turn,        -1.5708, "NAV2"))
                waypoints.append((x, self.y_bottom_turn,     -1.5708, "ROW_FOLLOW"))
                waypoints.append((x, self.y_approach_bottom, -1.5708, "NAV2"))
                if i < len(self.corridor_centers) - 1:
                    nx = self.corridor_centers[i + 1]
                    waypoints.append((nx, self.y_approach_bottom, 1.5708, "NAV2"))
        return waypoints

    # ── Sonar Callback'ler ───────────────────────────────────────
    def front_center_cb(self, msg): self.front_center = msg.range
    def front_left_cb(self,   msg): self.front_left   = msg.range
    def front_right_cb(self,  msg): self.front_right  = msg.range
    def back_cb(self,         msg): self.back         = msg.range

    # ── Yardımcılar ──────────────────────────────────────────────
    def front_obstacle_seen(self):
        if self.state == MissionState.ROW_FOLLOWING:
            if self.front_center is not None and not math.isnan(self.front_center):
                return self.front_center < self.front_obstacle_threshold
            return False
        vals = [v for v in [self.front_center, self.front_left, self.front_right]
                if v is not None and not math.isnan(v)]
        return min(vals) < self.front_obstacle_threshold if vals else False

    def back_is_safe(self):
        if self.back is None or math.isnan(self.back):
            return True
        return self.back > self.back_obstacle_threshold

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_cmd_vel(self, speed, angular=0.0):
        msg = Twist()
        msg.linear.x  = speed
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    # ── Engel Mantığı ────────────────────────────────────────────
    def skip_to_next_corridor(self):
        """Mevcut koridoru engelli olarak işaretle, sonraki koridora atla."""
        current_x = self.waypoints[self.current_wp][0]

        if current_x not in self.skipped_corridors:
            self.skipped_corridors.append(current_x)
            self.get_logger().warn(
                f"KORİDOR {current_x:.2f} ENGELLİ → atlandı. "
                f"Toplam atlanan: {len(self.skipped_corridors)}"
            )

        # Mevcut koridorun tüm waypoint'lerini geç
        for i in range(self.current_wp + 1, len(self.waypoints)):
            nx, ny, nyaw, ntype = self.waypoints[i]
            if abs(nx - current_x) > 0.5 and ntype == "NAV2":
                self.current_wp = i
                self.state = MissionState.SEND_GOAL
                self.get_logger().warn(f"YENİ HEDEF KORİDOR: X={nx:.2f}, Y={ny:.2f}")
                return

        # Başka ana koridor kalmadı — atlanmışları ekle
        if self.skipped_corridors:
            self.get_logger().warn(
                f"Ana koridorlar bitti. "
                f"{len(self.skipped_corridors)} atlanan koridor taranıyor."
            )
            self._append_skipped_corridor()
        else:
            self.get_logger().info("Tüm koridorlar tamamlandı.")
            self.state = MissionState.DONE

    def _append_skipped_corridor(self):
        """Atlanan ilk koridoru waypoint listesinin sonuna ekle."""
        skipped_x = self.skipped_corridors.pop(0)

        # Bu koridorun orijinal serpentine indeksini bul
        for i, x in enumerate(self.corridor_centers):
            if abs(x - skipped_x) < 0.1:
                going_up = (i % 2 == 0)
                if going_up:
                    new_wps = [
                        (x, self.y_approach_bottom, 1.5708,  "NAV2"),
                        (x, self.y_bottom_turn,     1.5708,  "NAV2"),
                        (x, self.y_top_turn,        1.5708,  "ROW_FOLLOW"),
                        (x, self.y_approach_top,    1.5708,  "NAV2"),
                    ]
                else:
                    new_wps = [
                        (x, self.y_approach_top,    -1.5708, "NAV2"),
                        (x, self.y_top_turn,        -1.5708, "NAV2"),
                        (x, self.y_bottom_turn,     -1.5708, "ROW_FOLLOW"),
                        (x, self.y_approach_bottom, -1.5708, "NAV2"),
                    ]

                # Mevcut pozisyonun hemen arkasına ekle
                ins = self.current_wp + 1
                for wp in reversed(new_wps):
                    self.waypoints.insert(ins, wp)

                self.get_logger().warn(
                    f"Atlanan koridor X={skipped_x:.2f} listeye eklendi. "
                    f"Kalan atlanan: {len(self.skipped_corridors)}"
                )
                self.state = MissionState.NEXT_WAYPOINT
                return

        # Bulunamazsa (olmamalı) bitir
        self.state = MissionState.DONE

    # ── Nav2 Hedef Gönder ────────────────────────────────────────
    def send_nav2_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.send_goal_future = self.nav_client.send_goal_async(goal)
        self.state = MissionState.WAIT_GOAL_RESPONSE

    def send_current_goal(self):
        if self.current_wp >= len(self.waypoints):
            self.state = MissionState.DONE
            return

        x, y, yaw, wp_type = self.waypoints[self.current_wp]

        if wp_type == "ROW_FOLLOW":
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.state = MissionState.ROW_FOLLOWING
            self.get_logger().info("NAV2 UYKUDA — HİBRİT PİLOT DEVREDE")
            return

        self.send_nav2_goal(x, y, yaw)

    def handle_goal_response(self):
        if self.send_goal_future.done():
            self.goal_handle = self.send_goal_future.result()
            if not self.goal_handle.accepted:
                self.get_logger().warn("Hedef reddedildi → sonraki waypoint")
                self.state = MissionState.NEXT_WAYPOINT
                return
            self.result_future = self.goal_handle.get_result_async()
            self.state = MissionState.WAIT_RESULT

    # ── Ana Döngü ────────────────────────────────────────────────
    def loop(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
        except:
            pass

        if self.state == MissionState.STARTUP_WAIT:
            if time.time() - self.startup_time >= self.startup_delay:
                self.state = MissionState.SEND_GOAL

        elif self.state == MissionState.SEND_GOAL:
            self.send_current_goal()

        elif self.state == MissionState.WAIT_GOAL_RESPONSE:
            self.handle_goal_response()

        elif self.state == MissionState.WAIT_RESULT:
            if self.front_obstacle_seen():
                self.obstacle_counter += 1
                if self.obstacle_counter >= self.obstacle_confirm_count:
                    self.get_logger().error("AÇIK ALANDA ENGEL → KORİDOR ATLANIYOR")
                    self.publish_stop()
                    self.obstacle_counter = 0
                    if self.goal_handle:
                        self.cancel_future = self.goal_handle.cancel_goal_async()
                        self.goal_handle   = None
                        self.state = MissionState.WAIT_CANCEL
                    else:
                        self.backup_start_time = time.time()
                        self.state = MissionState.BACKUP
            else:
                self.obstacle_counter = 0

            if self.result_future and self.result_future.done():
                self.state = MissionState.NEXT_WAYPOINT

        elif self.state == MissionState.ROW_FOLLOWING:
            if self.front_obstacle_seen():
                self.obstacle_counter += 1
                if self.obstacle_counter >= self.obstacle_confirm_count:
                    self.get_logger().error("KORİDOR TIKALI → TAHLİYE")
                    self.obstacle_counter = 0
                    self.publish_stop()
                    self.evacuate_start_time = time.time()
                    self.state = MissionState.EVACUATE
                    return
            else:
                self.obstacle_counter = 0

            # PID merkezleme
            Kp    = 1.2
            l_val = min(self.front_left  if self.front_left  and not math.isnan(self.front_left)  else 1.0, 1.0)
            r_val = min(self.front_right if self.front_right and not math.isnan(self.front_right) else 1.0, 1.0)
            error     = (l_val - r_val) / max(l_val + r_val, 0.1)
            angular_z = max(-0.5, min(0.5, Kp * error))
            self.publish_cmd_vel(0.18, angular_z)

            # Çıkış kontrolü
            _, target_y, yaw, _ = self.waypoints[self.current_wp]
            going_up = (yaw > 0)
            if (going_up     and self.current_y >= target_y - 0.3) or \
               (not going_up and self.current_y <= target_y + 0.3):
                self.publish_stop()
                self.get_logger().info("KORİDOR BİTTİ → NAV2 UYANDIRILIYOR")
                self.state = MissionState.NEXT_WAYPOINT

        elif self.state == MissionState.WAIT_CANCEL:
            if self.cancel_future and self.cancel_future.done():
                self.backup_start_time = time.time()
                self.state = MissionState.BACKUP

        elif self.state == MissionState.BACKUP:
            if not self.back_is_safe():
                self.publish_stop()
                self.get_logger().warn("Geri yol kapalı → koridoru atla")
                self.skip_to_next_corridor()
                return
            if time.time() - self.backup_start_time < self.backup_duration:
                self.publish_cmd_vel(self.backup_speed)
            else:
                self.publish_stop()
                self.skip_to_next_corridor()

        elif self.state == MissionState.EVACUATE:
            if not self.back_is_safe():
                if time.time() - self.evacuate_start_time > 10.0:
                    self.get_logger().error("Arkadan sıkıştık → zorla atla")
                    self.skip_to_next_corridor()
                    return
                self.publish_stop()
                return

            self.publish_cmd_vel(self.evacuate_speed)

            _, _, yaw, _ = self.waypoints[self.current_wp]
            if (yaw > 0  and self.current_y < self.y_approach_bottom + 0.5) or \
               (yaw < 0  and self.current_y > self.y_approach_top    - 0.5):
                self.publish_stop()
                self.get_logger().info("Koridordan çıkıldı → sonraki koridora")
                self.skip_to_next_corridor()

        elif self.state == MissionState.NEXT_WAYPOINT:
            self.current_wp += 1
            self.state = MissionState.SEND_GOAL if self.current_wp < len(self.waypoints) \
                         else MissionState.DONE

        elif self.state == MissionState.DONE:
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = RobotanikRowFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
