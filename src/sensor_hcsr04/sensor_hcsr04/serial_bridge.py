import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import re

class HCSR04SerialBridge(Node):
    def __init__(self):
        super().__init__('hcsr04_serial_bridge')

        # 4 ayrı sensör için Nav2 standartlarında yayıncılar (publishers)
        self.pub_s0 = self.create_publisher(Range, 'ultrasonic/left', 10)
        self.pub_s1 = self.create_publisher(Range, 'ultrasonic/front_left', 10)
        self.pub_s2 = self.create_publisher(Range, 'ultrasonic/front_right', 10)
        self.pub_s3 = self.create_publisher(Range, 'ultrasonic/right', 10)

        # Seri port ayarları
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self.get_logger().info(f"[{self.serial_port}] portuna {self.baud_rate} baud ile baglanildi. Dinleniyor...")
        except serial.SerialException as e:
            self.get_logger().error(f"Seri port acilamadi! Kabloyu kontrol et: {e}")
            return

        # 0.05 saniyede bir (20 Hz) seri portu kontrol et
        self.timer = self.create_timer(0.05, self.read_serial_data)

    def create_range_msg(self, frame_id, distance_cm):
        """ Gelen santimetre verisini ROS standartı olan Range (Metre) mesajına çevirir """
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26 # HC-SR04 görüş açısı (~15 derece)
        msg.min_range = 0.02     # Minimum 2 cm
        msg.max_range = 4.0      # Maksimum 4 metre
        
        # ROS 2 standartı her zaman SI birimleridir. cm -> metreye çevrilir.
        msg.range = float(distance_cm) / 100.0 
        return msg

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                matches = re.findall(r'S\d+:\s*(\d+)', line)

                if len(matches) == 4:
                    d0, d1, d2, d3 = [int(m) for m in matches]

                    msg_s0 = self.create_range_msg("ultrasonic_left_link", d0)
                    msg_s1 = self.create_range_msg("ultrasonic_front_left_link", d1)
                    msg_s2 = self.create_range_msg("ultrasonic_front_right_link", d2)
                    msg_s3 = self.create_range_msg("ultrasonic_right_link", d3)

                    self.pub_s0.publish(msg_s0)
                    self.pub_s1.publish(msg_s1)
                    self.pub_s2.publish(msg_s2)
                    self.pub_s3.publish(msg_s3)
                    
            except Exception as e:
                self.get_logger().warning(f"Veri okunurken ufak bir pürüz: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HCSR04SerialBridge()
    if hasattr(node, 'ser'): 
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()