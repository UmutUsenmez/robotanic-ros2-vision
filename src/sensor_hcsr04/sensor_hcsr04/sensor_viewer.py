import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import os

class SensorViewer(Node):
    def __init__(self):
        super().__init__('sensor_viewer')

        # Sensör değerlerini tutacağımız sözlük (başlangıçta 0 metre)
        self.dists = {'L': 0.0, 'FL': 0.0, 'FR': 0.0, 'R': 0.0}

        # 4 Kanalın da radyosunu dinliyoruz (Subscribe)
        self.create_subscription(Range, 'ultrasonic/left', self.cb_l, 10)
        self.create_subscription(Range, 'ultrasonic/front_left', self.cb_fl, 10)
        self.create_subscription(Range, 'ultrasonic/front_right', self.cb_fr, 10)
        self.create_subscription(Range, 'ultrasonic/right', self.cb_r, 10)

        # Saniyede 5 kez (0.2 sn) ekranı güncelleyen zamanlayıcı
        self.create_timer(0.2, self.print_dashboard)

    # Radyodan anlık veri geldikçe değişkenleri güncelleyen fonksiyonlar
    def cb_l(self, msg): self.dists['L'] = msg.range
    def cb_fl(self, msg): self.dists['FL'] = msg.range
    def cb_fr(self, msg): self.dists['FR'] = msg.range
    def cb_r(self, msg): self.dists['R'] = msg.range

    def print_dashboard(self):
        # ROS'taki resmi Metre verisini bizim okumamız için tekrar CM'ye çeviriyoruz
        l_cm = int(self.dists['L'] * 100)
        fl_cm = int(self.dists['FL'] * 100)
        fr_cm = int(self.dists['FR'] * 100)
        r_cm = int(self.dists['R'] * 100)

        # Ekranı temizle ve şık bir panel bas
        os.system('clear')
        print("="*65)
        print(" 🚜 ROBOTANIK CANLI SENSÖR PANELİ 🚜 ".center(65))
        print("="*65)
        print(f" 👈 SOL      : {l_cm:03d} cm")
        print(f" ↖️ SOL-ÖN   : {fl_cm:03d} cm")
        print(f" ↗️ SAĞ-ÖN   : {fr_cm:03d} cm")
        print(f" 👉 SAĞ      : {r_cm:03d} cm")
        print("="*65)

def main(args=None):
    rclpy.init(args=args)
    node = SensorViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()