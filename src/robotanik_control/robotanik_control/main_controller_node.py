import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os

class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        
        self.location_buffer = [] 
        self.csv_output_path = '/home/umut/robotanik_ws/src/robotanik_control/data/hastalik_haritasi_verileri.csv'
        os.makedirs(os.path.dirname(self.csv_output_path), exist_ok=True)
        
        if not os.path.exists(self.csv_output_path):
            with open(self.csv_output_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                # YENİ: Sütunlara Yayılma Oranı ve Risk Skoru eklendi
                writer.writerow(['Zaman', 'X_Koordinati', 'Y_Koordinati', 'Hastalik_Adi', 'Yayilma_Orani_%', 'Risk_Skoru'])
        # -----------------------------------

        self.loc_sub = self.create_subscription(String, 'robot/location', self.location_callback, 10)
        self.ai_sub = self.create_subscription(String, 'ai/detections', self.ai_callback, 10)
            
        self.get_logger().info('🧠 Main Controller devrede. Otonom CSV loglama aktif...')

    def location_callback(self, msg):
        loc_data = json.loads(msg.data)
        self.location_buffer.append(loc_data)
        if len(self.location_buffer) > 50:
            self.location_buffer.pop(0)

    def ai_callback(self, msg):
        ai_data = json.loads(msg.data)
        hastalik = ai_data.get("label", "Healthy")
        ai_time = ai_data.get("time", 0.0)
        
        # YENİ: AI Node'dan gelen taze verileri (skorları) JSON'dan çekiyoruz
        risk_skoru = ai_data.get("risk_score", 0.0)
        yayilma_orani = ai_data.get("spread_ratio", 0.0)
        
        if hastalik != "Healthy" and len(self.location_buffer) > 0:
            
            closest_loc = min(self.location_buffer, key=lambda loc: abs(loc["time"] - ai_time))
            
            # Terminaldeki Uyarıyı da Risk Skoruna göre şekillendirdik
            self.get_logger().warning(
                f"🚨 {hastalik} TESPİT EDİLDİ! (Risk: %{risk_skoru:.1f})\n"
                f"   📍 Konum -> X: {closest_loc['x']:.3f}, Y: {closest_loc['y']:.3f}"
            )

            # --- YENİ: HASTALIĞI, YAYILMA ORANINI VE RİSKİ CSV DOSYASINA YAZMA ---
            try:
                with open(self.csv_output_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    # Sütun sırasına göre veriyi yaz: Zaman, X, Y, Hastalık, Yayılma, Risk
                    writer.writerow([f"{ai_time:.2f}", closest_loc['x'], closest_loc['y'], hastalik, f"{yayilma_orani:.2f}", f"{risk_skoru:.2f}"])
                
                self.get_logger().info(f"💾 Veri başarıyla haritaya işlendi: Risk %{risk_skoru:.1f}")
            except Exception as e:
                self.get_logger().error(f"CSV Kayıt Hatası: {e}")
            # -------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
