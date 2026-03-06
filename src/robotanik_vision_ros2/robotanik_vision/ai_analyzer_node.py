import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class AIAnalyzerNode(Node):
    def __init__(self):
        super().__init__('ai_analyzer_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        # Ağırlık (.pt) dosyalarının yollarını belirle (kendi isimlerine göre güncelle)
        current_dir = os.path.dirname(os.path.realpath(__file__))
        leaf_model_path = '/home/aziz/Desktop/ros2_ws/src/robotanik_vision/models/leafdetectionfinal.pt'
        disease_model_path = '/home/aziz/Desktop/ros2_ws/src/robotanik_vision/models/yolo11s_leaf_disease.pt'
        
        # Modelleri yükle
        self.leaf_model = YOLO(leaf_model_path)
        self.disease_model = YOLO(disease_model_path)

    def listener_callback(self, msg):
        self.get_logger().info('Görüntü alındı, analiz ediliyor...')
        # ROS mesajını OpenCV formatına geri çevir
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 1. Aşama: Yaprak Tespiti
        leaf_results = self.leaf_model(frame, verbose=False)
        
        # Çıktıyı görselleştirmek için (Ultralytics'in kendi çizim fonksiyonu)
        annotated_frame = leaf_results[0].plot()

        # 2. Aşama: Hastalık Analizi (Tespit edilen yapraklar üzerinde)
        # Eğer yaprak modeli bir obje bulduysa, ikinci modeli çalıştır
        if len(leaf_results[0].boxes) > 0:
            disease_results = self.disease_model(frame, verbose=False)
            # Hastalık sonuçlarını da aynı karenin üzerine çiz
            annotated_frame = disease_results[0].plot(img=annotated_frame)

        # Sonucu ekranda göster
        cv2.imshow("Robotanik - Yapay Zeka Analizi", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AIAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()