import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from cv_bridge import CvBridge
import cv2
import json
import os
import numpy as np
import joblib
from ultralytics import YOLO

class AIAnalyzerNode(Node):
    def __init__(self):
        super().__init__('ai_analyzer_node')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.ai_publisher = self.create_publisher(String, 'ai/detections', 10)
        self.bridge = CvBridge()
        
        # 1. DOSYA YOLLARI 
        leaf_model_path = '/home/umut/robotanik_ws/src/robotanik_vision/models/leafdetectionfinal.pt'
        disease_model_path = '/home/umut/robotanik_ws/src/robotanik_vision/models/yolo11s_leaf_disease.pt'
        risk_model_path = '/home/umut/robotanik_ws/src/robotanik_vision/models/linear_risk_model_v4_noisy.pkl'
        
        # 2. MODELLERİ YÜKLEME
        self.leaf_model = YOLO(leaf_model_path)
        self.disease_model = YOLO(disease_model_path)
        self.risk_model = joblib.load(risk_model_path)

        # 3. TEHLİKE KATSAYILARI
        self.disease_danger_map = {
            "mosaic virus": 10, "yellow leaf curl virus": 10, "late blight": 8,
            "bacterial spot": 7, "early blight": 6, "spider mites": 5,
            "septoria": 4, "leaf mold": 3, "leaf miner": 2, "healthy": 0
        }
        self.get_logger().info("Robotanik AI Analyzer: Sistem senin PC için ayağa kalktı!")

    def get_danger_coefficient(self, class_name):
        name_lower = class_name.lower()
        for key, value in self.disease_danger_map.items():
            if key in name_lower:
                return value
        return 0

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        annotated_frame = frame.copy()
        
        # Toplam Görüntü Alanı (Yayılma oranı hesabı için payda)
        img_h, img_w = frame.shape[:2]
        total_frame_area = img_h * img_w
        
        leaf_results = self.leaf_model(frame, conf=0.5, verbose=False)
        
        if len(leaf_results[0].boxes) > 0:
            disease_results = self.disease_model(frame, conf=0.45, verbose=False)
            disease_boxes = disease_results[0].boxes
            disease_names = disease_results[0].names

            for leaf_box in leaf_results[0].boxes:
                lx1, ly1, lx2, ly2 = map(int, leaf_box.xyxy[0])
                
                # ALAN HESABI: Tespit edilen yaprak alanı / Toplam kare alanı
                leaf_area = (lx2 - lx1) * (ly2 - ly1)
                spread_ratio = (leaf_area / total_frame_area) * 100.0
                
                final_label = "Healthy"
                box_color = (0, 255, 0) 
                final_risk_score = 0.0

                for d_box in disease_boxes:
                    dx1, dy1, dx2, dy2 = map(int, d_box.xyxy[0])
                    dcx = (dx1 + dx2) / 2
                    dcy = (dy1 + dy2) / 2
                    
                    if lx1 <= dcx <= lx2 and ly1 <= dcy <= ly2:
                        cls_id = int(d_box.cls[0])
                        final_label = disease_names[cls_id] 
                        box_color = (0, 0, 255) 
                        
                        danger_coeff = self.get_danger_coefficient(final_label)
                        
                        # ML Risk Tahmini
                        if danger_coeff > 0:
                            X_input = np.array([[danger_coeff, spread_ratio]])
                            prediction = self.risk_model.predict(X_input)
                            final_risk_score = float(prediction[0])
                            final_risk_score = max(0.0, min(100.0, final_risk_score))
                        break
                        
                # Çizim ve Yayınlama
                cv2.rectangle(annotated_frame, (lx1, ly1), (lx2, ly2), box_color, 2)
                display_text = f"{final_label} (Risk: %{final_risk_score:.1f})"
                cv2.putText(annotated_frame, display_text, (lx1, ly1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                # Heatmap için yayınlanan çıktı verisi
                current_time = self.get_clock().now().nanoseconds / 1e9
                detection_data = {
                    "time": current_time, # Navigasyon verisiyle eşleşmek için kilit anahtar
                    "label": final_label,
                    "risk_score": final_risk_score,
                    "spread_ratio": spread_ratio
                }
                
                detection_msg = String()
                detection_msg.data = json.dumps(detection_data) 
                self.ai_publisher.publish(detection_msg)

        cv2.imshow("Robotanik - Analiz", annotated_frame)
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
