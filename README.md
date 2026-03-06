# 🌿 Robotanik: ROS 2 Real-Time Vision Subsystem

![ROS 2 YOLO Integration](docs/showcase.jpg)

## 📌 Project Overview
This repository contains the core vision and AI integration module for an autonomous plant disease detection robot. 

**This project represents the deployment phase of my previous YOLOv11 models.** It takes the custom-trained Leaf Detection and Disease Classification models and integrates them into a real-time **ROS 2** ecosystem for hardware deployment. It demonstrates the successful bridge between deep learning (AI) and robotic middleware (ROS 2).

## ⚙️ System Architecture (Nodes)
The architecture is built on a custom ROS 2 package containing primary nodes communicating in real-time:

* **camera_node**: Captures raw video frames from the hardware camera and publishes them to a specific ROS 2 topic.
* **ai_analyzer_node**: Subscribes to the camera topic, processes the frames through the YOLOv11 models (detecting leaves and specific diseases like Early Blight, Mosaic Virus, etc.), draws bounding boxes, and outputs the analyzed feed.

## 🚀 Key Features
* **Middleware:** ROS 2 (Real-time node communication)
* **Computer Vision:** OpenCV & YOLOv11
* **AI Integration:** Deployment of custom-trained plant disease models.
* **Performance:** Real-time inference optimized for Ubuntu-based robotic systems.

## 🛠️ Installation & Usage
Clone the repository into your ROS 2 workspace and build the package:

    cd ~/ros2_ws/src
    git clone [https://https://github.com/UmutUsenmez/robotanik-ros2-vision.git](https://https://github.com/UmutUsenmez/robotanik-ros2-vision.git)
    cd ~/ros2_ws
    colcon build --packages-select robotanik_vision_ros2
    source install/setup.bash

## 👨‍💻 Author
**Feyzullah Umut Üşenmez**
Mechatronics Engineering Student @ YTU
Focus Areas: Autonomous Systems, Computer Vision, Embedded AI
