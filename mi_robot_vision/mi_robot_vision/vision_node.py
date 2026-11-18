#!/usr/bin/env python3
import os
import json
import time
import numpy as np
import cv2
import joblib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from mi_robot_vision.train_and_eval_ai import CLASSES, extract_features, classify_classical_hsv

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        pkg_share = get_package_share_directory('mi_robot_vision')
        model_path = os.path.join(pkg_share, 'models', 'color_mlp_model.joblib')
        self.ai_model = joblib.load(model_path)
        self.get_logger().info(f'Modelo IA cargado correctamente desde {model_path}')

        self.fx = 361.193
        self.fy = 361.193
        self.cx = 320.0
        self.cy = 240.0

        self.cam_x_world = 0.52
        self.cam_y_world = 0.00
        self.cam_z_world = 0.40
        self.obj_z_world = 0.03
        self.depth_z = 0.31

        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, 10)
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)

        self.pub_detections = self.create_publisher(String, '/detected_objects', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/debug_image', 10)

    def info_cb(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug_img = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, (0, 55, 25), (180, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        now_stamp = self.get_clock().now().to_msg()
        obj_idx = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 400 or area > 25000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            u = M['m10'] / M['m00']
            v = M['m01'] / M['m00']

            pad = 6
            x1 = max(0, x - pad)
            y1 = max(0, y - pad)
            x2 = min(frame.shape[1], x + w + pad)
            y2 = min(frame.shape[0], y + h + pad)
            patch = frame[y1:y2, x1:x2]

            t0 = time.perf_counter()
            feats = extract_features(patch)
            probs = self.ai_model.predict_proba([feats])[0]
            ai_cls_idx = int(np.argmax(probs))
            ai_label = CLASSES[ai_cls_idx]
            ai_conf = float(probs[ai_cls_idx])
            ai_ms = (time.perf_counter() - t0) * 1000.0

            t0 = time.perf_counter()
            cv_cls_idx = classify_classical_hsv(patch)
            cv_label = CLASSES[cv_cls_idx]
            cv_ms = (time.perf_counter() - t0) * 1000.0

            x_opt = ((u - self.cx) * self.depth_z) / self.fx
            y_opt = ((v - self.cy) * self.depth_z) / self.fy
            z_opt = self.depth_z

            x_world = self.cam_x_world - y_opt
            y_world = self.cam_y_world - x_opt
            z_world = self.obj_z_world

            frame_name = f'detected_obj_{obj_idx}'

            t = TransformStamped()
            t.header.stamp = now_stamp
            t.header.frame_id = 'camera_optical_frame'
            t.child_frame_id = frame_name
            t.transform.translation.x = float(x_opt)
            t.transform.translation.y = float(y_opt)
            t.transform.translation.z = float(z_opt)
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

            detections.append({
                'id': frame_name,
                'color_ai': ai_label,
                'confidence_ai': round(ai_conf, 3),
                'color_classical': cv_label,
                'ai_time_ms': round(ai_ms, 3),
                'cv_time_ms': round(cv_ms, 3),
                'x': round(x_world, 4),
                'y': round(y_world, 4),
                'z': round(z_world, 4)
            })

            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.circle(debug_img, (int(u), int(v)), 4, (255, 255, 255), -1)
            label_str = f"{ai_label.upper()} ({ai_conf*100:.0f}%) [{x_world:.2f},{y_world:.2f}]"
            cv2.putText(debug_img, label_str, (x - 10, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

            obj_idx += 1

        out_msg = String()
        out_msg.data = json.dumps(detections)
        self.pub_detections.publish(out_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
