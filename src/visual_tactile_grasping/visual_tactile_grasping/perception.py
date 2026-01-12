#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import image_geometry
from rclpy.qos import qos_profile_sensor_data
import math
import tf_transformations
from visualization_msgs.msg import Marker

# 🌟 引入 YOLO-World (确保已 pip install ultralytics)
from ultralytics import YOLOWorld

class PerceptionModule:
    def __init__(self, node: Node, model_name='yolov8s-world.pt', conf_threshold=0.1):
        self.node = node
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        
        self.latest_color_img = None
        self.latest_depth_img = None
        self.camera_info = None
        
        # --- YOLO-World 初始化 ---
        self.node.get_logger().info(f"Loading YOLO-World: {model_name}...")
        try:
            # 第一次运行时会自动下载权重文件
            self.model = YOLOWorld(model_name)
            
            # 🌟【关键】设置你想检测的物体类别 (Open Vocabulary)
            
            
            self.target_classes = ["mouse", "cup", "bottle", "apple", "orange", "tofu block", "paper ball", "plush toy"]
            self.model.set_classes(self.target_classes)
            
            self.conf_threshold = conf_threshold
            self.node.get_logger().info(f"YOLO-World Loaded. Classes: {self.target_classes}")
            
        except Exception as e:
            self.node.get_logger().error(f"YOLO Load Failed: {e}")
            
        # 话题配置 (保持不变)
        try:
            from visual_tactile_grasping.utils import TOPIC_COLOR, TOPIC_DEPTH, TOPIC_INFO
        except ImportError:
            TOPIC_COLOR = '/camera/color/image_raw'
            TOPIC_DEPTH = '/camera/aligned_depth_to_color/image_raw'
            TOPIC_INFO = '/camera/color/camera_info'

        self.sub_info = node.create_subscription(CameraInfo, TOPIC_INFO, self.info_callback, 10)
        self.sub_color = node.create_subscription(Image, TOPIC_COLOR, self.color_callback, qos_profile_sensor_data)
        self.sub_depth = node.create_subscription(Image, TOPIC_DEPTH, self.depth_callback, qos_profile_sensor_data)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.debug_pub = node.create_publisher(Image, '/yolo/debug_image', 10)
        self.marker_pub = node.create_publisher(Marker, '/yolo/target_marker', 10)

    
    
    def publish_marker(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        marker = Marker()
        marker.header.frame_id = "base_link" 
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "grasp_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.10
        marker.scale.y = 0.01
        marker.scale.z = 0.01 
        marker.color.a = 1.0 
        marker.color.r = 1.0 
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_model.fromCameraInfo(msg)

    def color_callback(self, msg):
        try:
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e: pass

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e: pass

    def _calculate_orientation(self, image, bbox):
        x1, y1, x2, y2 = bbox
        h, w = image.shape[:2]
        pad = 5
        x1 = max(0, x1 - pad); y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad); y2 = min(h, y2 + pad)
        roi = image[y1:y2, x1:x2]
        if roi.size == 0: return 0.0
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 50: return 0.0 
        rect = cv2.minAreaRect(c) 
        center, size, angle_deg = rect
        width, height = size
        if width < height: angle_deg = 90 + angle_deg
        return math.radians(angle_deg)

    def _get_xyz(self, u, v):
        if self.latest_depth_img is None or self.camera_info is None: return None
        h, w = self.latest_depth_img.shape
        if not (0 <= u < w and 0 <= v < h): return None
        d_raw = self.latest_depth_img[v-1:v+2, u-1:u+2]
        if d_raw.size == 0: return None
        d_valid = d_raw[d_raw > 0]
        if d_valid.size == 0: return None
        depth_m = np.median(d_valid) * 0.001
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        pt_cam = np.array(ray) * depth_m
        ps = PointStamped()
        ps.header.frame_id = 'camera_color_optical_frame' 
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = pt_cam[0], pt_cam[1], pt_cam[2]
        try:
            trans = self.tf_buffer.lookup_transform('base_link', ps.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            pt_base = tf2_geometry_msgs.do_transform_point(ps, trans)
            return [pt_base.point.x, pt_base.point.y, pt_base.point.z]
        except Exception as e:
            self.node.get_logger().warn(f"TF Transform failed: {e}")
            return None

    def get_object_pose_base_frame(self, u, v, angle_rad):
        pos = self._get_xyz(u, v) 
        if not pos: return None
        bx, by, bz = pos
        # 默认向下姿态 [1, 0, 0, 0] (绕X轴转180度)
        q_base_vertical = [1.0, 0.0, 0.0, 0.0] 
        q_rot_z = tf_transformations.quaternion_from_euler(0, 0, angle_rad)
        q_final = tf_transformations.quaternion_multiply(q_base_vertical, q_rot_z)
        norm = np.linalg.norm(q_final)
        q_final = q_final / norm
        return (bx, by, bz, q_final[0], q_final[1], q_final[2], q_final[3])

    # ---------------------------------------------------------------
    # 🌟 核心修改：新的检测函数 (detect_object)
    # ---------------------------------------------------------------
    def detect_object(self):
        """
        YOLO-World 检测 + 角度计算
        """
        if self.latest_color_img is None:
            self.node.get_logger().warn("No image available for detection.", throttle_duration_sec=2.0)
            return False, 0, 0, 0.0

        detections = [] 

        try:
            # 🌟 YOLO-World 推理
            # conf: 置信度阈值, verbose: 是否打印日志
            results = self.model.predict(self.latest_color_img, conf=self.conf_threshold, verbose=False)
            
            # results[0] 包含第一张图的结果
            result = results[0] 
            
            # 将结果转为 numpy 方便处理
            # boxes.data 格式: [x1, y1, x2, y2, conf, cls]
            if len(result.boxes) > 0:
                detections = result.boxes.data.cpu().numpy()

            # 🌟 发布可视化 (带标注的图)
            # plot() 方法会自动画框和标签
            annotated_img = result.plot() 
            ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8") # plot()返回的是BGR
            ros_img.header.frame_id = "camera_color_optical_frame"
            ros_img.header.stamp = self.node.get_clock().now().to_msg()
            self.debug_pub.publish(ros_img)
            
        except Exception as e:
            self.node.get_logger().warn(f"Detection or visualization warning: {e}")

        if len(detections) == 0:
            return False, 0, 0, 0.0
        
        # 策略：选择置信度最高的那个
        # (YOLO-World 已经过滤了我们 set_classes 之外的东西，所以这里的都是我们想要的)
        best_det = None
        max_conf = -1.0 
        
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            if conf > max_conf:
                max_conf = conf
                best_det = det
        
        if best_det is None: return False, 0, 0, 0.0
            
        # 解析最佳目标
        x1, y1, x2, y2, conf, cls_id = best_det
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        
        # 计算角度
        bbox = [int(x1), int(y1), int(x2), int(y2)]
        angle = self._calculate_orientation(self.latest_color_img, bbox)
        
        # 获取类别名称
        name = self.model.names[int(cls_id)]
        
        self.node.get_logger().info(f"Found TARGET '{name}' at ({cx}, {cy}), Angle: {math.degrees(angle):.1f}°")
        return True, cx, cy, angle