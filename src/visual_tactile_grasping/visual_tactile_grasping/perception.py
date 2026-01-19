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

# 引入 YOLO-World
from ultralytics import YOLOWorld

class PerceptionModule:
    def __init__(self, node: Node, model_name='yolov8s-world.pt', conf_threshold=0.1):
        self.node = node
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        
        self.latest_color_img = None
        self.latest_depth_img = None
        self.camera_info = None
        self.model = None 
        
        self.conf_threshold = conf_threshold
        # 定义所有可能的物体
        self.target_classes = ["mouse", "cup", "bottle", "apple", "orange", "tofu block", "paper ball", "plush toy", "cube", "cylinder"]
        
        # 定义必须“侧抓”的高个子物体
        self.SIDE_GRASP_OBJECTS = ['cup', 'bottle', 'can', 'cylinder']
        
        self.node.get_logger().info(f"Loading YOLO-World: {model_name}...")
        try:
            self.model = YOLOWorld(model_name)
            self.model.set_classes(self.target_classes)
            self.node.get_logger().info(f"YOLO-World Loaded. Classes: {self.target_classes}")
        except Exception as e:
            self.node.get_logger().error(f"YOLO Load Failed: {e}")
            
        # 话题配置
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

    # ------------------------------------------------------------------
    # 🌟 核心算法：向量构建法 (Vector Construction)
    # 自动计算 Top 和 Side 两种策略的完美姿态
    # ------------------------------------------------------------------
    def get_object_pose_base_frame(self, u, v, angle_rad, grasp_strategy="TOP"):
        pos = self._get_xyz(u, v) 
        if not pos: return None
        bx, by, bz = pos
        
        # 使用 numpy 向量运算构建旋转矩阵
        
        if grasp_strategy == "TOP":
            # --- 场景 A: 顶抓 ---
            # 目标 Z 轴 (Approach): 垂直指向地面 [0, 0, -1]
            target_z_axis = np.array([0.0, 0.0, -1.0])
            
            # 目标 X 轴 (Orientation): 指向物体旋转方向 (cos, sin, 0)
            target_x_axis_temp = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
            
        elif grasp_strategy == "SIDE":
            # --- 场景 B: 侧抓 ---
            # 假设机械臂在 (0,0)，物体在 (bx, by)。
            # 目标 Z 轴 (Approach): 应该水平指向物体 (从 Robot -> Object)
            # 计算平面方向向量
            dist = math.sqrt(bx**2 + by**2)
            if dist < 0.001: return None # 重合了，无法计算方向
            
            dir_x = bx / dist
            dir_y = by / dist
            
            # Z轴：水平指向物体
            target_z_axis = np.array([dir_x, dir_y, 0.0])
            
            # X轴：垂直向下 [0, 0, -1] (这样夹爪是“握手”姿态/侧立姿态，不容易撞桌面)
            # 如果你想要夹爪平躺着去夹，把这里改为 [dir_y, -dir_x, 0] (水平切线)
            target_x_axis_temp = np.array([0.0, 0.0, -1.0]) 
            
        else:
            return None

        # --- 施密特正交化 (Gram-Schmidt) ---
        # 1. 算出 Y 轴 = Z 叉乘 X
        target_y_axis = np.cross(target_z_axis, target_x_axis_temp)
        norm_y = np.linalg.norm(target_y_axis)
        if norm_y < 0.001: target_y_axis = np.array([0.0, 1.0, 0.0]) # 防止奇异
        else: target_y_axis = target_y_axis / norm_y
            
        # 2. 重新算出严格垂直的 X 轴 = Y 叉乘 Z
        target_x_axis = np.cross(target_y_axis, target_z_axis)
        target_x_axis = target_x_axis / np.linalg.norm(target_x_axis)
        
        # 3. 构建矩阵
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, 0] = target_x_axis # Col 0 = X
        rotation_matrix[:3, 1] = target_y_axis # Col 1 = Y
        rotation_matrix[:3, 2] = target_z_axis # Col 2 = Z
        
        # 4. 转四元数
        q_final = tf_transformations.quaternion_from_matrix(rotation_matrix)
        q_final = q_final / np.linalg.norm(q_final)

        return (bx, by, bz, q_final[0], q_final[1], q_final[2], q_final[3])

    def detect_object(self):
        """
        YOLO-World 检测 + 决策 (Top vs Side)
        """
        if not hasattr(self, 'model') or self.model is None:
            self.node.get_logger().error("Model not loaded!", throttle_duration_sec=5.0)
            return False, 0, 0, 0.0, "TOP" # 默认返回 TOP 防止崩溃

        if self.latest_color_img is None:
            return False, 0, 0, 0.0, "TOP"

        detections = [] 
        try:
            results = self.model.predict(self.latest_color_img, conf=self.conf_threshold, verbose=False)
            result = results[0] 
            if len(result.boxes) > 0:
                detections = result.boxes.data.cpu().numpy()
            
            # 发布调试图
            annotated_img = result.plot() 
            ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
            ros_img.header.frame_id = "camera_color_optical_frame"
            self.debug_pub.publish(ros_img)
        except Exception: pass

        if len(detections) == 0:
            return False, 0, 0, 0.0, "TOP"
        
        # 策略：找置信度最高的
        best_det = None
        max_conf = -1.0 
        
        for det in detections:
            conf = det[4]
            if conf > max_conf:
                max_conf = conf
                best_det = det
        
        if best_det is None: return False, 0, 0, 0.0, "TOP"
            
        x1, y1, x2, y2, conf, cls_id = best_det
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        
        # 计算物体角度
        bbox = [int(x1), int(y1), int(x2), int(y2)]
        angle = self._calculate_orientation(self.latest_color_img, bbox)
        
        # 获取名字并决策
        name = self.model.names[int(cls_id)]
        
        # 🌟 决策逻辑：如果是高个子，就侧抓
        if name in self.SIDE_GRASP_OBJECTS:
            strategy = "SIDE"
        else:
            strategy = "TOP"
            
        self.node.get_logger().info(f"Target: '{name}', Strategy: {strategy}, Angle: {math.degrees(angle):.1f}°")
        
        # 🌟 返回值增加 strategy
        return True, cx, cy, angle, strategy

    
    def publish_marker(self, x, y, z, qx, qy, qz, qw):
        marker = Marker()
        marker.header.frame_id = "base_link" 
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "grasp_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = z
        marker.pose.orientation.x = qx; marker.pose.orientation.y = qy; marker.pose.orientation.z = qz; marker.pose.orientation.w = qw
        marker.scale.x = 0.1; marker.scale.y = 0.01; marker.scale.z = 0.01 
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def info_callback(self, msg):
        if self.camera_info is None: self.camera_info = msg; self.camera_model.fromCameraInfo(msg)
    def color_callback(self, msg):
        try: self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass
    def depth_callback(self, msg):
        try: self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except: pass
    def _calculate_orientation(self, image, bbox):
        x1, y1, x2, y2 = bbox; h, w = image.shape[:2]; pad = 5
        x1 = max(0, x1 - pad); y1 = max(0, y1 - pad); x2 = min(w, x2 + pad); y2 = min(h, y2 + pad)
        roi = image[y1:y2, x1:x2]
        if roi.size == 0: return 0.0
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 50: return 0.0 
        rect = cv2.minAreaRect(c); size = rect[1]; angle_deg = rect[2]
        if size[0] < size[1]: angle_deg = 90 + angle_deg
        return math.radians(angle_deg)
    
    def _get_xyz(self, u, v):
        if self.latest_depth_img is None or self.camera_info is None: return None
        h, w = self.latest_depth_img.shape
        if not (0 <= u < w and 0 <= v < h): return None
        d_raw = self.latest_depth_img[v-1:v+2, u-1:u+2]
        if d_raw.size == 0: return None
        depth_m = np.median(d_raw[d_raw > 0]) * 0.001
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        pt_cam = np.array(ray) * depth_m
        ps = PointStamped()
        ps.header.frame_id = 'camera_color_optical_frame' 
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = pt_cam[0], pt_cam[1], pt_cam[2]
        try:
            trans = self.tf_buffer.lookup_transform('base_link', ps.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            pt = tf2_geometry_msgs.do_transform_point(ps, trans)
            return [pt.point.x, pt.point.y, pt.point.z]
        except: return None