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
from ultralytics import YOLOWorld

class PerceptionModule:
    def __init__(self, node: Node, model_name='yolov8s-world.pt', conf_threshold=0.1):
        self.node = node
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        
        # 数据缓存
        self.latest_color_img = None
        self.latest_depth_img = None
        self.latest_header = None # 🌟 核心：时间戳同步
        self.camera_info = None
        self.model = None 
        
        self.conf_threshold = conf_threshold
        # 物体列表
        self.target_classes = ["mouse", "cup", "bottle", "apple", "orange", "tofu block", "paper ball", "plush toy", "cube", "cylinder", "pingpong ball"]
        self.SIDE_GRASP_OBJECTS = ['cup', 'bottle', 'can', 'cylinder']
        
        self.node.get_logger().info(f"Loading YOLO-World: {model_name}...")
        try:
            self.model = YOLOWorld(model_name)
            self.model.set_classes(self.target_classes)
            self.node.get_logger().info(f"YOLO-World Loaded.")
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
    # 🌟 核心算法 1：ROI 局部点云重心计算 (最准的方法)
    # ------------------------------------------------------------------
    def _get_xyz_roi_centroid(self, bbox):
        if self.latest_depth_img is None or self.camera_info is None or self.latest_header is None: 
            return None

        h_img, w_img = self.latest_depth_img.shape
        x1, y1, x2, y2 = map(int, bbox)
        
        # 1. 确定采样区域 (ROI)
        # 稍微缩小框，排除边缘背景
        scale = 0.15 
        bw, bh = x2 - x1, y2 - y1
        cx1 = max(0, x1 + int(bw * scale))
        cx2 = min(w_img, x2 - int(bw * scale))
        cy1 = max(0, y1 + int(bh * scale))
        cy2 = min(h_img, y2 - int(bh * scale))
        
        roi_depth = self.latest_depth_img[cy1:cy2, cx1:cx2]
        if roi_depth.size == 0: return None

        # 2. 提取有效深度
        valid_mask = (roi_depth > 0) & (roi_depth < 1500) # 1.5m以内
        if not np.any(valid_mask): return None
        
        z_vals = roi_depth[valid_mask] * 0.001 # mm -> m

        # 3. 🌟 空间去噪 (球冠滤波)
        # 只取距离最近的 3cm 数据，剔除桌面背景
        min_z = np.min(z_vals)
        sphere_mask = z_vals < (min_z + 0.03) 
        clean_z = z_vals[sphere_mask]
        
        if clean_z.size == 0: return None

        # 4. 反投影计算重心 (Centroid)
        grid_y, grid_x = np.indices(roi_depth.shape)
        # 还原到整图像素坐标
        pixel_u = (grid_x + cx1)[valid_mask][sphere_mask]
        pixel_v = (grid_y + cy1)[valid_mask][sphere_mask]

        fx = self.camera_model.fx()
        fy = self.camera_model.fy()
        cx = self.camera_model.cx()
        cy = self.camera_model.cy()

        # 向量化计算
        cam_x = (pixel_u - cx) * clean_z / fx
        cam_y = (pixel_v - cy) * clean_z / fy
        
        # 取平均值作为物理重心
        pt_cam_x = np.mean(cam_x)
        pt_cam_y = np.mean(cam_y)
        pt_cam_z = np.mean(clean_z)

        # 5. TF 时间同步变换
        ps = PointStamped()
        ps.header = self.latest_header # 🌟 必须用图像时间戳
        ps.point.x = pt_cam_x
        ps.point.y = pt_cam_y
        ps.point.z = pt_cam_z
        
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                ps.header.frame_id, 
                ps.header.stamp, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pt_base = tf2_geometry_msgs.do_transform_point(ps, trans)
            
            # 6. 坐标系修正 (如果TF反了)
            bx, by, bz = pt_base.point.x, pt_base.point.y, pt_base.point.z
            if bx < 0: bx = bx; by = by 
            
            return [bx, by, bz]
            
        except Exception as e:
            self.node.get_logger().error(f"TF Error: {e}")
            return None

    # ------------------------------------------------------------------
    # 🌟 核心算法 2：智能姿态决策 (融合版)
    # ------------------------------------------------------------------
    def get_object_pose_base_frame(self, u, v, angle_rad, grasp_strategy="TOP", bbox=None):
        # 1. 优先使用点云重心法获取位置
        if bbox:
            pos = self._get_xyz_roi_centroid(bbox)
        else:
            # 回退：如果没有bbox，暂时返回None或用旧方法
            return None
            
        if not pos: return None
        bx, by, bz = pos

        # 2. 智能补偿
        # 纸团：需要抓球心，且需要额外下压
        if grasp_strategy == "TOP":
            bx += 0.02  # X轴补偿 (手眼标定残差)
            by += 0.00
            bz += 0.01 # Z轴下压 (抓球心)
        else:
            # 侧抓物体通常比较高，不需要下压太多
            bx += 0.02
            bz += 0.00
        
        # 3. 姿态计算 (Vector Construction)
        if grasp_strategy == "TOP":
            # 方案 A: 垂直向下 (最稳，适合球/方块)
            # 即使物体旋转，夹爪也保持垂直，只旋转 Yaw 轴对齐物体长边
            # 构造目标坐标系：Z轴向下，X轴沿物体方向
            target_z_axis = np.array([0.0, 0.0, -1.0]) 
            target_x_axis_temp = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
            
        elif grasp_strategy == "SIDE":
            # 方案 B: 侧向水平 (适合瓶子) 
            # Z轴指向物体 (水平)
            dist = math.sqrt(bx**2 + by**2)
            dir_x = bx / dist
            dir_y = by / dist
            target_z_axis = np.array([dir_x, dir_y, 0.0])
            
            # X轴向下 (保持夹爪竖直)
            target_x_axis_temp = np.array([0.0, 0.0, -1.0])
        
        # 4. Gram-Schmidt 正交化 (保证旋转矩阵合法)
        target_y_axis = np.cross(target_z_axis, target_x_axis_temp)
        norm_y = np.linalg.norm(target_y_axis)
        if norm_y < 0.001: target_y_axis = np.array([0.0, 1.0, 0.0])
        else: target_y_axis = target_y_axis / norm_y
            
        target_x_axis = np.cross(target_y_axis, target_z_axis)
        target_x_axis = target_x_axis / np.linalg.norm(target_x_axis)
        # angle_rad = 0.0
        # 5. 转四元数
        R = np.eye(4)
        R[:3, 0] = target_x_axis
        R[:3, 1] = target_y_axis
        R[:3, 2] = target_z_axis
        q = tf_transformations.quaternion_from_matrix(R)
        q = q / np.linalg.norm(q)

        return (bx, by, bz, q[0], q[1], q[2], q[3])

    # 兼容接口
    def get_pose_compatible(self, u, v, angle, strategy):
        bbox = getattr(self, 'current_bbox', None)
        return self.get_object_pose_base_frame(u, v, angle, strategy, bbox)

    def detect_object(self):
        """
        YOLO-World 检测 (调试增强版)
        """
        # 1. 检查模型状态
        if not hasattr(self, 'model') or self.model is None:
            self.node.get_logger().error("Model not loaded!", throttle_duration_sec=5.0)
            return False, 0, 0, 0.0, "TOP"

        # 2. 检查图像数据是否存在
        if self.latest_color_img is None:
            self.node.get_logger().warn("No Image Data! Check camera topic and driver.", throttle_duration_sec=2.0)
            return False, 0, 0, 0.0, "TOP"

        detections = []
        # 默认调试图就是原图（防止没检测到时发黑屏，方便确认相机视野）
        annotated_img = self.latest_color_img.copy()

        # 3. 执行 YOLO 预测
        try:
            results = self.model.predict(self.latest_color_img, conf=self.conf_threshold, verbose=False)
            result = results[0] 
            
            # 如果有检测结果，用 YOLO 自带的绘图功能覆盖 annotated_img
            if len(result.boxes) > 0:
                detections = result.boxes.data.cpu().numpy()
                annotated_img = result.plot() 
        except Exception as e:
            self.node.get_logger().error(f"YOLO Prediction Error: {e}")

        # 4. 🔥🔥🔥 强制发布调试图像 (无论是否检测到物体)
        # 这样你在 RViz 订阅 /yolo/debug_image 就能看到相机到底在看哪里
        try:
            ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
            ros_img.header.frame_id = "camera_color_optical_frame"
            self.debug_pub.publish(ros_img)
        except Exception as e:
            self.node.get_logger().error(f"Publish Debug Image Error: {e}")

        # 5. 如果没检测到，打印日志并返回
        if len(detections) == 0: 
            self.node.get_logger().info("Detected nothing. (Check lighting or threshold)", throttle_duration_sec=2.0)
            return False, 0, 0, 0.0, "TOP"
        
        # 6. 处理检测结果 (找置信度最高的)
        best_det = max(detections, key=lambda x: x[4])
        x1, y1, x2, y2, conf, cls_id = best_det
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        bbox = [int(x1), int(y1), int(x2), int(y2)]
        
        # 保存 bbox 供后续深度计算使用
        self.current_bbox = bbox 
        
        # 计算角度
        angle = self._calculate_orientation(self.latest_color_img, bbox)
        
        # 决策策略
        name = self.model.names[int(cls_id)]
        strategy = "SIDE" if name in self.SIDE_GRASP_OBJECTS else "TOP"
            
        self.node.get_logger().info(f"Target: '{name}', Strategy: {strategy}, Angle: {math.degrees(angle):.1f}°")
        
        return True, cx, cy, angle, strategy

    # ... (publish_marker, callbacks, calculate_orientation 保持不变) ...
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
        try: 
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_header = msg.header
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