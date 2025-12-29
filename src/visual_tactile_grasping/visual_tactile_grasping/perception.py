#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import image_geometry
from rclpy.qos import qos_profile_sensor_data
import math

class PerceptionModule:
    def __init__(self, node: Node, model_name='yolov5s', conf_threshold=0.5):
        self.node = node
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        
        self.latest_color_img = None
        self.latest_depth_img = None
        self.camera_info = None
        
        # --- YOLOv5 初始化 ---
        self.node.get_logger().info(f"Loading YOLOv5: {model_name}...")
        try:
            # 这里的 source='ultralytics/yolov5' 根据你的网络情况，可能需要改为本地路径
            self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
            self.model.conf = conf_threshold
            self.model.iou = 0.45
            # 可选：限制只检测特定类别 (如杯子, 碗, 橘子等)
            # self.model.classes = [39, 41, 46, ... ] 
            self.node.get_logger().info("YOLOv5 Loaded.")
        except Exception as e:
            self.node.get_logger().error(f"YOLO Load Failed: {e}")
            
        # 订阅
        from visual_tactile_grasping.utils import TOPIC_COLOR, TOPIC_DEPTH, TOPIC_INFO
        self.sub_info = node.create_subscription(CameraInfo, TOPIC_INFO, self.info_callback, 10)
        self.sub_color = node.create_subscription(Image, TOPIC_COLOR, self.color_callback, qos_profile_sensor_data)
        self.sub_depth = node.create_subscription(Image, TOPIC_DEPTH, self.depth_callback, qos_profile_sensor_data)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

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
        """
        在 YOLO 的检测框内计算物体的主轴角度
        :param image: 原图
        :param bbox: [x1, y1, x2, y2]
        :return: angle (radians, relative to image x-axis)
        """
        x1, y1, x2, y2 = bbox
        
        # 1. 提取 ROI (Region of Interest)
        # 增加一点 padding 以防物体贴边
        h, w = image.shape[:2]
        pad = 5
        x1 = max(0, x1 - pad)
        y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad)
        y2 = min(h, y2 + pad)
        
        roi = image[y1:y2, x1:x2]
        if roi.size == 0: return 0.0
        
        # 2. 图像预处理 (转灰度 -> 模糊 -> 边缘/阈值)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 使用 Otsu 自动阈值
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 3. 找最大轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0
        
        c = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(c) < 50: return 0.0 # 轮廓太小，忽略
        
        # 4. 计算角度 (PCA 或 minAreaRect)
        # 方法 A: minAreaRect (适合矩形物体，)
        rect = cv2.minAreaRect(c) 
        # rect 格式: ((center_x, center_y), (width, height), angle)
        # OpenCV 4.5+ 返回的角度范围通常是 [0, 90] 或 [-90, 0]，定义比较混乱
        # 我们通常取长边对应的角度
        
        center, size, angle_deg = rect
        width, height = size
        
        # 修正角度，使得它是长轴与 X 轴的夹角
        if width < height:
            angle_deg = 90 + angle_deg
        
        angle_rad = math.radians(angle_deg)
        
        # 可视化调试 (在原图上画线)
        # 这里的坐标是相对于 ROI 的，如果要在全图画需要加 (x1, y1)
        # 为了简单，我们只在日志输出
        # self.node.get_logger().info(f"Object Angle: {angle_deg:.2f} deg")
        
        return angle_rad

    def detect_object(self):
        """
        YOLOv5 检测 + 角度计算
        :return: (found, u, v, angle_rad)
        """
        if self.latest_color_img is None:
            self.node.get_logger().warn("No image.")
            return False, 0, 0, 0.0

        # YOLO 推理
        results = self.model(self.latest_color_img)
        detections = results.xyxy[0].cpu().numpy()

        if len(detections) == 0:
            return False, 0, 0, 0.0
        
        # 取置信度最高的
        best_det = detections[np.argmax(detections[:, 4])]
        x1, y1, x2, y2, conf, cls_id = best_det
        
        # 计算中心
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        
        # 计算角度 (新增)
        bbox = [int(x1), int(y1), int(x2), int(y2)]
        angle = self._calculate_orientation(self.latest_color_img, bbox)
        
        # 打印信息
        name = self.model.names[int(cls_id)]
        self.node.get_logger().info(f"Found {name} at ({cx}, {cy}), Angle: {math.degrees(angle):.1f}°")
        
        return True, cx, cy, angle

    def get_object_pose_base_frame(self, u, v, angle_rad):
        """
        获取带角度的 6D 姿态 (Position + Orientation)
        :return: (x, y, z, qx, qy, qz, qw)
        """
        # 1. 获取位置
        pos = self._get_xyz(u, v) # 复用之前的逻辑提取为 _get_xyz
        if not pos: return None
        bx, by, bz = pos
        
        # 2. 计算四元数
        # 目标：夹爪垂直向下 (Z轴向下)，且绕Z轴旋转 angle_rad
        # 基础朝向 (垂直向下): 
        # 如果这是 UR5/RM65，通常末端 Z 轴是指出法兰的。要让它指向桌面，需要绕 Y 转 180 (或 X 转 180)
        # 这里我们需要构建一个旋转矩阵：
        #   Base Frame 下：
        #   Z_tool = (0, 0, -1)  (垂直向下)
        #   X_tool = (cos(ang), sin(ang), 0) (抓取方向)
        #   Y_tool = Z cross X
        
        # 使用 tf_transformations 或 scipy.spatial.transform
        # 这里手动构建简单的欧拉角 -> 四元数
        # 假设机械臂 Base Frame 的 Z 是向上的
        # 我们要: Roll=180(翻转让Z向下), Pitch=0, Yaw=angle_rad
        
        import tf_transformations
        # 注意：不同机械臂定义不同。
        # 这里假设：Roll=PI (让Z向下), Yaw=angle (抓取角度)
        q = tf_transformations.quaternion_from_euler(math.pi, 0, angle_rad)
        
        return (bx, by, bz, q[0], q[1], q[2], q[3])

    def _get_xyz(self, u, v):
        # ... (之前的 get_object_position_base_frame 逻辑，简化版) ...
        if self.latest_depth_img is None or self.camera_info is None: return None
        if not (0 <= u < self.latest_depth_img.shape[1] and 0 <= v < self.latest_depth_img.shape[0]): return None
        
        # 使用 3x3 区域中值滤波获取深度
        d_raw = self.latest_depth_img[v-1:v+2, u-1:u+2]
        if d_raw.size == 0: return None
        # 过滤0值
        d_valid = d_raw[d_raw > 0]
        if d_valid.size == 0: return None
        depth_m = np.median(d_valid) * 0.001
        
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        pt_cam = np.array(ray) * depth_m
        
        ps = PointStamped()
        ps.header.frame_id = self.camera_info.header.frame_id
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = pt_cam[0], pt_cam[1], pt_cam[2]
        
        try:
            trans = self.tf_buffer.lookup_transform('base_link', ps.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            pt_base = tf2_geometry_msgs.do_transform_point(ps, trans)
            return [pt_base.point.x, pt_base.point.y, pt_base.point.z]
        except:
            return None