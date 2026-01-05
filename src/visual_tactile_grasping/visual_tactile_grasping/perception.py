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
import os
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import image_geometry
from rclpy.qos import qos_profile_sensor_data
import math
import tf_transformations # 确保已安装: sudo apt install ros-humble-tf-transformations
from visualization_msgs.msg import Marker

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
            yolo_path = os.path.expanduser('~/vt_grasp_ws/src/yolov5') 
            self.model = torch.hub.load(
                yolo_path,      # 本地路径
                'custom',       # 模式改为 custom
                path=os.path.join(yolo_path, 'yolov5s.pt'), # 指定权重文件路径
                source='local'  # 本地源
            )
            self.model.conf = conf_threshold
            self.model.iou = 0.45
            self.node.get_logger().info("YOLOv5 Loaded.")
        except Exception as e:
            self.node.get_logger().error(f"YOLO Load Failed: {e}")
            
        # 订阅
        # 假设 visual_tactile_grasping.utils 存在，如果不存在请修改这里的 topic 字符串
        try:
            from visual_tactile_grasping.utils import TOPIC_COLOR, TOPIC_DEPTH, TOPIC_INFO
        except ImportError:
            # 默认 fallback
            TOPIC_COLOR = '/camera/color/image_raw'
            TOPIC_DEPTH = '/camera/aligned_depth_to_color/image_raw'
            TOPIC_INFO = '/camera/color/camera_info'

        self.sub_info = node.create_subscription(CameraInfo, TOPIC_INFO, self.info_callback, 10)
        self.sub_color = node.create_subscription(Image, TOPIC_COLOR, self.color_callback, qos_profile_sensor_data)
        self.sub_depth = node.create_subscription(Image, TOPIC_DEPTH, self.depth_callback, qos_profile_sensor_data)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # 调试图像发布器
        self.debug_pub = node.create_publisher(Image, '/yolo/debug_image', 10)
        # [新增] 用于在 RViz 中显示目标点的 3D 球体
        self.marker_pub = node.create_publisher(Marker, '/yolo/target_marker', 10)
    def publish_marker(self, x, y, z):
        """在 RViz 中发布一个红色的球，代表目标位置"""
        marker = Marker()
        marker.header.frame_id = "base_link" # 确保这是你的机械臂基座 Frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "object_location"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0 # 球体方向不重要
        
        marker.scale.x = 0.05 # 直径 5cm
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.a = 1.0 # 不透明
        marker.color.r = 1.0 # 红色
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
        """
        在 YOLO 的检测框内计算物体的主轴角度
        """
        x1, y1, x2, y2 = bbox
        
        h, w = image.shape[:2]
        pad = 5
        x1 = max(0, x1 - pad)
        y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad)
        y2 = min(h, y2 + pad)
        
        roi = image[y1:y2, x1:x2]
        if roi.size == 0: return 0.0
        
        # 图像预处理
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Otsu 阈值
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0
        
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 50: return 0.0 
        
        rect = cv2.minAreaRect(c) 
        center, size, angle_deg = rect
        width, height = size
        
        if width < height:
            angle_deg = 90 + angle_deg
        
        angle_rad = math.radians(angle_deg)
        return angle_rad

    # def detect_object(self):
    #     """
    #     YOLOv5 检测 + 角度计算
    #     :return: (found, u, v, angle_rad)
    #     """
    #     # 1. 安全检查
    #     if self.latest_color_img is None:
    #         self.node.get_logger().warn("No image available for detection.")
    #         return False, 0, 0, 0.0

    #     detections = [] # 初始化，防止报错

    #     try:
    #         # 2. YOLO 推理
    #         results = self.model(self.latest_color_img)
            
    #         # 3. 【关键修复】先获取数据，不论后面画图是否成功
    #         detections = results.xyxy[0].cpu().numpy()

    #         # 4. 尝试发布可视化 (放在 try 块中，即使失败也不影响主逻辑)
    #         annotated_img = results.render()[0]
    #         ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="rgb8")
    #         ros_img.header.frame_id = "calibrated_optical_frame"
    #         ros_img.header.stamp = self.node.get_clock().now().to_msg()
    #         self.debug_pub.publish(ros_img)
            
    #     except Exception as e:
    #         # 这里的异常只记录警告，不应打断流程
    #         self.node.get_logger().warn(f"Detection or visualization warning: {e}")

    #     # 5. 逻辑判断
    #     if len(detections) == 0:
    #         return False, 0, 0, 0.0
        
    #     # 取置信度最高的
    #     best_det = detections[np.argmax(detections[:, 4])]
    #     x1, y1, x2, y2, conf, cls_id = best_det
        
    #     cx = int((x1 + x2) / 2)
    #     cy = int((y1 + y2) / 2)
        
    #     bbox = [int(x1), int(y1), int(x2), int(y2)]
    #     angle = self._calculate_orientation(self.latest_color_img, bbox)
        
    #     name = self.model.names[int(cls_id)]
    #     self.node.get_logger().info(f"Found {name} at ({cx}, {cy}), Angle: {math.degrees(angle):.1f}°")
        
    #     return True, cx, cy, angle
    def detect_object(self):
        """
        YOLOv5 检测 + 角度计算 + 【新增】类别过滤
        :return: (found, u, v, angle_rad)
        """
        # 1. 安全检查
        if self.latest_color_img is None:
            self.node.get_logger().warn("No image available for detection.")
            return False, 0, 0, 0.0

        detections = [] # 初始化

        try:
            # 2. YOLO 推理
            results = self.model(self.latest_color_img)
            
            # 3. 获取数据
            detections = results.xyxy[0].cpu().numpy()

            # 4. 发布可视化 (即使失败也不影响逻辑)
            annotated_img = results.render()[0]
            ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="rgb8")
            ros_img.header.frame_id = "calibrated_optical_frame"
            ros_img.header.stamp = self.node.get_clock().now().to_msg()
            self.debug_pub.publish(ros_img)
            
        except Exception as e:
            self.node.get_logger().warn(f"Detection or visualization warning: {e}")

        # 5. 逻辑判断
        if len(detections) == 0:
            return False, 0, 0, 0.0
        
        # =================【修改开始：类别过滤逻辑】=================
        
        # 定义你想抓取的物体列表 (白名单)
        # 注意：名字必须和 YOLOv5 的 COCO 类别名完全一致 (如 'mouse', 'cup', 'bottle' 等)
        # 如果你想抓所有东西，就把下面的过滤逻辑去掉，改回原来的写法
        TARGET_CLASSES = ['mouse', 'cup', 'bottle', 'apple', 'orange', 'cell phone']
        
        best_det = None
        max_conf = -1.0 # 初始化最大置信度
        
        # 遍历所有检测到的物体
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            
            # 获取当前物体的名字
            class_name = self.model.names[int(cls_id)]
            
            # 筛选条件 1: 这个物体是否在我们的目标清单里？
            if class_name in TARGET_CLASSES:
                # 筛选条件 2: 如果有多个目标，取置信度最高的那个
                if conf > max_conf:
                    max_conf = conf
                    best_det = det
        
        # 如果遍历完一圈，发现虽然有检测到东西（比如人、笔记本），但没有我们要的（鼠标、杯子）
        if best_det is None:
            # 可以在这里打印日志调试
            # self.node.get_logger().info(f"Ignored non-target objects.")
            return False, 0, 0, 0.0
            
        # =================【修改结束】=================
        
        # 6. 解析最佳目标 (使用 best_det)
        x1, y1, x2, y2, conf, cls_id = best_det
        
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        
        # 计算抓取角度
        bbox = [int(x1), int(y1), int(x2), int(y2)]
        angle = self._calculate_orientation(self.latest_color_img, bbox)
        
        name = self.model.names[int(cls_id)]
        self.node.get_logger().info(f"Found TARGET '{name}' at ({cx}, {cy}), Angle: {math.degrees(angle):.1f}°")
        
        return True, cx, cy, angle
    

    def get_object_pose_base_frame(self, u, v, angle_rad):
        """
        获取带角度的 6D 姿态
        """
        pos = self._get_xyz(u, v) 
        if not pos: return None
        bx, by, bz = pos
        
        # 假设机械臂 Base Z 向上，End-Effector Z 垂直向下抓取
        # Roll=180 (翻转Z轴), Pitch=0, Yaw=angle_rad (抓取旋转)
        q = tf_transformations.quaternion_from_euler(math.pi, 0, angle_rad)
        
        return (bx, by, bz, q[0], q[1], q[2], q[3])

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
        # 注意：这里如果 camera_info 的 frame_id 和你的 TF tree 不一致，需要手动指定
        # 如果你的相机发布的是 'camera_color_optical_frame' 但你用 'calibrated_...' 
        # 请确保它们之间有 TF 连接
        ps.header.frame_id = 'calibrated_optical_frame' 
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = pt_cam[0], pt_cam[1], pt_cam[2]
        
        try:
            # 增加 timeout 到 1.0s 提高成功率
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                ps.header.frame_id, 
                rclpy.time.Time(), # 使用最新变换
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pt_base = tf2_geometry_msgs.do_transform_point(ps, trans)
            return [pt_base.point.x, pt_base.point.y, pt_base.point.z]
        except Exception as e:
            self.node.get_logger().warn(f"TF Transform failed: {e}")
            return None