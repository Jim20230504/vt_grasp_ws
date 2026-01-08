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
            
        # 订阅话题配置
        # 优先尝试从 utils 导入配置
        try:
            from visual_tactile_grasping.utils import TOPIC_COLOR, TOPIC_DEPTH, TOPIC_INFO
        except ImportError:
            # [适配] RealSense 默认话题
            TOPIC_COLOR = '/camera/color/image_raw'
            TOPIC_DEPTH = '/camera/aligned_depth_to_color/image_raw'
            TOPIC_INFO = '/camera/color/camera_info'

        self.sub_info = node.create_subscription(CameraInfo, TOPIC_INFO, self.info_callback, 10)
        self.sub_color = node.create_subscription(Image, TOPIC_COLOR, self.color_callback, qos_profile_sensor_data)
        self.sub_depth = node.create_subscription(Image, TOPIC_DEPTH, self.depth_callback, qos_profile_sensor_data)
        
        # TF 监听器 (用于查询手眼变换)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # 调试图像发布器
        self.debug_pub = node.create_publisher(Image, '/yolo/debug_image', 10)
        
        # Marker 发布器 (用于 RViz 可视化 目标箭头)
        self.marker_pub = node.create_publisher(Marker, '/yolo/target_marker', 10)

    def publish_marker(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        在 RViz 中发布一个箭头，代表目标位置和抓取方向
        """
        marker = Marker()
        # 画在 base_link 下，因为传入的坐标已经转换到了 base_link
        marker.header.frame_id = "base_link" 
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "grasp_target"
        marker.id = 0
        marker.type = Marker.ARROW # 使用箭头显示方向
        marker.action = Marker.ADD
        
        # 位置
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        
        # 方向 (四元数)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        
        # 尺寸
        marker.scale.x = 0.10 # 箭头长度 10cm
        marker.scale.y = 0.01 # 粗细
        marker.scale.z = 0.01 
        
        # 颜色 (红色)
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
        """
        在 YOLO 的检测框内计算物体的主轴角度 (PCA / MinAreaRect)
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
        
        # Otsu 阈值分割
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return 0.0
        
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 50: return 0.0 
        
        # 计算最小外接矩形
        rect = cv2.minAreaRect(c) 
        center, size, angle_deg = rect
        width, height = size
        
        # 修正角度：使角度对应长边方向
        if width < height:
            angle_deg = 90 + angle_deg
        
        angle_rad = math.radians(angle_deg)
        return angle_rad

    def detect_object(self):
        """
        YOLOv5 检测 + 角度计算 + 类别过滤
        :return: (found, u, v, angle_rad)
        """
        # 1. 安全检查
        if self.latest_color_img is None:
            self.node.get_logger().warn("No image available for detection.", throttle_duration_sec=2.0)
            return False, 0, 0, 0.0

        detections = [] 

        try:
            # 2. YOLO 推理
            results = self.model(self.latest_color_img)
            detections = results.xyxy[0].cpu().numpy()

            # 3. 发布可视化 (使用新的标准光心坐标系名称)
            annotated_img = results.render()[0]
            ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="rgb8")
            ros_img.header.frame_id = "camera_color_optical_frame" # 仅用于显示
            ros_img.header.stamp = self.node.get_clock().now().to_msg()
            self.debug_pub.publish(ros_img)
            
        except Exception as e:
            self.node.get_logger().warn(f"Detection or visualization warning: {e}")

        # 4. 逻辑判断
        if len(detections) == 0:
            return False, 0, 0, 0.0
        
        # ================= 类别过滤逻辑 =================
        # 只抓取列表中的物体
        TARGET_CLASSES = ['mouse', 'cup', 'bottle', 'apple', 'orange', 'cell phone']
        
        best_det = None
        max_conf = -1.0 
        
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            class_name = self.model.names[int(cls_id)]
            
            if class_name in TARGET_CLASSES:
                if conf > max_conf:
                    max_conf = conf
                    best_det = det
        
        if best_det is None:
            # 这里的 info 可以注释掉防止刷屏
            # self.node.get_logger().info(f"Objects detected but none in target list.")
            return False, 0, 0, 0.0
            
        # ===============================================
        
        # 5. 解析最佳目标
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
        获取带角度的 6D 姿态 (Position + Orientation)
        核心逻辑：Target_Quat = Q_grasp_vertical * Q_rotation_z
        """
        # 1. 获取位置 XYZ
        pos = self._get_xyz(u, v) 
        if not pos: return None
        bx, by, bz = pos
        
        # 2. 计算姿态
        
        # 【关键更新：基准垂直姿态】
        # 因为你的 SRDF 末端现在是 'grasp_link' (指尖)，
        # 所以这里的四元数应该是：当夹爪垂直向下时，'grasp_link' 相对于 'base_link' 的旋转。
        #
        # 调试方法：
        # 1. 手动把机械臂移动到夹爪垂直向下的姿态。
        # 2. 终端运行: ros2 run tf2_ros tf2_echo base_link grasp_link
        # 3. 把显示的 Rotation (x,y,z,w) 填入下面：
        
        # 默认值 [1, 0, 0, 0] 是绕 X 轴转 180 度。
        # 如果你的夹爪定义是 Z 轴朝前，那么这个默认值通常是正确的 (让 Z 轴朝下)。
        q_base_vertical = [1.0, 0.0, 0.0, 0.0] 
        
        # 【叠加旋转】
        # 计算 YOLO 带来的 Z 轴旋转 (物体在桌面上的旋转)
        q_rot_z = tf_transformations.quaternion_from_euler(0, 0, angle_rad)
        
        # 叠加：在垂直姿态的基础上，绕 Z 轴旋转
        q_final = tf_transformations.quaternion_multiply(q_base_vertical, q_rot_z)
        
        # 标准化
        norm = np.linalg.norm(q_final)
        q_final = q_final / norm
        
        return (bx, by, bz, q_final[0], q_final[1], q_final[2], q_final[3])

    def _get_xyz(self, u, v):
        """
        查询 TF 树，将像素点 (u,v) 转换为 base_link 下的 (x,y,z)
        """
        if self.latest_depth_img is None or self.camera_info is None: return None
        h, w = self.latest_depth_img.shape
        if not (0 <= u < w and 0 <= v < h): return None
        
        # 获取深度 (中值滤波去噪)
        d_raw = self.latest_depth_img[v-1:v+2, u-1:u+2]
        if d_raw.size == 0: return None
        d_valid = d_raw[d_raw > 0]
        if d_valid.size == 0: return None
        depth_m = np.median(d_valid) * 0.001
        
        # 像素 -> 相机坐标系 (3D射线)
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        pt_cam = np.array(ray) * depth_m
        
        # 构建点消息
        ps = PointStamped()
        # 【关键修改】使用 RealSense 驱动 + URDF 配合后的标准光心名称
        ps.header.frame_id = 'camera_color_optical_frame' 
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = pt_cam[0], pt_cam[1], pt_cam[2]
        
        try:
            # 查询 TF 变换 (光心 -> 机械臂基座)
            # 这一步会自动利用 robot_state_publisher 发布的实时链条
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                ps.header.frame_id, 
                rclpy.time.Time(), # 获取最新变换
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pt_base = tf2_geometry_msgs.do_transform_point(ps, trans)
            return [pt_base.point.x, pt_base.point.y, pt_base.point.z]
        except Exception as e:
            self.node.get_logger().warn(f"TF Transform failed: {e}")
            return None