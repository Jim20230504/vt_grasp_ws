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

class PerceptionModule:
    def __init__(self, node: Node):
        """
        初始化视觉模块
        :param node: 父ROS2节点句柄，用于创建订阅者和日志
        """
        self.node = node
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        
        # 数据缓存
        self.latest_color_img = None
        self.latest_depth_img = None
        self.camera_info = None
        
        # 订阅话题
        from visual_tactile_grasping.utils import TOPIC_COLOR, TOPIC_DEPTH, TOPIC_INFO
        
        self.sub_info = node.create_subscription(
            CameraInfo, TOPIC_INFO, self.info_callback, 10)
        self.sub_color = node.create_subscription(
            Image, TOPIC_COLOR, self.color_callback, 10)
        self.sub_depth = node.create_subscription(
            Image, TOPIC_DEPTH, self.depth_callback, 10)
            
        # TF Buffer 用于坐标变换 (Camera -> Base)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        self.node.get_logger().info("Perception Module Initialized.")

    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_model.fromCameraInfo(msg)
            self.node.get_logger().info("Camera Info Received.")

    def color_callback(self, msg):
        try:
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.node.get_logger().error(f"CV Bridge Error (Color): {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.node.get_logger().error(f"CV Bridge Error (Depth): {e}")

    def detect_object(self):
        """
        使用OpenCV检测图像中最显著的物体（例如鸡蛋或豆腐）。
        简单逻辑：转灰度 -> 模糊 -> 阈值 -> 轮廓 -> 最大面积
        :return: (found, u, v) found为bool, (u, v)为像素坐标
        """
        if self.latest_color_img is None:
            self.node.get_logger().warn("No image available for detection.")
            return False, 0, 0

        img = self.latest_color_img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 这里可以使用自适应阈值或Canny边缘检测，根据物体颜色调整
        # 假设物体与背景对比度较高 (例如白色豆腐在黑色桌面上)
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, 0, 0
            
        # 找到面积最大的轮廓
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        
        if area < 500: # 过滤噪点
            return False, 0, 0
            
        # 计算中心矩
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return True, cX, cY
        
        return False, 0, 0

    def get_object_position_base_frame(self, u, v):
        """
        获取像素点(u,v)在Robot Base Frame下的3D坐标
        :return: [x, y, z] (meters) or None
        """
        if self.latest_depth_img is None or self.camera_info is None:
            self.node.get_logger().warn("Depth or Camera Info missing.")
            return None

        # 1. 获取深度值 (单位 mm -> m)
        # 注意边界检查
        if 0 <= v < self.latest_depth_img.shape[0] and 0 <= u < self.latest_depth_img.shape[1]:
            depth_raw = self.latest_depth_img[v, u]
            depth_m = depth_raw * 0.001 # RealSense默认单位通常是mm
        else:
            return None

        if depth_m == 0:
            self.node.get_logger().warn("Invalid depth (0) at point.")
            return None

        # 2. 反投影: Pixel (u, v) -> Camera Frame (x, y, z)
        # ray 是归一化向量 (z=1)
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        camera_point = np.array(ray) * depth_m
        
        # 3. 构建 PointStamped 消息
        p_camera = PointStamped()
        p_camera.header.frame_id = self.camera_info.header.frame_id # 通常是 "camera_color_optical_frame"
        p_camera.header.stamp = self.node.get_clock().now().to_msg()
        p_camera.point.x = camera_point[0]
        p_camera.point.y = camera_point[1]
        p_camera.point.z = camera_point[2]

        # 4. TF 变换: Camera Frame -> base_link
        try:
            # 等待变换关系可用，超时1.0秒
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                p_camera.header.frame_id, 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            p_base = tf2_geometry_msgs.do_transform_point(p_camera, transform)
            
            self.node.get_logger().info(f"Target found at Base Frame: x={p_base.point.x:.3f}, y={p_base.point.y:.3f}, z={p_base.point.z:.3f}")
            return [p_base.point.x, p_base.point.y, p_base.point.z]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().error(f"TF Transform Error: {e}")
            return None