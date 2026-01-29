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
import open3d as o3d

class PerceptionModule:
    def __init__(self, node: Node, model_name='yolov8s-world.pt', conf_threshold=0.1):
        self.node = node
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        
        # 数据缓存
        self.latest_color_img = None
        self.latest_depth_img = None
        self.latest_header = None
        self.camera_info = None
        self.model = None 
        self.current_bbox = None
        
        self.conf_threshold = conf_threshold
        
        # === 1. 精准分类清单 (Tier 1) ===
        # 这里放你明确知道名字的东西
        self.target_classes = [
            "mouse", "cup", "bottle", "apple", "white charger", "sponge", 
            "tofu block", "crumpled paper", "paper ball", "trash", # 纸团建议用 crumpled paper
            "plush toy", "cube", "cylinder", "pingpong ball"
        ]
        
        # === 2. 通用泛化清单 (Tier 2) ===
        # 当精准识别失败时，用这些词兜底，YOLO 对这些词的召回率极高
        self.generic_classes = ["object", "item", "something", "obstacle"]

        # 策略配置
        self.SIDE_GRASP_OBJECTS = ['cup', 'bottle', 'can', 'cylinder']
        self.CYLINDER_OBJECTS = ['bottle', 'can', 'cup', 'cylinder']
        
        self.node.get_logger().info(f"Loading YOLO-World: {model_name}...")
        try:
            self.model = YOLOWorld(model_name)
            # 初始加载精准列表
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

    # ========================================================================================
    # 模块 1: 智能检测逻辑 (三级火箭)
    # ========================================================================================
    
    def detect_object(self):
        """
        三级检测策略：
        1. 精准 YOLO (Target Classes)
        2. 泛化 YOLO (Generic Classes)
        3. 几何兜底 (Depth Segmentation)
        """
        if not hasattr(self, 'model') or self.model is None:
            return False, 0, 0, 0.0, "TOP", "none"

        if self.latest_color_img is None or self.latest_depth_img is None:
            self.node.get_logger().warn("No Image Data!", throttle_duration_sec=2.0)
            return False, 0, 0, 0.0, "TOP", "none"

        annotated_img = self.latest_color_img.copy()
        final_bbox = None
        final_name = "unknown"
        detection_source = "NONE"

        # --- Tier 1: 精准检测 ---
        self.model.set_classes(self.target_classes)
        res_t1 = self.model.predict(self.latest_color_img, conf=self.conf_threshold, verbose=False)[0]
        
        if len(res_t1.boxes) > 0:
            # 找到置信度最高的
            best_box = max(res_t1.boxes.data.cpu().numpy(), key=lambda x: x[4])
            final_bbox = best_box[:4].astype(int)
            cls_id = int(best_box[5])
            final_name = self.model.names[cls_id]
            detection_source = "TIER_1_PRECISE"
            annotated_img = res_t1.plot()

        # --- Tier 2: 泛化检测 (如果 T1 失败) ---
        if final_bbox is None:
            # 切换到通用词汇
            self.model.set_classes(self.generic_classes)
            #稍微降低阈值，宁可抓错不可放过
            res_t2 = self.model.predict(self.latest_color_img, conf=0.05, verbose=False)[0]
            
            if len(res_t2.boxes) > 0:
                best_box = max(res_t2.boxes.data.cpu().numpy(), key=lambda x: x[4])
                final_bbox = best_box[:4].astype(int)
                final_name = "unknown_object" # 泛化物体统一叫 unknown
                detection_source = "TIER_2_GENERIC"
                annotated_img = res_t2.plot()

        # --- Tier 3: 几何兜底 (如果 AI 全挂了) ---
        if final_bbox is None:
            # 调用 Open3D 直接找凸起物
            bbox_geo = self._detect_unknown_by_geometry()
            if bbox_geo is not None:
                final_bbox = bbox_geo
                final_name = "geometric_blob"
                detection_source = "TIER_3_GEOMETRY"
                # 手动画框
                cv2.rectangle(annotated_img, (final_bbox[0], final_bbox[1]), (final_bbox[2], final_bbox[3]), (0, 0, 255), 2)
                cv2.putText(annotated_img, "Geo-Discovery", (final_bbox[0], final_bbox[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # --- 结果发布 ---
        try:
            ros_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
            ros_img.header.frame_id = "camera_color_optical_frame"
            self.debug_pub.publish(ros_img)
        except: pass

        if final_bbox is None:
            self.node.get_logger().info("Scan complete: No objects found.", throttle_duration_sec=2.0)
            return False, 0, 0, 0.0, "TOP", "none"

        # --- 后处理 ---
        x1, y1, x2, y2 = final_bbox
        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
        self.current_bbox = [int(x1), int(y1), int(x2), int(y2)]
        
        # 计算角度 (如果是 Tier 3，角度可能算不准，默认为 0)
        angle = 0.0
        if detection_source != "TIER_3_GEOMETRY":
            angle = self._calculate_orientation(self.latest_color_img, self.current_bbox)
        
        # 策略决策
        strategy = "SIDE" if final_name in self.SIDE_GRASP_OBJECTS else "TOP"
        if detection_source == "TIER_3_GEOMETRY":
            strategy = "TOP" # 几何发现的未知物体，顶抓最稳

        self.node.get_logger().info(f"[{detection_source}] Found: '{final_name}', Strategy: {strategy}")
        
        return True, cx, cy, angle, strategy, final_name

    # ========================================================================================
    # 模块 2: 几何感知引擎 (Tier 3 核心)
    # ========================================================================================
    
    def _detect_unknown_by_geometry(self):
        """
        [黑魔法] 不依赖 AI，直接用深度图找“桌子上凸出来的东西”
        返回: bbox [x1, y1, x2, y2] 或 None
        """
        if self.latest_depth_img is None: return None
        
        # 1. 降采样 (加速计算)
        scale = 0.5 
        small_depth = cv2.resize(self.latest_depth_img, (0,0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        h, w = small_depth.shape
        
        # 2. 转点云
        # 简单的逆投影 (为了速度，不构建完整 Pinhole)
        # 假设我们只关心相对高度，直接用 Z 值切分
        z_vals = small_depth * 0.001
        
        # 3. 桌面分割 (简单阈值法 或 RANSAC)
        # 假设桌面在 0.5m - 1.2m 之间
        # 任何比桌面“高”出 2cm 的东西都是物体
        # 注意：这里需要一个更鲁棒的方法：RANSAC 找最大的平面（桌子），然后取平面外的点
        
        # --- 使用 Open3D 进行快速 RANSAC ---
        # 必须先转成 o3d 点云，直接复用 _roi_to_pcd 但针对全图
        # 注意：这里我们传入整个缩小的图
        pcd = self._roi_to_pcd(small_depth, 0, 0) # 这里需要修改 _roi_to_pcd 的内参缩放，或者简化处理
        
        # 为了简单且不破坏 _roi_to_pcd，这里手写一个简易版转换
        # (真实工程中建议专门写一个全图转换函数)
        if pcd is None: return None
        
        # A. 移除桌面
        try:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
            pcd_objects = pcd.select_by_index(inliers, invert=True) # 剩下的就是物体
        except: return None
        
        # B. 聚类 (DBSCAN) 找最大的一坨
        if len(pcd_objects.points) < 50: return None
        
        # eps=0.02 (2cm), min_points=30
        labels = np.array(pcd_objects.cluster_dbscan(eps=0.03, min_points=30, print_progress=False))
        if len(labels) == 0: return None
        max_label = labels.max()
        if max_label < 0: return None # 全是噪声(-1)
        
        # 找到最大的簇 (假设是主要物体)
        counts = np.bincount(labels[labels >= 0])
        largest_cluster_idx = np.argmax(counts)
        
        # 提取该簇的点
        object_indices = np.where(labels == largest_cluster_idx)[0]
        pcd_final_obj = pcd_objects.select_by_index(object_indices)
        
        # C. 投影回 2D 生成 Bounding Box
        # 获取 3D 点的包围盒
        min_bound = pcd_final_obj.get_min_bound() # (x, y, z)
        max_bound = pcd_final_obj.get_max_bound()
        
        # 将 3D 边界点反投回 2D 像素
        # u = fx * x / z + cx
        fx = self.camera_model.fx()
        fy = self.camera_model.fy()
        cx = self.camera_model.cx()
        cy = self.camera_model.cy()
        
        # 投射 8 个角点找出 2D 极值 (简化版：直接投射中心和范围)
        # 只要找出 u_min, u_max, v_min, v_max
        points = np.asarray(pcd_final_obj.points)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        
        u = (x * fx / z) + cx
        v = (y * fy / z) + cy
        
        x1 = int(np.min(u))
        x2 = int(np.max(u))
        y1 = int(np.min(v))
        y2 = int(np.max(v))
        
        # 边界检查
        h_raw, w_raw = self.latest_depth_img.shape
        x1 = max(0, x1); y1 = max(0, y1)
        x2 = min(w_raw, x2); y2 = min(h_raw, y2)
        
        if (x2 - x1) < 10 or (y2 - y1) < 10: return None
        
        return [x1, y1, x2, y2]

    # ========================================================================================
    # 模块 3: RANSAC 与 OBB 核心算法 (保持你刚才确认过的版本)
    # ========================================================================================
    
    def _roi_to_pcd(self, roi_depth, cx_offset, cy_offset):
        if roi_depth is None or roi_depth.size == 0: return None
        valid_mask = (roi_depth > 0) & (roi_depth < 1500) 
        if not np.any(valid_mask): return None

        z_vals = roi_depth[valid_mask] * 0.001
        grid_y, grid_x = np.indices(roi_depth.shape)
        u_vals = (grid_x + cx_offset)[valid_mask]
        v_vals = (grid_y + cy_offset)[valid_mask]

        fx = self.camera_model.fx(); fy = self.camera_model.fy()
        cx = self.camera_model.cx(); cy = self.camera_model.cy()

        x_vals = (u_vals - cx) * z_vals / fx
        y_vals = (v_vals - cy) * z_vals / fy

        points_3d = np.stack((x_vals, y_vals, z_vals), axis=1)
        if points_3d.shape[0] < 50: return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)
        
        # 如果是全图扫描(Tier3)，降噪要狠一点；局部ROI则轻一点
        # 这里统一用较轻的参数，靠后续 RANSAC 过滤
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return pcd.select_by_index(ind)

    def _solve_cylinder_pose(self, pcd):
        if pcd is None or len(pcd.points) < 50: return None
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        pcd_object = pcd.select_by_index(inliers, invert=True)
        if len(pcd_object.points) < 50: return None
        try:
            model, inliers = pcd_object.segment_cylinder(radius_min=0.01, radius_max=0.08, ransac_n=20, num_iterations=1000)
        except: return None
        px, py, pz = model[0], model[1], model[2]
        nx, ny, nz = model[3], model[4], model[5]
        radius = model[6]
        cylinder_cloud = pcd_object.select_by_index(inliers)
        points_array = np.asarray(cylinder_cloud.points)
        axis_vec = np.array([nx, ny, nz]); axis_vec /= np.linalg.norm(axis_vec)
        ref_point = np.array([px, py, pz])
        t_values = np.dot(points_array - ref_point, axis_vec)
        t_center = (np.min(t_values) + np.max(t_values)) / 2.0
        return ref_point + t_center * axis_vec, axis_vec, radius

    def _solve_obb_pose(self, pcd):
        if pcd is None or len(pcd.points) < 50: return None
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        pcd_object = pcd.select_by_index(inliers, invert=True)
        if len(pcd_object.points) < 20: return None
        try:
            obb = pcd_object.get_oriented_bounding_box()
            return obb.center, obb.R, obb.extent
        except: return None

    # ========================================================================================
    # 模块 4: 位姿计算接口 (整合版)
    # ========================================================================================

    def get_object_pose_base_frame(self, u, v, angle_rad, name, grasp_strategy="TOP", bbox=None):
        bx, by, bz = 0.0, 0.0, 0.0
        final_q = None
        solver_success = False
        
        # 1. 尝试 3D 拟合
        if bbox:
            x1, y1, x2, y2 = map(int, bbox)
            pcd = self._roi_to_pcd(self.latest_depth_img[y1:y2, x1:x2], x1, y1)
            if pcd:
                if name in self.CYLINDER_OBJECTS:
                    result = self._solve_cylinder_pose(pcd)
                    if result:
                        center_cam, _, radius = result
                        pt_base = self._transform_point(center_cam)
                        if pt_base:
                            bx, by, bz = pt_base
                            self.node.get_logger().info(f"✅ Cylinder Fit: Radius={radius*100:.1f}cm")
                            solver_success = True
                else: # 通用物体 (包括 unknown)
                    result = self._solve_obb_pose(pcd)
                    if result:
                        center_cam, R_obb, size = result
                        pt_base = self._transform_point(center_cam)
                        if pt_base:
                            bx, by, bz = pt_base
                            if grasp_strategy == "TOP":
                                # size 是 [长, 宽, 高] 的无序数组 (OBB 特性)
                                
                                # 方案 A: 激进抬高 (防止撞桌子)
                                # 假设物体高度至少是 5cm，我们把 Z 强行抬到 3cm 以上
                                bz = max(bz, 0.03)
                            self.node.get_logger().info(f"✅ OBB Fit: Size={size*100} cm")
                            solver_success = True

        # 2. 兜底
        if not solver_success:
            if bbox:
                pos = self._get_xyz_roi_centroid(bbox)
                if pos:
                    bx, by, bz = pos
                    if grasp_strategy == "SIDE": bx += 0.0
                    elif grasp_strategy == "TOP": bz += 0.01
            else: return None

        # 3. 姿态 
        
        if grasp_strategy == "TOP":
            target_z_axis = np.array([0.0, 0.0, -1.0]) 
            target_x_axis_temp = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
            target_y_axis = np.cross(target_z_axis, target_x_axis_temp); target_y_axis /= np.linalg.norm(target_y_axis)
            target_x_axis = np.cross(target_y_axis, target_z_axis)
            R = np.eye(4); R[:3, 0]=target_x_axis; R[:3, 1]=target_y_axis; R[:3, 2]=target_z_axis
            final_q = tf_transformations.quaternion_from_matrix(R)

        elif grasp_strategy == "SIDE":
            target_z_axis = np.array([bx, by, 0.0]); target_z_axis /= np.linalg.norm(target_z_axis)
            target_y_axis = np.array([0.0, 0.0, -1.0])
            target_x_axis = np.cross(target_y_axis, target_z_axis); target_x_axis /= np.linalg.norm(target_x_axis)
            target_y_axis = np.cross(target_z_axis, target_x_axis)
            R = np.eye(4); R[:3, 0]=target_x_axis; R[:3, 1]=target_y_axis; R[:3, 2]=target_z_axis
            R_fix = tf_transformations.euler_matrix(0.0, 0.0, 0.0) 
            R_final = np.dot(R, R_fix)
            final_q = tf_transformations.quaternion_from_matrix(R_final)

        if final_q is None: return None
        final_q = final_q / np.linalg.norm(final_q)
        return (bx, by, bz, final_q[0], final_q[1], final_q[2], final_q[3])

    def _transform_point(self, point_cam):
        if self.latest_header is None: return None
        ps = PointStamped()
        ps.header = self.latest_header
        ps.point.x, ps.point.y, ps.point.z = point_cam[0], point_cam[1], point_cam[2]
        try:
            trans = self.tf_buffer.lookup_transform('base_link', ps.header.frame_id, ps.header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
            pt_base = tf2_geometry_msgs.do_transform_point(ps, trans)
            return [pt_base.point.x, pt_base.point.y, pt_base.point.z]
        except: return None

    def _get_xyz_roi_centroid(self, bbox):
        if self.latest_depth_img is None: return None
        x1, y1, x2, y2 = map(int, bbox)
        scale = 0.15 
        cx1 = max(0, x1 + int((x2-x1)*scale)); cx2 = min(self.latest_depth_img.shape[1], x2 - int((x2-x1)*scale))
        cy1 = max(0, y1 + int((y2-y1)*scale)); cy2 = min(self.latest_depth_img.shape[0], y2 - int((y2-y1)*scale))
        roi_depth = self.latest_depth_img[cy1:cy2, cx1:cx2]
        valid_mask = (roi_depth > 0) & (roi_depth < 1500)
        if not np.any(valid_mask): return None
        z_vals = roi_depth[valid_mask] * 0.001
        min_z = np.min(z_vals)
        sphere_mask = z_vals < (min_z + 0.03) 
        clean_z = z_vals[sphere_mask]
        if clean_z.size == 0: return None
        grid_y, grid_x = np.indices(roi_depth.shape)
        pixel_u = (grid_x + cx1)[valid_mask][sphere_mask]
        pixel_v = (grid_y + cy1)[valid_mask][sphere_mask]
        pt_cam_x = np.mean((pixel_u - self.camera_model.cx()) * clean_z / self.camera_model.fx())
        pt_cam_y = np.mean((pixel_v - self.camera_model.cy()) * clean_z / self.camera_model.fy())
        pt_cam_z = np.mean(clean_z)
        return self._transform_point([pt_cam_x, pt_cam_y, pt_cam_z])

    # ... (publish_marker, callbacks, calculate_orientation 保持原样)
    def publish_marker(self, x, y, z, qx, qy, qz, qw):
        marker = Marker()
        marker.header.frame_id = "base_link" 
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "grasp_target"; marker.id = 0; marker.type = Marker.ARROW; marker.action = Marker.ADD
        marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = z
        marker.pose.orientation.x = qx; marker.pose.orientation.y = qy; marker.pose.orientation.z = qz; marker.pose.orientation.w = qw
        marker.scale.x = 0.1; marker.scale.y = 0.01; marker.scale.z = 0.01 
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def info_callback(self, msg):
        if self.camera_info is None: self.camera_info = msg; self.camera_model.fromCameraInfo(msg)
    def color_callback(self, msg):
        try: self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8"); self.latest_header = msg.header
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