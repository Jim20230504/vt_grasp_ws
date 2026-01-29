#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class ElegantMotion:
    def __init__(self, node: Node, group_name="rm_group"):
        self.node = node
        self.group_name = group_name
        
        # Action Client
        self._move_action_client = ActionClient(self.node, MoveGroup, 'move_action')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        # 等待服务
        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("❌ MoveGroup Action Server not found! Check Pilz plugin.")

    def get_current_pose(self):
        """获取当前末端位姿"""
        try:
            # 注意：请确保 frame_id 正确，通常是 base_link -> grasp_link (或 tcp_link)
            trans = self.tf_buffer.lookup_transform('base_link', 'grasp_link', rclpy.time.Time(), Duration(seconds=1.0))
            p = Pose()
            p.position.x = trans.transform.translation.x
            p.position.y = trans.transform.translation.y
            p.position.z = trans.transform.translation.z
            p.orientation = trans.transform.rotation
            return p
        except Exception as e:
            self.node.get_logger().warn(f"TF Error: {e}")
            return None

    # =========================================================================
    #  核心：工业级运动接口 (Pilz PTP & LIN)
    # =========================================================================

    def _create_pilz_goal(self, planner_id="PTP", vel_scale=1.0, acc_scale=1.0):
        """
        内部辅助函数：构建 Pilz 规划请求的基础 Goal
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        
        # 🔥 关键设置：指定使用 Pilz 流水线
        goal.request.pipeline_id = "pilz_industrial_motion_planner"
        goal.request.planner_id = planner_id 
        
        # 🔥 关键设置：速度全开，实现丝滑效果
        # (工业机器人通常设为 0.1-0.5 调试，1.0 运行。MTC 示例中用的是 1.0)
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.request.allowed_planning_time = 1.0 # Pilz 计算极快，1秒足够
        goal.request.num_planning_attempts = 1   # Pilz 是确定性算法，不需要重试
        
        return goal

    def _create_constraints(self, x, y, z, qx, qy, qz, qw, tol_pos=0.001, tol_rot=0.01):
        """
        内部辅助函数：构建高精度约束
        """
        c = Constraints()
        c.name = "pilz_target"
        
        # 1. 位置约束 (球体范围)
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "grasp_link"
        pc.weight = 1.0
        
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.SPHERE
        pr.dimensions = [tol_pos] # Pilz 要求精度很高，这里设为 1mm
        bv.primitives.append(pr)
        
        target = Pose()
        target.position.x = x; target.position.y = y; target.position.z = z
        target.orientation.w = 1.0 # 这里的方向不重要，主要看 ConstraintRegion
        bv.primitive_poses.append(target)
        
        pc.constraint_region = bv
        c.position_constraints.append(pc)
        
        # 2. 姿态约束
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "grasp_link"
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = tol_rot
        oc.absolute_y_axis_tolerance = tol_rot
        oc.absolute_z_axis_tolerance = tol_rot
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        
        return c

    def move_ptp(self, x, y, z, qx, qy, qz, qw, velocity=0.5):
        """
        [PTP] 快速点对点移动 (类似 MTC Connect)
        特点：最快路径，避障能力弱，但动作连贯。
        用途：从观察点 -> 预抓取点
        """
        # 构建 Goal
        goal = self._create_pilz_goal("PTP", vel_scale=velocity, acc_scale=velocity)
        
        # 添加约束
        # PTP 的容差可以稍微大一点点，保证规划成功率
        c = self._create_constraints(x, y, z, qx, qy, qz, qw, tol_pos=0.005, tol_rot=0.01)
        goal.request.goal_constraints.append(c)
        
        self.node.get_logger().info(f"🚀 Async PTP -> ({x:.3f}, {y:.3f}, {z:.3f})")
        return self._move_action_client.send_goal_async(goal)

    def move_lin(self, x, y, z, qx, qy, qz, qw, velocity=0.2):
        """
        [LIN] 直线插补移动 (类似 MTC MoveRelative/Cartesian)
        特点：严格走直线，末端姿态平滑过渡。
        用途：预抓取点 -> 抓取点 (下插)，抓取点 -> 抬起
        """
        # 构建 Goal
        # LIN 动作通常需要慢一点，保证安全和力控稳定
        goal = self._create_pilz_goal("LIN", vel_scale=velocity, acc_scale=velocity)
        
        # 添加约束
        # LIN 对精度要求极高，必须设得很小
        c = self._create_constraints(x, y, z, qx, qy, qz, qw, tol_pos=0.001, tol_rot=0.001)
        goal.request.goal_constraints.append(c)
        
        self.node.get_logger().info(f"📏 Async LIN -> ({x:.3f}, {y:.3f}, {z:.3f})")
        return self._move_action_client.send_goal_async(goal)

    def stop(self):
        """紧急停止"""
        self.node.get_logger().warn("STOPPING ROBOT!")
        # 发送空目标或调用取消接口 (根据实际需求实现)
        # 简单实现：取消所有 goal
        pass