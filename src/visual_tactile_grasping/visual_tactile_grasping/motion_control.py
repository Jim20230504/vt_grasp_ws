#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
import math

class MotionControl:
    def __init__(self, node: Node, group_name="arm"):
        self.node = node
        self.group_name = group_name
        
        # MoveGroup Action Client
        self._action_client = ActionClient(self.node, MoveGroup, 'move_action')
        
        self.node.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server not available!")
        else:
            self.node.get_logger().info("MoveGroup Interface Initialized.")
        

    def move_to_joint_pose(self, joint_values):
        """
        关节空间规划
        :param joint_values: list of float [j1, j2, j3, j4, j5, j6]
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # 构建关节约束
        constraints = Constraints()
        # RM65 6个关节
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        for i, val in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        return self._send_goal(goal_msg)

    def move_to_cartesian_pose(self, x, y, z, qx, qy, qz, qw):
        """
        笛卡尔空间规划 (移动到指定位置和姿态) - Low Level Action 版
        """
        # 1. 创建 Goal 消息
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 5.0
        
        
        # 速度 0.0 ~ 1.0。
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # 2. 构建约束
        constraints = Constraints()
        
        # --- 位置约束 (Position Constraint) ---
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "Link6" # 确保这里是你的末端 Link 名称
        pc.weight = 1.0
        
        # 定义目标区域 (极小框)
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        pr.dimensions = [0.001, 0.001, 0.001] # 1mm 容差
        bv.primitives.append(pr)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # 区域本身的姿态(默认即可)
        bv.primitive_poses.append(pose)
        
        pc.constraint_region = bv
        constraints.position_constraints.append(pc)
        
        # --- 姿态约束 (Orientation Constraint) ---
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "Link6"
        # 使用传入的四元数参数
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        # 容差：越小越精准，但也越难规划
        oc.absolute_x_axis_tolerance = 0.05 
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05 
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        # 将约束加入目标
        goal_msg.request.goal_constraints.append(constraints)
        
        # 发送目标 (假设你类里有 _send_goal 方法)
        return self._send_goal(goal_msg)
    
    def _send_goal(self, goal_msg):
        """发送Action并等待结果"""
        self.node.get_logger().info("Sending Motion Goal...")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # 注意：在同步逻辑中，我们需要等待Future完成。
        # 如果在主节点的Timer回调中调用，这里不能使用 rclpy.spin_until_future_complete
        # 否则会死锁。理想架构是主控制器基于Future回调切换状态。
        # 为简化，这里返回Future对象，交给Main Controller检查状态。
        return send_goal_future