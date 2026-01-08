#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState
from moveit_msgs.srv import GetCartesianPath 
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration
import copy

class MotionControl:
    def __init__(self, node: Node, group_name="rm_group"):
        self.node = node
        self.group_name = group_name
        
        # 1. Action Clients
        self._move_action_client = ActionClient(self.node, MoveGroup, 'move_action')
        self._execute_action_client = ActionClient(self.node, ExecuteTrajectory, 'execute_trajectory')
        
        # 2. Service Client (关键：用于计算直线路径)
        self._cartesian_client = self.node.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        if not self._move_action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn("MoveGroup Action not ready")
        
        # 3. TF & State
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.current_joint_state = None
        self.joint_sub = self.node.create_subscription(JointState, 'joint_states', self.js_cb, 10)

    def js_cb(self, msg):
        self.current_joint_state = msg

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'grasp_link', rclpy.time.Time(), Duration(seconds=1.0))
            p = Pose()
            p.position.x = trans.transform.translation.x
            p.position.y = trans.transform.translation.y
            p.position.z = trans.transform.translation.z
            p.orientation = trans.transform.rotation
            return p
        except Exception:
            return None

    def _create_elbow_constraint(self):
        """
        [防怪异核心] 创建肘部约束
        RM65 的 Joint 3 通常是肘部。防止它翻到下面去。
        假设 Joint 3 > 0 是肘部朝上（根据你的实际机器人调整正负）
        """
        jc = JointConstraint()
        jc.joint_name = "joint3" 
        jc.position = 1.57 # 偏向 90度
        jc.tolerance_above = 3.14 
        jc.tolerance_below = 1.5  # 不允许小于 0 (假设 0 是伸直)
        jc.weight = 0.5 
        return jc

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        """
        [长距离移动] 从观察点飞到预备点
        使用 PTP 规划，但加入姿态约束和关节约束，防止乱飞。
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1 # 速度适中
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # 1. 目标约束
        c = Constraints()
        
        # 位置
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "grasp_link"
        pc.weight = 1.0
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        pr.dimensions = [0.01, 0.01, 0.01]
        bv.primitives.append(pr)
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        bv.primitive_poses.append(p)
        pc.constraint_region = bv
        c.position_constraints.append(pc)
        
        # 姿态 
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "grasp_link"
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 3 # 允许Z轴旋转，解算更容易
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        
        # [防乱舞] 加入关节约束 (如果需要)
        # c.joint_constraints.append(self._create_elbow_constraint())

        goal.request.goal_constraints.append(c)
        return self._move_action_client.send_goal_async(goal)

    def compute_linear_path(self, target_pose):
        """
        [短距离移动] 真正的直线规划！
        直接计算笛卡尔路径插值。
        """
        if not self.current_joint_state: return None
        
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.group_name
        req.link_name = "grasp_link"
        
        req.start_state.joint_state = self.current_joint_state
        req.waypoints = [target_pose]
        req.max_step = 0.01       # 1cm 插补一次，保证直线
        req.jump_threshold = 0.0  # 禁止关节跳变 (防止反转)
        req.avoid_collisions = True
        
        self.node.get_logger().info("Computing Linear Path...")
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        res = future.result()
        if res and res.fraction > 0.9: # 至少规划出 90% 的路径才算成功
            return res.solution
        else:
            fraction = res.fraction if res else 0.0
            self.node.get_logger().warn(f"Linear path failed! Fraction: {fraction}")
            return None

    def execute_trajectory(self, trajectory):
        """执行计算好的直线轨迹"""
        if not trajectory: return False
        
        # 减速处理 (安全第一)
        trajectory = self._scale_trajectory(trajectory, 0.1)
        
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.node.get_logger().info("Executing Linear Path...")
        future = self._execute_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        return True

    def _scale_trajectory(self, traj, scale):
        """拉伸时间戳以减速"""
        new_traj = copy.deepcopy(traj)
        for point in new_traj.joint_trajectory.points:
            t = point.time_from_start
            new_time = (t.sec + t.nanosec * 1e-9) / scale
            point.time_from_start.sec = int(new_time)
            point.time_from_start.nanosec = int((new_time - int(new_time)) * 1e9)
        return new_traj
    
    def move_to_joint_pose(self, joint_values):
        """回原点专用"""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        c = Constraints()
        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for i, v in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = names[i]; jc.position = float(v); jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        return self._move_action_client.send_goal_async(goal)