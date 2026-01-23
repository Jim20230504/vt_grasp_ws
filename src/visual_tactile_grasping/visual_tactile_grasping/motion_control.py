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
        
        # 2. Service Client
        self._cartesian_client = self.node.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # 检查服务可用性 (只在初始化时检查一次，非阻塞等待由外部逻辑保证或忽略)
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
        """查询当前末端姿态 (同步查询 TF，因为这很快且只读本地缓存)"""
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

    # =========================================================================
    #  异步动作接口 (Async Action Interfaces)
    #  所有函数直接返回 Future，绝不阻塞！
    # =========================================================================

    def move_to_joint_pose(self, joint_values):
        """
        [异步] 发起关节空间移动
        :return: ActionGoalHandle 的 Future
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        c = Constraints()
        # 请根据实际机器人定义修改关节名称
        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for i, v in enumerate(joint_values):
            if i >= len(names): break
            jc = JointConstraint()
            jc.joint_name = names[i]
            jc.position = float(v)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        goal.request.goal_constraints.append(c)
        
        self.node.get_logger().info("Async: Sending Joint Goal...")
        return self._move_action_client.send_goal_async(goal)

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        """
        [异步] 发起笛卡尔空间 PTP 移动 (MoveIt 规划)
        :return: ActionGoalHandle 的 Future
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.num_planning_attempts = 10

        # goal.planning_options.planner_id = "RRTConnectkConfigDefault"
        
        # 目标约束
        c = Constraints()
        c.name = "target_pose"
        
        # 位置
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "grasp_link"
        pc.weight = 1.0
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.SPHERE
        pr.dimensions = [0.02] 
        bv.primitives.append(pr)
        p_target = Pose()
        p_target.position.x, p_target.position.y, p_target.position.z = x, y, z
        p_target.orientation.w = 1.0
        bv.primitive_poses.append(p_target)
        pc.constraint_region = bv
        c.position_constraints.append(pc)
        
        # 姿态
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "grasp_link"
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 1.0
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        
        goal.request.goal_constraints.append(c)
        
        self.node.get_logger().info("Async: Sending Pose Goal...")
        return self._move_action_client.send_goal_async(goal)

    def execute_trajectory(self, trajectory):
        """
        [异步] 执行给定的轨迹 (带起点修补)
        """
        if not trajectory:
            self.node.get_logger().warn("Execute called with empty trajectory")
            return None
        
        # 🌟 关键步骤：修补轨迹起点 (Stitching)
        # 将轨迹的第一个点强制修改为当前的实时关节状态
        # 这样可以欺骗控制器，消除 "start point deviates" 错误
        trajectory_patched = self._patch_trajectory_start(trajectory)
        
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory_patched
        
        self.node.get_logger().info("Async: Executing Trajectory (Patched)...")
        return self._execute_action_client.send_goal_async(goal)

    # =========================================================================
    #  计算接口 (Compute Interfaces)
    #  分为两步：发起计算 -> 处理结果
    # =========================================================================
    def _patch_trajectory_start(self, trajectory):
        """
        内部工具：强制将轨迹起点对齐到当前机械臂状态
        """
        if not self.current_joint_state:
            return trajectory # 如果没状态，就不修补了
            
        # 创建深拷贝，防止修改原数据
        patched_traj = copy.deepcopy(trajectory)
        
        # 获取当前的关节映射 {name: position}
        current_map = {name: pos for name, pos in zip(
            self.current_joint_state.name, self.current_joint_state.position)}
        
        # 遍历轨迹涉及的关节
        traj_joint_names = patched_traj.joint_trajectory.joint_names
        start_point = patched_traj.joint_trajectory.points[0]
        
        # 修改第一个点的位置
        for i, name in enumerate(traj_joint_names):
            if name in current_map:
                # 强制覆盖为当前真实值
                # 注意：偏差不能太大(比如超过5度)，否则会瞬移。
                # 但这里的偏差通常只有 0.02 弧度，完全可以接受。
                start_point.positions[i] = current_map[name]
                
                # 既然修改了位置，最好把该点的速度/加速度设为0，保证平稳启动
                if start_point.velocities: start_point.velocities[i] = 0.0
                if start_point.accelerations: start_point.accelerations[i] = 0.0
                
        return patched_traj
    def compute_linear_path_async(self, target_pose):
        """
        [异步] 发起直线路径计算请求
        :return: Service Future
        """
        if not self.current_joint_state: 
            self.node.get_logger().warn("No joint state received yet!")
            return None
        
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.group_name
        req.link_name = "grasp_link"
        
        rs = RobotState()
        rs.joint_state = self.current_joint_state
        req.start_state = rs
        
        req.waypoints = [target_pose]
        req.max_step = 0.01       # 1cm 插补
        req.jump_threshold = 0.0 
        req.avoid_collisions = True
        
        self.node.get_logger().info("Async: Computing Linear Path...")
        return self._cartesian_client.call_async(req)

    def process_linear_path_result(self, response):
        """
        [同步非阻塞] 处理直线规划结果，添加时间戳
        :param response: compute_linear_path_async 返回的 Future 的 result()
        :return: 加工好的 RobotTrajectory 或 None
        """
        if not response:
            return None
            
        if response.fraction < 0.9:
            self.node.get_logger().warn(f"Linear path incomplete (fraction: {response.fraction})")
            return None
            
        # 成功，添加时间参数化
        traj_with_time = self._scale_trajectory(response.solution, 0.1)
        return traj_with_time

    def _scale_trajectory(self, traj, scale_factor=0.1):
        """内部工具：给轨迹加上时间戳"""
        new_traj = copy.deepcopy(traj)
        n_points = len(new_traj.joint_trajectory.points)
        if n_points == 0: return new_traj
        
        dt = 0.5 # 每个点间隔 0.5秒
        
        for i in range(n_points):
            time_from_start = i * dt
            new_traj.joint_trajectory.points[i].time_from_start.sec = int(time_from_start)
            new_traj.joint_trajectory.points[i].time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            
        return new_traj