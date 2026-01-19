#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
import copy
import math

# 引入自定义模块
from visual_tactile_grasping.utils import SystemState, OBSERVATION_JOINT_POSE, DROP_OFF_JOINT_POSE
from visual_tactile_grasping.perception import PerceptionModule
from visual_tactile_grasping.motion_control import MotionControl
from visual_tactile_grasping.tactile_gripper import TactileGripper
from visual_tactile_grasping.utils import GripperState as GState

class VisualTactileController(Node):
    def __init__(self):
        super().__init__('visual_tactile_controller')
        self.state = SystemState.INITIALIZING
        
        # 核心模块
        self.perception = PerceptionModule(self, model_name='yolov8s-world.pt')
        self.motion = MotionControl(self, group_name="rm_group") 
        self.gripper = TactileGripper(self)
        
        # 状态机变量
        self.target_pose_list = None 
        self.current_strategy = "TOP"
        self.wait_time = 0
        self.sub_step = 0 # 用于管理异步操作的微步骤 (0:发送, 1:等Handle, 2:等Result)
        
        # 异步句柄
        self.async_future = None
        self.async_goal_handle = None
        self.computed_trajectory = None

        # 主循环 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        """主状态机循环 (绝对非阻塞)"""
        
        # ---------------------------------------------------------------------
        # 1. 系统初始化
        # ---------------------------------------------------------------------
        if self.state == SystemState.INITIALIZING:
            self.gripper.tick()
            self.wait_time += 1
            if self.wait_time > 20:
                if self.gripper.state != GState.INIT:
                    self.get_logger().info("System Ready.")
                    self.state = SystemState.MOVING_TO_OBSERVE
                    self.sub_step = 0 # 重置微步骤

        # ---------------------------------------------------------------------
        # 2. 移动到观察点 (Async Action)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.MOVING_TO_OBSERVE:
            # [Step 0] 发送请求
            if self.sub_step == 0:
                self.get_logger().info("Moving to Observation Pose...")
                self.async_future = self.motion.move_to_joint_pose(OBSERVATION_JOINT_POSE)
                self.sub_step = 1 # 进入等待 Goal Accepted 阶段
            
            # [Step 1] 等待 Goal 被接受
            elif self.sub_step == 1:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if not goal_handle.accepted:
                        self.get_logger().error("Move Rejected!")
                        self.sub_step = 0 # 重试
                        return
                    # 获取结果 Future
                    self.async_future = goal_handle.get_result_async()
                    self.sub_step = 2 # 进入等待执行结果阶段

            # [Step 2] 等待执行完成
            elif self.sub_step == 2:
                if self.async_future.done():
                    result = self.async_future.result().result
                    if result.error_code.val == 1: # SUCCESS
                        self.get_logger().info("Arrived. Opening Gripper...")
                        self.gripper.state = GState.OPEN
                        self.gripper.tick()
                        self.wait_time = 15 # 给一点时间稳定画面
                        self.state = SystemState.DETECTING_WAIT
                    else:
                        self.get_logger().error(f"Move Failed: {result.error_code.val}")
                        self.state = SystemState.INITIALIZING # 出错复位

        # ---------------------------------------------------------------------
        # 2.5 等待检测 (防抖)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.DETECTING_WAIT:
            if self.wait_time > 0: self.wait_time -= 1; return
            self.state = SystemState.DETECTING

        # ---------------------------------------------------------------------
        # 3. 视觉识别
        # ---------------------------------------------------------------------
        elif self.state == SystemState.DETECTING:
            found, u, v, angle, strategy = self.perception.detect_object()
            
            if found:
                self.current_strategy = strategy
                pose_7d = self.perception.get_object_pose_base_frame(u, v, angle, strategy)
                
                if pose_7d:
                    self.target_pose_list = list(pose_7d)
                    tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
                    
                    self.get_logger().info(f"Target: {strategy} Grasp at Z={tz:.3f}")
                    self.perception.publish_marker(tx, ty, tz, qx, qy, qz, qw)
                    
                    self.sub_step = 0 # 准备进入下一阶段
                    self.state = SystemState.PLANNING_APPROACH

        # ---------------------------------------------------------------------
        # 4. 飞到预备点 (Async Action - PTP)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.PLANNING_APPROACH:
            tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
            
            # [Step 0] 计算并发送请求
            if self.sub_step == 0:
                target_x, target_y, target_z = tx, ty, tz + 0.15 # 默认 Top
                
                if self.current_strategy == "SIDE":
                    # 侧抓：修正高度到腰部，修正位置到前方
                    side_grasp_z = max(tz, 0.05)
                    self.target_pose_list[2] = side_grasp_z # 更新 Z
                    
                    dist = math.sqrt(tx**2 + ty**2)
                    ratio = (dist - 0.15) / dist if dist > 0.15 else 1.0
                    target_x, target_y, target_z = tx * ratio, ty * ratio, side_grasp_z
                    self.get_logger().info("Strategy SIDE: Moving to Front...")
                else:
                    self.get_logger().info("Strategy TOP: Moving to Above...")

                # 发送异步 Action
                self.async_future = self.motion.move_to_pose(target_x, target_y, target_z, qx, qy, qz, qw)
                self.sub_step = 1

            # [Step 1] 等待 Goal
            elif self.sub_step == 1:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if not goal_handle.accepted:
                        self.get_logger().error("Approach Rejected!")
                        self.state = SystemState.MOVING_TO_OBSERVE
                        return
                    self.async_future = goal_handle.get_result_async()
                    self.sub_step = 2

            # [Step 2] 等待结果
            elif self.sub_step == 2:
                if self.async_future.done():
                    res = self.async_future.result().result
                    if res.error_code.val == 1: # SUCCESS
                        self.get_logger().info("Pre-Grasp Reached. Calculating Linear...")
                        self.sub_step = 0
                        self.state = SystemState.APPROACHING
                    else:
                        self.get_logger().warn("Approach Failed (Planning Error)")
                        self.state = SystemState.MOVING_TO_OBSERVE

        # ---------------------------------------------------------------------
        # 5. 直线进近 (Async Service + Async Action)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.APPROACHING:
            tx, ty, _, qx, qy, qz, qw = self.target_pose_list
            final_z = self.target_pose_list[2]
            
            # [Step 0] 发起直线路径计算 (Service Call)
            if self.sub_step == 0:
                current_pose = self.motion.get_current_pose()
                if not current_pose: return # 等TF
                
                grasp_pose = Pose()
                grasp_pose.orientation = current_pose.orientation # 锁姿态
                
                if self.current_strategy == "TOP":
                    grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = tx, ty, final_z - 0.005
                else:
                    grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = tx, ty, final_z
                
                # 异步调用 Service
                self.async_future = self.motion.compute_linear_path_async(grasp_pose)
                self.sub_step = 1
                
            # [Step 1] 等待计算结果，并处理
            elif self.sub_step == 1:
                if self.async_future.done():
                    resp = self.async_future.result()
                    # 处理结果 (添加时间戳)
                    self.computed_trajectory = self.motion.process_linear_path_result(resp)
                    
                    if self.computed_trajectory:
                        self.get_logger().info("Linear Path Computed. Executing...")
                        # 发起执行 Action
                        self.async_future = self.motion.execute_trajectory(self.computed_trajectory)
                        self.sub_step = 2
                    else:
                        self.get_logger().error("Linear Plan Failed!")
                        self.state = SystemState.MOVING_TO_OBSERVE
                        self.sub_step = 0

            # [Step 2] 等待 Goal Accepted (执行 Action)
            elif self.sub_step == 2:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if not goal_handle.accepted:
                        self.get_logger().error("Linear Execution Rejected!")
                        self.state = SystemState.MOVING_TO_OBSERVE
                        return
                    self.async_future = goal_handle.get_result_async()
                    self.sub_step = 3

            # [Step 3] 等待执行完成
            elif self.sub_step == 3:
                if self.async_future.done():
                    # 只要不报错就算成功，误差由夹爪弥补
                    self.get_logger().info("Approach Done. Grasping...")
                    self.gripper.start_grasping_sequence()
                    self.state = SystemState.TACTILE_GRASPING
                    self.sub_step = 0

        # ---------------------------------------------------------------------
        # 6. 触觉抓取
        # ---------------------------------------------------------------------
        elif self.state == SystemState.TACTILE_GRASPING:
            status = self.gripper.tick()
            if status == "SUCCESS":
                self.state = SystemState.LIFTING
                self.sub_step = 0

        # ---------------------------------------------------------------------
        # 7. 抬起 (Async Service + Async Action)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.LIFTING:
            self.gripper.tick()
            
            # [Step 0] 计算抬起路径
            if self.sub_step == 0:
                current = self.motion.get_current_pose()
                lift_pose = copy.deepcopy(current)
                lift_pose.position.z += 0.15 
                
                self.async_future = self.motion.compute_linear_path_async(lift_pose)
                self.sub_step = 1
                
            # [Step 1] 处理计算结果
            elif self.sub_step == 1:
                if self.async_future.done():
                    resp = self.async_future.result()
                    self.computed_trajectory = self.motion.process_linear_path_result(resp)
                    if self.computed_trajectory:
                        self.async_future = self.motion.execute_trajectory(self.computed_trajectory)
                        self.sub_step = 2
                    else:
                        # 抬起失败也往下走，防止卡死
                        self.state = SystemState.PLACING 
                        self.sub_step = 0

            # [Step 2] 等待 Goal
            elif self.sub_step == 2:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if goal_handle.accepted:
                        self.async_future = goal_handle.get_result_async()
                        self.sub_step = 3
                    else:
                        self.state = SystemState.PLACING
                        self.sub_step = 0

            # [Step 3] 等待完成
            elif self.sub_step == 3:
                if self.async_future.done():
                    self.get_logger().info("Lifted.")
                    self.state = SystemState.PLACING
                    self.sub_step = 0

        # ---------------------------------------------------------------------
        # 8. 放置 (Async Action)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.PLACING:
            self.gripper.tick()
            
            if self.sub_step == 0:
                self.get_logger().info("Moving to Drop-off...")
                self.async_future = self.motion.move_to_joint_pose(DROP_OFF_JOINT_POSE)
                self.sub_step = 1
                
            elif self.sub_step == 1:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if goal_handle.accepted:
                        self.async_future = goal_handle.get_result_async()
                        self.sub_step = 2
                        
            elif self.sub_step == 2:
                if self.async_future.done():
                    self.get_logger().info("Dropped off. Releasing...")
                    self.gripper.state = GState.RELEASE
                    self.gripper.tick()
                    
                    self.wait_time = 20 # 等待释放
                    self.state = SystemState.IDLE
                    
        elif self.state == SystemState.IDLE:
            self.gripper.tick()
            if self.wait_time > 0: self.wait_time -= 1
            else:
                self.get_logger().info("Mission Cycle Complete. Restarting...")
                self.state = SystemState.MOVING_TO_OBSERVE
                self.sub_step = 0

def main(args=None):
    rclpy.init(args=args)
    controller = VisualTactileController()
    
    # 🌟 必须使用 MultiThreadedExecutor 防止 Action 回调被 Timer 卡死
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()