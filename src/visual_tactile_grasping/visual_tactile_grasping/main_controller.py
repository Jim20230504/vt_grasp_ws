#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Quaternion

# 导入我们的模块
from visual_tactile_grasping.utils import SystemState, OBSERVATION_JOINT_POSE, DROP_OFF_JOINT_POSE
from visual_tactile_grasping.perception import PerceptionModule
from visual_tactile_grasping.motion_control import MotionControl
from visual_tactile_grasping.tactile_gripper import TactileGripper

class VisualTactileController(Node):
    def __init__(self):
        super().__init__('visual_tactile_controller')
        
        self.state = SystemState.IDLE
        self.target_coords = None # [x, y, z]
        
        # 初始化模块
        self.perception = PerceptionModule(self)
        self.motion = MotionControl(self, group_name="rm_group") # 组名需匹配MoveIt配置
        self.gripper = TactileGripper(self)
        
        # 控制循环定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("System Ready. Waiting for start...")
        
        # 自动开始延迟
        self.start_delay = 0
        self.state = SystemState.INITIALIZING

    def control_loop(self):
        """主状态机循环"""
        
        # --- 1. 初始化 ---
        if self.state == SystemState.INITIALIZING:
            # 确保Gripper激活并打开
            self.gripper.send_gripper_cmd(1000) # Open
            self.start_delay += 1
            if self.start_delay > 20: # 等待2秒硬件就绪
                self.state = SystemState.MOVING_TO_OBSERVE

        # --- 2. 移动到观察姿态 ---
        elif self.state == SystemState.MOVING_TO_OBSERVE:
            self.get_logger().info("Moving to Observation Pose...")
            future = self.motion.move_to_joint_pose(OBSERVATION_JOINT_POSE)
            # 不阻塞等待Future，而是假设MoveIt会处理。嘛
            # 更好的做法是检查Future状态，这里简化处理，增加一个等待时间状态
            self.wait_time = 50 # 5秒等待运动完成
            self.state = SystemState.DETECTING # 实际上应该有个 WAIT_FOR_MOVE 状态

        # --- 3. 视觉识别 ---
        elif self.state == SystemState.DETECTING:
            if self.wait_time > 0:
                self.wait_time -= 1
                return # 还在移动中

            self.get_logger().info("Detecting object...")
            found, u, v = self.perception.detect_object()
            
            if found:
                self.get_logger().info(f"Object detected at pixel ({u}, {v})")
                coords = self.perception.get_object_position_base_frame(u, v)
                
                if coords:
                    self.target_coords = coords
                    self.state = SystemState.PLANNING_APPROACH
                else:
                    self.get_logger().warn("Could not calculate 3D coordinates. Retrying...")
            else:
                self.get_logger().info("No object detected.")

        # --- 4. 规划抓取 ---
        elif self.state == SystemState.PLANNING_APPROACH:
            from visual_tactile_grasping.utils import GRIPPER_LENGTH # 导入长度

            tx, ty, tz = self.target_coords
            
            # ---------------------------------------------------
            # [关键修正] 计算机械臂法兰(link6)的目标位置
            # ---------------------------------------------------
            # 我们的目标是让 "指尖" 到达物体位置。
            # 所以 "法兰" 必须停在物体上方 "夹爪长度" 的地方。
            
            # 抓取点 Z (指尖目标 Z) = 物体 Z 
            # 法兰目标 Z = 抓取点 Z + 夹爪长度 + 预备高度 (10cm)
            
            approach_height = 0.10 # 预备阶段，在物体上方10cm
            
            # 法兰的目标高度
            target_flange_z = tz + GRIPPER_LENGTH + approach_height
            
            self.get_logger().info(f"Target Object Z: {tz:.3f}")
            self.get_logger().info(f"Gripper Length: {GRIPPER_LENGTH:.3f}")
            self.get_logger().info(f"Commanding Flange to Z: {target_flange_z:.3f}")
            
            # (四元数部分)
            q_down = [1.0, 0.0, 0.0, 0.0] 
            
            # 发送指令给 link6
            self.motion.move_to_cartesian_pose(tx, ty, target_flange_z, q_down[0], q_down[1], q_down[2], q_down[3])
            
            self.wait_time = 40 
            self.state = SystemState.APPROACHING

        # --- 5. 执行接近 ---
        elif self.state == SystemState.APPROACHING:
            if self.wait_time > 0:
                self.wait_time -= 1
                return
            
            # 此时机械臂在物体上方 (Approach Height)
            # 我们需要让它慢慢下降，直到指尖包围物体
            
            self.get_logger().info("Visual Approach Done. Moving down to grasp...")
            
            # 计算最终抓取时的法兰高度
            # 最终法兰 Z = 物体 Z + 夹爪长度 - 稍微一点过压(或正好接触)
            final_flange_z = self.target_coords[2] + GRIPPER_LENGTH
            
            # 调用运动控制直线下降 (这里直接调用 move_to_cartesian 简单处理)
            # 实际项目中建议使用直线规划 (compute_cartesian_path)
            q_down = [1.0, 0.0, 0.0, 0.0]
            self.motion.move_to_cartesian_pose(
                self.target_coords[0], 
                self.target_coords[1], 
                final_flange_z, 
                q_down[0], q_down[1], q_down[2], q_down[3]
            )
            
            # 等待下降完成
            # 注意：这里应该有一个等待逻辑，或者在 TACTILE_GRASPING 里的 INIT 阶段等待
            # 为了简单，我们强制给一点等待时间
            time.sleep(2.0) 
            
            # 启动触觉抓取
            self.gripper.start_grasping_sequence()
            self.state = SystemState.TACTILE_GRASPING

        # --- 6. 触觉自适应抓取 (核心) ---
        elif self.state == SystemState.TACTILE_GRASPING:
            # 调用 Gripper 的 tick
            status = self.gripper.tick()
            
            if status == "SUCCESS":
                self.get_logger().info("Grasp Successful!")
                self.state = SystemState.LIFTING
                self.wait_time = 0
            elif status == "FAILURE":
                self.get_logger().error("Grasp Failed. Resetting.")
                self.state = SystemState.ERROR
            else:
                # 还在运行中，可以在这里命令机械臂微动下降 (Visual-Tactile Servoing)
                # 这是一个高级功能：如果夹爪还没接触，机械臂可以继续以极慢速度下降 (Z轴 -0.001m/s)
                # self.motion.jog_down() # 需要在 MotionControl 实现 jog 功能
                pass

        # --- 7. 抬起物体 ---
        elif self.state == SystemState.LIFTING:
            if self.wait_time == 0:
                self.get_logger().info("Lifting object...")
                # 简单实现：回到观察点或抬高Z轴
                # 这里不仅要移动，还要开启滑动检测
                self.gripper.state = GState.SLIP_DETECTION # 强制进入滑动检测模式
                
                # 向上运动
                current_z = self.target_coords[2] + 0.2
                self.motion.move_to_cartesian_pose(self.target_coords[0], self.target_coords[1], current_z, 1.0, 0.0, 0.0, 0.0)
                self.wait_time = 40
            
            else:
                self.wait_time -= 1
                # 在抬起过程中持续检查滑动
                self.gripper.tick()
                if self.wait_time <= 0:
                    self.state = SystemState.PLACING

        # --- 8. 放置 ---
        elif self.state == SystemState.PLACING:
            self.get_logger().info("Moving to drop-off zone...")
            self.motion.move_to_joint_pose(DROP_OFF_JOINT_POSE)
            # 等待到位后释放...
            # 这里省略具体等待逻辑，直接示意
            self.gripper.send_gripper_cmd(1000) # Open
            self.state = SystemState.IDLE

        elif self.state == SystemState.ERROR:
            self.get_logger().error("System in Error State. Check logs.")
            # 停止定时器或等待人工重置

def main(args=None):
    rclpy.init(args=args)
    controller = VisualTactileController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()