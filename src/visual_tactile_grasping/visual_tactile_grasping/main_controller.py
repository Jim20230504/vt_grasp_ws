#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import tf_transformations
import numpy as np
import math
import time
import copy
from visual_tactile_grasping.utils import OBSERVATION_JOINT_POSE

# 引入你的模块 (请确保文件名匹配)
from visual_tactile_grasping.motion_control import ElegantMotion
from visual_tactile_grasping.perception import PerceptionModule
from visual_tactile_grasping.tactile_gripper import TactileGripper # 假设你保存了那个自适应夹爪代码
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint  

class ElegantPickTask(Node):
    def __init__(self):
        super().__init__('elegant_pick_task')
        
        # 1. 初始化各子模块
        self.get_logger().info("Initializing Modules...")
        self.motion = ElegantMotion(self, group_name="rm_group") # 确保 group_name 对
        self.perception = PerceptionModule(self)
        self.gripper = TactileGripper(self)
        
        # 2. 系统参数
        self.pre_grasp_distance = 0.15  # 预备点距离 (15cm)
        self.lift_height = 0.20         # 抬起高度 (20cm)
        

    def run(self):
        self.get_logger().info("System Start. Moving to Observation Pose...")
        
        # --- 步骤 0: 归位 ---
        # 使用 Action 接口发送关节指令 (需要你在 ElegantMotion 里加一个 move_to_joints，或者直接用 PTP 到某个 Pose)
        # 这里为了演示，假设你已经到位，或者手动控制过去
        self.motion.move_to_joints(OBSERVATION_JOINT_POSE) 
        time.sleep(2.0)

        # 开启抓取循环
        while rclpy.ok():
            self.get_logger().info("\n>>> Scanning for objects...")
            
            # --- 步骤 1: 感知 ---
            # 尝试检测 5 次，直到找到物体
            target_pose_7d = None
            obj_name = "unknown"
            strategy = "TOP"
            
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1) # 处理图像回调
                found, _, _, angle, strategy, name = self.perception.detect_object()
                
                if found:
                    # 获取 7D 位姿 (x, y, z, qx, qy, qz, qw)
                    target_pose_7d = self.perception.get_object_pose_base_frame(0, 0, angle, name, strategy)
                    if target_pose_7d:
                        obj_name = name
                        break
                time.sleep(0.5)
            
            if not target_pose_7d:
                self.get_logger().warn("No object found. Retrying...")
                continue

            self.get_logger().info(f"Target Locked: {obj_name} ({strategy})")
            
            # --- 步骤 2: 执行 MTC 风格流水线 ---
            success = self.execute_mtc_pipeline(target_pose_7d, strategy)
            
            if success:
                self.get_logger().info("Task Complete! Restarting...")
                # 回到观察点 (PTP)
                # 假设有一个固定的 Pose 或 Joint
                pass 
            else:
                self.get_logger().error("Task Failed during execution.")
            
            time.sleep(2.0)

    def execute_mtc_pipeline(self, target_pose_7d, strategy):
        """
        核心函数：复刻 MTC 的 Connect -> MoveRelative 逻辑
        """
        tx, ty, tz, qx, qy, qz, qw = target_pose_7d
        
        # 1. 构建关键帧 (Keyframes)
        # ---------------------------------------------------------
        
        # [A] 抓取点 (Grasp Pose) - 终点
        p_grasp = Pose()
        p_grasp.position.x = tx
        p_grasp.position.y = ty
        p_grasp.position.z = tz
        p_grasp.orientation.x = qx
        p_grasp.orientation.y = qy
        p_grasp.orientation.z = qz
        p_grasp.orientation.w = qw
        
        # [B] 预备点 (Pre-Grasp Pose) - 起点
        # 这一步通过“进给向量”反推，保证姿态完全一致
        p_pre = copy.deepcopy(p_grasp)
        
        if strategy == "TOP":
            # 顶抓：进给方向是 Z 轴负方向 (向下)
            # 往回退就是往 Z 轴正方向 (向上)
            p_pre.position.z += self.pre_grasp_distance
            
        elif strategy == "SIDE":
            # 侧抓：进给方向是水平指向物体
            # 我们需要沿着 (0,0) -> (tx, ty) 的向量往回退
            # 计算平面距离
            dist = math.sqrt(tx**2 + ty**2)
            if dist < 0.001: return False
            
            # 比例缩放因子
            # 目标位置 = 当前位置 * (距离 - 回退量) / 距离
            scale = (dist - self.pre_grasp_distance) / dist
            p_pre.position.x *= scale
            p_pre.position.y *= scale
            # Z 轴保持不变 (水平切入)

        # 2. 执行流水线 (Pipeline Execution)
        # ---------------------------------------------------------
        
        # --- Stage 1: 快速接近 (MTC Connect) ---
        # 使用 Pilz PTP：快速、平滑，不在乎直线，只在乎到位
        self.get_logger().info(f"1. [PTP] Moving to Pre-Grasp...")
        # 速度设高一点 (0.5 - 0.8) 以获得丝滑感
        future = self.motion.move_ptp(
            p_pre.position.x, p_pre.position.y, p_pre.position.z,
            p_pre.orientation.x, p_pre.orientation.y, p_pre.orientation.z, p_pre.orientation.w,
            velocity=0.6
        )
        rclpy.spin_until_future_complete(self, future)
        if not future.result().result.error_code.val == 1: return False

        # --- Stage 2: 直线插入 (MTC MoveRelative/Approach) ---
        # 使用 Pilz LIN：严格直线，姿态保持不变
        # 因为起点(Pre)和终点(Grasp)姿态完全一样，这里机械臂只会伸展，不会扭腕，非常稳
        self.get_logger().info(f"2. [LIN] Inserting...")
        
        # 开启夹爪到最大
        self.gripper.send_gripper_cmd(1000) 
        
        # 慢速直线插入 (0.2 左右，模拟小心翼翼的感觉)
        future = self.motion.move_lin(
            p_grasp.position.x, p_grasp.position.y, p_grasp.position.z,
            p_grasp.orientation.x, p_grasp.orientation.y, p_grasp.orientation.z, p_grasp.orientation.w,
            velocity=0.2
        )
        rclpy.spin_until_future_complete(self, future)
        if not future.result().result.error_code.val == 1: return False

        # --- Stage 3: 抓取 (Grasp) ---
        self.get_logger().info(f"3. [GRIP] Closing gripper...")
        # 这里可以调用你的自适应抓取逻辑
        # 简单起见，先用闭合指令
        self.gripper.send_gripper_cmd(0, speed=20, force=50)
        time.sleep(1.0) # 等待抓稳

        # --- Stage 4: 直线抬起 (MTC Lift) ---
        # 同样使用 LIN，保证垂直抬起不撒漏
        self.get_logger().info(f"4. [LIN] Lifting...")
        
        p_lift = copy.deepcopy(p_grasp)
        p_lift.position.z += self.lift_height
        
        future = self.motion.move_lin(
            p_lift.position.x, p_lift.position.y, p_lift.position.z,
            p_lift.orientation.x, p_lift.orientation.y, p_lift.orientation.z, p_lift.orientation.w,
            velocity=0.3
        )
        rclpy.spin_until_future_complete(self, future)
        if not future.result().result.error_code.val == 1: return False
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ElegantPickTask()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()