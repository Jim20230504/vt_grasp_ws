#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np

# --- 导入真实的消息类型 ---
try:
    # 假设你的大寰夹爪包名为 dh_gripper_msgs (如果不是请修改此处)
    from dh_gripper_msgs.msg import GripperCtrl, GripperState as DHGripperState
except ImportError:
    # 如果本地没有大寰消息包，使用Mock防止报错(仅调试用)
    class GripperCtrl:
        def __init__(self): self.initialize = False; self.position = 0.0; self.force = 0.0; self.speed = 0.0
    class DHGripperState:
        pass

# --- 导入你提供的触觉消息类型 ---
# 对应 src/tactile_sensor_ros/msg/TactileArray.msg 和 FingerData.msg
from tactile_sensor_ros.msg import TactileArray, FingerData

from visual_tactile_grasping.utils import GripperState as GState

class TactileGripper:
    def __init__(self, node: Node):
        self.node = node
        
        # --- 参数配置 ---
        self.open_pos = 1000.0       # 夹爪打开位置 (0-1000)
        self.contact_threshold = 0.5 # 法向力(nf)接触阈值 (需根据 float32[] nf 的实际量级调整，假设单位是N)
        self.slip_force_threshold = 2.0 # 切向力(tf)滑动报警阈值
        self.stable_contact_count_req = 5 # 需要连续多少帧检测到接触才算稳定
        
        # --- 内部变量 ---
        self.state = GState.INIT
        self.current_gripper_pos = 0.0
        self.target_gripper_pos = 0.0
        
        # 触觉状态存储
        # 结构: { sensor_index: {'nf': max_nf_val, 'tf': max_tf_val, 'is_touch': bool} }
        self.tactile_status = {} 
        
        # 稳定计数器
        self.contact_stability_counter = 0
        self.tick_counter = 0
        self.first_touch_pos = None
        
        # --- ROS 接口 ---
        
        # 1. 夹爪控制 Publisher (大寰)
        self.pub_gripper = node.create_publisher(GripperCtrl, '/gripper/ctrl', 10)
        
        # 2. 夹爪状态 Subscriber
        self.sub_gripper_state = node.create_subscription(
            DHGripperState, '/gripper/states', self.gripper_state_callback, 10)
            
        # 3. 触觉数据 Subscriber (根据 tactile_node.py 里的发布话题 'tactile_data')
        self.sub_tactile = node.create_subscription(
            TactileArray, 
            '/tactile_data', # 对应 tactile_node.py 中的 topic name
            self.tactile_callback, 
            10
        )
            
        self.node.get_logger().info("Tactile Gripper Module Initialized with Real Tactile Driver.")

    def gripper_state_callback(self, msg):
        """更新夹爪实时位置"""
        if hasattr(msg, 'position'):
            self.current_gripper_pos = msg.position

    def tactile_callback(self, msg: TactileArray):
        """
        处理真实的触觉传感器数据
        msg: TactileArray 包含多个 FingerData
        """
        # 遍历所有手指数据
        for finger in msg.fingers:
            # finger 是 tactile_sensor_ros/FingerData 类型
            # 包含: int32 sensor_index, float32[] nf, float32[] tf ...
            
            idx = finger.sensor_index
            
            # 1. 提取法向力 (Normal Force)
            # nf 是一个数组，代表该手指上不同感测单元的压力。取最大值作为该手指的受力指标。
            max_nf = 0.0
            if len(finger.nf) > 0:
                max_nf = max(finger.nf)
            
            # 2. 提取切向力 (Tangential Force)
            max_tf = 0.0
            if len(finger.tf) > 0:
                max_tf = max(finger.tf)
            
            # 3. 判断接触状态
            is_touching = max_nf > self.contact_threshold
            
            # 4. 更新状态
            self.tactile_status[idx] = {
                'nf': max_nf,
                'tf': max_tf,
                'is_touch': is_touching
            }
            
        # 调试日志 (可选，防止刷屏)
        # self.node.get_logger().debug(f"Tactile: {self.tactile_status}")

    def send_gripper_cmd(self, pos, speed=50, force=50):
        """发送大寰夹爪控制指令"""
        cmd = GripperCtrl()
        cmd.initialize = True
        cmd.position = float(pos)
        cmd.speed = float(speed)
        cmd.force = float(force)
        self.pub_gripper.publish(cmd)

    def check_any_touch(self):
        """检查是否有任意手指接触"""
        for idx, data in self.tactile_status.items():
            if data['is_touch']:
                return True
        return False

    def check_both_touch(self):
        """检查是否左右手指(假设index 0 和 1)同时接触"""
        # 假设 sensor_index 0 是左指, 1 是右指 (需根据实际硬件ID调整)
        t0 = self.tactile_status.get(0, {}).get('is_touch', False)
        t1 = self.tactile_status.get(1, {}).get('is_touch', False)
        return t0 and t1

    def detect_slip(self):
        """
        基于切向力(tf)的简单滑动检测逻辑。
        如果切向力突然增大或者超过安全阈值，认为发生滑动。
        """
        is_slipping = False
        for idx, data in self.tactile_status.items():
            # 逻辑1: 绝对阈值判断 (假设 tf 单位对应摩擦力)
            if data['tf'] > self.slip_force_threshold:
                self.node.get_logger().warn(f"Finger {idx} Slip Detected! TF={data['tf']:.2f}")
                is_slipping = True
        return is_slipping

    def start_grasping_sequence(self):
        self.state = GState.INIT
        self.node.get_logger().info("Starting Adaptive Grasping Sequence.")

    def tick(self):
        """
        状态机循环 (被 Main Controller 定时调用)
        """
        
        # 1. 初始化
        if self.state == GState.INIT:
            self.node.get_logger().info("Grasping: Initializing...")
            self.state = GState.OPEN
            return "RUNNING"

        # 2. 打开夹爪
        elif self.state == GState.OPEN:
            self.send_gripper_cmd(self.open_pos)
            self.tick_counter = 0
            self.state = GState.WAIT_OPEN
            return "RUNNING"

        # 3. 等待打开
        elif self.state == GState.WAIT_OPEN:
            self.tick_counter += 1
            # 简单延时，实际可判断 abs(current - open) < epsilon
            if self.tick_counter > 20: 
                self.state = GState.CALIBRATION
            return "RUNNING"

        # 4. 传感器标定 (如果硬件需要)
        elif self.state == GState.CALIBRATION:
            # 实际驱动中可能不需要手动标定，这里作为占位
            self.node.get_logger().info("Grasping: Sensors Ready.")
            self.target_gripper_pos = self.open_pos
            self.state = GState.CLOSING_UNTIL_TOUCH
            return "RUNNING"

        # 5. 闭合直到初次接触
        elif self.state == GState.CLOSING_UNTIL_TOUCH:
            # 步进闭合
            step_size = 5.0 
            self.target_gripper_pos -= step_size
            self.send_gripper_cmd(self.target_gripper_pos, speed=30) 
            
            # 使用真实的触觉判断
            if self.check_any_touch():
                self.node.get_logger().info(f"First Touch Detected at Pos {self.current_gripper_pos:.1f}")
                self.first_touch_pos = self.current_gripper_pos
                # 立即停止夹爪 (发送当前位置)
                self.send_gripper_cmd(self.current_gripper_pos, speed=100)
                self.contact_stability_counter = 0
                self.state = GState.ADJUSTING_FOR_BOTH_TOUCH
            
            elif self.target_gripper_pos <= 0.0:
                self.node.get_logger().warn("Gripper fully closed but no object detected.")
                self.state = GState.RELEASE
                
            return "RUNNING"

        # 6. 调整直至双指稳定接触
        elif self.state == GState.ADJUSTING_FOR_BOTH_TOUCH:
            # 如果两指都接触
            if self.check_both_touch():
                self.contact_stability_counter += 1
                if self.contact_stability_counter >= self.stable_contact_count_req:
                    self.node.get_logger().info("Both fingers stable contact.")
                    self.state = GState.GENTLE_CLAMPING
            else:
                # 只有一边接触，继续微小步进
                self.contact_stability_counter = 0
                self.target_gripper_pos -= 1.0 # 极慢速逼近
                self.send_gripper_cmd(self.target_gripper_pos, speed=5)
                
            return "RUNNING"

        # 7. 柔顺抓紧 (Egg/Tofu Mode)
        elif self.state == GState.GENTLE_CLAMPING:
            # 鸡蛋/豆腐策略：在接触点基础上，只增加极小的过压量
            # 这里的 10.0 需要根据夹爪单位（如脉冲或mm）调整，确保不会捏碎
            final_grip_pos = self.current_gripper_pos - 10.0 
            
            # 力控模式：限制最大力 force=20%
            self.send_gripper_cmd(final_grip_pos, speed=10, force=20)
            
            self.node.get_logger().info("Gentle Grasp Applied. Ready to lift.")
            self.state = GState.SLIP_DETECTION
            return "SUCCESS" 

        # 8. 动态滑动检测 (抬起过程中监控)
        elif self.state == GState.SLIP_DETECTION:
            # 这是一个持续监控状态
            if self.detect_slip():
                self.node.get_logger().warn(">>> SLIP DETECTED! Increasing Grip Force! <<<")
                # 动态调整：夹紧一点，并增加力矩限制
                self.current_gripper_pos -= 5.0
                self.send_gripper_cmd(self.current_gripper_pos, speed=100, force=50) # 增加力
            
            return "RUNNING"

        # 9. 释放
        elif self.state == GState.RELEASE:
            self.send_gripper_cmd(self.open_pos)
            return "FAILURE"

        return "RUNNING"