#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import numpy as np

# 导入消息类型
try:
    # 尝试导入大寰夹爪的官方消息类型
    from dh_gripper_msgs.msg import GripperCtrl, GripperState as DHGripperState
except ImportError:
    # 如果没有安装驱动包，使用 Mock 类防止报错 (仅用于代码静态检查)
    class GripperCtrl:
        def __init__(self): self.initialize = False; self.position = 0.0; self.force = 0.0; self.speed = 0.0
    class DHGripperState:
        pass

from tactile_sensor_ros.msg import TactileArray
from visual_tactile_grasping.utils import GripperState as GState

class TactileGripper:
    def __init__(self, node: Node):
        self.node = node
        
        # === 核心参数配置 (针对易碎品优化) ===
        # 抓取速度 (0-100): 
        # 建议设为 10-20，太快会导致刹车距离变长，容易捏碎物体
        self.grasp_speed = 15.0 
        
        # 抓取力 (20-100): 
        # 注意：大寰夹爪的力控通常有死区，低于 20 可能无法动作
        self.grasp_force = 30.0 
        
        # 触觉触发阈值 (去皮后的数值)
        # 根据传感器灵敏度调整，太低容易误触，太高反应迟钝
        self.contact_threshold = 0.5 
        
        # === 内部状态 ===
        self.state = GState.IDLE
        self.current_gripper_pos = 0.0
        self.target_gripper_pos = 0.0
        
        # 默认最大量程 (DH-AG95 通常是 1000)
        self.max_open_pos = 1000.0       
        
        # 标志位
        self.range_calibrated = False
        self.gripper_feedback_received = False # 是否收到过夹爪反馈
        self.command_sent = False 
        self.calibration_start_time = 0.0
        self.closing_start_time = 0.0
        
        # 触觉数据
        self.raw_tactile_data = {}
        self.tactile_baseline = {}
        self.is_calibrated = False
        
        # 计数器
        self.cali_sample_count = 0
        self.cali_samples_req = 20
        
        # === ROS接口 ===
        self.pub_gripper = node.create_publisher(GripperCtrl, '/gripper/ctrl', 10)
        
        self.sub_gripper_state = node.create_subscription(
            DHGripperState, '/gripper/states', self.gripper_state_callback, 10)
            
        self.sub_tactile = node.create_subscription(
            TactileArray, '/tactile_data', self.tactile_callback, 10)
            
        self.node.get_logger().info(f"Tactile Gripper initialized: V11 (Guarded Move)")

    def gripper_state_callback(self, msg):
        """读取夹爪实时反馈"""
        self.gripper_feedback_received = True
        if hasattr(msg, 'position'):
            new_pos = float(msg.position)
            # 自动更新最大张开位置 (仅在初始化阶段信任这个值)
            if not self.range_calibrated:
                if new_pos > self.max_open_pos:
                    self.max_open_pos = new_pos
            self.current_gripper_pos = new_pos

    def tactile_callback(self, msg: TactileArray):
        """读取触觉数据"""
        for finger in msg.fingers:
            idx = finger.sensor_index
            max_nf = max(finger.nf) if len(finger.nf) > 0 else 0.0
            max_tf = max(finger.tf) if len(finger.tf) > 0 else 0.0
            self.raw_tactile_data[idx] = {'nf': max_nf, 'tf': max_tf}

    def send_gripper_cmd(self, pos, speed=None, force=None, do_init=False):
        """发送控制指令"""
        cmd = GripperCtrl()
        cmd.initialize = do_init 
        cmd.position = float(pos)
        
        # 如果不指定，使用类内部配置的参数
        cmd.speed = float(speed) if speed is not None else self.grasp_speed
        cmd.force = float(force) if force is not None else self.grasp_force
        
        self.pub_gripper.publish(cmd)

    def get_normalized_nf(self, idx):
        """获取去皮后的法向力"""
        if idx not in self.raw_tactile_data: return 0.0
        raw_val = self.raw_tactile_data[idx]['nf']
        baseline = self.tactile_baseline.get(idx, 0.0)
        return max(0.0, raw_val - baseline)

    def check_any_touch(self):
        """检测任意接触"""
        for idx in self.raw_tactile_data:
            if self.get_normalized_nf(idx) > self.contact_threshold:
                return True
        return False

    def calibrate_gripper_range(self):
        """启动时初始化夹爪"""
        if self.range_calibrated: return True
        
        # 还没收到过反馈，说明驱动可能没启动，先等待
        if not self.gripper_feedback_received:
            self.node.get_logger().warn("Waiting for gripper driver...", throttle_duration_sec=2.0)
            return False

        # 发送初始化指令
        if not self.command_sent:
            self.node.get_logger().info("Initializing Gripper (Pos=1000, Init=True)...")
            # 初始化时使用较大力矩和速度确保到位
            self.send_gripper_cmd(1000.0, speed=50, force=100, do_init=True)
            self.calibration_start_time = time.time()
            self.command_sent = True
            return False
        
        # 等待 3 秒
        if time.time() - self.calibration_start_time > 3.0: 
            self.range_calibrated = True
            # 如果反馈值太小（说明夹爪可能没开），强制设为 1000
            if self.max_open_pos < 500: 
                self.max_open_pos = 1000.0
            
            self.node.get_logger().info(f"Gripper Ready. Range: 0 - {self.max_open_pos:.0f}")
            self.command_sent = False 
            return True
        return False

    def start_grasping_sequence(self):
        """外部调用：开始一次抓取任务"""
        self.state = GState.INIT
        self.command_sent = False
        self.node.get_logger().info("Starting Grasp Sequence.")

    def tick(self):
        """
        主逻辑循环
        返回: "RUNNING", "SUCCESS", "FAILURE"
        """
        
        # ----------------------------------------------------
        # 1. 系统初始化
        # ----------------------------------------------------
        if self.state == GState.INIT:
            if not self.calibrate_gripper_range(): return "RUNNING"
            self.state = GState.OPEN
            self.command_sent = False
            return "RUNNING"

        # ----------------------------------------------------
        # 2. 确保完全张开
        # ----------------------------------------------------
        elif self.state == GState.OPEN:
            if not self.command_sent:
                self.send_gripper_cmd(self.max_open_pos, do_init=False)
                self.command_sent = True
            
            # 简单延时等待
            self.tick_counter = 0
            self.state = GState.WAIT_OPEN
            return "RUNNING"

        # ----------------------------------------------------
        # 3. 等待 + 准备触觉标定
        # ----------------------------------------------------
        elif self.state == GState.WAIT_OPEN:
            self.tick_counter += 1
            if self.tick_counter > 10: # 约0.5-1秒
                self.state = GState.CALIBRATION
                self.cali_sample_count = 0
                self.tactile_baseline = {}
                self.command_sent = False
            return "RUNNING"

        # ----------------------------------------------------
        # 4. 触觉传感器去皮
        # ----------------------------------------------------
        elif self.state == GState.CALIBRATION:
            if self.cali_sample_count == 0:
                self.node.get_logger().info("Taring Sensors...")
            
            for idx, data in self.raw_tactile_data.items():
                if idx not in self.tactile_baseline: self.tactile_baseline[idx] = 0.0
                self.tactile_baseline[idx] += data['nf']
            self.cali_sample_count += 1
            
            if self.cali_sample_count >= self.cali_samples_req:
                # 计算均值
                for idx in self.tactile_baseline:
                    self.tactile_baseline[idx] /= self.cali_samples_req
                
                self.is_calibrated = True
                self.state = GState.CLOSING_UNTIL_TOUCH
                self.command_sent = False
                self.closing_start_time = time.time()
                self.node.get_logger().info("Calibration Done. START CLOSING.")
            return "RUNNING"

        # ----------------------------------------------------
        # 5. 闭合直到接触 (Guarded Move)
        # ----------------------------------------------------
        elif self.state == GState.CLOSING_UNTIL_TOUCH:
            
            # [A] 动作发起
            if not self.command_sent:
                self.send_gripper_cmd(0.0, speed=self.grasp_speed, force=self.grasp_force, do_init=False)
                self.command_sent = True
                self.node.get_logger().info(f"Closing... Speed={self.grasp_speed}")
            
            # [B] 实时监测接触
            if self.check_any_touch():
                self.node.get_logger().info(f"!!! TOUCH DETECTED at {self.current_gripper_pos:.0f} !!!")
                
                # [刹车逻辑]
                stop_pos = self.current_gripper_pos
                self.target_gripper_pos = stop_pos 
                # 急停：大速度，当前力矩
                self.send_gripper_cmd(stop_pos, speed=100, force=self.grasp_force, do_init=False)
                
                self.state = GState.GENTLE_CLAMPING
                self.command_sent = False
                return "RUNNING"
            
            # [C] 超时/到底检测
            elapsed = time.time() - self.closing_start_time
            # 增加到 5.0 秒，防止慢速抓取时误判
            if elapsed > 5.0 or self.current_gripper_pos < 10.0:
                self.node.get_logger().warn(f"Fully closed or Timed out. No object.")
                self.state = GState.RELEASE
                self.command_sent = False
            
            return "RUNNING"

        # ----------------------------------------------------
        # 6. 柔顺保持 (Gentle Hold)
        # ----------------------------------------------------
        elif self.state == GState.GENTLE_CLAMPING:
            if not self.command_sent:
                # 稍微加一点点预紧力 (挤压 2%)
                squeeze = self.max_open_pos * 0.02
                final_pos = max(0.0, self.target_gripper_pos - squeeze)
                
                # 低速保持，力控维持
                self.send_gripper_cmd(final_pos, speed=10, force=self.grasp_force, do_init=False)
                self.target_gripper_pos = final_pos
                self.command_sent = True
                
                self.node.get_logger().info(f"Grasp Secured at {final_pos:.0f}")
                
                # 转入被动防滑状态
                self.state = GState.SLIP_DETECTION
                
            # 这里返回 SUCCESS，通知主控可以抬起了
            return "SUCCESS"

        # ----------------------------------------------------
        # 7. 防滑监测 (被动状态)
        # ----------------------------------------------------
        elif self.state == GState.SLIP_DETECTION:
            # 持续运行，如果需要可以在这里添加滑移检测逻辑
            # 目前只负责占位，防止状态机跑飞
            pass 
            return "RUNNING"

        # ----------------------------------------------------
        # 8. 释放
        # ----------------------------------------------------
        elif self.state == GState.RELEASE:
            if not self.command_sent:
                self.send_gripper_cmd(self.max_open_pos, do_init=False)
                self.command_sent = True
                self.node.get_logger().info("Releasing Gripper.")
            
            # 即使已经发了指令，这里通常返回 FAILURE 或者 IDLE 供主控判断
            return "FAILURE"

        elif self.state == GState.IDLE:
            return "RUNNING"

        return "RUNNING"