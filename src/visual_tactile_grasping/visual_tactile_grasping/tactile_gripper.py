#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import numpy as np

# 导入消息类型
try:
    from dh_gripper_msgs.msg import GripperCtrl, GripperState as DHGripperState
except ImportError:
    # Mock for testing without driver
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
        # 抓取速度 (0-100): 越慢越安全，给传感器反应时间
        self.grasp_speed = 20.0 
        # 抓取力 (0-100): 设为较小值，硬件层保护
        self.grasp_force = 30.0 
        # 触觉触发阈值
        self.contact_threshold = 0.3 
        
        # === 内部状态 ===
        self.state = GState.INIT
        self.current_gripper_pos = 0.0
        self.target_gripper_pos = 0.0
        
        self.max_open_pos = None       
        self.range_calibrated = False
        self.calibration_start_time = None
        self.closing_start_time = None
        self.command_sent = False # 关键标志位：确保指令只发一次
        
        # 触觉数据
        self.raw_tactile_data = {}
        self.tactile_baseline = {}
        self.is_calibrated = False
        
        # 计数器
        self.cali_sample_count = 0
        self.cali_samples_req = 20
        self.contact_stability_counter = 0
        
        # === ROS接口 ===
        self.pub_gripper = node.create_publisher(GripperCtrl, '/gripper/ctrl', 10)
        
        self.sub_gripper_state = node.create_subscription(
            DHGripperState, '/gripper/states', self.gripper_state_callback, 10)
            
        self.sub_tactile = node.create_subscription(
            TactileArray, '/tactile_data', self.tactile_callback, 10)
            
        self.node.get_logger().info(f"Tactile Gripper: V11 (Guarded Move - One Shot)")

    def gripper_state_callback(self, msg):
        """读取夹爪实时反馈"""
        if hasattr(msg, 'position'):
            new_pos = float(msg.position)
            # 自动记录最大张开位置
            if self.max_open_pos is None or new_pos > self.max_open_pos:
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
        """
        发送控制指令
        模仿命令行: {initialize: false, position: ..., force: ..., speed: ...}
        """
        cmd = GripperCtrl()
        cmd.initialize = do_init 
        cmd.position = float(pos)
        
        # 如果不指定速度/力，使用默认的安全值
        cmd.speed = float(speed) if speed is not None else self.grasp_speed
        cmd.force = float(force) if force is not None else self.grasp_force
        
        self.pub_gripper.publish(cmd)
        
        # 仅用于调试日志
        # init_str = "INIT=True" if do_init else "Init=False"
        # self.node.get_logger().info(f"CMD -> Pos:{pos:.0f} Spd:{cmd.speed:.0f} {init_str}")

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

    def check_both_touch(self):
        """检测双指接触"""
        touch_count = 0
        for idx in self.raw_tactile_data:
            if self.get_normalized_nf(idx) > self.contact_threshold:
                touch_count += 1
        return touch_count >= 2

    def calibrate_gripper_range(self):
        """启动时先让夹爪完全张开，确定量程"""
        if self.range_calibrated: return True
        
        # 第一次进入，发送初始化指令
        if not self.command_sent:
            self.node.get_logger().info("Initializing Gripper (Pos=100000, Init=True)...")
            self.send_gripper_cmd(1000.0, speed=50, force=50, do_init=True)
            self.calibration_start_time = time.time()
            self.command_sent = True
            return False
        
        # 等待 3 秒让夹爪动完
        if time.time() - self.calibration_start_time > 3.0: 
            if self.max_open_pos is not None:
                self.range_calibrated = True
                self.node.get_logger().info(f"Gripper Ready. Max Pos: {self.max_open_pos}")
                self.command_sent = False # 重置标志位，供后续使用
                return True
        return False

    def start_grasping_sequence(self):
        self.state = GState.INIT
        self.node.get_logger().info("Starting V11 Sequence.")

    def tick(self):
        """主逻辑循环 (建议频率 20Hz-50Hz)"""
        
        # ----------------------------------------------------
        # 1. 系统初始化与范围校准
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
                # 使用 Init=False 发送运动指令
                self.send_gripper_cmd(self.max_open_pos, do_init=False)
                self.command_sent = True
            
            # 简单延时等待到位
            self.tick_counter = 0
            self.state = GState.WAIT_OPEN
            return "RUNNING"

        # ----------------------------------------------------
        # 3. 等待 + 准备触觉标定
        # ----------------------------------------------------
        elif self.state == GState.WAIT_OPEN:
            self.tick_counter += 1
            if self.tick_counter > 20: # 约1秒
                self.state = GState.CALIBRATION
                self.cali_sample_count = 0
                self.tactile_baseline = {}
                self.command_sent = False
            return "RUNNING"

        # ----------------------------------------------------
        # 4. 触觉传感器去皮 (Taring)
        # ----------------------------------------------------
        elif self.state == GState.CALIBRATION:
            if self.cali_sample_count == 0:
                self.node.get_logger().info("Taring Sensors...")
            
            for idx, data in self.raw_tactile_data.items():
                if idx not in self.tactile_baseline: self.tactile_baseline[idx] = 0.0
                self.tactile_baseline[idx] += data['nf']
            self.cali_sample_count += 1
            
            if self.cali_sample_count >= self.cali_samples_req:
                for idx in self.tactile_baseline:
                    self.tactile_baseline[idx] /= self.cali_samples_req
                
                self.is_calibrated = True
                self.state = GState.CLOSING_UNTIL_TOUCH
                self.command_sent = False
                self.closing_start_time = time.time()
                self.node.get_logger().info("Calibration Done. START CLOSING (Guarded Move).")
            return "RUNNING"

        # ----------------------------------------------------
        # 5. 闭合直到接触 (Guarded Move)
        # ----------------------------------------------------
        elif self.state == GState.CLOSING_UNTIL_TOUCH:
            
            # [A] 动作发起：只发一次指令
            if not self.command_sent:
                # 目标：0.0 (全闭)
                # 速度：self.grasp_speed (20%) -> 慢速闭合，方便刹车
                # 力：self.grasp_force (30%) -> 硬件保护
                self.send_gripper_cmd(0.0, speed=self.grasp_speed, force=self.grasp_force, do_init=False)
                self.command_sent = True
                self.node.get_logger().info(f"Sent Slow Close CMD (Target=0, Speed={self.grasp_speed})")
            
            # [B] 实时监测：如果碰到，立即刹车
            if self.check_any_touch():
                self.node.get_logger().info(f"!!! TOUCH DETECTED at {self.current_gripper_pos:.0f} !!!")
                
                # [刹车逻辑]：立即发送当前位置作为目标
                stop_pos = self.current_gripper_pos
                self.target_gripper_pos = stop_pos 
                # 使用较快的速度刹车，并维持当前力矩
                self.send_gripper_cmd(stop_pos, speed=100, force=self.grasp_force, do_init=False)
                
                self.contact_stability_counter = 0
                self.state = GState.ADJUSTING_FOR_BOTH_TOUCH
                self.command_sent = False
                return "RUNNING"
            
            # [C] 到底检测：如果长时间运行且位置接近0
            elapsed = time.time() - self.closing_start_time
            threshold = self.max_open_pos * 0.02 # 2%
            
            if elapsed > 2.0 and self.current_gripper_pos < threshold:
                self.node.get_logger().warn(f"Fully closed (Pos={self.current_gripper_pos:.0f}). No object.")
                self.state = GState.RELEASE
                self.command_sent = False
            
            return "RUNNING"

        # ----------------------------------------------------
        # 6. 双指平衡调整 (可选)
        # ----------------------------------------------------
        elif self.state == GState.ADJUSTING_FOR_BOTH_TOUCH:
            # 简单策略：如果已经接触，就直接进入保持，不再微调，防止捏碎鸡蛋
            # 对于易碎品，检测到接触后立即停止是最安全的
            self.node.get_logger().info("Touch detected. Holding position.")
            self.state = GState.GENTLE_CLAMPING
            self.command_sent = False
            return "RUNNING"

        # ----------------------------------------------------
        # 7. 柔顺保持
        # ----------------------------------------------------
        elif self.state == GState.GENTLE_CLAMPING:
            if not self.command_sent:
                # 稍微加一点点预紧力 (挤压 2%)
                squeeze = self.max_open_pos * 0.02
                final_pos = max(0.0, self.target_gripper_pos - squeeze)
                
                # 保持较小的力控
                self.send_gripper_cmd(final_pos, speed=10, force=self.grasp_force, do_init=False)
                self.target_gripper_pos = final_pos
                self.command_sent = True
                
                self.node.get_logger().info(f"Grasped Secured at {final_pos:.0f}")
                self.state = GState.SLIP_DETECTION
                
            return "SUCCESS"

        # ----------------------------------------------------
        # 8. 防滑监测 (被动)
        # ----------------------------------------------------
        elif self.state == GState.SLIP_DETECTION:
            # 在这里，我们只监测，不主动发指令，除非检测到滑动
            # 如果之前发的指令有效，夹爪应该会保持在 final_pos
            
            # 这里简单做个维持：每隔一段时间发一次心跳？
            # 实际上大寰夹爪不需要心跳，只要不掉电就会保持。
            
            # TODO: 如果检测到滑动，可以增加 target_gripper_pos 并重新发送
            pass 
            
            return "RUNNING"

        # ----------------------------------------------------
        # 9. 释放
        # ----------------------------------------------------
        elif self.state == GState.RELEASE:
            if not self.command_sent:
                self.send_gripper_cmd(self.max_open_pos, do_init=False)
                self.command_sent = True
                self.node.get_logger().info("Releasing")
            return "FAILURE"

        return "RUNNING"