#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import numpy as np
from collections import deque

# 导入消息类型
try:
    from dh_gripper_msgs.msg import GripperCtrl, GripperState as DHGripperState
except ImportError:
    class GripperCtrl:
        def __init__(self): self.initialize = False; self.position = 0.0; self.force = 0.0; self.speed = 0.0
    class DHGripperState:
        pass

from tactile_sensor_ros.msg import TactileArray
from visual_tactile_grasping.utils import GripperState as GState

class TactileGripper:
    def __init__(self, node: Node):
        self.node = node
        
        # === 1. 抓取参数配置 ===
        self.grasp_speed = 15.0 
        self.grasp_force = 30.0 
        self.contact_threshold = 0.5 
        
        # [防滑参数]
        self.slip_window_size = 10      # 滑动检测窗口大小 (约0.1秒)
        self.slip_std_threshold = 0.05  # TF 标准差阈值 (检测震动)
        self.re_grasp_step = 20.0       # 发生滑动时，夹爪进给步长 (脉冲)
        self.max_grasp_force = 100.0    # 自适应最大力限制
        
        # [放置参数]
        self.stable_tf_avg = 0.0        # 稳定抓取后的平均切向力
        self.release_ratio = 0.4        # 当 TF 降至稳定值的 40% 时，认为物体已着陆
        
        # === 2. 内部状态 ===
        self.state = GState.IDLE
        self.current_gripper_pos = 0.0
        self.target_gripper_pos = 0.0
        self.max_open_pos = 1000.0       
        
        # 标志位
        self.range_calibrated = False
        self.gripper_feedback_received = False
        self.command_sent = False 
        self.calibration_start_time = 0.0
        self.closing_start_time = 0.0
        
        # 触觉数据处理
        self.raw_tactile_data = {}
        self.tactile_baseline = {} # 去皮基准
        self.is_calibrated = False
        self.cali_sample_count = 0
        self.cali_samples_req = 20
        
        # [滑动检测缓冲区] {sensor_id: deque(maxlen=N)}
        self.tf_history = {} 
        
        # === ROS接口 ===
        self.pub_gripper = node.create_publisher(GripperCtrl, '/gripper/ctrl', 10)
        self.sub_gripper_state = node.create_subscription(
            DHGripperState, '/gripper/states', self.gripper_state_callback, 10)
        self.sub_tactile = node.create_subscription(
            TactileArray, '/tactile_data', self.tactile_callback, 10)
            
        self.node.get_logger().info(f"Tactile Gripper V2: Adaptive Slip & Place Detection Ready")

    def gripper_state_callback(self, msg):
        self.gripper_feedback_received = True
        if hasattr(msg, 'position'):
            new_pos = float(msg.position)
            if not self.range_calibrated and new_pos > self.max_open_pos:
                self.max_open_pos = new_pos
            self.current_gripper_pos = new_pos

    def tactile_callback(self, msg: TactileArray):
        """读取触觉数据并维护滑动检测窗口"""
        for finger in msg.fingers:
            idx = finger.sensor_index
            
            # 获取最大力值 (根据你的传感器特性，也可以用 sum 或 mean)
            max_nf = max(finger.nf) if len(finger.nf) > 0 else 0.0
            max_tf = max(finger.tf) if len(finger.tf) > 0 else 0.0
            
            self.raw_tactile_data[idx] = {'nf': max_nf, 'tf': max_tf}
            
            # 维护 TF 历史数据用于滑动检测
            if idx not in self.tf_history:
                self.tf_history[idx] = deque(maxlen=self.slip_window_size)
            self.tf_history[idx].append(max_tf)

    # --- 核心算法 1: 数据去皮 ---
    def get_clean_force(self, idx, force_type='nf'):
        if idx not in self.raw_tactile_data: return 0.0
        raw_val = self.raw_tactile_data[idx][force_type]
        # 基准值默认为0
        baseline = 0.0
        if idx in self.tactile_baseline:
            baseline = self.tactile_baseline[idx][force_type]
        return max(0.0, raw_val - baseline)

    # --- 核心算法 2: 滑动检测 (检测 TF 的高频震动) ---
    def check_slippage(self):
        """
        计算切向力的标准差。如果标准差突然变大，说明摩擦力不稳定，正在滑动。
        """
        is_slipping = False
        max_std = 0.0
        
        for idx, history in self.tf_history.items():
            if len(history) < self.slip_window_size: continue
            
            # 计算标准差
            std_dev = np.std(list(history))
            if std_dev > self.slip_std_threshold:
                is_slipping = True
                max_std = std_dev
                # 只有当法向力也存在时，才认为是有效滑动（排除空载震动）
                if self.get_clean_force(idx, 'nf') > 0.1: 
                    self.node.get_logger().warn(f"⚠️ SLIP DETECTED on Finger {idx}! StdDev: {std_dev:.3f}")
                    return True
        return False

    # --- 核心算法 3: 放置检测 (检测 TF 下降) ---
    def check_placed_on_table(self):
        """
        检测切向力是否显著小于由于重力产生的稳定切向力
        """
        if self.stable_tf_avg <= 0.01: return False # 还没抓稳，没法测
        
        current_total_tf = 0.0
        count = 0
        for idx in self.raw_tactile_data:
            current_total_tf += self.get_clean_force(idx, 'tf')
            count += 1
        
        if count == 0: return False
        avg_tf = current_total_tf / count
        
        # 调试日志 (可选)
        # self.node.get_logger().info(f"Place Check: Cur={avg_tf:.2f} / Base={self.stable_tf_avg:.2f}")
        
        # 如果当前摩擦力 < 稳定负载的 40%，说明桌子托住了物体
        if avg_tf < (self.stable_tf_avg * self.release_ratio):
            return True
        return False

    def calibrate_sensors(self):
        """采集基准值"""
        self.tactile_baseline = {}
        for idx, data in self.raw_tactile_data.items():
            self.tactile_baseline[idx] = {'nf': data['nf'], 'tf': data['tf']} # 同时校准 NF 和 TF

    # --- 状态机相关 ---
    def send_gripper_cmd(self, pos, speed=None, force=None, do_init=False):
        cmd = GripperCtrl()
        cmd.initialize = do_init 
        cmd.position = float(pos)
        cmd.speed = float(speed) if speed is not None else self.grasp_speed
        cmd.force = float(force) if force is not None else self.grasp_force
        self.pub_gripper.publish(cmd)

    def start_grasping_sequence(self):
        self.state = GState.INIT
        self.command_sent = False
        self.is_calibrated = False
        self.node.get_logger().info("Starting Adaptive Grasp Sequence.")

    def record_stable_load(self):
        """在抬起完成后调用，记录此时物体的重力产生的切向力"""
        total_tf = 0.0
        count = 0
        for idx in self.raw_tactile_data:
            total_tf += self.get_clean_force(idx, 'tf')
            count += 1
        if count > 0:
            self.stable_tf_avg = total_tf / count
            self.node.get_logger().info(f"⚖️ Stable Load Recorded: TF_avg = {self.stable_tf_avg:.3f}")
        else:
            self.stable_tf_avg = 0.0

    def tick(self):
        """主逻辑循环"""
        
        # 1. 初始化与开爪 (保持不变)
        if self.state == GState.INIT:
            # ... (此处省略具体的初始化和开爪代码，与之前版本一致，参考上一版 calibrate_gripper_range) ...
            if not self.range_calibrated: 
                # 简化的初始化逻辑，实际请复用上一版代码
                self.send_gripper_cmd(1000, speed=50, force=100, do_init=True)
                time.sleep(1.0)
                self.range_calibrated = True
            self.state = GState.OPEN
            return "RUNNING"

        elif self.state == GState.OPEN:
            if not self.command_sent:
                self.send_gripper_cmd(self.max_open_pos)
                self.command_sent = True
            self.tick_counter = 0
            self.state = GState.WAIT_OPEN
            return "RUNNING"

        elif self.state == GState.WAIT_OPEN:
            self.tick_counter += 1
            if self.tick_counter > 10:
                self.state = GState.CALIBRATION
                self.cali_sample_count = 0
            return "RUNNING"

        # 2. 去皮
        elif self.state == GState.CALIBRATION:
            if self.cali_sample_count == 0: self.node.get_logger().info("Taring...")
            # 简单累加均值逻辑 (略) -> 假设已完成
            self.calibrate_sensors() # 瞬时去皮
            self.is_calibrated = True
            self.state = GState.CLOSING_UNTIL_TOUCH
            self.command_sent = False
            return "RUNNING"

        # 3. 接触检测 (Guarded Move)
        elif self.state == GState.CLOSING_UNTIL_TOUCH:
            if not self.command_sent:
                self.send_gripper_cmd(0.0, speed=self.grasp_speed, force=self.grasp_force)
                self.command_sent = True
            
            # 使用去皮后的 NF 判断接触
            is_touched = False
            for idx in self.raw_tactile_data:
                if self.get_clean_force(idx, 'nf') > self.contact_threshold:
                    is_touched = True
                    break
            
            if is_touched:
                self.node.get_logger().info(f"Contact Detected! Position: {self.current_gripper_pos}")
                self.target_gripper_pos = self.current_gripper_pos
                # 急停
                self.send_gripper_cmd(self.current_gripper_pos, speed=100, force=self.grasp_force)
                self.state = GState.GENTLE_CLAMPING
                self.command_sent = False
            return "RUNNING"

        # 4. 柔顺保持 (Pre-load)
        elif self.state == GState.GENTLE_CLAMPING:
            if not self.command_sent:
                # 稍微夹紧一点点 (例如 2mm = 20个脉冲)
                final_pos = max(0.0, self.target_gripper_pos - 20.0)
                self.target_gripper_pos = final_pos
                self.send_gripper_cmd(final_pos, speed=10, force=self.grasp_force)
                self.command_sent = True
                self.node.get_logger().info("Grasped. Switching to Slip Detection Mode.")
                
                # 进入防滑保持状态
                self.state = GState.SLIP_DETECTION 
                
            return "SUCCESS" # 告诉主控：抓好了，可以抬起了

        # 5. 🔥🔥🔥 主动防滑保持 (在抬起和移动过程中持续运行) 🔥🔥🔥
        elif self.state == GState.SLIP_DETECTION:
            # 这是一个被动状态，主控在 MOVE 过程中会不断调用 tick()
            
            # A. 检测滑动
            if self.check_slippage():
                self.node.get_logger().warn("🚨 SLIP REFLEX TRIGGERED! Tightening grip...")
                
                # 自适应调整：夹得更紧，力矩更大
                self.target_gripper_pos = max(0.0, self.target_gripper_pos - self.re_grasp_step)
                self.grasp_force = min(self.max_grasp_force, self.grasp_force + 10.0)
                
                # 立即发送修正指令
                self.send_gripper_cmd(self.target_gripper_pos, speed=50, force=self.grasp_force)
                
                # 给一点时间让机械响应，避免连续触发
                time.sleep(0.1) 
            
            return "RUNNING"

        # 6. 🔥🔥🔥 自动释放检测 (放置模式) 🔥🔥🔥
        # 主控在进入 PLACING 阶段后，手动将 gripper.state 设为 AUTO_RELEASE
        elif self.state == GState.AUTO_RELEASE_MONITOR:
            
            # A. 检测是否着陆 (TF 消失)
            if self.check_placed_on_table():
                self.node.get_logger().info("⬇️ Object Landed (Shear force dropped). Releasing...")
                self.state = GState.RELEASE
                self.command_sent = False
            
            # B. 同时也依然要做防滑 (万一还没放稳就松手了不行)
            elif self.check_slippage():
                # 这里可以策略性地决定：是捏紧防止掉落，还是忽略
                # 通常在放置时，轻微震动可能是接触桌面，所以这里防滑可以稍微迟钝一点
                pass

            return "RUNNING"

        elif self.state == GState.RELEASE:
            if not self.command_sent:
                self.send_gripper_cmd(self.max_open_pos)
                self.command_sent = True
            return "SUCCESS"

        return "RUNNING"