#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import threading
import termios
import tty

# 导入核心模块
from visual_tactile_grasping.tactile_gripper import TactileGripper
from visual_tactile_grasping.utils import GripperState as GState

class TactileAlgoTester(Node):
    def __init__(self):
        super().__init__('tactile_algo_tester')
        
        self.gripper = TactileGripper(self)
        self.running = True
        
        # 测试流程状态
        self.test_phase = "IDLE" 
        
        self.get_logger().info("==========================================")
        self.get_logger().info("   自适应触觉抓取算法测试 (无视觉/无运动)")
        self.get_logger().info("==========================================")
        self.get_logger().info("按键说明:")
        self.get_logger().info("  [Enter] : 开始下一步 (放入物体后按此键)")
        self.get_logger().info("  [R]     : 释放/重置 (Release)")
        self.get_logger().info("  [Q]     : 退出")
        self.get_logger().info("==========================================")

        # 启动键盘监听线程
        self.input_thread = threading.Thread(target=self.keyboard_listener)
        self.input_thread.daemon = True
        self.input_thread.start()

        # 启动主控定时器 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        """核心控制循环"""
        
        # 1. 等待阶段
        if self.test_phase == "IDLE":
            # 确保夹爪打开
            if self.gripper.state != GState.OPEN and self.gripper.state != GState.WAIT_OPEN:
                self.gripper.send_gripper_cmd(1000) # Open
                self.test_phase = "WAIT_FOR_OBJECT"
                print("\n>>> 夹爪已打开。请手动将物体(鸡蛋/豆腐)放入指尖之间。")
                print(">>> 准备好后，按 [Enter] 键开始抓取...")

        # 2. 等待用户放入物体
        elif self.test_phase == "WAIT_FOR_OBJECT":
            pass # 等待键盘事件触发 start_grasping_sequence

        # 3. 执行抓取算法
        elif self.test_phase == "GRASPING":
            # 调用核心算法的 tick
            status = self.gripper.tick()
            
            if status == "SUCCESS":
                self.get_logger().info("\n>>> 抓取成功！进入 [模拟搬运/防滑监测] 阶段 <<<")
                self.get_logger().info(">>> 此时你可以尝试轻轻向下拉扯物体，测试防滑功能。")
                self.get_logger().info(">>> 按 [R] 键放置物体。")
                self.test_phase = "HOLDING_AND_MONITORING"
                
            elif status == "FAILURE":
                self.get_logger().error("\n>>> 抓取失败 (未检测到物体或超时) <<<")
                self.test_phase = "IDLE"

        # 4. 模拟搬运过程 (持续监测滑动)
        elif self.test_phase == "HOLDING_AND_MONITORING":
            # 即使抓取成功了，我们也要继续 tick 以运行防滑逻辑 (Slip Detection)
            # 在 tactile_gripper.py 中，我们需要手动将状态设为 SLIP_DETECTION 才能持续监测
            if self.gripper.state != GState.SLIP_DETECTION:
                self.gripper.state = GState.SLIP_DETECTION
            
            self.gripper.tick()
            # 这里没有任何退出条件，直到用户按 R

        # 5. 放置
        elif self.test_phase == "RELEASING":
            self.get_logger().info("正在放置...")
            self.gripper.send_gripper_cmd(1000) # Open
            time.sleep(1.0)
            self.test_phase = "IDLE"

    def keyboard_listener(self):
        """非阻塞键盘监听"""
        def get_key():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        while self.running:
            key = get_key()
            if key == '\r': # Enter key
                if self.test_phase == "WAIT_FOR_OBJECT":
                    self.get_logger().info("启动抓取算法...")
                    self.gripper.start_grasping_sequence()
                    self.test_phase = "GRASPING"
            elif key.lower() == 'r':
                self.get_logger().info("重置系统...")
                self.test_phase = "RELEASING"
            elif key.lower() == 'q':
                self.get_logger().info("退出程序")
                self.running = False
                rclpy.shutdown()
                sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    tester = TactileAlgoTester()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    except Exception as e: # 捕获 rclpy.shutdown 导致的异常
        pass
    finally:
        if rclpy.ok():
            tester.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()