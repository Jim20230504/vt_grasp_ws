#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Quaternion

# 导入消息类型
from visualization_msgs.msg import Marker


from visual_tactile_grasping.utils import SystemState, OBSERVATION_JOINT_POSE, DROP_OFF_JOINT_POSE, GRIPPER_LENGTH
from visual_tactile_grasping.perception import PerceptionModule
from visual_tactile_grasping.motion_control import MotionControl
from visual_tactile_grasping.tactile_gripper import TactileGripper
from visual_tactile_grasping.utils import GripperState as GState

class VisualTactileController(Node):
    def __init__(self):
        super().__init__('visual_tactile_controller')
        
        self.state = SystemState.IDLE
        
        # [新增] 用于存储目标 6D 姿态 (x, y, z, qx, qy, qz, qw)
        self.target_pose = None 
        
        # 初始化模块
        # 注意：这里加载 YOLOv5 模型
        self.perception = PerceptionModule(self, model_name='yolov5s') 
        
        # 确保组名正确 (根据之前的调试，SRDF中通常是 rm_group)
        self.motion = MotionControl(self, group_name="rm_group") 
        
        self.gripper = TactileGripper(self)
        
        # 控制循环定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("System Ready. Waiting for start...")
        
        # 自动开始延迟
        self.start_delay = 0
        self.wait_time = 0
        self.state = SystemState.INITIALIZING

        # 创建一个 Marker 发布器
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

    def publish_debug_marker(self, x, y, z):
        """发布一个红色小球到目标位置"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "grasp_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 位置
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        # 尺寸 (2cm 的小球)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        # 颜色 (红色)
        marker.color.r = 1.0
        marker.color.a = 1.0 # 不透明
        
        self.marker_pub.publish(marker)
    
    def control_loop(self):
        """主状态机循环"""
        
        # --- 1. 初始化 ---
        if self.state == SystemState.INITIALIZING:
            # 运行夹爪的 tick 以执行初始化逻辑 (V11 需要)
            self.gripper.tick()
            
            # 简单延时等待硬件就绪
            self.start_delay += 1
            if self.start_delay > 30: # 3秒
                if self.gripper.state != GState.INIT: # 确保夹爪初始化完成
                    self.get_logger().info("Hardware Ready. Moving to Observation Pose.")
                    self.state = SystemState.MOVING_TO_OBSERVE

        # --- 2. 移动到观察姿态 ---
        elif self.state == SystemState.MOVING_TO_OBSERVE:
            # 这里的观察姿态建议设高一点，视野好
            future = self.motion.move_to_joint_pose(OBSERVATION_JOINT_POSE)
            self.wait_time = 50 # 5秒等待运动
            self.state = SystemState.DETECTING_WAIT # 中间状态等待移动完成

        elif self.state == SystemState.DETECTING_WAIT:
            if self.wait_time > 0:
                self.wait_time -= 1
                return
            self.state = SystemState.DETECTING

        # --- 3. 视觉识别 (YOLOv5 + 角度计算) ---
        elif self.state == SystemState.DETECTING:
            self.get_logger().info("Detecting object (YOLOv5)...")
            
            # [关键修改] 获取 4 个返回值：found, u, v, angle
            found, u, v, angle = self.perception.detect_object()
            
            if found:
                self.get_logger().info(f"Object found at ({u}, {v}), Angle: {angle:.2f} rad")
                
                # [关键修改] 获取带角度的 7D 姿态 (Position + Orientation)
                pose_7d = self.perception.get_object_pose_base_frame(u, v, angle)
                
                if pose_7d:
                    self.target_pose = pose_7d
                    # 打印一下目标姿态确认
                    tx, ty, tz, qx, qy, qz, qw = pose_7d
                    self.get_logger().info(f"Target Pose: Pos=[{tx:.3f}, {ty:.3f}, {tz:.3f}], Quat=[{qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f}]")
                    
                    self.state = SystemState.PLANNING_APPROACH
                else:
                    self.get_logger().warn("TF Transform failed or Depth invalid. Retrying...")
            else:
                self.get_logger().info("No object detected. Retrying...")
                # 可以在这里增加重试计数，多次失败后报错

        # --- 4. 规划抓取 (使用计算出的角度) ---
        elif self.state == SystemState.PLANNING_APPROACH:
            # 解包 7D 姿态
            tx, ty, tz, qx, qy, qz, qw = self.target_pose

            # [调用] 发布可视化标记
            self.publish_debug_marker(tx, ty, tz)
            
            # 计算法兰目标高度
            # 目标：指尖到达物体上方 10cm (预备点)
            # 法兰 Z = 物体 Z + 夹爪长度 + 10cm
            approach_height = 0.10
            target_flange_z = tz + GRIPPER_LENGTH + approach_height
            
            self.get_logger().info(f"Approaching... Flange Z target: {target_flange_z:.3f}")
            
            # [关键修改] 使用视觉计算出的 Quaternion (qx, qy, qz, qw)
            # 这样机械臂会旋转手腕，使夹爪垂直于物体长轴
            self.motion.move_to_cartesian_pose(tx, ty, target_flange_z, qx, qy, qz, qw)
            
            self.wait_time = 40 # 等待运动
            self.state = SystemState.APPROACHING

        # --- 5. 执行接近与下降 ---
        elif self.state == SystemState.APPROACHING:
            if self.wait_time > 0:
                self.wait_time -= 1
                return
            
            # 此时已到达预备点，准备下降抓取
            self.get_logger().info("Moving down to grasp...")
            
            tx, ty, tz, qx, qy, qz, qw = self.target_pose
            
            # 最终抓取高度：物体表面 (或者稍微往下一点点，依赖触觉急停)
            # 法兰 Z = 物体 Z + 夹爪长度
            # 注意：这里的 0.005 是为了稍微过压一点点，确保接触，反正有触觉保护
            final_flange_z = tz + GRIPPER_LENGTH - 0.005 
            
            # 直线下降
            self.motion.move_to_cartesian_pose(tx, ty, final_flange_z, qx, qy, qz, qw)
            
            # 这里简单给个延时等待下降，实际应该检查 MoveIt 执行结果
            # 但为了配合触觉，我们可以在下降后直接启动触觉逻辑
            time.sleep(2.0) 
            
            # 启动触觉抓取流程
            self.gripper.start_grasping_sequence()
            self.state = SystemState.TACTILE_GRASPING

        # --- 6. 触觉自适应抓取 (V11 Guarded Move) ---
        elif self.state == SystemState.TACTILE_GRASPING:
            # 持续调用夹爪状态机
            status = self.gripper.tick()
            
            if status == "SUCCESS":
                self.get_logger().info("Grasp Secured! Lifting...")
                self.state = SystemState.LIFTING
                self.wait_time = 0
            elif status == "FAILURE":
                self.get_logger().error("Grasp Failed. Retrying or Aborting.")
                self.state = SystemState.MOVING_TO_OBSERVE # 回去重试
            else:
                # RUNNING 状态，继续等待
                pass

        # --- 7. 抬起物体 ---
        elif self.state == SystemState.LIFTING:
            # 抬起时，继续运行夹爪 tick 以保持防滑 (Slip Detection)
            self.gripper.tick()
            
            if self.wait_time == 0:
                tx, ty, tz, qx, qy, qz, qw = self.target_pose
                
                # 抬起 20cm
                lift_z = tz + GRIPPER_LENGTH + 0.20
                self.motion.move_to_cartesian_pose(tx, ty, lift_z, qx, qy, qz, qw)
                self.wait_time = 40
            
            else:
                self.wait_time -= 1
                if self.wait_time <= 0:
                    self.state = SystemState.PLACING

        # --- 8. 放置 ---
        elif self.state == SystemState.PLACING:
            # 放置时也继续防滑，直到到达目的地
            self.gripper.tick()
            
            self.get_logger().info("Moving to drop-off zone...")
            self.motion.move_to_joint_pose(DROP_OFF_JOINT_POSE)
            
            # 到达后释放
            # 这里简单用 sleep 模拟运动时间
            # 实际上应该检查 moveit 结果
            if self.wait_time == 0: # 复用 wait_time 逻辑
                 self.wait_time = 50
            else:
                 self.wait_time -= 1
                 if self.wait_time <= 0:
                     self.gripper.state = GState.RELEASE # 触发释放
                     self.gripper.tick() # 执行一次
                     self.state = SystemState.IDLE

        elif self.state == SystemState.ERROR:
            self.get_logger().error("System Error.")

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