#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visual_tactile_grasping.utils import SystemState, OBSERVATION_JOINT_POSE, DROP_OFF_JOINT_POSE
from visual_tactile_grasping.perception import PerceptionModule
from visual_tactile_grasping.motion_control import MotionControl
from visual_tactile_grasping.tactile_gripper import TactileGripper
from visual_tactile_grasping.utils import GripperState as GState

class VisualTactileController(Node):
    def __init__(self):
        super().__init__('visual_tactile_controller')
        self.state = SystemState.INITIALIZING
        
        # 【关键】标准抓取姿态 (垂直向下)
        # 如果发现爪子朝天，请改为 [0, 1, 0, 0] 试试
        self.GRASP_ORIENTATION = [1.0, 0.0, 0.0, 0.0] 
        
        self.target_pose_list = None 
        self.perception = PerceptionModule(self, model_name='yolov5s') 
        self.motion = MotionControl(self, group_name="rm_group") 
        self.gripper = TactileGripper(self)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.wait_time = 0
        self.sub_step = 0

    def control_loop(self):
        # 1. 初始化
        if self.state == SystemState.INITIALIZING:
            self.gripper.tick()
            self.wait_time += 1
            if self.wait_time > 20:
                if self.gripper.state != GState.INIT:
                    self.get_logger().info("System Ready.")
                    self.state = SystemState.MOVING_TO_OBSERVE

        # 2. 去观察点
        elif self.state == SystemState.MOVING_TO_OBSERVE:
            self.motion.move_to_joint_pose(OBSERVATION_JOINT_POSE)
            self.wait_time = 50 
            self.state = SystemState.DETECTING_WAIT 

        elif self.state == SystemState.DETECTING_WAIT:
            if self.wait_time > 0: self.wait_time -= 1; return
            self.state = SystemState.DETECTING

        # 3. 识别
        elif self.state == SystemState.DETECTING:
            found, u, v, angle = self.perception.detect_object()
            if found:
                pose_7d = self.perception.get_object_pose_base_frame(u, v, angle)
                if pose_7d:
                    tx, ty, tz, _, _, _, _ = pose_7d
                    # 强制使用我们定义的标准姿态，忽略视觉算的旋转
                    qx, qy, qz, qw = self.GRASP_ORIENTATION
                    self.target_pose_list = [tx, ty, tz, qx, qy, qz, qw]
                    
                    self.get_logger().info(f"Target Found at Z={tz:.3f}. Planning Approach...")
                    self.perception.publish_marker(tx, ty, tz, qx, qy, qz, qw)
                    self.state = SystemState.PLANNING_APPROACH

        # 4. 第一阶段：飞到预备点 (物体上方 15cm)
        elif self.state == SystemState.PLANNING_APPROACH:
            tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
            
            # 【关键】预备点不要太高，防止够不着
            # 15cm 是个比较舒服的距离，机械臂容易伸展开
            pre_grasp_z = tz + 0.15
            
            self.get_logger().info(f"1. Moving to Pre-Grasp (Z={pre_grasp_z:.3f})...")
            
            # 使用 move_to_pose (PTP)，让机械臂自由调整姿态飞过去
            # 只要到了那里是指尖朝下就行
            self.motion.move_to_pose(tx, ty, pre_grasp_z, qx, qy, qz, qw)
            
            self.wait_time = 0 
            self.sub_step = 0
            self.state = SystemState.APPROACHING

        # 5. 第二阶段：直线下降
        elif self.state == SystemState.APPROACHING:
            # 防抖
            if self.wait_time > 0: self.wait_time -= 1; return
            
            current_pose = self.motion.get_current_pose()
            if not current_pose: return

            tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
            
            # [Step 0] 检查是否到达预备点
            if self.sub_step == 0:
                dist_z = abs(current_pose.position.z - (tz + 0.15))
                # XY 平面也要检查一下，防止虽然高度对了但位置偏了
                dist_xy = ((current_pose.position.x - tx)**2 + (current_pose.position.y - ty)**2)**0.5
                
                if dist_z < 0.03 and dist_xy < 0.03:
                    self.get_logger().info("Reached Pre-Grasp. 2. Computing Linear Descent...")
                    
                    # 构建直线下降的目标 Pose
                    # 直接沿用当前的姿态！(既然已经到了预备点，姿态肯定是对的，直接保持住)
                    descent_pose = Pose()
                    descent_pose.position.x = tx
                    descent_pose.position.y = ty
                    descent_pose.position.z = tz - 0.005 # 稍微往下压，确保抓到
                    descent_pose.orientation = current_pose.orientation # 【关键】继承当前姿态
                    
                    # 计算直线路径
                    trajectory = self.motion.compute_linear_path(descent_pose)
                    
                    if trajectory:
                        self.get_logger().info("Linear Path Found! Executing...")
                        self.motion.execute_trajectory(trajectory)
                        self.sub_step = 1
                        self.wait_time = 20 # 等待执行完成
                    else:
                        self.get_logger().error("Linear Planning Failed! (Unreachable?)")
                        self.state = SystemState.MOVING_TO_OBSERVE # 回去重试
                else:
                    # 还在飞向预备点的路上...
                    pass

            # [Step 1] 检查是否到达抓取点
            elif self.sub_step == 1:
                dist = abs(current_pose.position.z - (tz - 0.005))
                if dist < 0.02:
                    self.get_logger().info("Ready to Grasp.")
                    self.gripper.start_grasping_sequence()
                    self.state = SystemState.TACTILE_GRASPING

        # 6. 抓取与后续
        elif self.state == SystemState.TACTILE_GRASPING:
            status = self.gripper.tick()
            if status == "SUCCESS":
                self.state = SystemState.LIFTING
                self.sub_step = 0

        elif self.state == SystemState.LIFTING:
            self.gripper.tick()
            # 同样用直线规划抬起
            if self.sub_step == 0:
                current = self.motion.get_current_pose()
                lift_pose = copy.deepcopy(current)
                lift_pose.position.z += 0.15 # 垂直抬起 15cm
                
                traj = self.motion.compute_linear_path(lift_pose)
                if traj:
                    self.motion.execute_trajectory(traj)
                    self.sub_step = 1
                    self.wait_time = 30
            elif self.sub_step == 1:
                if self.wait_time <= 0:
                    self.state = SystemState.PLACING
                else:
                    self.wait_time -= 1

        elif self.state == SystemState.PLACING:
            self.gripper.tick()
            self.motion.move_to_joint_pose(DROP_OFF_JOINT_POSE)
            self.wait_time += 1
            if self.wait_time > 80:
                 self.gripper.state = GState.RELEASE
                 self.gripper.tick()
                 self.state = SystemState.IDLE
                 self.get_logger().info("Done.")

def main(args=None):
    rclpy.init(args=args)
    controller = VisualTactileController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()