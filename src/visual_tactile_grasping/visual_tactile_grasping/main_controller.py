#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
import copy
import math

# 引入自定义模块
# 确保你的 utils.py 里有这些定义
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
        self.sub_step = 0 # 用于管理异步操作的微步骤
        
        # 异步句柄
        self.async_future = None
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
                    self.sub_step = 0

        # ---------------------------------------------------------------------
        # 2. 移动到观察点 (Async Action)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.MOVING_TO_OBSERVE:
            # [Step 0] 发送请求
            if self.sub_step == 0:
                self.get_logger().info("Moving to Observation Pose...")
                self.async_future = self.motion.move_to_joint_pose(OBSERVATION_JOINT_POSE)
                self.sub_step = 1
            
            # [Step 1] 等待 Goal Accepted
            elif self.sub_step == 1:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if not goal_handle.accepted:
                        self.get_logger().error("Move Rejected!")
                        self.sub_step = 0
                        return
                    self.async_future = goal_handle.get_result_async()
                    self.sub_step = 2

            # [Step 2] 等待执行完成
            elif self.sub_step == 2:
                if self.async_future.done():
                    result = self.async_future.result().result
                    if result.error_code.val == 1: # SUCCESS
                        self.get_logger().info("Arrived at Observation Pose. Clearing stale cache...")
                        
                        # 🔥 关键：清除感知缓存，确保下次用新图
                        self.perception.latest_color_img = None
                        self.perception.latest_depth_img = None
                        self.perception.latest_header = None
                        
                        self.gripper.state = GState.OPEN
                        self.gripper.tick()
                        
                        # 等待相机曝光稳定 (2秒)
                        self.wait_time = 20 
                        self.state = SystemState.DETECTING_WAIT
                    else:
                        self.get_logger().error("Move Failed!")
                        self.state = SystemState.INITIALIZING

        # ---------------------------------------------------------------------
        # 2.5 等待检测 (防抖)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.DETECTING_WAIT:
            if self.wait_time > 0: self.wait_time -= 1; return
            self.state = SystemState.DETECTING

        # ---------------------------------------------------------------------
        # 3. 视觉识别 (✅ 已适配新接口)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.DETECTING:
            # 🌟 修复：解包 6 个返回值
            found, u, v, angle, strategy, obj_name = self.perception.detect_object()
            
            if found:
                self.current_strategy = strategy
                
                # 🌟 修复：传入 obj_name，以便 perception 选择算法 (RANSAC vs OBB)
                # 使用 self.perception.current_bbox 是最准的，
                # 但 get_object_pose_base_frame 内部已经处理了 bbox 参数
                # 我们这里显式传入 bbox=self.perception.current_bbox 确保万无一失
                
                pose_7d = self.perception.get_object_pose_base_frame(
                    u, v, angle, 
                    name=obj_name, 
                    grasp_strategy=strategy,
                    bbox=self.perception.current_bbox # 显式传入 bbox
                )
                
                if pose_7d:
                    self.target_pose_list = list(pose_7d)
                    tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
                    
                    # 打印坐标
                    self.get_logger().info(f"Target Pose: X={tx:.3f}, Y={ty:.3f}, Z={tz:.3f}")
                    self.get_logger().info(f"Target: {obj_name} ({strategy}) at Z={tz:.3f}")
                    
                    # 发布 RViz Marker
                    self.perception.publish_marker(tx, ty, tz, qx, qy, qz, qw)
                    
                    self.sub_step = 0 
                    self.state = SystemState.PLANNING_APPROACH

        # ---------------------------------------------------------------------
        # 4. 飞到预备点 (Async Action - PTP)
        # ---------------------------------------------------------------------
        elif self.state == SystemState.PLANNING_APPROACH:
            tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
            
            # [Step 0] 计算并发送请求
            if self.sub_step == 0:
                # 🌟 RANSAC 算出来的是中心点，我们需要根据策略调整预备点
                
                if self.current_strategy == "SIDE":
                    # 侧抓：Z轴保持不变(瓶子中心高度)，XY平面往回退 15cm
                    # perception 返回的 tz 已经是瓶子中心高度，非常准，不用动
                    side_grasp_z = tz 
                    
                    # 计算水平退后向量
                    dist = math.sqrt(tx**2 + ty**2)
                    if dist > 0.15:
                        ratio = (dist - 0.15) / dist
                        target_x, target_y, target_z = tx * ratio, ty * ratio, side_grasp_z
                    else:
                        # 距离太近，无法退后，直接报错或就在原地
                        self.get_logger().warn("Object too close!")
                        target_x, target_y, target_z = tx, ty, side_grasp_z

                    self.get_logger().info("Strategy SIDE: Moving to Front...")
                    
                else:
                    # 顶抓：Z轴往上提 10cm
                    target_x, target_y = tx, ty
                    target_z = tz + 0.10
                    self.get_logger().info("Strategy TOP: Moving to Above...")

                # 更新 target_pose_list 用于下一步直线插补
                # 注意：这里我们只更新 Z，因为 TOP 抓取最终要去的是 tx, ty, tz
                # 而 SIDE 抓取最终要去的是 tx, ty, tz
                # 下一步 approaching 会用到原始的 tx, ty, tz 作为终点
                
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
            tx, ty, tz, qx, qy, qz, qw = self.target_pose_list
            final_z = self.target_pose_list[2]
            
            # [Step 0] 🛑 新增：防抖延时 🛑
            # 刚飞过来可能没停稳，或者 joint_states 数据有延迟
            # 强制等待 0.5秒 (5个周期)，确保 current_joint_state 是最新的静止状态
            if self.sub_step == 0:
                if not hasattr(self, 'approach_wait_count'):
                    self.approach_wait_count = 0
                
                self.approach_wait_count += 1
                if self.approach_wait_count < 5: # 等待 5 * 0.1s = 0.5s
                    return 
                
                # 等待结束，进入计算
                self.sub_step = 1 

            # [Step 1] 发起直线路径计算
            elif self.sub_step == 1:
                # 此时获取的 pose 才是真正停稳后的 pose
                current_pose = self.motion.get_current_pose()
                if not current_pose: return
                
                grasp_pose = Pose()
                # 策略 A：为了丝滑，沿用当前机械臂的姿态（忽略感知的微小旋转误差）
                grasp_pose.orientation = current_pose.orientation 
                
                # 策略 B：如果你非常信任感知算出的 RANSAC 姿态，就用这个：
                # grasp_pose.orientation.x = qx
                # grasp_pose.orientation.y = qy
                # grasp_pose.orientation.z = qz
                # grasp_pose.orientation.w = qw
                
                # 位置设置
                if self.current_strategy == "TOP":
                    grasp_pose.position.x = tx
                    grasp_pose.position.y = ty
                    grasp_pose.position.z = final_z 
                else:
                    grasp_pose.position.x = tx
                    grasp_pose.position.y = ty
                    grasp_pose.position.z = final_z
                
                self.async_future = self.motion.compute_linear_path_async(grasp_pose)
                self.sub_step = 2 # 跳转到等待结果
                
            # [Step 2] 等待计算结果
            elif self.sub_step == 2:
                if self.async_future.done():
                    resp = self.async_future.result()
                    self.computed_trajectory = self.motion.process_linear_path_result(resp)
                    
                    if self.computed_trajectory:
                        self.get_logger().info("Linear Path Computed. Executing...")
                        self.async_future = self.motion.execute_trajectory(self.computed_trajectory)
                        self.sub_step = 3
                    else:
                        self.get_logger().error("Linear Plan Failed!")
                        self.state = SystemState.MOVING_TO_OBSERVE
                        self.sub_step = 0

            # [Step 3] 等待 Goal Accepted
            elif self.sub_step == 3:
                if self.async_future.done():
                    goal_handle = self.async_future.result()
                    if not goal_handle.accepted:
                        self.get_logger().error("Linear Execution Rejected!")
                        self.state = SystemState.MOVING_TO_OBSERVE
                        return
                    self.async_future = goal_handle.get_result_async()
                    self.sub_step = 4

            # [Step 4] 等待执行完成
            elif self.sub_step == 4:
                if self.async_future.done():
                    # 记得重置计数器，供下次使用
                    self.approach_wait_count = 0
                    
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
                lift_pose.position.z += 0.20 # 抬高一点
                
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
                    # 记录负载 (如果你用了新版 gripper)
                    if hasattr(self.gripper, 'record_stable_load'):
                        self.gripper.record_stable_load()
                        
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