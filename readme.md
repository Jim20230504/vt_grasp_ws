export ROS_DOMAIN_ID=33
colcon build --symlink-install

source install/setup.bash

ros2 launch visual_tactile_grasping system.launch.py

# 触觉：
# 启动节点
ros2 launch tactile_sensor_ros sensor.launch.py
# 在一个新终端中查看数据：
ros2 topic echo /tactile_data



# 测试抓取: 
ros2 run visual_tactile_grasping test_tactile_algo

# 查看机械臂姿态：
ros2 topic echo /joint_states



# 列出当前系统所有图像 topic
ros2 topic list | grep image


# 验证 RealSense 是否在发图
ros2 topic echo /camera/camera/color/image_raw --once

# 查看当前坐标变换
ros2 run tf2_ros tf2_echo base_link camera_link

# 或者查看所有坐标变换
ros2 run tf2_ros tf2_monitor

# 生成 TF 树可视化文件：
ros2 run tf2_tools view_frames
