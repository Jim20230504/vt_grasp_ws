colcon build --symlink-install

source install/setup.bash

ros2 launch visual_tactile_grasping system.launch.py

触觉：

# 启动节点
ros2 launch tactile_sensor_ros sensor.launch.py

# 在一个新终端中查看数据：

ros2 topic echo /tactile_data



测试抓取: ros2 run visual_tactile_grasping test_tactile_algo