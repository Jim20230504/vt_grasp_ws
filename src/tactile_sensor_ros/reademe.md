# source 环境
source install/setup.bash

# 启动节点
ros2 launch tactile_sensor_ros sensor.launch.py

# 在一个新终端中查看数据：

ros2 topic echo /tactile_data
