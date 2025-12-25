colcon build --symlink-install

source install/setup.bash

ros2 launch visual_tactile_grasping system.launch.py