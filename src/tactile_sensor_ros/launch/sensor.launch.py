import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'tactile_sensor_ros'
    
    # 配置文件路径 (如果以后有 .yaml 配置文件)
    # config = os.path.join(
    #     get_package_share_directory(pkg_name),
    #     'config',
    #     'params.yaml'
    # )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='tactile_node.py', # 注意：这里对应 CMakeLists.txt install(PROGRAMS ...) 的名字
            name='tactile_driver',
            output='screen',
            parameters=[
                {'polling_rate': 100.0},
                {'max_finger_num': 5},
                # {'use_sim_time': False}
            ]
        )
    ])