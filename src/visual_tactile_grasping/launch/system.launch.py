import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution  # <--- 检查这个

  # <--- 还有这个

def generate_launch_description():
    
    # ==========================================
    # 1. 硬件驱动层
    # ==========================================

    # [A] 睿尔曼机械臂 (RM65) - 使用修改后的 Bringup
    # 注意：前提是你已经修改了 rm_65_bringup.launch.py 
    # 让它加载 'ts_robot_description' 包里的 'ts_robot.urdf.xacro'
    # 这样它就会发布包含 [机械臂+夹爪+相机] 的完整 TF 树。
    rm_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_bringup'), 'launch', 'rm_65_bringup.launch.py')
        ]),
    )

    # 关键修改：显式开启 TF 发布，确保相机数据帧能连上机器人 TF 树
    # [B] RealSense 相机
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'base_frame_id': 'camera_link',
            'publish_tf': 'false',             
            'tf_publish_rate': '30.0',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'unite_imu_method': 'copy'
        }.items(),
    )

    # [C] 大寰夹爪驱动
    gripper_node = Node(
        package='dh_gripper_driver',
        executable='dh_gripper_driver',
        name='dh_gripper_driver',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0'}] # 请确认端口
    )

    # [D] 触觉传感器驱动
    tactile_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tactile_sensor_ros'), 'launch', 'sensor.launch.py')
        ])
    )
    # 找到 MoveIt 的 RViz 配置文件
    # 假设你的配置文件在 rm_65_moveit_config/config/moveit.rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rm_65_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file], # 加载配置文件
        # parameters=[
        #     robot_description,
        #     robot_description_semantic,
        #     kinematics_yaml,
            
        # ],
    )
    # ==========================================
    # 3. 应用层
    # ==========================================

    # 我们的主控制器 (延迟15秒启动，确保 MoveIt 完全就绪)
    main_controller = Node(
        package='visual_tactile_grasping',
        executable='main_controller',
        name='visual_tactile_controller',
        output='screen'
    )
    
    delayed_controller = TimerAction(
        period=15.0, 
        actions=[main_controller]
    )

    return LaunchDescription([
        rm_bringup_launch,   # 负责：机械臂控制 + 发布完整 URDF TF 树
        realsense_launch,    # 负责：相机数据 + 光心 TF
        gripper_node,
        tactile_launch,
        # gripper_tcp_tf,     
        delayed_controller,
        rviz_node
    ])