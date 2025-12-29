import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # ==========================================
    # 1. 硬件驱动层
    # ==========================================

    # [A] RealSense 相机
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true'
        }.items(),
    )

    # [B] 睿尔曼机械臂 (RM65) - 使用官方 Bringup
    # 文档说 rm_bringup 可以一键启动 driver, control 和 moveit
    rm_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_bringup'), 'launch', 'rm_65_bringup.launch.py')
        ]),
        # 确保传递正确的 IP 
        # 注意：如果 bringup 内部写死了参数，你可能需要去修改 rm_driver/config/rm_65_config.yaml
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

   # ==========================================
    # 2. 坐标变换 (TF)
    # ==========================================
    # 手眼标定: Link6 (法兰) -> camera_link
    # 使用你计算出的精确数值
    # 格式: x y z qx qy qz qw frame_id child_frame_id
    # T(平移): 0.03357, 0.07922, -0.18146 (来自标定JSON)
    # R(旋转): 0.00419, 0.00374, 0.99998, -0.00072 (来自你计算的Quat)
    
    hand_eye_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf',
        arguments=[
            '0.03357', '0.07922', '-0.18146',   # x, y, z (单位: 米)
            '0.00419', '0.00374', '0.99998', '-0.00072', # qx, qy, qz, qw
            'Link6',            # 父坐标系 (法兰)
            'camera_link'       # 子坐标系 (相机)
        ]
    )

    # 夹爪TCP: link6 -> gripper_tcp (指尖)
    # 大寰 AG-95 长度
    gripper_tcp_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_tcp_tf',
        arguments=['0.0', '0.0', '0.30', '0.0', '0.0', '0.0', 'Link6', 'gripper_tcp']
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
        realsense_launch,
        rm_bringup_launch,
        gripper_node,
        tactile_launch,
        hand_eye_tf,
        gripper_tcp_tf,
        delayed_controller
    ])