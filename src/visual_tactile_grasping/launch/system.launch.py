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
    
    # hand_eye_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='hand_eye_tf',
    #     arguments=[
    #         # '0.03357', '0.07922', '-0.18146',   # x, y, z (单位: 米)
    #         # '0.00419', '0.00374', '0.99998', '-0.00072', # qx, qy, qz, qw
    #         # 'Link6',            # 父坐标系 (法兰)
    #         # 'camera_link'       # 子坐标系 (相机)
    #         '0.037', '0.269', '0.591',  # 注意y坐标可能取反
    #         '0.670', '0.667', '-0.248', '-0.213',  # 从tf2_echo获取的四元数
    #         'Link6', 'camera_link'
    #     ]
    # )

    # ==========================================
    # 2. 坐标变换 (TF)
    # ==========================================
    # 手眼标定: Link6 (法兰) -> calibrated_optical_frame
    
    hand_eye_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf',
        arguments=[
            '--x', '0.03357', '--y', '0.07922', '--z', '-0.18146',
            '--qx', '0.00419', '--qy', '0.00374', '--qz', '0.99998', '--qw', '-0.00072',
            '--frame-id', 'Link6',          # 父坐标系
            '--child-frame-id', 'calibrated_optical_frame' 
        ]
    )

    # ==========================================
    # [新增] 3. 相机内部桥接 (Bridge TF)
    # ==========================================
    # 作用: 将 RealSense 的 TF 树 (以 camera_link 为根) 
    #      挂载到我们的标定帧 (calibrated_optical_frame) 上。
    # 变换: Optical Frame -> Camera Link 是一个固定的旋转。
    # 平移: 设为 0 (忽略光心到外壳螺丝孔的微小距离，约1.5cm，对抓取影响不大)
    # 旋转: qx=0.5, qy=-0.5, qz=0.5, qw=0.5
    
    camera_bridge_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_bridge_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
            '--frame-id', 'calibrated_optical_frame',  # 父 (我们的标定末端)
            '--child-frame-id', 'camera_link'          # 子 (RealSense 的根)
        ]
    )

    # 夹爪TCP: Link6 -> gripper_tcp (指尖)
    gripper_tcp_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_tcp_tf',
        arguments=['0.0', '0.0', '0.22', '0.0', '0.0', '0.0', 'Link6', 'gripper_tcp']
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
        camera_bridge_tf,
        gripper_tcp_tf,
        delayed_controller
    ])