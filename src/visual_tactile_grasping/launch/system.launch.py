import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. RealSense 相机 ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true'
        }.items(),
    )

    # --- 2. 睿尔曼机械臂 ---
    rm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_65_config'), 'launch', 'real_moveit_demo.launch.py')
        ]),
        launch_arguments={'rm_ip': '192.168.1.18'}.items(), 
    )

    # --- 3. 大寰夹爪驱动 ---
    gripper_node = Node(
        package='dh_gripper_driver',
        executable='dh_gripper_driver',
        name='dh_gripper_driver',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0'}] 
    )

    # --- 4. 触觉传感器驱动 ---
    tactile_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tactile_sensor_ros'), 'launch', 'sensor.launch.py')
        ])
    )

    # ==========================================
    # [新增] 手眼标定静态 TF 发布节点
    # ==========================================
    # 作用：连接机械臂末端(link6)和相机(camera_link)
    # 参数格式：X Y Z (米) Yaw Pitch Roll (弧度) parent_frame child_frame
    # 请根据实际安装位置修改前三个数字 (X, Y, Z)
    hand_eye_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf_publisher',
        # 假设相机镜头中心在法兰中心前方5cm(x=0.05), 上方2cm(z=0.02)
        arguments=['0.05', '0.0', '0.02', '0.0', '0.0', '0.0', 'link6', 'camera_link']
    )
    # 定义夹爪指尖中心相对于法兰(link6)的位置
    # AG-95 长度约为 0.16m (Z轴向下为负? 注意：RM65 link6 的 Z 轴通常是垂直法兰面向外的)
    # 假设 link6 Z轴朝外，那么指尖就在 Z = 0.16m 处
    gripper_tcp_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_tcp_tf',
        # X=0, Y=0, Z=0.20 (20cm), 后面是旋转 (0,0,0)
        arguments=['0.0', '0.0', '0.30', '0.0', '0.0', '0.0', 'link6', 'gripper_tcp']
    )

    # --- 5. 主控制器 ---
    main_controller_node = Node(
        package='visual_tactile_grasping',
        executable='main_controller',
        name='visual_tactile_controller',
        output='screen'
    )
    
    delayed_main_controller = TimerAction(
        period=10.0, 
        actions=[main_controller_node]
    )

    
    return LaunchDescription([
        realsense_launch,
        rm_moveit_launch,
        gripper_node,
        tactile_launch,
        hand_eye_tf_node,  
        gripper_tcp_tf,
        delayed_main_controller
    ])