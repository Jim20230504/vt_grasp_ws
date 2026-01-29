import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # ==========================================
    # 1. 核心配置 (MoveIt + Pilz)
    # ==========================================
    # 🔥 关键：这里必须加载 Pilz，否则 elegant_motion.py 会报错
    
    moveit_config = (
        MoveItConfigsBuilder("ts_robot", package_name="rm_65_config")
        .robot_description(file_path="config/rm_65_description.urdf.xacro")
        .robot_description_semantic(file_path="config/rm_65_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"]) # 👈 启用 Pilz
        .to_moveit_configs()
    )

    # MoveGroup 节点
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz 节点 (加载 MoveIt 配置)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        arguments=["-d", os.path.join(get_package_share_directory("rm_65_config"), "config", "moveit.rviz")],
    )

    # ==========================================
    # 2. 硬件驱动层
    # ==========================================

    # [A] 睿尔曼机械臂驱动
    rm_driver_node = Node(
        package='rm_driver',
        executable='rm_driver',
        name='rm_driver',
        output='screen',
    )
    
    # [B] RealSense 相机
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'base_frame_id': 'camera_link',
            'publish_tf': 'true',  # 这里建议开 True，除非你有专门的 robot_state_publisher 发布了相机TF
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
        parameters=[{'port': '/dev/ttyUSB0'}] 
    )

    # [D] 触觉传感器驱动
    tactile_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tactile_sensor_ros'), 'launch', 'sensor.launch.py')
        ])
    )

    # [E] Robot State Publisher (如果 rm_bringup 没发，这里要发)
    # 如果上面的 rm_driver 不发 TF，你需要这个
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ==========================================
    # 3. 应用层 (新的 MTC 风格任务)
    # ==========================================

    # 🔥 替换：旧的 main_controller -> 新的 elegant_task
    task_node = Node(
        package='visual_tactile_grasping',
        # 注意：你需要去 setup.py 把 elegant_task.py 注册为可执行文件
        executable='main_controller', 
        name='elegant_pick_task',
        output='screen',
        parameters=[moveit_config.to_dict()] # 让节点能读到 robot_description
    )
    
    # 延迟启动，等待 MoveGroup 就绪
    delayed_task = TimerAction(
        period=10.0, 
        actions=[task_node]
    )

    return LaunchDescription([
        rsp_node,
        rm_driver_node,
        realsense_launch,
        gripper_node,
        tactile_launch,
        run_move_group_node, 
        rviz_node,
        delayed_task
    ])