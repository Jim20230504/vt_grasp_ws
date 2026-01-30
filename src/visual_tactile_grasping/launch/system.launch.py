import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # ==========================================
    # 1. 核心配置 (MoveIt + Pilz)
    # ==========================================
    # 我们仍然需要构建 MoveIt 配置，以便让 MoveGroup 节点知道我们要用 Pilz
    moveit_config = (
        MoveItConfigsBuilder("ts_robot", package_name="rm_65_config")
        .robot_description(file_path="config/rm_65_description.urdf.xacro")
        .robot_description_semantic(file_path="config/rm_65_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"]) # 👈 启用 Pilz
        .to_moveit_configs()
    )

    # ==========================================
    # 2. 硬件驱动层 (使用厂家的一键启动)
    # ==========================================
    
    # [A] 睿尔曼机械臂 (RM65) - 复用旧版本的 Bringup
    # 理由：厂家提供的 launch 文件里包含了驱动连接参数、URDF加载、RSP发布等关键步骤。
    # 不要手动写 rm_driver 节点，除非你非常清楚所有参数。
    rm_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_bringup'), 'launch', 'rm_65_bringup.launch.py')
        ]),
    )

    # [B] RealSense 相机
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'base_frame_id': 'camera_link',
            'publish_tf': 'true', 
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

    # ==========================================
    # 3. 软件控制层 (MoveIt)
    # ==========================================

    # [E] MoveGroup 节点 (核心修改点)
    # 我们不使用 rm_bringup 自带的 move_group (如果有的话)，而是运行我们配置好的这个
    # 这样才能确保加载了 pilz 管道
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            # 强制告诉 MoveIt 不要自己发布 TF，因为 rm_bringup 已经发了
            # 如果发现 TF 闪烁，请检查这里
            {"publish_robot_description_semantic": True} 
        ],
    )

    # [F] RViz
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
    # 4. 应用层 (MTC 任务)
    # ==========================================

    # [G] 任务节点
    task_node = Node(
        package='visual_tactile_grasping',
        executable='main_controller', # 确保 setup.py 里 entry point 指向了 elegant_task.py
        name='elegant_pick_task',
        output='screen',
        parameters=[moveit_config.to_dict()] # 这一点很重要，让节点能查到运动学
    )
    
    delayed_task = TimerAction(
        period=10.0, 
        actions=[task_node]
    )

    return LaunchDescription([
        rm_bringup_launch,   # 1. 先启动硬件，确保 /joint_states 和 TF 正常
        realsense_launch,
        gripper_node,
        tactile_launch,
        run_move_group_node, # 2. 启动带 Pilz 的 MoveIt
        rviz_node,           # 3. 启动可视化
        delayed_task         # 4. 最后启动任务
    ])