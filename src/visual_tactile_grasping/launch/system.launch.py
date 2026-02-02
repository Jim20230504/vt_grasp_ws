import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from moveit_configs_utils import MoveItConfigsBuilder

# 辅助函数：安全加载 YAML
def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading yaml: {e}")
        return None

def generate_launch_description():
    
    # ==========================================================
    # 1. 关键修复：精确定位 URDF 文件
    # ==========================================================
    # 根据你的目录结构，URDF 在 ts_robot_description/urdf/ts_robot.urdf.xacro
    ts_desc_share = get_package_share_directory('ts_robot_description')
    xacro_file = os.path.join(ts_desc_share, 'urdf', 'ts_robot.urdf.xacro')
    
    # 确保文件存在
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"URDF not found at: {xacro_file}")

    # 生成 XML 内容 (供 RobotStatePublisher 使用)
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file]
    )
    robot_description = {'robot_description': robot_description_content}

    # ==========================================================
    # 2. 配置 MoveIt (带 Pilz)
    # ==========================================================
    # 这里我们告诉 MoveItConfigsBuilder：
    # "配置参数去 rm_65_config 找，但机器人模型用我指定的这个路径"
    moveit_config = (
        MoveItConfigsBuilder("ts_robot", package_name="rm_65_config")
        .robot_description(file_path=xacro_file) # 🔥 传入绝对路径
        .robot_description_semantic(file_path="config/rm_65_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # ==========================================================
    # 3. 补全控制器参数 (解决 "Unable to identify controllers")
    # ==========================================================
    # MoveItConfigsBuilder 有时不会自动展开 controllers.yaml 的内容
    # 我们手动读出来，合并到参数里
    controllers_yaml = load_yaml("rm_65_config", "config/moveit_controllers.yaml")
    
    move_group_params = moveit_config.to_dict()
    
    # 显式指定管理器
    move_group_params.update({
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
    })
    
    # 注入控制器详细配置 (controller_names, rm_group_controller 等)
    if controllers_yaml:
        move_group_params.update(controllers_yaml)

    # ==========================================================
    # 4. 启动节点
    # ==========================================================

    # [A] 硬件驱动 (复用官方 launch，确保 IP/端口 参数正确)
    rm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_driver'), 'launch', 'rm_65_driver.launch.py')
        ])
    )
    
    rm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rm_control'), 'launch', 'rm_65_control.launch.py')
        ])
    )

    # [B] Robot State Publisher (修复 TF 断裂的关键)
    # 它会读取 URDF 中的 <joint name="camera_joint" type="fixed">
    # 从而发布 base_link -> camera_link 的变换
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # [C] Joint State Publisher
    # 监听真实机械臂关节，并补充夹爪等被动关节的默认值(0)
    # 这能消除 "No link elements found" 及其引发的 TF 树不完整
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['/joint_states']}, 
            {'rate': 30},
            robot_description
        ],
    )

    # [D] MoveGroup (带 Pilz)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_params, # 🔥 使用补全后的参数
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True} 
        ],
    )

    # [E] 其他组件 (相机、夹爪、触觉、可视化)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'base_frame_id': 'camera_link', # 这里的 link 名必须与 URDF 里的一致
            'publish_tf': 'true', 
            'tf_publish_rate': '30.0',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'unite_imu_method': 'copy'
        }.items(),
    )

    gripper_node = Node(
        package='dh_gripper_driver',
        executable='dh_gripper_driver',
        name='dh_gripper_driver',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0'}] 
    )

    tactile_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tactile_sensor_ros'), 'launch', 'sensor.launch.py')
        ])
    )

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

    # [F] 任务节点
    task_node = Node(
        package='visual_tactile_grasping',
        executable='main_controller', 
        name='elegant_pick_task',
        output='screen',
        parameters=[moveit_config.to_dict()] 
    )
    
    delayed_task = TimerAction(
        period=10.0, 
        actions=[task_node]
    )

    return LaunchDescription([
        rm_driver_launch,
        rm_control_launch,
        rsp_node,
        joint_state_publisher_node,
        realsense_launch,
        gripper_node,
        tactile_launch,
        run_move_group_node, 
        rviz_node,
        delayed_task
    ])