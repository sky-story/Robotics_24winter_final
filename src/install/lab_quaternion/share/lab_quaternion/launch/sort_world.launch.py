from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
    robot_type = LaunchConfiguration("robot_type")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    robot_traj_controller = LaunchConfiguration("robot_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    robot_lite_hand_controller = LaunchConfiguration("robot_lite_hand_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper = LaunchConfiguration("gripper")

    # Define correct path to `sort_world.sdf`
    world_file = PathJoinSubstitution([
        FindPackageShare("lab_quaternion"),
        "worlds",
        "sort_world.sdf"
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(description_package),
        "arms",
        robot_type,
        dof,
        "dof/config",
        controllers_file,
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "rviz",
        "view_robot.rviz"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "robots", description_file]),
        " ",
        "robot_ip:=xxx.yyy.zzz.www",
        " ",
        "name:=", robot_name,
        " ",
        "arm:=", robot_type,
        " ",
        "dof:=", dof,
        " ",
        "vision:=", vision,
        " ",
        "prefix:=", prefix,
        " ",
        "sim_gazebo:=", sim_gazebo,
        " ",
        "sim_ignition:=", sim_ignition,
        " ",
        "simulation_controllers:=", robot_controllers,
        " ",
        "gripper:=", gripper,
        " ",
    ])
    
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_pos_controller, "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager"],
        condition=UnlessCondition(PythonExpression(["'", robot_type, "' == 'gen3_lite'"])),
    )

    robot_lite_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_lite_hand_controller, "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", robot_type, "' == 'gen3_lite'"])),
    )

    gzserver = ExecuteProcess(
        cmd=["gzserver", "--verbose", world_file],  # ✅ 直接用 world_file
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"ign_args": ["-r", "-v", "3", world_file]}.items(),
        condition=IfCondition(sim_ignition),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        robot_lite_hand_controller_spawner,
        gzserver,
        gzclient,
        ignition_launch_description,
    ]

    return nodes_to_start



def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="kortex_description", description="机器人描述包."),
        DeclareLaunchArgument("description_file", default_value="kinova.urdf.xacro", description="URDF/XACRO 文件."),
        DeclareLaunchArgument("robot_controller", default_value="gen3_lite_joint_trajectory_controller", description="机器人控制器."),
        DeclareLaunchArgument("robot_pos_controller", default_value="twist_controller", description="位置控制器."),
        DeclareLaunchArgument("robot_hand_controller", default_value="robotiq_gripper_controller", description="机械手控制器."),
        DeclareLaunchArgument("robot_lite_hand_controller", default_value="gen3_lite_2f_gripper_controller", description="Lite 机械手控制器."),
        DeclareLaunchArgument("robot_name", default_value="gen3", description="机器人名称."),
        DeclareLaunchArgument("robot_type", choices=["gen3", "gen3_lite"], default_value="gen3", description="机器人型号."),
        DeclareLaunchArgument("dof", choices=["6", "7"], default_value="7", description="自由度."),
        DeclareLaunchArgument("gripper", default_value="gen3_lite_2f", description="机械爪类型."),
        DeclareLaunchArgument("vision", choices=["true", "false"], default_value="false", description="是否使用相机."),
        DeclareLaunchArgument("sim_gazebo", default_value="false", description="是否使用 Gazebo 模拟."),
        DeclareLaunchArgument("sim_ignition", default_value="true", description="是否使用 Ignition 模拟."),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="是否启动 RViz."),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="是否使用模拟时间."),
        DeclareLaunchArgument("prefix", default_value='""', description="关节名称前缀，用于多机器人设置."),
        DeclareLaunchArgument("controllers_file", default_value="ros2_controllers.yaml", description="YAML file with controllers."),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


