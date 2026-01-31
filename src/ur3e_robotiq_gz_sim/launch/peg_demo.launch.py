import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")

    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    start_joint_controller = LaunchConfiguration("start_joint_controller")

    # --- paths ---
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    initial_positions_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )

    world_sdf = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "worlds", "peg_in_hole_demo.sdf"]
    )

    # --- robot_description ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "prefix:=", prefix,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=", controllers_yaml,
            " ",
            "initial_positions_file:=", initial_positions_abs,
            " ",
            "include_ros2_control:=true",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- Gazebo Sim launch ---
    gz_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-r -v 3 ", world_sdf]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gz_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-s -r -v 3 ", world_sdf]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # Clock bridge
    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # ---- Spawn robot (MATCH your working file: -string <xacro output>) ----
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
            "-topic",
            "-x", "0",
            "-y", "-0.25",
            "-z", "0.74",
        ],
    )


    # Controllers (spawn AFTER robot is created)
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    arm_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    arm_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Demo node (arm + gripper + peg follow)
    demo = Node(
        package="ur3e_robotiq_gz_sim",
        executable="peg_pick_insert_demo.py",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_name": "peg_in_hole_demo",
            "peg_name": "peg",
            "tool_frame": "tool0",
            "traj_topic": "/joint_trajectory_controller/joint_trajectory",
            "gripper_action": "/gripper_position_controller/gripper_cmd",
            "tcp_offset_z": 0.244
        }],
    )

    start_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn,
            on_exit=[
                TimerAction(period=0.5, actions=[jsb]),
                TimerAction(period=1.5, actions=[arm_stopped]),
                TimerAction(period=1.5, actions=[arm_started]),
                TimerAction(period=2.0, actions=[gripper]),
                TimerAction(period=3.0, actions=[demo]),
            ],
        )
    )

    return [
        gz_with_gui,
        gz_without_gui,
        gz_clock_bridge,
        rsp,
        gz_spawn,
        start_after_spawn,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),

        DeclareLaunchArgument("runtime_config_package", default_value="ur3e_robotiq_gz_sim"),
        DeclareLaunchArgument("controllers_file", default_value="ur3e_robotiq_ros2_controllers.yaml"),
        DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml"),

        DeclareLaunchArgument("description_package", default_value="ur3e_robotiq_gz_sim"),
        DeclareLaunchArgument("description_file", default_value="gz_ur_robotiq_camera.urdf.xacro"),

        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("prefix", default_value='""'),

        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
