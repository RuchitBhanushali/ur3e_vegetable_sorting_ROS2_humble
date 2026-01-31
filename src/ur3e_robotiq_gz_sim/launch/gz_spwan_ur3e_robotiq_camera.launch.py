import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml_required(pkg: str, relpath: str):
    pkg_share = FindPackageShare(pkg).find(pkg)
    abspath = os.path.join(pkg_share, relpath)
    if not os.path.exists(abspath):
        raise FileNotFoundError(f"YAML not found: {abspath}")
    with open(abspath, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    # ---- Args (keep your working ones) ----
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---- Paths for sim runtime ----
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    initial_positions_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )

    # If world_file is just "empty.sdf", use it from your sim pkg worlds folder.
    # If user passes an absolute path, Gazebo will accept it as-is.
    world_sdf = PathJoinSubstitution(
    [FindPackageShare(runtime_config_package), "worlds", "pick_place_table_world.sdf"]
)

    # ---- Robot description (MATCH your working spawn file) ----
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_positions_file_abs,
            " ",
            "include_ros2_control:=true",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    gz_launch_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-r -v 4 ", world_sdf]}.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-s -r -v 4 ", world_sdf]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # ---- /clock bridge ----
    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # ---- robot_state_publisher (sim time ON) ----
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # ---- Spawn robot (MATCH your working file: -string <xacro output>) ----
    gz_spawn_entity = Node(
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
        ],
    )

    # ---- Controller spawners (MATCH your working file arguments) ----
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start controllers only after spawn finishes (reliable ordering)
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(period=0.5, actions=[joint_state_broadcaster_spawner]),
                TimerAction(period=1.5, actions=[initial_joint_controller_spawner_stopped]),
                TimerAction(period=1.5, actions=[initial_joint_controller_spawner_started]),
                TimerAction(period=2.5, actions=[gripper_controller_spawner]),
            ],
        )
    )



    return [
        gz_launch_with_gui,
        gz_launch_without_gui,
        gz_clock_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        start_controllers_after_spawn,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from Gazebo.",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur5", "ur10",
                "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e",
                "ur20", "ur30",
            ],
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur3e_robotiq_gz_sim",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur3e_robotiq_ros2_controllers.yaml",
            description="Controllers YAML under <runtime_config_package>/config/",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="Initial positions YAML under <runtime_config_package>/config/",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="ur3e_robotiq_gz_sim",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="gz_ur_robotiq_camera.urdf.xacro",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
        ),
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="World filename in <runtime_config_package>/worlds/",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
