# gz_moveit_ur3e_robotiq_camera.launch.py
#
# Single-file launch (no reuse of your other launch files):
# - Starts Gazebo Sim via ros_gz_sim/gz_sim.launch.py (GUI on/off)
# - Spawns robot via ros_gz_sim/create using "-string <xacro output>" (same pattern as your working spawn)
# - Spawns controllers EXACTLY like your working file (same controller-manager args)
# - Starts MoveIt move_group + MoveIt RViz using configs from moveit_config pkg
# - Forces use_sim_time = True everywhere
#
# Requirements:
# - moveit_config package exists and contains:
#   config/ur.srdf
#   config/kinematics.yaml
#   config/joint_limits.yaml
#   config/moveit_controllers.yaml
#   rviz/moveit.rviz
# - Your sim/runtime pkg contains the xacro and controller yaml you already use.

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
    [FindPackageShare(runtime_config_package), "worlds", "conveyor_base.sdf"]
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
            "-topic",
            "-x", "0",
            "-y", "0",
            "-z", "0.755",
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

    # -------------------------------------------------------------------------
    # MoveIt (NO ompl_planning.yaml dependency; uses only required files)
    # -------------------------------------------------------------------------
    moveit_pkg = LaunchConfiguration("moveit_config_package").perform(context)
    moveit_share = FindPackageShare(moveit_pkg).find(moveit_pkg)

    srdf_path = os.path.join(moveit_share, "config", "ur.srdf")
    rviz_config_path = os.path.join(moveit_share, "config", "moveit.rviz")

    kinematics = _load_yaml_required(moveit_pkg, "config/kinematics.yaml")
    joint_limits = _load_yaml_required(moveit_pkg, "config/joint_limits.yaml")
    moveit_controllers = _load_yaml_required(moveit_pkg, "config/moveit_controllers.yaml")

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_path]),
            value_type=str,
        )
    }

    # Minimal planning pipeline config so MoveIt doesn’t need ompl_planning.yaml
    # (MoveIt still uses OMPL plugin, but this avoids the missing file error.)
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
    }
    ompl_min = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning": joint_limits},
            moveit_controllers,
            planning_pipelines,
            ompl_min,
            {
                "planning_scene_monitor": {
                    "publish_robot_description": True,
                    "publish_robot_description_semantic": True,
                    "publish_planning_scene": True,
                }
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning": joint_limits},
            planning_pipelines,
            ompl_min,
        ],
    )

    # Start MoveIt after controllers are up (prevents SRDF timeout / empty doc)
    start_moveit_after_controllers = TimerAction(
        period=8.0,
        actions=[
            move_group_node,
            TimerAction(period=1.0, actions=[rviz_node]),
        ],
    )

    return [
        gz_launch_with_gui,
        gz_launch_without_gui,
        gz_clock_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        start_controllers_after_spawn,
        start_moveit_after_controllers,
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
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="moveit_config",
            description="MoveIt config package name (contains config/ and rviz/).",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
