import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
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

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    moveit_start_delay = LaunchConfiguration("moveit_start_delay")
    mtc_start_delay = LaunchConfiguration("mtc_start_delay")

    arm_group = LaunchConfiguration("arm_group")
    eef_frame = LaunchConfiguration("eef_frame")
    world_frame = LaunchConfiguration("world_frame")
    tcp_offset_z = LaunchConfiguration("tcp_offset_z")
    execute = LaunchConfiguration("execute")
    max_solutions = LaunchConfiguration("max_solutions")

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    initial_positions_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=", "ur", " ",
            "ur_type:=", ur_type, " ",
            "prefix:=", prefix, " ",
            "sim_ignition:=true", " ",
            "simulation_controllers:=", initial_joint_controllers, " ",
            "initial_positions_file:=", initial_positions_file_abs, " ",
            "include_ros2_control:=true",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_pkg = LaunchConfiguration("moveit_config_package").perform(context)
    moveit_share = FindPackageShare(moveit_pkg).find(moveit_pkg)

    srdf_file = LaunchConfiguration("srdf_file").perform(context)
    srdf_path = os.path.join(moveit_share, "config", srdf_file)

    if not os.path.exists(srdf_path):
        raise FileNotFoundError(f"SRDF not found: {srdf_path}")

    this_pkg_share = FindPackageShare("ur3e_pick_place_mtc").find("ur3e_pick_place_mtc")
    rviz_config_path = os.path.join(this_pkg_share, "config", "mtc.rviz")

    kinematics = _load_yaml_required(moveit_pkg, "config/kinematics.yaml")
    joint_limits = _load_yaml_required(moveit_pkg, "config/joint_limits.yaml")
    moveit_controllers = _load_yaml_required(moveit_pkg, "config/moveit_controllers.yaml")

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_path]),
            value_type=str,
        )
    }

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
        condition=IfCondition(start_rviz),
    )

    mtc_params_yaml = os.path.join(this_pkg_share, "config", "pick_place_mtc_parameters.yaml")

    mtc_node = Node(
        package="ur3e_pick_place_mtc",
        executable="pick_place_mtc",
        name="ur3e_pick_place_mtc",
        output="screen",
        parameters=[
            mtc_params_yaml,
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning": joint_limits},
            moveit_controllers,
            planning_pipelines,
            ompl_min,
            {
                "arm_group": arm_group,
                "eef_frame": eef_frame,
                "world_frame": world_frame,
                "tcp_offset_z": tcp_offset_z,
                "execute": execute,
                "max_solutions": max_solutions,
            },
        ],
    )

    start_moveit = TimerAction(
        period=moveit_start_delay,
        actions=[
            move_group_node,
            TimerAction(period=1.0, actions=[rviz_node]),
        ],
    )

    start_mtc = TimerAction(
        period=mtc_start_delay,
        actions=[mtc_node],
    )

    return [start_moveit, start_mtc]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            choices=[
                "ur3", "ur5", "ur10",
                "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e",
                "ur20", "ur30",
            ],
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="ur3e_robotiq_gz_sim"),
        DeclareLaunchArgument("controllers_file", default_value="ur3e_robotiq_ros2_controllers.yaml"),
        DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml"),
        DeclareLaunchArgument("description_package", default_value="ur3e_robotiq_gz_sim"),
        DeclareLaunchArgument("description_file", default_value="gz_ur_robotiq_camera.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("moveit_config_package", default_value="moveit_config"),
        DeclareLaunchArgument("srdf_file", default_value="ur.srdf"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_start_delay", default_value="2.0"),
        DeclareLaunchArgument("mtc_start_delay", default_value="10.0"),  # Increased

        # CRITICAL: Changed default world_frame to base_link
        DeclareLaunchArgument("arm_group", default_value="ur_arm"),
        DeclareLaunchArgument("eef_frame", default_value="robotiq_base_link"),
        DeclareLaunchArgument("world_frame", default_value="base_link"),  # FIXED

        DeclareLaunchArgument("tcp_offset_z", default_value="0.244"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("max_solutions", default_value="1"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])