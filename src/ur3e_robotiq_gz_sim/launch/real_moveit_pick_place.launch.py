# real_moveit_pick_place.launch.py
#
# Real robot bringup (NO Gazebo):
# - Starts Universal Robots ROS2 driver (ur_robot_driver)
# - Starts MoveIt move_group using ur3e_moveit_config
# - Starts RViz (optional)
# - Starts your pick_place node (optional), delayed after move_group
#
# Notes:
# - No controller spawners here. The UR driver provides controller_manager + controllers.
# - No /clock bridge, no use_sim_time.
# - You must have the External Control URCap running on the teach pendant.
# - Update robot_ip when launching.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # -------------------------
    # Args
    # -------------------------
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_pick_place = LaunchConfiguration("launch_pick_place")

    # -------------------------
    # UR driver (REAL robot)
    # -------------------------
    # ur_robot_driver provides ur_control.launch.py in Humble.
    # This launches ros2_control + controller_manager for the real robot.
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": "false",
            "launch_rviz": "false",
        }.items(),
    )

    # -------------------------
    # MoveIt config (same package)
    # -------------------------
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur3e_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()
    config_dict.update({"use_sim_time": False})

    # -------------------------
    # Move group
    # -------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # -------------------------
    # RViz (optional)
    # -------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory("ur3e_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(launch_rviz),
    )

    # -------------------------
    # Your pick_place node (optional, delayed)
    # -------------------------
    pick_place_node = Node(
        package="ur3e_robotiq_gz_sim",   # keeping your existing package/executable
        executable="pick_place",
        output="screen",
        parameters=[config_dict],
        condition=IfCondition(launch_pick_place),
    )

    delay_pick_place = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[TimerAction(period=3.0, actions=[pick_place_node])],
        )
    )

    delay_rviz = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[rviz_node],
        )
    )

    return [
        ur_driver_launch,
        move_group_node,
        delay_rviz,
        delay_pick_place,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.10",
            description="IP address of the real UR robot.",
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
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz with MoveIt config.",
        ),
        DeclareLaunchArgument(
            "launch_pick_place",
            default_value="true",
            description="Start your pick_place node.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
