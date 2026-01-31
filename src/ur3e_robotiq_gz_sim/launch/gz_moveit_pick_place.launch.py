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
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit



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

    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur3e_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    cfg = moveit_config.to_dict()
    print("MOVEIT KEYS:", sorted(cfg.keys()))
    print("HAS KINEMATICS:", "robot_description_kinematics" in cfg)
    print("HAS SRDF:", "robot_description_semantic" in cfg)
    print("HAS URDF:", "robot_description" in cfg)

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

    ## Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    pick_place_node = Node(
        package="ur3e_robotiq_gz_sim",
        executable="pick_place",
        output="screen",
        parameters=[config_dict],   # IMPORTANT: same dict you pass to move_group
    )

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
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # delay_pick_place = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=move_group_node,
    #         on_start=[pick_place_node],
    #     )
    # )

    delay_pick_place = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[TimerAction(period=3.0, actions=[pick_place_node])],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=arm_trajectory_controller_spawner,
            on_start=[gripper_controller_spawner],
        )
    )

    delay_move_group = RegisterEventHandler(
        OnProcessStart(
            target_action=arm_trajectory_controller_spawner,
            on_start=[move_group_node],
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_trajectory_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )


    return [
        gz_launch_with_gui,
        gz_clock_bridge,
        gz_spawn_entity,
        robot_state_publisher,
        delay_rviz_node,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
        delay_move_group,
        delay_pick_place,
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
            default_value="ur3e_moveit_config",
            description="MoveIt config package name (contains config/ and rviz/).",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
