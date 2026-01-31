import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def _load_yaml_required(pkg: str, relpath: str):
    pkg_share = FindPackageShare(pkg).find(pkg)
    abspath = os.path.join(pkg_share, relpath)
    if not os.path.exists(abspath):
        raise FileNotFoundError(f"YAML not found: {abspath}")
    with open(abspath, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    # -------------------- args --------------------
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # MoveIt/MTC
    moveit_pkg = LaunchConfiguration("moveit_config_package").perform(context)
    srdf_file = LaunchConfiguration("srdf_file").perform(context)
    start_rviz = LaunchConfiguration("start_rviz")

    moveit_start_delay = float(LaunchConfiguration("moveit_start_delay").perform(context))
    mtc_start_delay = float(LaunchConfiguration("mtc_start_delay").perform(context))

    # -------------------- paths --------------------
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    initial_positions_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )

    # Match your current working world choice
    world_sdf = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "worlds", "pick_place_table_world.sdf"]
    )

    # -------------------- robot_description --------------------
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
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # -------------------- MoveIt config loads --------------------
    moveit_share = FindPackageShare(moveit_pkg).find(moveit_pkg)
    srdf_path = os.path.join(moveit_share, "config", srdf_file)
    if not os.path.exists(srdf_path):
        raise FileNotFoundError(f"SRDF not found: {srdf_path}")

    kinematics = _load_yaml_required(moveit_pkg, "config/kinematics.yaml")
    joint_limits = _load_yaml_required(moveit_pkg, "config/joint_limits.yaml")
    moveit_controllers = _load_yaml_required(moveit_pkg, "config/moveit_controllers.yaml")

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(Command(["cat ", srdf_path]), value_type=str)
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

    # -------------------- Gazebo Sim --------------------
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


    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # Spawn robot (FIXED: no broken "-topic")
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

    # -------------------- Controllers --------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["gripper_position_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

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

    qos_fix = {
                "qos_overrides./joint_states.subscription.reliability": "reliable",
                "qos_overrides./joint_states.subscription.durability": "volatile",
                "qos_overrides./joint_states.subscription.history": "keep_last",
                "qos_overrides./joint_states.subscription.depth": 10,
            }

    # -------------------- MoveIt + RViz + MTC --------------------
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
            qos_fix,
        ],
    )

    mtc_pkg_share = FindPackageShare("ur3e_pick_place_mtc").find("ur3e_pick_place_mtc")
    rviz_config_path = os.path.join(mtc_pkg_share, "config", "mtc.rviz")
    mtc_params_yaml = os.path.join(mtc_pkg_share, "config", "pick_place_mtc_parameters.yaml")

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

    mtc_node = Node(
        package="ur3e_pick_place_mtc",
        executable="pick_place_mtc",
        name="ur3e_pick_place_mtc",
        output="screen",
        remappings=[
            ("joint_states", "/joint_states"),
            ("collision_object", "/collision_object"),
            ("planning_scene", "/planning_scene"),
            ],
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
            qos_fix,
        ],
    )

    # Start MoveIt + RViz + MTC after spawn/controllers (single graph, deterministic)
    start_moveit_and_mtc = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=moveit_start_delay,
                    actions=[
                        move_group_node,
                        TimerAction(period=1.0, actions=[rviz_node]),
                    ],
                ),
                TimerAction(
                    period=mtc_start_delay,
                    actions=[mtc_node],
                ),
            ],
        )
    )

    return [
        gz_launch_with_gui,
        # gz_launch_without_gui,
        gz_clock_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        start_controllers_after_spawn,
        start_moveit_and_mtc,
    ]


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
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        # MoveIt
        DeclareLaunchArgument("moveit_config_package", default_value="moveit_config"),
        DeclareLaunchArgument("srdf_file", default_value="ur.srdf"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        # ordering
        DeclareLaunchArgument("moveit_start_delay", default_value="6.0"),
        DeclareLaunchArgument("mtc_start_delay", default_value="14.0"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
