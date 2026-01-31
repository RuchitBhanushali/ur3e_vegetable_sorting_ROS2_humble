import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
    # =========================================================================
    # CONFIGURATION
    # =========================================================================
    
    # Robot configuration
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # Package configuration
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # Controller configuration
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # Simulation configuration
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # MoveIt/MTC configuration
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    srdf_file = LaunchConfiguration("srdf_file")
    start_rviz = LaunchConfiguration("start_rviz")

    # MTC specific
    arm_group = LaunchConfiguration("arm_group")
    eef_frame = LaunchConfiguration("eef_frame")
    world_frame = LaunchConfiguration("world_frame")
    tcp_offset_z = LaunchConfiguration("tcp_offset_z")
    execute = LaunchConfiguration("execute")
    max_solutions = LaunchConfiguration("max_solutions")

    # =========================================================================
    # PATHS AND ROBOT DESCRIPTION
    # =========================================================================

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    initial_positions_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", initial_positions_file]
    )

    world_sdf = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "worlds", world_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
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

    # =========================================================================
    # MOVEIT CONFIGURATION
    # =========================================================================

    moveit_pkg = LaunchConfiguration("moveit_config_package").perform(context)
    moveit_share = FindPackageShare(moveit_pkg).find(moveit_pkg)

    srdf_file_name = LaunchConfiguration("srdf_file").perform(context)
    srdf_path = os.path.join(moveit_share, "config", srdf_file_name)

    if not os.path.exists(srdf_path):
        raise FileNotFoundError(f"SRDF not found: {srdf_path}")

    mtc_pkg_share = FindPackageShare("ur3e_pick_place_mtc").find("ur3e_pick_place_mtc")
    rviz_config_path = os.path.join(mtc_pkg_share, "config", "mtc.rviz")
    mtc_params_yaml = os.path.join(mtc_pkg_share, "config", "pick_place_mtc_parameters.yaml")

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

    # =========================================================================
    # STEP 1: GAZEBO SIMULATION
    # =========================================================================

    gz_launch_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world_sdf]}.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": ["-s -r -v 4 ", world_sdf]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # =========================================================================
    # STEP 2: CLOCK BRIDGE (CRITICAL FOR SIM TIME)
    # =========================================================================

    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # =========================================================================
    # STEP 3: ROBOT STATE PUBLISHER
    # =========================================================================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # =========================================================================
    # STEP 4: SPAWN ROBOT IN GAZEBO
    # =========================================================================

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

    # =========================================================================
    # STEP 5: CONTROLLERS (SPAWN AFTER ROBOT)
    # =========================================================================

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

    # =========================================================================
    # STEP 6: MOVE_GROUP (START AFTER CONTROLLERS ARE READY)
    # =========================================================================

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

    # =========================================================================
    # STEP 7: RVIZ (START WITH MOVE_GROUP)
    # =========================================================================

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

    # =========================================================================
    # STEP 8: MTC NODE (START LAST, AFTER EVERYTHING IS READY)
    # =========================================================================

    mtc_node = Node(
        package="ur3e_pick_place_mtc",
        executable="pick_place_trial",
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

    # =========================================================================
    # ORCHESTRATION: START EVERYTHING IN THE RIGHT ORDER
    # =========================================================================

    # Start controllers after robot spawn
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[joint_state_broadcaster_spawner],
                ),
                TimerAction(
                    period=1.5,
                    actions=[
                        initial_joint_controller_spawner_stopped,
                        initial_joint_controller_spawner_started,
                    ],
                ),
                TimerAction(
                    period=2.5,
                    actions=[gripper_controller_spawner],
                ),
            ],
        )
    )

    # Start MoveIt after gripper controller is ready
    start_moveit_after_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=gripper_controller_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        move_group_node,
                        TimerAction(period=1.0, actions=[rviz_node]),
                    ],
                ),
            ],
        )
    )

    # Start MTC after MoveIt is ready
    start_mtc_after_moveit = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[
                TimerAction(
                    period=5.0,  # Wait 5 seconds for MoveIt to fully initialize
                    actions=[mtc_node],
                ),
            ],
        )
    )

    # =========================================================================
    # RETURN ALL NODES AND EVENT HANDLERS
    # =========================================================================

    return [
        # Gazebo
        gz_launch_with_gui,
        gz_launch_without_gui,
        gz_clock_bridge,
        
        # Robot
        robot_state_publisher_node,
        gz_spawn_entity,
        
        # Event-driven startup sequence
        start_controllers_after_spawn,
        start_moveit_after_controllers,
        start_mtc_after_moveit,
    ]


def generate_launch_description():
    declared_arguments = [
        # Simulation
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("world_file", default_value="pick_place_table_world.sdf"),
        
        # Robot
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
        DeclareLaunchArgument("prefix", default_value='""'),
        
        # Packages
        DeclareLaunchArgument("runtime_config_package", default_value="ur3e_robotiq_gz_sim"),
        DeclareLaunchArgument("description_package", default_value="ur3e_robotiq_gz_sim"),
        DeclareLaunchArgument("moveit_config_package", default_value="moveit_config"),
        
        # Files
        DeclareLaunchArgument("controllers_file", default_value="ur3e_robotiq_ros2_controllers.yaml"),
        DeclareLaunchArgument("initial_positions_file", default_value="initial_positions.yaml"),
        DeclareLaunchArgument("description_file", default_value="gz_ur_robotiq_camera.urdf.xacro"),
        DeclareLaunchArgument("srdf_file", default_value="ur.srdf"),
        
        # Controllers
        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        
        # Visualization
        DeclareLaunchArgument("start_rviz", default_value="true"),
        
        # MTC Parameters
        DeclareLaunchArgument("arm_group", default_value="ur_arm"),
        DeclareLaunchArgument("eef_frame", default_value="robotiq_base_link"),
        DeclareLaunchArgument("world_frame", default_value="base_link"),
        DeclareLaunchArgument("tcp_offset_z", default_value="0.244"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("max_solutions", default_value="1"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])