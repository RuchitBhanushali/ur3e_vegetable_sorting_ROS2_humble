from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare("ur3e_robotiq_gz_sim")

    world = PathJoinSubstitution([pkg, "worlds", "empty.sdf"])
    urdf = PathJoinSubstitution([pkg, "urdf", "ur3e_with_robotiq_gz.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "ur3e_robotiq_ros2_controllers.yaml"])

    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf,
            " simulation_controllers:=",
            controllers
        ])
    }

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world]
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur",
            "-topic", "robot_description",
            "-z", "0.0",
        ],
        output="screen",
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    spawner_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller"],
    )

    return LaunchDescription([
        gz,
        rsp,
        spawn,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[jsb, traj],
            )
        ),
        spawner_gripper,
    ])
