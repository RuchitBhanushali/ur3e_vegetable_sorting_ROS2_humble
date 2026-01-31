from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Shared args (keep defaults aligned with your existing launches)
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
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
        DeclareLaunchArgument("world_file", default_value="empty.sdf"),

        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),

        # MTC/MoveIt side
        DeclareLaunchArgument("moveit_config_package", default_value="moveit_config"),
        DeclareLaunchArgument("srdf_file", default_value="ur.srdf"),
        DeclareLaunchArgument("start_rviz", default_value="true"),

        # IMPORTANT: give Gazebo/controllers time before move_group
        DeclareLaunchArgument("moveit_start_delay", default_value="6.0"),
        DeclareLaunchArgument("mtc_start_delay", default_value="14.0"),

        DeclareLaunchArgument("arm_group", default_value="ur_arm"),
        DeclareLaunchArgument("eef_frame", default_value="robotiq_base_link"),
        DeclareLaunchArgument("world_frame", default_value="base_link"),
        DeclareLaunchArgument("tcp_offset_z", default_value="0.244"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("max_solutions", default_value="1"),
    ]

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur3e_robotiq_gz_sim"),
                "launch",
                "gz_spwan_ur3e_robotiq_camera.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "ur_type": LaunchConfiguration("ur_type"),
            "safety_limits": LaunchConfiguration("safety_limits"),
            "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
            "safety_k_position": LaunchConfiguration("safety_k_position"),
            "runtime_config_package": LaunchConfiguration("runtime_config_package"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "initial_positions_file": LaunchConfiguration("initial_positions_file"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "prefix": LaunchConfiguration("prefix"),
            "start_joint_controller": LaunchConfiguration("start_joint_controller"),
            "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "world_file": LaunchConfiguration("world_file"),
        }.items(),
    )

    mtc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur3e_pick_place_mtc"),
                "launch",
                "pick_place_mtc.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "ur_type": LaunchConfiguration("ur_type"),
            "safety_limits": LaunchConfiguration("safety_limits"),
            "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
            "safety_k_position": LaunchConfiguration("safety_k_position"),
            "runtime_config_package": LaunchConfiguration("runtime_config_package"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "initial_positions_file": LaunchConfiguration("initial_positions_file"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "prefix": LaunchConfiguration("prefix"),
            "moveit_config_package": LaunchConfiguration("moveit_config_package"),
            "srdf_file": LaunchConfiguration("srdf_file"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "moveit_start_delay": LaunchConfiguration("moveit_start_delay"),
            "mtc_start_delay": LaunchConfiguration("mtc_start_delay"),
            "arm_group": LaunchConfiguration("arm_group"),
            "eef_frame": LaunchConfiguration("eef_frame"),
            "world_frame": LaunchConfiguration("world_frame"),
            "tcp_offset_z": LaunchConfiguration("tcp_offset_z"),
            "execute": LaunchConfiguration("execute"),
            "max_solutions": LaunchConfiguration("max_solutions"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [gz_launch, mtc_launch])
