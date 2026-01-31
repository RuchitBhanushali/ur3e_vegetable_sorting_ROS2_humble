import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define Arguments
    ur_type_arg = LaunchConfiguration('ur_type', default='ur3e') 
    pkg_description = FindPackageShare('ur3e_robotiq_gz_sim')

    # 2. Define XACRO file path
    xacro_file = PathJoinSubstitution([
        pkg_description,
        'urdf',
        'gz_ur_robotiq_camera.urdf.xacro'
    ])

    # 3. Process XACRO into URDF (Ensures spaces are correctly placed for shell command)
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ', # CRITICAL FIX: Space between executable and file
        xacro_file,
        ' ',
        # Pass the ur_type argument expected by the XACRO file
        'ur_type:=', 
        ur_type_arg,
    ])

    robot_description = {'robot_description': robot_description_content}

    # 4. Robot State Publisher Node (Converts URDF to TF frames)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 5. Joint State Publisher GUI Node (Allows interactive control in RViz)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # 6. RViz Node for visualization
    rviz_config_path = PathJoinSubstitution([
        pkg_description, 
        'rviz', 
        'ur_config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])