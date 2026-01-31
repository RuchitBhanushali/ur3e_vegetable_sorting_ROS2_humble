from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("moveit_config")

    def inc(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", name))
        )

    return LaunchDescription([
        inc("rsp.launch.py"),
        inc("static_virtual_joint_tfs.launch.py"),
        # inc("spawn_controllers.launch.py"),
        inc("move_group.launch.py"),
        inc("moveit_rviz.launch.py"),
    ])
