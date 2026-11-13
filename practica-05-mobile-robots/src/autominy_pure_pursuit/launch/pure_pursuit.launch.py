from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(
        get_package_share_directory("autominy_pure_pursuit")
    )
    config = str(share / "config" / "pure_pursuit.yaml")

    return LaunchDescription(
        [
            Node(
                package="autominy_pure_pursuit",
                executable="pure_pursuit",
                name="autominy_pure_pursuit",
                output="screen",
                parameters=[config],
            )
        ]
    )
