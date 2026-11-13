from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(
        get_package_share_directory("autominy_metrics")
    )
    config = str(share / "config" / "metrics.yaml")

    return LaunchDescription(
        [
            Node(
                package="autominy_metrics",
                executable="metrics_logger",
                name="autominy_metrics",
                output="screen",
                parameters=[config],
            )
        ]
    )
