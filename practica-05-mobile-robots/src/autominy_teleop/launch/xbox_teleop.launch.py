from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("autominy_teleop"))
    config = str(share / "config" / "xbox.yaml")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "coalesce_interval_ms": 1,
            }
        ],
    )

    teleop_node = Node(
        package="autominy_teleop",
        executable="xbox_teleop",
        name="xbox_teleop",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([joy_node, teleop_node])
