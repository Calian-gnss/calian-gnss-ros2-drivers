"""Launch: Disabled (single-antenna) configuration.

Starts a GPS node in Disabled mode alongside the NTRIP client and visualizer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from calian_gnss_ros2.launch_common import gps_node, ntrip_node, visualizer_node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viz_port", default_value="8080",
                                 description="HTTP port for the map visualizer"),
            gps_node(name="gps_publisher", mode="Disabled"),
            ntrip_node(),
            visualizer_node(),
        ]
    )
