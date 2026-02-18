"""Launch: Moving-baseline configuration.

Starts a Base GPS node, a Rover GPS node, an NTRIP client, and the visualizer.
The base publishes RTCM corrections to the ``rtcm_topic`` which the rover consumes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from calian_gnss_ros2.launch_common import gps_node, ntrip_node, visualizer_node

_RTCM_REMAP = [("rtcm_corrections", "rtcm_topic")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viz_port", default_value="8080",
                                 description="HTTP port for the map visualizer"),
            gps_node(name="base", mode="Heading_Base", remappings=_RTCM_REMAP),
            ntrip_node(),
            gps_node(name="rover", mode="Rover", remappings=_RTCM_REMAP),
            visualizer_node(),
        ]
    )
