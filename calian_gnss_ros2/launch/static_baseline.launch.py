"""Launch: Static-baseline configuration.

Starts the Ably-based remote RTCM handler, a Rover GPS node, and the visualizer.
A TruPrecision base station pushes RTCM to an Ably channel; the handler relays
it to the rover via the ``rtcm_topic``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from calian_gnss_ros2.launch_common import config_path, logs_config_path, gps_node, visualizer_node

_RTCM_REMAP = [("rtcm_corrections", "rtcm_topic")]
_PKG = "calian_gnss_ros2"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viz_port", default_value="8080",
                                 description="HTTP port for the map visualizer"),
            Node(
                package=_PKG,
                executable="remote_rtcm_corrections_handler",
                name="rtcm_handler",
                output="screen",
                emulate_tty=False,
                parameters=[config_path(), logs_config_path()],
                namespace="calian_gnss",
                remappings=_RTCM_REMAP,
            ),
            gps_node(name="rover", mode="Rover", remappings=_RTCM_REMAP),
            visualizer_node(LaunchConfiguration("viz_port")),
        ]
    )
