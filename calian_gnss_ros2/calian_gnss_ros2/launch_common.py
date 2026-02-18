"""Shared helpers for Calian GNSS launch files.

Centralises the config-path resolution, visualizer node, and NTRIP node
definitions so that the individual launch files stay DRY.
"""

import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_PKG = "calian_gnss_ros2"


def _share(*parts: str) -> str:
    """Resolve a path inside the installed package share directory."""
    return os.path.join(get_package_share_directory(_PKG), *parts)


def config_path() -> str:
    return _share("params", "config.yaml")


def ntrip_config_path() -> str:
    return _share("params", "ntrip.yaml")


def logs_config_path() -> str:
    return _share("params", "logs.yaml")


def gps_node(name: str, mode: str, *, remappings: list | None = None) -> Node:
    """Return a GPS Node action.

    Parameters
    ----------
    name : str
        ROS node name (e.g. ``"gps_publisher"``, ``"base"``, ``"rover"``).
    mode : str
        Operating mode passed as CLI argument (``"Disabled"``, ``"Heading_Base"``,
        or ``"Rover"``).
    remappings : list, optional
        ROS topic remappings.
    """
    return Node(
        package=_PKG,
        executable="calian_gnss_gps",
        name=name,
        output="screen",
        emulate_tty=True,
        parameters=[config_path(), logs_config_path()],
        namespace="calian_gnss",
        remappings=remappings or [],
        arguments=[mode],
    )


def ntrip_node() -> Node:
    """Return the NTRIP client Node action."""
    return Node(
        package=_PKG,
        executable="ntrip_client",
        name="ntrip_client",
        output="screen",
        emulate_tty=True,
        parameters=[ntrip_config_path(), logs_config_path()],
        namespace="calian_gnss",
    )


def visualizer_node(port: int = 8080) -> Node:
    """Return the GPS Visualizer Node action."""
    return Node(
        package=_PKG,
        executable="calian_gnss_gps_visualizer",
        name="gps_visualizer",
        output="screen",
        emulate_tty=False,
        parameters=[{"port": port}],
        namespace="calian_gnss",
    )
