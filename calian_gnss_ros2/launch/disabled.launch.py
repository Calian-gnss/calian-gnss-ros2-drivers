import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#     Log Level values for different logging levels
#     NotSet = 0
#     Debug = 10
#     Info = 20
#     Warn = 30
#     Error = 40
#     Critical = 50


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "config.yaml"
    )

    pp_corrections_config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "pointperfect.yaml"
    )
    ntrip_corrections_config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "ntrip.yaml"
    )
    logs_config = os.path.join(
        get_package_share_directory("calian_gnss_ros2"), "params", "logs.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="calian_gnss_ros2",
                executable="calian_gnss_gps",
                name="gps_publisher",
                output="screen",
                emulate_tty=True,
                parameters=[config, logs_config],
                namespace="calian_gnss",
                arguments=["Disabled"],
            ),
            # Node(
            #     package="calian_gnss_ros2",
            #     executable="pointperfect",
            #     name="pointperfect",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[corrections_config, logs_config],
            #     namespace="calian_gnss",
            # ),
            Node(
                package="calian_gnss_ros2",
                executable="ntrip_client",
                name="ntrip_client",
                output="screen",
                emulate_tty=True,
                parameters=[ntrip_corrections_config, logs_config],
                namespace="calian_gnss",
            ),
            Node(
                package="calian_gnss_ros2",
                executable="calian_gnss_gps_visualizer",
                name="gps_visualizer",
                output="screen",
                emulate_tty=False,
                parameters=[{"port": 8080}],
                namespace="calian_gnss",
            ),
        ]
    )
