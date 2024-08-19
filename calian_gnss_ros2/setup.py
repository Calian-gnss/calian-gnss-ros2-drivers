from setuptools import find_packages, setup
import os

package_name = "calian_gnss_ros2"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]


def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for path, directories, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=package_files(data_files, ["launch/", "params/"]),
    install_requires=[
        "setuptools<=58.2.0",
        "cs-events",
        "paho-mqtt",
        "pyubx2",
        "folium",
        "pyserial",
        "sensor_msgs",
    ],
    zip_safe=True,
    maintainer="pkgodugunuri",
    maintainer_email="pavan.godugunuri@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calian_gnss_gps = calian_gnss_ros2.gps:main",
            "calian_gnss_gps_visualizer = calian_gnss_ros2.gps_visualizer:main",
            "remote_rtcm_corrections_handler = calian_gnss_ros2.remote_rtcm_corrections_handler:main",
            "unique_id_finder = calian_gnss_ros2.unique_id_finder:main",
            "pointperfect = calian_gnss_ros2.pointperfect_module:main",
            "ntrip_client = calian_gnss_ros2.ntrip_module:main",
        ],
    },
)
