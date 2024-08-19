# Calian GNSS ROS2 Custom Message Types

This package contains the custom message types used in the `calian_gnss_ros2` package. These message types are specifically designed to enhance communication and data exchange related to GNSS (Global Navigation Satellite System) operations within a ROS2 environment.

## Message Types

### 1. CorrectionMessage.msg
This message is used for communicating correction information. It facilitates the transmission of data required for GNSS correction, ensuring accurate positioning and navigation.

### 2. GnssSignalStatus.msg
This message provides detailed GNSS information from the antenna, including satellite and signal quality data. It is built on the standard `NavSatFix` message, extending its functionality to include additional status details.

### 3. NavSatInfo.msg
This message delivers information about satellite identity and signal quality. It is designed to work alongside other GNSS messages to provide comprehensive satellite data, supporting high-quality GNSS operations.

## Usage
To use these custom message types in your ROS2 project, include this package as a dependency in your `package.xml` and `CMakeLists.txt` files. You can then import and utilize these messages in your ROS2 nodes as needed.

## Installation
Clone this repository into your ROS2 workspace and build it using `colcon`.
