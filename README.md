# Calian GNSS ROS2 Drivers

This repository contains ROS2 drivers for integrating Calian GNSS receivers. The drivers are split into two main projects:

1. **calian_gnss_ros2**: This project includes ROS2 nodes that configure the GNSS receivers and publish GNSS information onto specific topics.

2. **calian_gnss_ros2_msg**: This project defines custom message types used by the `calian_gnss_ros2` project to publish GNSS information.

## Repository Structure

- **calian_gnss_ros2**: Contains ROS2 nodes for GNSS receiver integration.
  - [README](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/calian_gnss_ros2/README.md)
- **calian_gnss_ros2_msg**: Contains custom message definitions for GNSS data.
  - [README](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/calian_gnss_ros2_msg/README.md)

## Getting Started

### Prerequisites

- ROS2 Humble
- A compatible Calian GNSS receiver
- Build tools for ROS2 (`colcon`, `rosdep`, etc.)

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/Calian-gnss/calian-gnss-ros2-drivers.git
   cd calian-gnss-ros2-drivers
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Build the packages:
   ```bash
   colcon build
   ```

4. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Usage

To use the ROS2 nodes for configuring the GNSS receivers and publishing data:

```bash
ros2 launch calian_gnss_ros2 disabled.launch.py
```

Make sure to adjust the parameters in the launch file according to your receiver's configuration.

## Documentation

- For more detailed instructions on configuring and using the `calian_gnss_ros2` project, refer to the [README](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/calian_gnss_ros2/README.md) in the `calian_gnss_ros2` directory.
- For information on the custom message types, see the [README](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/calian_gnss_ros2_msg/README.md) in the `calian_gnss_ros2_msg` directory.

## Contributing

We welcome contributions! Please see our [CONTRIBUTING.md](CONTRIBUTING.md) for more details.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
