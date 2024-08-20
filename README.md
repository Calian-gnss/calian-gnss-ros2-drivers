# :world_map: Calian GNSS ROS 2 Driver

This repository contains the ROS 2 package for integrating Calian Smart GNSS Antennas with ROS 2-based systems.

# Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Requirements](#requirements)
- [Nodes](#nodes)
- [Parameters](#parameters)
- [PointPerfect Setup](#pointperfect-setup)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [Purchase Call](#purchase-call)
- [Reference](#reference)
- [License](#license)

# Introduction

Calian GNSS ROS 2 is a ROS 2 package that provides functionality for interfacing with Calian Smart GNSS antennas. It allows you to receive and process GNSS data within your ROS 2-based systems. This package provides ROS 2 nodes and utilities to interact with Calian Smart GNSS antennas, enabling accurate localization, navigation, and time synchronization for your robotic projects.

This package depends on the message package. 

# Features

- **Real-Time Positioning:** Achieve high-precision real-time positioning for your robots, essential for tasks like autonomous navigation and mapping
- **ROS 2 Integration:** Seamlessly integrate GNSS data into your ROS 2 ecosystem, making it easy to access and use GNSS information within your ROS 2-based applications.
- **Customizable Configuration:** Tailor the configuration to your specific requirements, with options for different GNSS constellations, frequencies, and data output formats.
- **Monitoring Tool:** Monitor your robot's location with a map view.

# Requirements

- [Calian GNSS Antenna](https://www.calian.com/advanced-technologies/gnss/technologies/gnss-smart-antennas/)
- [Ubuntu 22](https://releases.ubuntu.com/jammy/)
- [ROS2 (Humble)](https://docs.ros.org/en/humble/index.html)
- [Python](https://docs.python.org/3/)

  ```
    NOTE: Ensure to have clear skies to get good precision values
  ```
  
# Nodes
Calian GNSS ROS 2 package contains multiple nodes. One or more nodes are used at a time to use Calian GNSS Smart antenna in different configurations.
### :one: GPS
The GPS node is responsible for the configuration of Calian GNSS Smart antenna based on the mode of operation and publishing location data, antenna feedback to the respective topics. It takes the **Log parameters** and **Config parameters** (Described in the Parameters section) as the input parameters.
- Node arguments: Operating Mode (Accepted values are ****)
### :two: PointPerfect
The PointPerfect node is responsible to provide [PPP-RTK](https://www.u-blox.com/en/product/pointperfect) GNSS correction data to the Calian GNSS Smart antenna. It publishes the correction data to the **"corrections"** topic. It takes the  **Log parameters** and **Pointperfect parameters** as the input parameters.
### :three: Ntrip
The Ntrip node is responsible to provide RTCM GNSS correction data to the Calian GNSS Smart antenna. It publishes the correction data to the **"corrections"** topic. It takes the **Log parameters** and **Ntrip parameters** as the input parameters.
### :four: Remote Message Handler
The RTCM Remote Message Handler node is responsible for providing RTCM messages from Ably to the Calian GNSS Smart antenna. From our windows **TruPrecision** Application, A Base can be setup at a static location and RTCM corrections can be published onto the remote ably server. Those messages can be transmitted to the antennas using this node. It takes the **Config parameters** as the input parameters.
### :five: GPS Visualizer
The GPS Visualizer node visualizes the location data of the Calian GNSS Smart antenna published in the **gps** topic onto the map.
### :six: Unique Id Finder
The Unique Id Finder node extracts the unique id of the antenna from the SEC-UNIQID Ubx message. It assumes the antennas have the default baudrate (230400).

# Parameters

### :one: Log parameters
Path: `src/calian_gnss_ros2/params/logs.yaml`
1. **`save_logs (boolean)`:**
   - Flag to save logs. If true, all the logs will be saved to the logs folder.
2. **`log_level (integer)`:**
   - Logging level. Log level values are of ROS2 logging standards. Default is `Info`.
     - `(NotSet: 0, Debug: 10, Info: 20, Warn: 30, Error: 40, Critical: 50)`.
### :two: Config parameters:
Path: `src/calian_gnss_ros2/params/config.yaml`
1. **`unique_id (string)`:**
   - Unique Id of Calian Gnss receiver. Run the `Unique_id_finder` node (assumes default baudrate) to get the unique ids of all connected antennas.
2. **`baud_rate (integer)`:**
   - Baud rate for serial communication. Default value should be 230400.
3. **`use_corrections (boolean)`:**
   - Flag indicating whether PPP-RTK corrections should be used.
4. **`corrections_source (string)`:**
   - The type of augmentation source used. Accepted values are `PointPerfect, Ntrip`. Only used when the `use_corrections` is **true**.
### :three: Pointperfect parameters
Path: `src/calian_gnss_ros2/params/pointperfect.yaml`

1. **`config_path (string)`:**
   - Path to PPP-RTK configuration file. details to obtain config file is given in PointPerfect Setup section.
2. **`region (string)`:**
   - Region information. Accepted values are `us, eu, kr, au`.
### :four: Ntrip parameters
Path: `src/calian_gnss_ros2/params/ntrip.yaml`
1. **`hostname`:**
	-	Hostname or IP address of the NTRIP server to connect to.
2. **`port`:**
	- Port to connect to on the server. Default: `2101`
3. **`mountpoint`:**
	- Mountpoint to connect to on the NTRIP server.
4.  **`ntrip_version`:**
	- Value to use for the `Ntrip-Version` header in the initial HTTP request to the caster.
5. **`authenticate`:**
	- Whether to authenticate with the server, or send an unauthenticated request. If set to true, `username`, and `password` must be supplied.
6. **`username`:**
	-	Username to use when authenticating with the NTRIP server. Only used if `authenticate` is true
7. **`password`:**
	- Password to use when authenticating with the NTRIP server. Only used if `authenticate` is true
8. **`ssl`:** 
	- Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
9. **`cert`:**
	- If the NTRIP caster is configured to use cert based authentication, you can use this option to specify the client certificate
10. **`key`:**
	- If the NTRIP caster is configured to use cert based authentication, you can use this option to specify the private key
11. **`ca_cert`:**
	- If the NTRIP caster uses self signed certs, or you need to use a different CA chain, this option can be used to specify a CA file

# PointPerfect Setup

To achieve centimeter-level accuracy in real-time, PPP-RTK corrections are essential. These corrections can be obtained through the Pointperfect subscription service, accessible at **`https://www.u-blox.com/en/product/pointperfect`**. Follow the steps below to acquire and configure the necessary files:
- Visit the website to subscribe to the Pointperfect service.
- Once subscribed, navigate to the "Credentials" tab under the "Location Thing Details" section on the website.
- Download the ucenter configuration file provided and rename it to **`ucenter-config.json`**.

- Create a new folder named **`pointperfect_files`** at the following directory: **`humble_ws/src/calian_gnss_ros2/pointperfect_files/`**.

- Place the **`ucenter-config.json`** file inside the newly created **`pointperfect_files`** folder.
- When you run the node, it will generate several files within the **`pointperfect_files`** folder, which are necessary for establishing a connection to the subscription service.

# Installation

To install Calian GNSS ROS2, follow these steps:

1. **ROS Workspace:** Move to your ROS2 workspace (example: mine is humble_ws)

   ```bash
    cd ~/humble_ws/src
    ```

2. **Clone the repository:**

    ```bash
    git clone git@github.com:Calian-gnss/calian-gnss-ros2-drivers.git
    ```
   
3. **Build the package using colcon:**

    ```bash
    cd ~/humble_ws
    colcon build
    ```

4. **source the setup file:**

    ```bash
    source install/setup.bash
    ```

5. Go to your project, and install requirements.txt for installing all the required libraries at once. (for exampe mine is under humble_ws/src/Calian_ros2)

    ```bash
    cd humble_ws/src/calian_gnss_ros2
    pip install -r requirements.txt
      ```

  ```diff
  + IMP NOTE: Source your package every time you make change or open a new terminal. 

  + Else you will see Error like <<Package 'calian_gnss_ros2' not found>> even if you have cloned it.

  ```

# Usage

The Calian GNSS ROS2 package provides flexibility in its configurations, and example launch files for different setups can be found in the **`launch`** folder (**`/src/calian_gnss_ros2/launch/`**). The package includes the **`gps_visualizer`** node, designed to run alongside the **`gps`** node, enabling the visualization of the published location data. Ensure to change the **`unique_id`** parameter in the launch files to the desired gnss receiver.

## :one: Basic configuration.

- To use corrections, Ensure the params file is changed accordingly.
- If PointPerfect needs to be used as correction source, The steps mentioned in the PointPerfect Setup sections needs to be performed. If Ntrip needs to be used as correction source, The ntrip parameters needs to be changed to supply ntrip configuration.
- Set the parameters in the params file (`src/calian_gnss_ros2/params/`):
  - **`use_corrections`** to True if the corrections service needs to be used.
  - **`corrections_source` to either `Ntrip` or `PointPerfect`
- Build the workspace using **`colcon build`** and source the setup file with **`source install/setup.bash`**. Repeat this step for any changes in the launch file.
- Launch the nodes using the following command:
   ```
   ros2 launch calian_gnss_ros2 disabled.launch.py
   ```

- Upon execution, the **`Calian_gps`** node starts in disabled configuration, publishing location data to the **`gps`** topic.
- The visualizer node is also initiated, and you can view the mapped location data at **http://localhost:8080**.

- **Note**: You can modify the default port number (8080) of the visualizer node by adjusting the **`port`** parameter in the launch file.

## :two: Moving Baseline configuration:

For the Moving Baseline configuration, which involves two Calian antennas (one base and one rover), and only antennas with Zed-f9p chips acting as the base:

- Follow similar steps as the RTK disabled configuration and make necessary changes to the params file.
- Build and source the terminal.
- Launch the nodes with the command:

   ```
   ros2 launch calian_gnss_ros2 moving_baseline.launch.py
   ```
- Upon execution, the Calian_gps nodes start in Base and Rover modes in moving baseline configuration, publishing location data to the **`gps`** topic and extended information like heading, quality and accuracies to **`gps_extended`** topics.
- Access the location data at **http://localhost:8080**.
- Ensure to have clear skies to get good precision values.

## :three: Static Baseline configuration:

For the Static Baseline configuration, which involves one Calian antenna setup using [TruPrecision](https://tallysman.com/downloads/TruPrecision.zip) as base at a known location and one or more devices/robots with one antenna acting as rovers connected to the Base.

- Make sure to use the same key used for the TruPrecision (Prefilled do not change unless if you have a separate source). Change the channel name to the one given in TruPrecision application.
- Build and source the terminal.
- Launch the nodes with the command:
   ```
   ros2 launch calian_gnss_ros2 static_baseline.launch.py
   ```
- Upon execution, The remote rtcm corrections handler and Calian_gps nodes start in Rover mode in static baseline configuration, publishing location data to the **`gps`** topic and extended information like heading, quality and accuracies to **`gps_extended`** topics.
- Access the location data at **http://localhost:8080**.
- Ensure to have clear skies to get good precision values.
To view active topics use command
```bash
ros2 topic list
```

# Contributing

Contributions to Calian ROS2 are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

# Purchase Call

For inquiries or to purchase our Calian's Antenna, please contact us at [Calian](gnss.sales@Calian.com). We are excited to assist you and provide further information about our offerings.

# Reference

For more information about the Calian ROS2 driver please refer to [GitHub](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/)

# License

This project is licensed under the terms of the [MIT License](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/blob/main/LICENSE)
