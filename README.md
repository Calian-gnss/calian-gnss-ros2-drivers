# 🗺️ Calian GNSS ROS 2 Driver

ROS 2 driver for [Calian Smart GNSS Antennas](https://www.calian.com/advanced-technologies/gnss/technologies/gnss-smart-antennas/). Supports single-antenna, moving-baseline, and static-baseline configurations with real-time RTK corrections via NTRIP or Ably.

---

## Table of Contents

- [Features](#features)
- [Supported Hardware](#supported-hardware)
- [Requirements](#requirements)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [1 — Discover Antenna IDs](#1--discover-antenna-ids)
  - [2 — Disabled (Single Antenna)](#2--disabled-single-antenna)
  - [3 — Moving Baseline (Two Antennas)](#3--moving-baseline-two-antennas)
  - [4 — Static Baseline (TruPrecision + Rover)](#4--static-baseline-truprecision--rover)
- [ROS Topics & Messages](#ros-topics--messages)
  - [Published Topics](#published-topics)
  - [Custom Messages](#custom-messages)
- [Nodes](#nodes)
- [Parameters Reference](#parameters-reference)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Features

| Feature | Description |
|---------|-------------|
| **Real-Time Positioning** | High-precision RTK positioning via NTRIP or Ably RTCM streams |
| **Three Configurations** | Disabled (single antenna), Moving Baseline (dual antenna heading), Static Baseline (TruPrecision base + rover) |
| **ROS 2 Integration** | Publishes `sensor_msgs/NavSatFix`, custom `GnssSignalStatus`, `ReceiverHealthStatus` |
| **Map Visualizer** | Built-in HTTP server renders a Folium map of live GPS positions |
| **Antenna Health Monitoring** | Periodically publishes receiver health and satellite constellation status |

---

## Supported Hardware

- Calian Smart GNSS Antennas with **u-blox ZED-F9P** chipset (required for moving-baseline base mode)
- Any Calian antenna supported by **pyubx2** for rover / disabled modes

---

## Requirements

| Requirement | Version |
|-------------|---------|
| **Ubuntu** | 24.04 (Noble) |
| **ROS 2** | Jazzy Jalisco |
| **Python** | 3.12+ |
| **Hardware** | Calian GNSS Smart Antenna connected via USB |

> ⚠️ **Clear-sky conditions** are required for accurate RTK positioning.

---

## Repository Structure

```
calian-gnss-ros2-drivers/
├── calian_gnss_ros2/             # Python ROS 2 package (main driver)
│   ├── calian_gnss_ros2/         # Source modules
│   │   ├── gps.py                # Main GPS node
│   │   ├── serial_module.py      # Serial communication & UBX parsing
│   │   ├── ntrip_module.py       # NTRIP client node
│   │   ├── remote_rtcm_corrections_handler.py  # Ably RTCM handler
│   │   ├── gps_visualizer.py     # HTTP map visualizer node
│   │   ├── unique_id_finder.py   # Antenna ID scanner utility
│   │   └── logging.py            # Shared logging helpers
│   ├── launch/                   # Launch files
│   │   ├── launch_common.py      # Shared launch helpers
│   │   ├── disabled.launch.py    # Single-antenna launch
│   │   ├── moving_baseline.launch.py   # Dual-antenna launch
│   │   └── static_baseline.launch.py   # TruPrecision + rover launch
│   ├── params/                   # YAML parameter files
│   │   ├── config.yaml           # Antenna unique IDs & correction flags
│   │   ├── ntrip.yaml            # NTRIP caster credentials
│   │   └── logs.yaml             # Logging level & file-save toggle
│   ├── setup.py
│   ├── package.xml
│   └── requirements.txt
├── calian_gnss_ros2_msg/         # Custom message definitions (C++)
│   ├── msg/
│   │   ├── GnssSignalStatus.msg
│   │   ├── NavSatInfo.msg
│   │   ├── CorrectionMessage.msg
│   │   └── ReceiverHealthStatus.msg
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md
```

---

## Installation

### 1. Create / navigate to your ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

### 2. Clone the repository

```bash
git clone https://github.com/Calian-gnss/calian-gnss-ros2-drivers.git
```

### 3. Install Python dependencies

```bash
cd calian-gnss-ros2-drivers/calian_gnss_ros2
pip install -r requirements.txt
```

### 4. Build the workspace

```bash
cd ~/ros2_ws
colcon build
```

### 5. Source the workspace

```bash
source install/setup.bash
```

> 📌 **Important:** Source the workspace in every new terminal, or add the line above to your `~/.bashrc`.

---

## Configuration

All parameter files live in `calian_gnss_ros2/params/`.

### `config.yaml` — Antenna settings

Each section maps to a **node name** used in the launch files:

```yaml
calian_gnss:
  base:
    ros__parameters:
      use_corrections: true
      baud_rate: 230400
      unique_id: "<YOUR_BASE_UNIQUE_ID>"
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `unique_id` | string | Hex ID printed by the `unique_id_finder` node |
| `baud_rate` | int | Serial baud rate (default `230400`) |
| `use_corrections` | bool | Enable corrections |

### `ntrip.yaml` — NTRIP caster credentials

```yaml
calian_gnss:
  ntrip_client:
    ros__parameters:
      hostname: "<YOUR_NTRIP_HOST>"
      port: 2101
      mountpoint: "<YOUR_MOUNTPOINT>"
      username: "<YOUR_USERNAME>"
      password: "<YOUR_PASSWORD>"
```

### `logs.yaml` — Logging configuration

```yaml
calian_gnss:
  base:
    ros__parameters:
      save_logs: false    # Write logs to file
      log_level: 20       # 0=NotSet, 10=Debug, 20=Info, 30=Warn, 40=Error, 50=Critical
```

---

## Usage

### 1 — Discover Antenna IDs

Before running any configuration, find the unique ID of each connected antenna:

```bash
source install/setup.bash
ros2 run calian_gnss_ros2 unique_id_finder
```

Copy the printed IDs into `params/config.yaml`, rebuild, and source again.

---

### 2 — Disabled (Single Antenna)

A single antenna publishing GPS data with optional NTRIP corrections.

```bash
ros2 launch calian_gnss_ros2 disabled.launch.py
```

**What starts:**

| Node | Purpose |
|------|---------|
| `gps_publisher` | GPS node in **Disabled** mode |
| `ntrip_client` | NTRIP correction stream |
| `gps_visualizer` | Map at [http://localhost:8080](http://localhost:8080) |

**Published topics:**

```
/calian_gnss/gps_publisher/gps               # sensor_msgs/NavSatFix
/calian_gnss/gps_publisher/gps_extended       # calian_gnss_ros2_msg/GnssSignalStatus
/calian_gnss/gps_publisher/antenna_health     # calian_gnss_ros2_msg/ReceiverHealthStatus
```

---

### 3 — Moving Baseline (Two Antennas)

Two antennas: one base, one rover. The base generates RTCM corrections; the rover consumes them for centimetre-level heading.

```bash
ros2 launch calian_gnss_ros2 moving_baseline.launch.py
```

**What starts:**

| Node | Purpose |
|------|---------|
| `base` | GPS node in **Heading_Base** mode (ZED-F9P required) |
| `rover` | GPS node in **Rover** mode |
| `ntrip_client` | NTRIP correction stream |
| `gps_visualizer` | Map at [http://localhost:8080](http://localhost:8080) |

**Published topics:**

```
/calian_gnss/base/gps_extended         # calian_gnss_ros2_msg/GnssSignalStatus
/calian_gnss/base/antenna_health       # calian_gnss_ros2_msg/ReceiverHealthStatus
/calian_gnss/base/rtcm_corrections     # calian_gnss_ros2_msg/CorrectionMessage (remapped → rtcm_topic)
/calian_gnss/rover/gps                 # sensor_msgs/NavSatFix
/calian_gnss/rover/gps_extended        # calian_gnss_ros2_msg/GnssSignalStatus
/calian_gnss/rover/antenna_health      # calian_gnss_ros2_msg/ReceiverHealthStatus
```

---

### 4 — Static Baseline (TruPrecision + Rover)

A Windows TruPrecision base at a known location pushes RTCM to Ably; the rover receives it.

1. Set up a base using the [TruPrecision](https://tallysman.com/downloads/TruPrecision.zip) application.
2. Note the Ably channel name and use the same API key.
3. Update `params/config.yaml` with the key and channel under `rtcm_handler`.

```bash
ros2 launch calian_gnss_ros2 static_baseline.launch.py
```

**What starts:**

| Node | Purpose |
|------|---------|
| `rtcm_handler` | Ably → ROS RTCM bridge |
| `rover` | GPS node in **Rover** mode |
| `gps_visualizer` | Map at [http://localhost:8080](http://localhost:8080) |

---

## ROS Topics & Messages

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `~gps` | `sensor_msgs/NavSatFix` | Latitude, longitude, altitude, covariance |
| `~gps_extended` | `calian_gnss_ros2_msg/GnssSignalStatus` | Full fix info: heading, accuracy, quality, satellite breakdown |
| `~antenna_health` | `calian_gnss_ros2_msg/ReceiverHealthStatus` | Antenna health string |
| `~rtcm_corrections` | `calian_gnss_ros2_msg/CorrectionMessage` | Raw RTCM byte stream (base → rover) |
| `corrections` | `calian_gnss_ros2_msg/CorrectionMessage` | NTRIP-sourced RTCM data |

### Custom Messages

#### `GnssSignalStatus.msg`

Extended GPS fix with heading, accuracy, quality string, and per-constellation satellite counts.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame |
| `status` | `sensor_msgs/NavSatStatus` | Fix status |
| `latitude` | `float64` | Degrees (positive = north) |
| `longitude` | `float64` | Degrees (positive = east) |
| `altitude` | `float64` | Metres above WGS-84 |
| `position_covariance` | `float64[9]` | ENU covariance (m²) |
| `position_covariance_type` | `uint8` | 0=unknown, 1=approx, 2=diagonal, 3=known |
| `accuracy_2d` | `float64` | Horizontal accuracy (m) |
| `accuracy_3d` | `float64` | 3-D accuracy (m) |
| `heading` | `float64` | Heading relative to base (°) |
| `length` | `float64` | Baseline length (m) |
| `quality` | `string` | Human-readable fix quality |
| `augmentations_used` | `bool` | RTCM / SPARTN corrections active |
| `valid_fix` | `bool` | Fix validity flag |
| `no_of_satellites` | `uint16` | Total satellites in solution |
| `satellite_information` | `NavSatInfo[]` | Per-constellation breakdown |

#### `NavSatInfo.msg`

| Field | Type | Description |
|-------|------|-------------|
| `gnss_id` | `uint8` | Constellation (0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 4=IMES, 5=QZSS, 6=GLONASS) |
| `count` | `uint8` | Satellite count for this constellation |

#### `CorrectionMessage.msg`

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame |
| `message` | `uint8[]` | Raw RTCM byte payload |

#### `ReceiverHealthStatus.msg`

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame |
| `health` | `string` | Receiver health status string |

---

## Nodes

| Node | Executable | Description |
|------|-----------|-------------|
| GPS | `calian_gnss_gps` | Main node — configures antenna, publishes fix data |
| NTRIP Client | `ntrip_client` | Connects to NTRIP caster, publishes RTCM corrections |
| Remote RTCM Handler | `remote_rtcm_corrections_handler` | Receives RTCM from Ably, publishes to ROS |
| GPS Visualizer | `calian_gnss_gps_visualizer` | Serves Folium map of live positions |
| Unique ID Finder | `unique_id_finder` | One-shot scanner that prints antenna IDs |

---

## Parameters Reference

### GPS Node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `unique_id` | string | `""` | Antenna unique ID (hex string) |
| `baud_rate` | int | `230400` | Serial baud rate |
| `use_corrections` | bool | `true` | Enable SPARTN / RTK corrections |
| `save_logs` | bool | `false` | Persist logs to disk |
| `log_level` | int | `20` | Logging verbosity (10–50) |

### NTRIP Client

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hostname` | string | `127.0.0.1` | NTRIP caster hostname |
| `port` | int | `2101` | Caster port |
| `mountpoint` | string | `mount` | Mountpoint name |
| `username` | string | `""` | Auth username |
| `password` | string | `""` | Auth password |
| `ntrip_version` | string | `""` | Protocol version header |
| `ssl` | bool | `false` | Use TLS |
| `cert` / `key` / `ca_cert` | string | `""` | Cert-based auth paths |

### Remote RTCM Handler

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `key` | string | `""` | Ably API key |
| `channel` | string | `""` | Ably channel name |
| `frame_id` | string | `rtcm` | Header frame_id |

### GPS Visualizer

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | int | `8080` | HTTP server port |

---

## Architecture

```
┌────────────────┐        ┌──────────────┐
│  NTRIP Caster  │───────▶│ ntrip_client │──▶ /corrections
└────────────────┘        └──────────────┘
                                              │
┌────────────────┐        ┌──────────────┐    ▼
│ USB Antenna(s) │◀──────▶│   gps (base) │──▶ /rtcm_corrections ──▶ rtcm_topic
└────────────────┘        │              │──▶ /gps, /gps_extended, /antenna_health
                          └──────────────┘
                                              │
                          ┌──────────────┐    │
                          │  gps (rover) │◀───┘ (subscribes to rtcm_topic)
                          │              │──▶ /gps, /gps_extended, /antenna_health
                          └──────────────┘
                                              │
                          ┌──────────────┐    │
                          │ gps_visualizer│◀──┘ (subscribes to /gps)
                          │  :8080       │
                          └──────────────┘
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Package 'calian_gnss_ros2' not found` | Workspace not sourced | Run `source install/setup.bash` |
| `No ports connected` | Antenna not plugged in or no USB permissions | Check USB cable; add user to `dialout` group: `sudo usermod -aG dialout $USER` |
| Unique ID returns error at all baud rates | Wrong cable or antenna not powered | Try a different USB port; ensure antenna has power |
| NTRIP: `401 Unauthorized` | Bad credentials | Verify `username`, `password`, and `mountpoint` in `ntrip.yaml` |
| NTRIP: `SOURCETABLE 200 OK` | Invalid mountpoint | Check available mountpoints with your provider |
| GPS quality shows `"No Fix"` | Obstructed sky | Move to an open-sky location |
| Visualizer page blank | No fix data yet | Wait for satellite lock; check `/gps` topic with `ros2 topic echo` |
| `_frame_id` AttributeError in static baseline | Old code version | Rebuild the workspace after pulling latest |

---

## Viewing Live Data

```bash
# List all active topics
ros2 topic list

# Echo GPS fix
ros2 topic echo /calian_gnss/gps_publisher/gps

# Echo extended info (heading, quality, satellites)
ros2 topic echo /calian_gnss/gps_publisher/gps_extended

# Echo antenna health
ros2 topic echo /calian_gnss/gps_publisher/antenna_health

# View map
xdg-open http://localhost:8080
```

---

## Contributing

Contributions are welcome! Please open an issue or submit a pull request on [GitHub](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/).

---

## Purchase

For inquiries or to purchase Calian GNSS antennas, contact [gnss.sales@calian.com](mailto:gnss.sales@calian.com).

---

## License

This project is licensed under the [MIT License](https://github.com/Calian-gnss/calian-gnss-ros2-drivers/blob/main/LICENSE).
