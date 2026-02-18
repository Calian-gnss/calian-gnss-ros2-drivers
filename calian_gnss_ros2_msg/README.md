# Calian GNSS ROS 2 Custom Messages

Custom message types used by the `calian_gnss_ros2` driver package.

## Message Types

### CorrectionMessage.msg
Raw RTCM correction payload with a timestamped header.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame |
| `message` | `uint8[]` | Raw RTCM byte payload |

### GnssSignalStatus.msg
Extended GPS fix built on `NavSatFix`, adding heading, accuracy, quality string, augmentation status, and per-constellation satellite counts.

### NavSatInfo.msg
Satellite constellation identity and count (used inside `GnssSignalStatus`).

| Field | Type | Description |
|-------|------|-------------|
| `gnss_id` | `uint8` | Constellation ID (0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 4=IMES, 5=QZSS, 6=GLONASS) |
| `count` | `uint8` | Satellite count |

### ReceiverHealthStatus.msg
Antenna receiver health status string.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame |
| `health` | `string` | Health status description |

## Usage

Add `calian_gnss_ros2_msg` as a dependency in your `package.xml`, then import:

```python
from calian_gnss_ros2_msg.msg import GnssSignalStatus, CorrectionMessage, ReceiverHealthStatus
```
