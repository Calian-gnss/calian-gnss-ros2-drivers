"""GPS node — bridges u-blox serial data to ROS 2 topics.

Operates in three modes:
- **Disabled** - single antenna, no RTK heading.
- **Heading_Base** - base antenna in a moving-baseline pair.
- **Rover** - rover antenna receiving RTCM corrections.
"""

import sys
import threading
from typing import Literal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from pynmeagps import NMEAMessage
from pyrtcm import RTCMReader
from nmea_msgs.msg import Sentence

from calian_gnss_ros2_msg.msg import (
    CorrectionMessage,
    GnssSignalStatus,
    ReceiverHealthStatus,
)
from calian_gnss_ros2.logging import setup_node_logging
from calian_gnss_ros2.serial_module import UbloxSerial


class Gps(Node):
    """Main GPS node that reads from a u-blox antenna and publishes ROS 2 topics."""

    def __init__(
        self, mode: Literal["Disabled", "Heading_Base", "Rover"] = "Disabled"
    ) -> None:
        super().__init__("calian_gnss_gps")

        self.mode: Literal["Disabled", "Heading_Base", "Rover"] = mode

        # ---- Parameters -------------------------------------------------
        self.declare_parameter("unique_id", "")
        self.declare_parameter("baud_rate", 230400)
        self.declare_parameter("use_corrections", True)
        self.declare_parameter("frame_id", "gps")

        self.unique_id: str = (
            self.get_parameter("unique_id").get_parameter_value().string_value
        )
        self.baud_rate: int = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )
        self.use_corrections: bool = (
            self.get_parameter("use_corrections").get_parameter_value().bool_value
        )
        self._frame_id: str = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        # ---- Logging (shared helper) ------------------------------------
        _, self.logger = setup_node_logging(self, f"{self.mode}_GPS")

        # ---- Serial module -----------------------------------------------
        self.ser = UbloxSerial(
            self.unique_id,
            self.baud_rate,
            self.mode,
            self.use_corrections,
        )

        # ---- Lock for thread-safe RTCM pool access ----------------------
        self._rtcm_lock = threading.Lock()

        # ---- Mode-specific publishers / subscribers ----------------------
        if self.mode == "Heading_Base":
            self.rtcm_publisher = self.create_publisher(
                CorrectionMessage, "rtcm_corrections", 100
            )
            self.base_status_publisher = self.create_publisher(
                GnssSignalStatus, "base_gps_extended", 50
            )
            self.ser.rtcm_message_found += self.handle_rtcm_message
            self.rtcm_msg_pool: list[CorrectionMessage] = []
            self.create_timer(1, self.publish_pooled_rtcm)

        elif self.mode in ("Rover", "Disabled"):
            self.gps_publisher = self.create_publisher(NavSatFix, "gps", 50)
            self.gps_status_publisher = self.create_publisher(
                GnssSignalStatus, "gps_extended", 50
            )
            if self.mode == "Rover":
                self.rtcm_subscriber = self.create_subscription(
                    CorrectionMessage,
                    "rtcm_corrections",
                    self.handle_rtcm_correction_from_base,
                    100,
                )

        # ---- Common publishers / timers ----------------------------------
        self.health_publisher = self.create_publisher(
            ReceiverHealthStatus, "health", 50
        )
        self.create_timer(1, self.get_health_status)
        self.create_timer(1, self.get_status)

        # ---- Corrections (NTRIP / SPARTN) --------------------------------
        if self.use_corrections:
            self.create_subscription(
                CorrectionMessage, "corrections", self.handle_correction_message, 100
            )
            self.nmea_publisher = self.create_publisher(Sentence, "nmea", 100)
            self.create_timer(1, self.send_nmea_message)
            self._recent_nmea_gga: str = ""
            self.ser.nmea_message_found += self.handle_nmea_message

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _make_header(self) -> Header:
        """Create a stamped Header with the node's frame_id."""
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame_id)

    # ------------------------------------------------------------------
    # Correction handling
    # ------------------------------------------------------------------

    def handle_correction_message(self, message: CorrectionMessage) -> None:
        """Forward correction data (NTRIP / SPARTN) to the antenna over serial."""
        self.logger.debug(
            f"Sending correction message: {message.message.tobytes().hex(' ')}"
        )
        self.ser.send(message.message.tobytes())

    def handle_nmea_message(self, nmea_message: NMEAMessage) -> None:
        """Cache the latest GGA sentence for NTRIP."""
        if nmea_message.identity == "GNGGA" and self.use_corrections:
            self._recent_nmea_gga = nmea_message.serialize().decode("utf-8")

    # ------------------------------------------------------------------
    # RTCM (base → rover)
    # ------------------------------------------------------------------

    def handle_rtcm_message(self, rtcm_message) -> None:
        """Called on the base: pool outgoing RTCM for periodic publish."""
        msg = CorrectionMessage(
            header=self._make_header(),
            message=rtcm_message.serialize(),
        )
        with self._rtcm_lock:
            self.rtcm_msg_pool.append(msg)

    def handle_rtcm_correction_from_base(self, message: CorrectionMessage) -> None:
        """Called on the rover: forward RTCM from the base topic to the antenna."""
        parsed = RTCMReader.parse(message.message)
        self.ser.send(parsed.serialize())
        self.logger.debug(f"Received RTCM message with identity: {parsed.identity}")

    def publish_pooled_rtcm(self) -> None:
        """Publish all pooled RTCM messages and clear the pool (thread-safe)."""
        with self._rtcm_lock:
            pool_snapshot = list(self.rtcm_msg_pool)
            self.rtcm_msg_pool.clear()
        for msg in pool_snapshot:
            self.rtcm_publisher.publish(msg)
            self.logger.debug(f"Published RTCM message -> {msg.message}")

    # ------------------------------------------------------------------
    # Health & status
    # ------------------------------------------------------------------

    def get_health_status(self) -> None:
        """Publish antenna health status (Good / Bad)."""
        msg = ReceiverHealthStatus(
            header=self._make_header(),
            health="Good" if self.ser.get_antenna_health_status else "Bad",
        )
        self.health_publisher.publish(msg)

    def get_status(self) -> None:
        """Poll the serial module for signal status and publish NavSatFix + extended."""
        status: GnssSignalStatus = self.ser.get_status()
        status.header = self._make_header()

        if not status.valid_fix:
            return

        if self.mode in ("Rover", "Disabled"):
            msg = NavSatFix(
                header=status.header,
                latitude=status.latitude,
                longitude=status.longitude,
                altitude=status.altitude,
                position_covariance=status.position_covariance,
                position_covariance_type=status.position_covariance_type,
                status=status.status,
            )
            self.gps_publisher.publish(msg)
            self.gps_status_publisher.publish(status)
            self.logger.debug(
                f"Published GPS data - Lat: {status.latitude:.6f}, "
                f"Lon: {status.longitude:.6f}"
            )
        else:
            self.base_status_publisher.publish(status)

    def send_nmea_message(self) -> None:
        """Publish the most recent GGA sentence for NTRIP."""
        self.nmea_publisher.publish(
            Sentence(header=self._make_header(), sentence=self._recent_nmea_gga)
        )


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------


def main():
    rclpy.init()
    args = rclpy.utilities.remove_ros_args(sys.argv)
    gps = Gps(mode=args[1])
    try:
        rclpy.spin(gps)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        gps.logger.error(f"Unexpected error: {e}")
    finally:
        gps.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
