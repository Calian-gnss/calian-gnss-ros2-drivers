from typing import Literal
import rclpy
import sys
import base64
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from calian_gnss_ros2.pointperfect_module import PointPerfectModule
from calian_gnss_ros2.serial_module import UbloxSerial
from pynmeagps import NMEAMessage
from pyrtcm import RTCMReader
from calian_gnss_ros2_msg.msg import GnssSignalStatus, CorrectionMessage
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger
from std_srvs.srv import Empty
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence


class Gps(Node):
    def __init__(
        self, mode: Literal["Disabled", "Heading_Base", "Rover"] = "Disabled"
    ) -> None:
        super().__init__("calian_gnss_gps")

        internal_logger = Logger(self.get_logger())
        # region Parameters declaration
        self.declare_parameter("unique_id", "")
        self.declare_parameter("baud_rate", 230400)
        # Parameter {config_path} needs to be present if this is True.
        self.declare_parameter("use_corrections", True)
        self.declare_parameter("corrections_source", "PointPerfect")
        self.declare_parameter("save_logs", False)
        self.declare_parameter("log_level", LoggingLevel.Info)
        self.declare_parameter("frame_id", "gps")
        # endregion

        # region Parameters Initialization
        self.unique_id = (
            self.get_parameter("unique_id").get_parameter_value().string_value
        )
        self.baud_rate = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )
        self.use_corrections = (
            self.get_parameter("use_corrections").get_parameter_value().bool_value
        )
        self.corrections_source: Literal["PointPerfect", "Ntrip"] = (
            self.get_parameter("corrections_source").get_parameter_value().string_value
        )
        self.save_logs = (
            self.get_parameter("save_logs").get_parameter_value().bool_value
        )
        self.log_level: LoggingLevel = LoggingLevel(
            self.get_parameter("log_level").get_parameter_value().integer_value
        )
        self.mode: Literal["Disabled", "Heading_Base", "Rover"] = mode
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        # endregion

        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger(self.mode + "_GPS")
        self.ser = UbloxSerial(
            self.unique_id,
            self.baud_rate,
            self.mode,
            self.use_corrections,
            self.corrections_source,
        )
        # region Conditional attachments to events based on rover/base
        if self.mode == "Heading_Base":
            # Publisher to publish RTCM corrections to Rover
            self.rtcm_publisher = self.create_publisher(
                CorrectionMessage, "rtcm_corrections", 100
            )
            self.base_status_publisher = self.create_publisher(
                GnssSignalStatus, "base_gps_extended", 50
            )
            self.ser.rtcm_message_found += self.handle_rtcm_message
            # Timer to send rtcm messages from pool.
            self.rtcm_msg_pool: list = []
            self.rtcm_publish_timer = self.create_timer(0.5, self.publish_pooled_rtcm)
            pass
        elif self.mode == "Rover":
            # Subscriber for receiving RTCM corrections from base.
            self.rtcm_subscriber = self.create_subscription(
                CorrectionMessage, "rtcm_corrections", self.handle_rtcm_message, 100
            )
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.gps_publisher = self.create_publisher(NavSatFix, "gps", 50)
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.gps_status_publisher = self.create_publisher(
                GnssSignalStatus, "gps_extended", 50
            )
            pass
        elif self.mode == "Disabled":
            self.gps_publisher = self.create_publisher(NavSatFix, "gps", 50)
            self.gps_status_publisher = self.create_publisher(
                GnssSignalStatus, "gps_extended", 50
            )
            pass

        # Timer to poll status messages from base/rover for every sec.
        self.status_timer = self.create_timer(1, self.get_status)
        # Establishing PointPerfect connection only if it's enabled. Required parameters needs to be sent.
        if self.use_corrections:
            self.on_correction_message = self.create_subscription(
                CorrectionMessage, "corrections", self.handle_correction_message, 100
            )
            if self.corrections_source == "PointPerfect":
                self._pp_client = self.create_client(Empty, "restart")
                self.reconnect_timer = self.create_timer(
                    5, self.__reconnect_pointperfect_if_needed
                )
            else:
                self.nmea_publisher = self.create_publisher(Sentence, "nmea", 100)
                self.rtcm_publish_timer = self.create_timer(1, self.send_nmea_message)
                self._recent_nmea_gga = ""
                self.ser.nmea_message_found += self.handle_nmea_message

        # endregion
        pass

    """
        Handles the correction messages received from PointPerfect MQTT connection.

        The decoding of correction messages and applying them to the location is done by the antenna itself.
        Need to make sure we send entire message to the antenna without a delay, Since the correction messages are time dependent.
    """

    def handle_correction_message(self, message) -> None:
        self.logger.debug(
            message="Sending correction message: " + message.message.tobytes().hex(" ")
        )
        self.ser.send(message.message.tobytes())
        pass

    """
        Handles the received NMEAMessage.

        Location information is taken from RMC and GGA messages and published to the {topic_name} topic via NavSatFix message
    """

    def handle_nmea_message(self, nmeaMessage: NMEAMessage) -> None:
        if (
            nmeaMessage.identity == "GNGGA"
            and self.use_corrections
            and self.corrections_source == "Ntrip"
        ):
            self._recent_nmea_gga = nmeaMessage.serialize().decode("utf-8")
        pass

    """
        Handles received RTCM message. 
        
        If the antenna is base, The received RTCM message is from antenna and is published to the {rtcm_topic_name} topic via ByteMultiArray message.
        If the antenna is rover, The received RTCM message is from {rtcm_topic_name} topic via ByteMultiArray message and is sent to the antenna through serial port.
    """

    def handle_rtcm_message(self, rtcmMessage) -> None:
        if self.mode == "Heading_Base":
            msg = CorrectionMessage(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self._frame_id,
                ),
                message=rtcmMessage.serialize(),
            )
            self.rtcm_msg_pool.append(msg)
        else:
            rmg = RTCMReader.parse(rtcmMessage.message)
            self.ser.send(rmg.serialize())
            self.logger.debug("Received RTCM message with identity: " + rmg.identity)
        pass

    """
        gets the status of the signal and outputs into the topic with NavSatFix message
    """

    def get_status(self) -> None:
        status = self.ser.get_status()

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame_id)
        status.header = header

        if not status.valid_fix:
            return

        if self.mode == "Rover" or self.mode == "Disabled":
            msg = NavSatFix()
            msg.header = header
            msg.latitude = status.latitude
            msg.longitude = status.longitude
            msg.altitude = status.altitude
            msg.position_covariance = status.position_covariance
            msg.position_covariance_type = status.position_covariance_type
            msg.status = status.status
            self.gps_publisher.publish(msg)
            self.gps_status_publisher.publish(status)
            self.logger.debug(
                "Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}".format(
                    status.latitude, status.longitude
                )
            )
        else:
            self.base_status_publisher.publish(status)

    def send_nmea_message(self) -> None:
        self.nmea_publisher.publish(
            Sentence(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self._frame_id,
                ),
                sentence=self._recent_nmea_gga,
            )
        )

    def publish_pooled_rtcm(self) -> None:
        pooled_msgs = list(self.rtcm_msg_pool)
        self.rtcm_msg_pool.clear()
        for msg in pooled_msgs:
            self.rtcm_publisher.publish(msg)
            self.logger.debug("Published RTCM message -> " + str(msg.message))

    """
        Spartn keys are checked in the antenna. if no keys are present, pointperfect is reconnected to get a new pair of keys.
    """

    def __reconnect_pointperfect_if_needed(self):
        if self.use_corrections:
            sptn_key = self.ser.poll_once("RXM", "RXM-SPARTN-KEY")
            if (
                sptn_key is not None
                and sptn_key.numKeys == 0
                and self._pp_client.service_is_ready()
            ):
                self._pp_client.call_async(Empty.Request())
                pass


def main():
    rclpy.init()
    args = rclpy.utilities.remove_ros_args(sys.argv)
    gps = Gps(mode=args[1])
    try:
        rclpy.spin(gps)
    except KeyboardInterrupt:
        pass
    except:
        pass
    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
