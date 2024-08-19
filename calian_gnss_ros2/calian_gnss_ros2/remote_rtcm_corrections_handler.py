import base64
import rclpy
import asyncio
from ably import AblyRealtime
from ably.types.message import Message
from ably.types.connectionstate import (
    ConnectionEvent,
    ConnectionState,
    ConnectionStateChange,
)
from rclpy.node import Node
from pyrtcm import RTCMReader
from calian_gnss_ros2_msg.msg import RtcmMessage
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger


class RemoteRtcmCorrectionsHandler(Node):
    def __init__(self) -> None:
        super().__init__("remote_rtcm_corrections_handler")

        # region Parameters declaration
        self.declare_parameter("key", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("save_logs", False)
        self.declare_parameter("log_level", LoggingLevel.Info)
        # endregion

        # region Parameters Initialization
        self.key = self.get_parameter("key").get_parameter_value().string_value
        self.channel = self.get_parameter("channel").get_parameter_value().string_value
        self.save_logs = (
            self.get_parameter("save_logs").get_parameter_value().bool_value
        )
        self.log_level: LoggingLevel = LoggingLevel(
            self.get_parameter("log_level").get_parameter_value().integer_value
        )
        # endregion
        internal_logger = Logger(self.get_logger())
        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger("remote_rtcm_corrections_handler")
        # Publisher to publish RTCM corrections to Rover
        self.rtcm_publisher = self.create_publisher(RtcmMessage, "rtcm_corrections", 50)
        asyncio.run(self.__process())
        pass

    pass

    async def __process(self):
        self.logger.info("Connecting to real-time...")
        await self._connect_to_ably()
        self.__channel = self.ably.channels.get(self.channel)
        await self.__channel.subscribe(
            "RTCM Corrections", self.__process_incoming_messages
        )
        while rclpy.ok():
            # await self.ably.connection.ping()
            await self.__channel.publish_message(
                Message(
                    name="Rover Ping",
                    data=self.ably.connection.connection_manager.connection_id,
                )
            )
            await asyncio.sleep(15)
        await self.ably.connection.once_async(ConnectionState.CLOSED)

    async def _connect_to_ably(self):
        # connects automatically after initialization. no need to manually connect to realtime.
        self.ably = AblyRealtime(self.key, auto_connect=False)
        self.ably.connect()
        self.ably.connection.on(self.log_events)
        await self.ably.connection.once_async(ConnectionState.CONNECTED)

    def log_events(self, args: ConnectionStateChange):
        self.logger.info(
            "connection state changed from " + args.previous + " to " + args.current
        )
        pass

    def __process_incoming_messages(self, message: Message):
        try:
            for msg in message.data:
                rtcmMessage = RTCMReader.parse(base64.b64decode(msg))
                rtcm = RtcmMessage()
                rtcm.identity = rtcmMessage.identity
                rtcm.payload = msg
                self.rtcm_publisher.publish(rtcm)
                self.logger.info("RTCM message with identity " + rtcmMessage.identity)
            pass
        except:
            self.logger.error(
                "Exception while processing received message. message skipped."
            )
            pass


def main():
    rclpy.init()
    rtcm_data_handler = RemoteRtcmCorrectionsHandler()
    try:
        rclpy.spin(rtcm_data_handler)
    except KeyboardInterrupt:
        pass
    rtcm_data_handler.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
