"""Remote RTCM corrections handler — receives RTCM from Ably real-time and publishes on ROS.

Used in the **static baseline** configuration where a Windows TruPrecision
application pushes RTCM messages to an Ably channel.
"""

import base64
import asyncio
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from ably import AblyRealtime
from ably.types.message import Message
from ably.types.connectionstate import (
    ConnectionEvent,
    ConnectionState,
    ConnectionStateChange,
)

from calian_gnss_ros2_msg.msg import CorrectionMessage
from calian_gnss_ros2.logging import setup_node_logging


class RemoteRtcmCorrectionsHandler(Node):
    """ROS 2 node that subscribes to an Ably channel and publishes RTCM corrections."""

    def __init__(self) -> None:
        super().__init__("remote_rtcm_corrections_handler")

        # ---- Parameters --------------------------------------------------
        self.declare_parameters(
            namespace="",
            parameters=[
                ("key", ""),
                ("channel", ""),
                ("frame_id", "rtcm"),
            ],
        )

        self._key = self.get_parameter("key").get_parameter_value().string_value
        self._channel_name = self.get_parameter("channel").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # ---- Logging (shared helper) -------------------------------------
        _, self.logger = setup_node_logging(self, "RemoteRTCM")

        # ---- Publisher ----------------------------------------------------
        self._rtcm_pub = self.create_publisher(
            CorrectionMessage, "rtcm_corrections", 50
        )

        # Run the Ably event loop in a background daemon thread so
        # rclpy.spin() on the main thread is never blocked.
        self._ably_thread = threading.Thread(
            target=self._run_ably_loop, name="ably_loop", daemon=True
        )
        self._ably_thread.start()

    # ------------------------------------------------------------------
    # Ably event loop (runs in background thread)
    # ------------------------------------------------------------------

    def _run_ably_loop(self) -> None:
        """Create a new asyncio event loop in this thread and run the Ably client."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._ably_process())
        except Exception as e:
            self.logger.error(f"Ably loop error: {e}")
        finally:
            loop.close()

    async def _ably_process(self) -> None:
        """Connect to Ably, subscribe, and keep pinging while ROS is alive."""
        self.logger.info("Connecting to Ably real-time...")
        self._ably = AblyRealtime(self._key, auto_connect=False)
        self._ably.connect()
        self._ably.connection.on(self._log_connection_event)
        await self._ably.connection.once_async(ConnectionState.CONNECTED)

        channel = self._ably.channels.get(self._channel_name)
        await channel.subscribe("RTCM Corrections", self._on_rtcm_message)

        while rclpy.ok():
            await channel.publish_message(
                Message(
                    name="Rover Ping",
                    data=self._ably.connection.connection_manager.connection_id,
                )
            )
            await asyncio.sleep(15)

        await self._ably.connection.once_async(ConnectionState.CLOSED)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _log_connection_event(self, event: ConnectionStateChange) -> None:
        """Log Ably connection state transitions."""
        self.logger.info(
            f"Ably connection state changed: {event.previous} -> {event.current}"
        )

    def _on_rtcm_message(self, message: Message) -> None:
        """Decode base-64 RTCM fragments and publish each as a CorrectionMessage."""
        try:
            for fragment in message.data:
                msg = CorrectionMessage(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id=self._frame_id,
                    ),
                    message=base64.b64decode(fragment),
                )
                self._rtcm_pub.publish(msg)
        except Exception as e:
            self.logger.error(f"Error processing Ably RTCM message: {e}")


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------


def main():
    rclpy.init()
    handler = RemoteRtcmCorrectionsHandler()
    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        handler.logger.error(f"Unexpected error: {e}")
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
