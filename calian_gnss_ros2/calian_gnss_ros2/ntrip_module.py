"""NTRIP client node — connects to an NTRIP caster and publishes RTCM corrections.

Contains:
- ``NTRIPClient`` - low-level socket wrapper with SSL, reconnect, and NMEA send.
- ``NtripModule``  - ROS 2 node that bridges the client to the ``corrections`` topic.
"""

import threading
import time
import base64
import socket
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence
import ssl

from calian_gnss_ros2_msg.msg import CorrectionMessage
from calian_gnss_ros2.logging import setup_node_logging


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_CHUNK_SIZE = 1024
_SOURCETABLE_RESPONSES = ["SOURCETABLE 200 OK"]
_SUCCESS_RESPONSES = ["ICY 200 OK", "HTTP/1.0 200 OK", "HTTP/1.1 200 OK"]
_UNAUTHORIZED_RESPONSES = ["401"]


# ---------------------------------------------------------------------------
# NTRIPClient
# ---------------------------------------------------------------------------


class NTRIPClient:
    """Low-level NTRIP v1/v2 socket client with reconnect and NMEA relay."""

    DEFAULT_RECONNECT_ATTEMPT_MAX = 10
    DEFAULT_RECONNECT_ATTEMPT_WAIT_SECONDS = 5
    DEFAULT_RTCM_TIMEOUT_SECONDS = 4

    def __init__(self, host, port, mountpoint, ntrip_version, username, password, logger):
        self._logger = logger
        self._logerr = self._logger.error
        self._logwarn = self._logger.warn
        self._loginfo = self._logger.info
        self._logdebug = self._logger.debug

        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._ntrip_version = ntrip_version

        if username is not None and password is not None:
            self._basic_credentials = base64.b64encode(
                f"{username}:{password}".encode("utf-8")
            ).decode("utf-8")
        else:
            self._basic_credentials = None

        self._raw_socket = None
        self._server_socket = None

        # SSL configuration (public)
        self.ssl = False
        self.cert = None
        self.key = None
        self.ca_cert = None

        # State
        self._shutdown = False
        self._connected = False

        # Reconnect bookkeeping
        self._reconnect_attempt_count = 0
        self._nmea_send_failed_count = 0
        self._nmea_send_failed_max = 5
        self._read_zero_bytes_count = 0
        self._read_zero_bytes_max = 5
        self._first_rtcm_received = False
        self._recv_rtcm_last_packet_timestamp = 0

        # Public reconnect tunables
        self.reconnect_attempt_max = self.DEFAULT_RECONNECT_ATTEMPT_MAX
        self.reconnect_attempt_wait_seconds = self.DEFAULT_RECONNECT_ATTEMPT_WAIT_SECONDS
        self.rtcm_timeout_seconds = self.DEFAULT_RTCM_TIMEOUT_SECONDS

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open a socket to the NTRIP caster and perform the HTTP handshake."""
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.settimeout(5)

        try:
            self._server_socket.connect((self._host, self._port))
        except Exception as e:
            self._logerr(f"Unable to connect to http://{self._host}:{self._port}")
            self._logerr(f"Exception: {e}")
            return False

        # SSL wrapping
        if self.ssl:
            ssl_ctx = ssl.create_default_context()
            if self.cert:
                ssl_ctx.load_cert_chain(self.cert, self.key)
            if self.ca_cert:
                ssl_ctx.load_verify_locations(self.ca_cert)
            self._raw_socket = self._server_socket
            self._server_socket = ssl_ctx.wrap_socket(
                self._raw_socket, server_hostname=self._host
            )

        # Send HTTP request
        try:
            self._server_socket.send(self._form_request())
        except Exception as e:
            self._logerr(f"Unable to send request to http://{self._host}:{self._port}")
            self._logerr(f"Exception: {e}")
            return False

        # Read response
        response = ""
        try:
            response = self._server_socket.recv(_CHUNK_SIZE).decode("ISO-8859-1")
        except Exception as e:
            self._logerr(f"Unable to read response from http://{self._host}:{self._port}")
            self._logerr(f"Exception: {e}")
            return False

        # Check for success
        if any(s in response for s in _SUCCESS_RESPONSES):
            self._connected = True

        # Diagnose known errors
        known_error = False
        if any(s in response for s in _SOURCETABLE_RESPONSES):
            self._logwarn(
                "Received sourcetable response — the mountpoint is probably invalid."
            )
            known_error = True
        elif any(s in response for s in _UNAUTHORIZED_RESPONSES):
            self._logwarn(
                "Received 401 Unauthorized — check username, password, and mountpoint."
            )
            known_error = True
        elif not self._connected and not self._ntrip_version:
            self._logwarn(
                "Unknown error — consider specifying the NTRIP version in your config."
            )
            known_error = True

        if known_error or not self._connected:
            self._logerr(
                f"Invalid response from http://{self._host}:{self._port}/{self._mountpoint}"
            )
            self._logerr(f"Response: {response}")
            return False

        self._loginfo(
            f"Connected to http://{self._host}:{self._port}/{self._mountpoint}"
        )
        return True

    def disconnect(self) -> None:
        """Gracefully shut down and close the socket."""
        self._connected = False
        for sock in (self._server_socket, self._raw_socket):
            try:
                if sock:
                    sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                if sock:
                    sock.close()
            except Exception:
                pass

    def reconnect(self) -> None:
        """Disconnect and reconnect, retrying up to ``reconnect_attempt_max`` times."""
        if not self._connected:
            self._logdebug("Reconnect called while already disconnected, ignoring")
            return

        while not self._shutdown:
            self._reconnect_attempt_count += 1
            self.disconnect()
            if self.connect():
                self._reconnect_attempt_count = 0
                break
            if self._reconnect_attempt_count >= self.reconnect_attempt_max:
                self._reconnect_attempt_count = 0
                raise ConnectionError(
                    f"Reconnect failed after {self.reconnect_attempt_max} attempts"
                )
            self._logerr(
                f"Reconnect failed. Retrying in {self.reconnect_attempt_wait_seconds}s"
            )
            time.sleep(self.reconnect_attempt_wait_seconds)

    # ------------------------------------------------------------------
    # NMEA send
    # ------------------------------------------------------------------

    def send_nmea(self, sentence: str) -> None:
        """Send an NMEA sentence to the NTRIP caster (for VRS)."""
        if not self._connected:
            self._logwarn("NMEA sent before client was connected, discarding")
            return

        if sentence[-4:] == "\\r\\n":
            sentence = sentence[:-4] + "\r\n"
        elif sentence[-2:] != "\r\n":
            sentence += "\r\n"

        try:
            self._server_socket.send(sentence.encode("utf-8"))
        except Exception as e:
            self._logwarn(f"Unable to send NMEA: {e}")
            self._nmea_send_failed_count += 1
            if self._nmea_send_failed_count >= self._nmea_send_failed_max:
                self._logwarn(
                    f"NMEA send failed {self._nmea_send_failed_count} times, reconnecting"
                )
                self.reconnect()
                self._nmea_send_failed_count = 0
                self.send_nmea(sentence)

    # ------------------------------------------------------------------
    # RTCM receive
    # ------------------------------------------------------------------

    def recv_rtcm(self) -> bytes | None:
        """Return available RTCM data from the socket, or ``None``."""
        if not self._connected:
            self._logwarn("RTCM requested before connected, returning None")
            return None

        # Timeout check
        if (
            self._first_rtcm_received
            and time.time() - self.rtcm_timeout_seconds
            >= self._recv_rtcm_last_packet_timestamp
        ):
            self._logerr(
                f"RTCM data not received for {self.rtcm_timeout_seconds}s, reconnecting"
            )
            self.reconnect()
            self._first_rtcm_received = False

        # Check for available data
        read_sockets, _, _ = select.select([self._server_socket], [], [], 0)
        if not read_sockets:
            return None

        # Read all available data
        data = b""
        while True:
            try:
                chunk = self._server_socket.recv(_CHUNK_SIZE)
                data += chunk
                if len(chunk) < _CHUNK_SIZE:
                    break
            except Exception:
                if not self._socket_is_open():
                    self._logerr("Socket closed. Reconnecting")
                    self.reconnect()
                    return None
                break

        if len(data) == 0:
            self._read_zero_bytes_count += 1
            if self._read_zero_bytes_count >= self._read_zero_bytes_max:
                self._logwarn(
                    f"Received 0 bytes {self._read_zero_bytes_count} times, reconnecting"
                )
                self.reconnect()
                self._read_zero_bytes_count = 0
                return None
        else:
            self._recv_rtcm_last_packet_timestamp = time.time()
            self._first_rtcm_received = True

        return data

    def shutdown(self) -> None:
        """Signal the client to shut down and disconnect."""
        self._shutdown = True
        self.disconnect()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _form_request(self) -> bytes:
        """Build the HTTP GET request for the NTRIP caster."""
        if self._ntrip_version:
            request = (
                f"GET /{self._mountpoint} HTTP/1.0\r\n"
                f"Ntrip-Version: {self._ntrip_version}\r\n"
                "User-Agent: NTRIP ntrip_client_ros\r\n"
            )
        else:
            request = (
                f"GET /{self._mountpoint} HTTP/1.0\r\n"
                "User-Agent: NTRIP ntrip_client_ros\r\n"
            )
        if self._basic_credentials is not None:
            request += f"Authorization: Basic {self._basic_credentials}\r\n"
        request += "\r\n"
        return request.encode("utf-8")

    def _socket_is_open(self) -> bool:
        """Check whether the socket is still open without consuming data."""
        try:
            data = self._server_socket.recv(
                _CHUNK_SIZE, socket.MSG_DONTWAIT | socket.MSG_PEEK
            )
            return len(data) != 0
        except BlockingIOError:
            return True
        except ConnectionResetError:
            self._logwarn("Connection reset by peer")
            return False
        except socket.timeout:
            return True
        except Exception as e:
            self._logwarn(f"Socket appears closed: {e}")
            return False


# ---------------------------------------------------------------------------
# NtripModule (ROS 2 node)
# ---------------------------------------------------------------------------


class NtripModule(Node):
    """ROS 2 node that runs an NTRIPClient and publishes RTCM corrections."""

    def __init__(self) -> None:
        super().__init__("ntrip_module")

        # ---- Parameters -------------------------------------------------
        self.declare_parameters(
            namespace="",
            parameters=[
                ("hostname", "127.0.0.1"),
                ("port", 2101),
                ("mountpoint", "mount"),
                ("username", ""),
                ("password", ""),
                ("frame_id", "ntrip"),
                ("ntrip_version", ""),
                ("authenticate", True),
                ("ssl", False),
                ("cert", ""),
                ("key", ""),
                ("ca_cert", ""),
            ],
        )

        host = self.get_parameter("hostname").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        mountpoint = self.get_parameter("mountpoint").get_parameter_value().string_value
        username = self.get_parameter("username").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        ntrip_version = self.get_parameter("ntrip_version").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # ---- Logging (shared helper) ------------------------------------
        _, self.logger = setup_node_logging(self, "NtripClient")

        # ---- NTRIP client ------------------------------------------------
        self._client = NTRIPClient(
            host=host,
            port=port,
            ntrip_version=ntrip_version,
            mountpoint=mountpoint,
            username=username,
            password=password,
            logger=self.logger
        )
        self._client.ssl = self.get_parameter("ssl").get_parameter_value().bool_value
        self._client.cert = self.get_parameter("cert").get_parameter_value().string_value or None
        self._client.key = self.get_parameter("key").get_parameter_value().string_value or None
        self._client.ca_cert = self.get_parameter("ca_cert").get_parameter_value().string_value or None

        # ---- ROS interfaces -----------------------------------------------
        self._nmea_sub = self.create_subscription(
            Sentence, "nmea", self._subscribe_nmea, 10
        )
        self._correction_pub = self.create_publisher(
            CorrectionMessage, "corrections", 100
        )

        self._initialized = False
        threading.Thread(
            target=self._process_loop, name="ntrip_process_thread", daemon=True
        ).start()

    # ------------------------------------------------------------------

    def _process_loop(self) -> None:
        """Background loop: connect then continuously receive RTCM."""
        while rclpy.ok():
            if not self._initialized:
                if self._client.connect():
                    self._initialized = True
                else:
                    self.logger.error("Unable to connect to NTRIP server")
            else:
                data = self._client.recv_rtcm()
                if data is not None:
                    self._correction_pub.publish(
                        CorrectionMessage(
                            header=Header(
                                stamp=self.get_clock().now().to_msg(),
                                frame_id=self._frame_id,
                            ),
                            message=data,
                        )
                    )
            time.sleep(0.5)

    def _subscribe_nmea(self, nmea: Sentence) -> None:
        """Forward the received NMEA sentence to the NTRIP caster."""
        self._client.send_nmea(nmea.sentence)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------


def main():
    rclpy.init()
    ntrip = NtripModule()
    try:
        rclpy.spin(ntrip)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        ntrip.logger.error(f"Unexpected error: {e}")
    finally:
        ntrip.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
