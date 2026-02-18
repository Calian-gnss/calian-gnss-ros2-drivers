"""Serial communication module for u-blox GNSS antennas.

Provides:
- ``Event``           - simple C#-style multicast delegate.
- ``SerialUtilities`` - helpers for port / unique-ID discovery.
- ``UbloxSerial``     - full serial lifecycle: config, read, write, status.
"""

from collections import UserList
import threading
import time
from typing import Literal, Callable

import serial
from serial.tools.list_ports import comports
import rclpy
from pyubx2 import ubxreader, UBXMessage, POLL
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage
import concurrent.futures

from calian_gnss_ros2_msg.msg import GnssSignalStatus, NavSatInfo
from sensor_msgs.msg import NavSatStatus
from calian_gnss_ros2.logging import SimplifiedLogger


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

UBX_STALENESS_SEC = 10
"""Discard cached UBX messages older than this many seconds."""

MAX_RETRY_COUNT = 3
"""Maximum retries for antenna config / poll operations."""

MM_TO_M = 0.001
"""Millimetre-to-metre scaling factor (used for accuracy & altitude)."""

CM_TO_M = 0.01
"""Centimetre-to-metre scaling factor (used for baseline length)."""

_QUALITY_STRINGS = {
    0: "No Fix",
    1: "Autonomous Gnss Fix",
    2: "Differential Gnss Fix",
    3: "PPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "Dead Reckoning Fix",
    7: "Manual",
    8: "Simulated",
}

_FIX_TYPE_STRINGS = {
    0: "NoFix",
    1: "DR",
    2: "2D",
    3: "3D",
    4: "GDR",
    5: "TF",
}


# ---------------------------------------------------------------------------
# Event
# ---------------------------------------------------------------------------


class Event:
    """Simple C#-style event: supports ``+=`` to subscribe, ``()`` to fire."""

    def __init__(self):
        self._handlers: list[Callable] = []

    def __iadd__(self, handler: Callable):
        self._handlers.append(handler)
        return self

    def __isub__(self, handler: Callable):
        self._handlers.remove(handler)
        return self

    def __call__(self, *args, **kwargs):
        for handler in self._handlers:
            handler(*args, **kwargs)


# ---------------------------------------------------------------------------
# SerialUtilities
# ---------------------------------------------------------------------------


class SerialUtilities:
    """Static helpers for discovering u-blox antennas on serial ports."""

    @staticmethod
    def extract_unique_id_of_port(standard_port: serial.Serial, timeout: int) -> str:
        """Poll SEC-UNIQID and return the hex ID, or ``""`` on failure."""
        deadline = time.time() + timeout
        reader = ubxreader.UBXReader(standard_port, protfilter=2)
        standard_port.write(UBXMessage("SEC", "SEC-UNIQID", POLL).serialize())
        while time.time() < deadline:
            try:
                (_raw_data, parsed_data) = reader.read()
                if isinstance(parsed_data, UBXMessage) and (
                    parsed_data.identity == "SEC-UNIQID"
                ):
                    return parsed_data.uniqueId.to_bytes(5, "little").hex(" ")
            except Exception:
                return ""
        return ""

    @staticmethod
    def get_port_from_unique_id(
        unique_id: str,
        baudrate: Literal[
            1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
        ],
    ) -> str:
        """Return the serial port whose antenna matches *unique_id*, or ``""``."""
        port_name = ""
        probe_baudrates = [38400, 230400]
        ubx: UBXMessage = UBXMessage.config_set(
            1, 0, [("CFG_UART1_BAUDRATE", baudrate)]
        )
        ports = comports()
        failed_ports: list[str] = []

        if not ports:
            return ""

        with concurrent.futures.ThreadPoolExecutor() as executor:
            for port in ports:
                if port.description.find("Standard") == -1 or port_name:
                    continue
                for baud in probe_baudrates:
                    standard_port = None
                    try:
                        standard_port = serial.Serial(port.device, baud)
                        future = executor.submit(
                            SerialUtilities.extract_unique_id_of_port,
                            standard_port=standard_port,
                            timeout=3,
                        )
                        uid = future.result(3)
                        if uid.upper() == unique_id.upper():
                            port_name = port.device
                            if baud != baudrate:
                                standard_port.write(ubx.serialize())
                                time.sleep(2)
                    except Exception:
                        failed_ports.append(port.device)
                    finally:
                        if standard_port is not None:
                            standard_port.close()
                        if port_name:
                            break

            # Retry failed ports (race conditions with port locking)
            for port_device in failed_ports:
                if port_name:
                    break
                standard_port = None
                try:
                    standard_port = serial.Serial(port_device, baudrate)
                    future = executor.submit(
                        SerialUtilities.extract_unique_id_of_port,
                        standard_port=standard_port,
                        timeout=3,
                    )
                    uid = future.result(3)
                    if uid.upper() == unique_id.upper():
                        port_name = port_device
                except Exception:
                    pass
                finally:
                    if standard_port is not None:
                        standard_port.close()

        return port_name


# ---------------------------------------------------------------------------
# UbloxSerial
# ---------------------------------------------------------------------------


class UbloxSerial:
    """Manages the serial connection, configuration, and data dispatch for a u-blox antenna.

    Parameters
    ----------
    unique_id : str
        Hex string identifying the target antenna (from SEC-UNIQID).
    baudrate : int
        Serial baud rate (e.g. 230400).
    rtk_mode : str
        One of ``"Disabled"``, ``"Heading_Base"``, ``"Rover"``.
    use_corrections : bool
        Whether RTCM / SPARTN corrections are expected.
    """

    def __init__(
        self,
        unique_id: str,
        baudrate: Literal[
            1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
        ],
        rtk_mode: Literal["Disabled", "Heading_Base", "Rover"],
        use_corrections: bool = False,
    ):
        self.logger = SimplifiedLogger(f"{rtk_mode}_Serial")

        # Event handlers for message types
        self.ublox_message_found: Event = Event()
        self.nmea_message_found: Event = Event()
        self.rtcm_message_found: Event = Event()

        self.baudrate = baudrate
        self.unique_id = unique_id
        self.port_name: str | None = None
        self.__rtk_mode = rtk_mode
        self.__status = GnssSignalStatus()
        self.__quality: int | None = None
        self.__use_corrections = use_corrections
        self.__config_status = False
        self.__recent_ubx_message: dict[str, tuple[float, UBXMessage]] = {}
        self.__service_constellations = 0
        self.__poll_messages: set[tuple[str, str]] = set()
        self._run_time = 0
        self.__is_antenna_healthy = False
        self.__port: serial.Serial | None = None
        self.__read_thread: threading.Thread | None = None

        self.__process = threading.Thread(
            target=self.__serial_process, name="serial_process_thread", daemon=True
        )
        self.__process.start()

    # ------------------------------------------------------------------
    # Main process loop
    # ------------------------------------------------------------------

    def __serial_process(self) -> None:
        """Background loop that keeps the serial connection alive and healthy."""
        while rclpy.ok():
            if not self.port_name:
                self.logger.info("Finding port...")
                self.port_name = SerialUtilities.get_port_from_unique_id(
                    self.unique_id, self.baudrate
                )
            elif self.__port is None or not self.__config_status:
                self.__setup_serial_port_and_reader(self.port_name, self.baudrate)
            elif not self.__port.is_open:
                self.__port.open()
            elif self.__read_thread is None or not self.__read_thread.is_alive():
                self.__read_thread = threading.Thread(
                    target=self.__receive_thread,
                    name=f"receive_thread_{self.__port.name}",
                    daemon=True,
                )
                self.__read_thread.start()
            else:
                mon_sys = self.get_recent_ubx_message("MON-SYS")
                if mon_sys is not None:
                    if self._run_time <= mon_sys.runTime:
                        self._run_time = mon_sys.runTime
                        self.__is_antenna_healthy = True
                    else:
                        self.logger.warn("Antenna rebooted. Reconfiguring the antenna")
                        self._run_time = 0
                        self.__is_antenna_healthy = False
                        self.config()
                else:
                    self.check_antenna_health()
            time.sleep(1)

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    @property
    def get_antenna_health_status(self) -> bool:
        """Return ``True`` if the antenna is considered healthy."""
        return self.__is_antenna_healthy

    def check_antenna_health(self) -> None:
        """Try to re-establish communication with the antenna after it stops responding."""
        ubx: UBXMessage = UBXMessage.config_set(
            1, 0, [("CFG_UART1_BAUDRATE", self.baudrate)]
        )
        probe_baudrates = [38400, 230400]
        self.__port.close()
        self.__config_status = False
        self.__port = None
        with concurrent.futures.ThreadPoolExecutor() as executor:
            for baud in probe_baudrates:
                standard_port = None
                try:
                    self.logger.debug(
                        f"Connecting to port {self.port_name} using baud: {baud}"
                    )
                    standard_port = serial.Serial(self.port_name, baud)
                    future = executor.submit(
                        SerialUtilities.extract_unique_id_of_port,
                        standard_port=standard_port,
                        timeout=3,
                    )
                    uid = future.result(3)
                    if uid:
                        standard_port.write(ubx.serialize())
                        break
                except Exception as e:
                    self.logger.warn(
                        f"Cannot communicate to port {self.port_name} "
                        f"using baud: {baud} — {e}"
                    )
                finally:
                    if standard_port is not None:
                        standard_port.close()

    # ------------------------------------------------------------------
    # Serial port setup
    # ------------------------------------------------------------------

    def __setup_serial_port_and_reader(
        self,
        port_name: str,
        baudrate: int,
    ) -> None:
        """Open the serial port, start the receive thread, and configure the antenna."""
        try:
            self.__port = serial.Serial(port_name, baudrate)
            # protfilter=7 → UBX + NMEA + RTCM
            self.__ubr = ubxreader.UBXReader(self.__port, protfilter=7, validate=0)

            self.__read_thread = threading.Thread(
                target=self.__receive_thread,
                name=f"receive_thread_{port_name}",
                daemon=True,
            )
            self.__read_thread.start()
            self.config()
            self.__service_constellations = self.__get_service_constellations()
        except serial.SerialException as se:
            self.logger.error(str(se))
            self.__port = None
        except Exception as e:
            self.logger.error(
                f"Exception while setting up the Serial Module: {e}"
            )

    def open(self) -> None:
        """Open the serial port if it is not already open."""
        if self.__port is not None and not self.__port.is_open:
            self.__port.open()

    def close(self) -> None:
        """Close the serial port if it is currently open."""
        if self.__port is not None and self.__port.is_open:
            self.__port.close()

    # ------------------------------------------------------------------
    # Receive thread & message handlers
    # ------------------------------------------------------------------

    def __receive_thread(self) -> None:
        """Continuously read from the serial port and dispatch messages."""
        while rclpy.ok():
            if self.__port is None or not self.__port.is_open:
                self.logger.warn("Port is not configured/open.")
                break
            try:
                (_raw_data, parsed_data) = self.__ubr.read()
                if isinstance(parsed_data, NMEAMessage):
                    self.__nmea_message_received(parsed_data)
                elif isinstance(parsed_data, UBXMessage):
                    self.__ublox_message_received(parsed_data)
                elif isinstance(parsed_data, RTCMMessage):
                    self.__rtcm_message_received(parsed_data)
            except Exception as e:
                self.logger.warn(
                    f"Port is open but unable to read from port. Error: {e}"
                )
                break

    def __ublox_message_received(self, message: UBXMessage) -> None:
        """Cache and dispatch a UBX message."""
        self.logger.debug(f" <- [Ubx:{message.identity}] : {message}")
        # Only cache RXM-COR when corrections are actually used
        if message.identity == "RXM-COR" and message.msgUsed != 2:
            pass  # skip stale RXM-COR
        else:
            self.__recent_ubx_message[message.identity] = (time.time(), message)
        self.ublox_message_found(message)

    def __nmea_message_received(self, message: NMEAMessage) -> None:
        """Parse lat/lon from RMC/GGA and dispatch the NMEA message."""
        self.logger.debug(f" <- [Nmea:{message.identity}] : {message}")
        if message.identity in ("GNRMC", "GNGGA"):
            self.__status.latitude = (
                round(float(message.lat), 6) if message.lat else self.__status.latitude
            )
            # BUG-FIX: was incorrectly falling back to self.__status.latitude
            self.__status.longitude = (
                round(float(message.lon), 6)
                if message.lon
                else self.__status.longitude
            )
            if message.identity == "GNGGA":
                self.__quality = message.quality
        self.nmea_message_found(message)

    def __rtcm_message_received(self, message: RTCMMessage) -> None:
        """Dispatch an RTCM message."""
        self.logger.debug(f" <- [Rtcm:{message.identity}] : {message}")
        self.rtcm_message_found(message)

    # ------------------------------------------------------------------
    # Send / poll
    # ------------------------------------------------------------------

    def send(self, data: bytes) -> None:
        """Write raw bytes to the serial port."""
        try:
            if self.__port is not None and self.__port.is_open:
                self.logger.debug(f" -> {data.hex(' ')}")
                self.__port.write(data)
            else:
                self.logger.warn("Port is not configured/open.")
        except Exception as e:
            self.logger.error(f"Exception while writing to port: {e}")

    def poll(self) -> None:
        """Send all registered poll messages to the antenna."""
        if self.__port is None:
            self.logger.warn("Port is not configured/open.")
            return
        self.logger.debug(" -> Polling Messages")
        for class_name, msg_name in self.__poll_messages:
            ubx = UBXMessage(class_name, msg_name, POLL)
            self.send(ubx.serialize())

    def poll_once(self, class_name: str, msg_name: str) -> UBXMessage | None:
        """Poll a single message, retrying up to ``MAX_RETRY_COUNT`` times."""
        ubx = UBXMessage(class_name, msg_name, POLL)
        for _ in range(MAX_RETRY_COUNT):
            self.send(ubx.serialize())
            time.sleep(0.5)
            result = self.get_recent_ubx_message(msg_name)
            if result is not None:
                return result
        return None

    def add_to_poll(self, class_name: str, msg_name: str) -> None:
        """Register a message to be polled periodically."""
        self.__poll_messages.add((class_name, msg_name))

    def remove_from_poll(self, class_name: str, msg_name: str) -> None:
        """Remove a message from the periodic poll set."""
        self.__poll_messages.discard((class_name, msg_name))

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def config(self) -> bool:
        """Apply the antenna configuration. Retries up to ``MAX_RETRY_COUNT`` times."""
        config_data = self.__get_config_set(
            mode_of_operation=self.__rtk_mode,
            use_corrections=self.__use_corrections,
        )
        ubx: UBXMessage = UBXMessage.config_set(1, 0, config_data)

        for _ in range(MAX_RETRY_COUNT):
            self.send(ubx.serialize())
            time.sleep(0.5)
            if self.get_recent_ubx_message("ACK-ACK") is not None:
                self.__config_status = True
                self.logger.info("Configuration successful.")
                return True

        self.__config_status = False
        self.logger.error("Configuration failed.")
        return False

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def get_status(self) -> GnssSignalStatus:
        """Build and return the current ``GnssSignalStatus`` from cached UBX data."""
        try:
            if (
                not self.__status.latitude
                or not self.__status.longitude
            ):
                self.__status.valid_fix = False
                return self.__status

            self.__status.valid_fix = True

            nav_hpposllh = self.get_recent_ubx_message("NAV-HPPOSLLH")
            nav_pvt = self.get_recent_ubx_message("NAV-PVT")
            nav_relposned = self.get_recent_ubx_message("NAV-RELPOSNED")
            nav_hpposecef = self.get_recent_ubx_message("NAV-HPPOSECEF")
            nav_sig = self.get_recent_ubx_message("NAV-SIG")
            nav_cov = self.get_recent_ubx_message("NAV-COV")

            # Augmentation status (deduplicated — Rover implies corrections)
            augmentations_used = False
            if self.__use_corrections or self.__rtk_mode == "Rover":
                rxm_cor = self.get_recent_ubx_message("RXM-COR")
                augmentations_used = bool(rxm_cor and rxm_cor.msgUsed == 2)
            self.__status.augmentations_used = augmentations_used

            # Heading / baseline length
            if (
                nav_relposned is not None
                and nav_relposned.relPosValid == 1
                and nav_relposned.relPosHeadingValid == 1
            ):
                self.__status.heading = round(nav_relposned.relPosHeading, 2)
                self.__status.length = round(
                    float(nav_relposned.relPosLength * CM_TO_M), 2
                )

            # Accuracy / altitude
            if (
                nav_hpposecef is not None
                and nav_hpposecef.invalidEcef == 0
                and nav_hpposllh is not None
                and nav_hpposllh.invalidLlh == 0
            ):
                self.__status.altitude = round(
                    float(nav_hpposllh.height * MM_TO_M), 4
                )
                self.__status.accuracy_2d = round(
                    float(nav_hpposllh.hAcc * MM_TO_M), 4
                )
                self.__status.accuracy_3d = round(
                    float(nav_hpposecef.pAcc * MM_TO_M), 4
                )

            # Position covariance
            if nav_cov is not None and nav_cov.posCovValid == 1:
                variance = [
                    round(float(nav_cov.posCovNN), 4),
                    round(float(nav_cov.posCovNE), 4),
                    round(float(nav_cov.posCovND), 4),
                    round(float(nav_cov.posCovNE), 4),
                    round(float(nav_cov.posCovEE), 4),
                    round(float(nav_cov.posCovED), 4),
                    round(float(nav_cov.posCovND), 4),
                    round(float(nav_cov.posCovED), 4),
                    round(float(nav_cov.posCovDD), 4),
                ]
                self.__status.position_covariance = UserList(variance)
                self.__status.position_covariance_type = (
                    self.__status.COVARIANCE_TYPE_KNOWN
                )

            # NavSatStatus
            if nav_pvt is not None:
                status = NavSatStatus()
                if nav_pvt.gnssFixOk == 1:
                    if nav_pvt.carrSoln != 0:
                        status.status = NavSatStatus.STATUS_GBAS_FIX
                    else:
                        status.status = NavSatStatus.STATUS_FIX
                else:
                    status.status = NavSatStatus.STATUS_NO_FIX
                status.service = self.__service_constellations
                self.__status.status = status
                self.__status.quality = self.__get_quality_string(nav_pvt)

            # Satellite information
            if nav_sig is not None:
                nav_sat_info_list: list[NavSatInfo] = []
                no_of_satellites = 0
                gnss_sat_count: dict[int, int] = {}
                for attr in dir(nav_sig):
                    if attr.startswith("gnss"):
                        cno_attr = f"cno_{attr.split('_')[1]}"
                        if getattr(nav_sig, cno_attr, 0) > 0:
                            gnss_id = getattr(nav_sig, attr)
                            gnss_sat_count[gnss_id] = gnss_sat_count.get(gnss_id, 0) + 1
                            no_of_satellites += 1

                for gnss_id, count in gnss_sat_count.items():
                    info = NavSatInfo()
                    info.gnss_id = gnss_id
                    info.count = count
                    nav_sat_info_list.append(info)

                self.__status.no_of_satellites = no_of_satellites
                self.__status.satellite_information = UserList(nav_sat_info_list)

        except Exception as ex:
            self.logger.error(f"Error building status: {ex}")

        return self.__status

    # ------------------------------------------------------------------
    # UBX message cache
    # ------------------------------------------------------------------

    def get_recent_ubx_message(self, msg_id: str) -> UBXMessage | None:
        """Return a cached UBX message only if it arrived within ``UBX_STALENESS_SEC``."""
        try:
            timestamp, ubx_message = self.__recent_ubx_message[msg_id]
            if time.time() - timestamp < UBX_STALENESS_SEC:
                return ubx_message
            return None
        except KeyError:
            return None

    # ------------------------------------------------------------------
    # Quality string (dictionary-driven)
    # ------------------------------------------------------------------

    def __get_quality_string(self, nav_pvt: UBXMessage) -> str:
        """Build a human-readable quality string from NMEA quality + NAV-PVT."""
        if nav_pvt is None:
            return ""

        quality = _QUALITY_STRINGS.get(self.__quality, "Unknown")
        fix_type = _FIX_TYPE_STRINGS.get(nav_pvt.fixType, "?")
        quality += f"({fix_type}"

        if nav_pvt.gnssFixOk == 1:
            quality += "/DGNSS"
        if nav_pvt.carrSoln == 1:
            quality += "/Float"
        elif nav_pvt.carrSoln == 2:
            quality += "/Fixed"

        quality += ")"
        return quality

    # ------------------------------------------------------------------
    # Antenna configuration set
    # ------------------------------------------------------------------

    @staticmethod
    def __get_config_set(
        mode_of_operation: Literal["Disabled", "Heading_Base", "Rover"],
        use_corrections: bool = False,
    ) -> list:
        """Return the list of (key, value) pairs for ``UBXMessage.config_set``."""
        config_data = [
            ("CFG_UART1INPROT_NMEA", 1),
            ("CFG_UART1INPROT_UBX", 1),
            ("CFG_UART1OUTPROT_NMEA", 1),
            ("CFG_UART1OUTPROT_UBX", 1),
            ("CFG_MSGOUT_UBX_MON_SYS_UART1", 1),
            ("CFG_NAVSPG_DYNMODEL", 0),
            ("CFG_MSGOUT_UBX_NAV_SIG_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_COV_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1),
        ]

        if mode_of_operation == "Heading_Base":
            config_data.extend([
                ("CFG_UART1INPROT_RTCM3X", 0),
                ("CFG_UART1OUTPROT_RTCM3X", 1),
                # Common RTCM message types for Base
                ("CFG_MSGOUT_RTCM_3X_TYPE1074_UART1", 0x1),
                ("CFG_MSGOUT_RTCM_3X_TYPE1084_UART1", 0x1),
                ("CFG_MSGOUT_RTCM_3X_TYPE1124_UART1", 0x1),
                ("CFG_MSGOUT_RTCM_3X_TYPE1094_UART1", 0x1),
                ("CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1", 0x1),
                ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 0x1),
                ("CFG_TMODE_MODE", 0x0),
            ])
        elif mode_of_operation == "Rover":
            config_data.extend([
                ("CFG_UART1INPROT_RTCM3X", 1),
                ("CFG_MSGOUT_UBX_RXM_RTCM_UART1", 0x1),
                ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
                ("CFG_MSGOUT_UBX_RXM_COR_UART1", 1),
            ])

        if use_corrections:
            config_data.extend([
                ("CFG_UART1INPROT_RTCM3X", 1),
                ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
                ("CFG_MSGOUT_UBX_RXM_COR_UART1", 1),
            ])

        return config_data

    # ------------------------------------------------------------------
    # Service constellations
    # ------------------------------------------------------------------

    def __get_service_constellations(self) -> int:
        """Query the antenna for enabled GNSS constellations and return a bitmask."""
        gnss_config_poll: UBXMessage = UBXMessage.config_poll(
            0,
            0,
            [
                "CFG_SIGNAL_GPS_ENA",
                "CFG_SIGNAL_GLO_ENA",
                "CFG_SIGNAL_BDS_ENA",
                "CFG_SIGNAL_GAL_ENA",
            ],
        )

        gnss_config_msg = None
        for _ in range(MAX_RETRY_COUNT):
            self.send(gnss_config_poll.serialize())
            time.sleep(0.5)
            gnss_config_msg = self.get_recent_ubx_message("CFG-VALGET")
            if gnss_config_msg is not None:
                break

        if gnss_config_msg is None:
            self.logger.error("Antenna is not responding.")
            return 0

        constellations = 0
        if gnss_config_msg.CFG_SIGNAL_GPS_ENA == 1:
            constellations |= NavSatStatus.SERVICE_GPS
        if gnss_config_msg.CFG_SIGNAL_GLO_ENA == 1:
            constellations |= NavSatStatus.SERVICE_GLONASS
        if gnss_config_msg.CFG_SIGNAL_BDS_ENA == 1:
            constellations |= NavSatStatus.SERVICE_COMPASS
        if gnss_config_msg.CFG_SIGNAL_GAL_ENA == 1:
            constellations |= NavSatStatus.SERVICE_GALILEO
        return constellations
