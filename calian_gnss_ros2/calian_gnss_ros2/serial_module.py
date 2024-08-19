from collections import UserList
import threading
import time
from typing import Literal
import serial
from serial.tools.list_ports import comports
from events import Event
import rclpy
from pyubx2 import ubxreader, UBXMessage, POLL
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage
from calian_gnss_ros2_msg.msg import GnssSignalStatus
from calian_gnss_ros2_msg.msg import NavSatInfo
from sensor_msgs.msg import NavSatStatus
from calian_gnss_ros2.logging import SimplifiedLogger
import concurrent.futures


class SerialUtilities:
    """
    Extracts unique id of the given port.

    returns port if found with in timeout, Empty string otherwise
    """

    @staticmethod
    def extract_unique_id_of_port(standard_port: serial.Serial, timeout: int) -> str:
        current_time = time.time()
        reader = ubxreader.UBXReader(standard_port, protfilter=2)
        standard_port.write(UBXMessage("SEC", "SEC-UNIQID", POLL).serialize())
        while current_time + timeout >= time.time():
            try:
                (raw_data, parsed_data) = reader.read()
                if isinstance(parsed_data, UBXMessage) and (
                    parsed_data.identity == "SEC-UNIQID"
                ):
                    return parsed_data.uniqueId.to_bytes(5, "little").hex(" ")
            except Exception as e:
                return ""
        return ""

    """
        Finds out the serial port which has antenna connected with the given unique id
    """

    @staticmethod
    def get_port_from_unique_id(
        unique_id: str,
        baudrate: Literal[
            1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
        ],
    ) -> str:
        port_name = ""
        ports = comports()
        failed_ports = []
        if len(ports) != 0:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for port in ports:
                    if port.description.find("Standard") != -1 and not port_name:
                        try:
                            standard_port = serial.Serial(port.device, baudrate)
                            thread = executor.submit(
                                SerialUtilities.extract_unique_id_of_port,
                                standard_port=standard_port,
                                timeout=3,
                            )
                            unique_id_of_port = thread.result(3)
                            if unique_id_of_port.upper() == unique_id.upper():
                                port_name = port.device
                            pass
                        except:
                            failed_ports.append(port.device)
                            pass
                        finally:
                            standard_port.close()
                            standard_port = None
                            if port_name:
                                break
                            pass
                    pass

                # trying again for the failed port...need to do this because of race conditions.
                for port in failed_ports:
                    try:
                        standard_port = serial.Serial(port, baudrate)
                        thread = executor.submit(
                            SerialUtilities.extract_unique_id_of_port,
                            standard_port=standard_port,
                            timeout=3,
                        )
                        unique_id_of_port = thread.result(3)
                        if unique_id_of_port.upper() == unique_id.upper():
                            port_name = port
                        pass
                    except:
                        pass
                    finally:
                        standard_port.close()
                        standard_port = None
                        if port_name:
                            break
                        pass
        if not port_name:
            return ""
        else:
            return port_name


class UbloxSerial:
    """
    Serial module to handle read and write operations for the antenna.

    port_name: Bidirectional port through which node communicates with antenna.
    baudrate: The speed at which data is transmitted.
    logger: Logger must contain basic log functions.
    """

    def __init__(
        self,
        unique_id: str,
        baudrate: Literal[
            1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
        ],
        rtk_mode: Literal["Disabled", "Heading_Base", "Rover"],
        use_corrections: bool = False,
        corrections_source=Literal["PointPerfect", "Ntrip"],
    ):
        self.logger = SimplifiedLogger(rtk_mode + "_Serial")
        # Event handlers for Ublox, Nmea and RTCM messages.
        self.ublox_message_found: Event[UBXMessage] = Event()
        self.nmea_message_found: Event[NMEAMessage] = Event()
        self.rtcm_message_found: Event[RTCMMessage] = Event()

        self.baudrate = baudrate
        self.unique_id = unique_id
        self.port_name = None
        self.__rtk_mode = rtk_mode
        self.__status = GnssSignalStatus()
        self.__quality = None
        self.__use_corrections = use_corrections
        self.__corrections_source = corrections_source
        self.__config_status = False
        self.__recent_ubx_message = dict[str, (float, UBXMessage)]()
        self.__service_constellations = 0
        self.runTime = 0

        self.__process = threading.Thread(
            target=self.__serial_process, name="serial_process_thread", daemon=True
        )
        self.__process.start()
        self.__port = None

    """
        This is continuosly running loop to make sure everything is working fine in serial module.

        This will restart the receive thread if by any case it stops due to exception.
        This will also detect if the antenna is rebooted and does the configuration to the antenna again.
    """

    def __serial_process(self) -> None:
        while rclpy.ok():
            if not self.port_name:
                self.logger.info("Finding port..")
                self.port_name = SerialUtilities.get_port_from_unique_id(
                    self.unique_id, self.baudrate
                )
            elif self.__port is None or not self.__config_status:
                self.__setup_serial_port_and_reader(self.port_name, self.baudrate)
                pass
            elif not self.__port.is_open:
                self.__port.open()
                pass
            elif self.__read_thread is None or not self.__read_thread.is_alive():
                self.__read_thread = threading.Thread(
                    target=self.__receive_thread,
                    name="receive_thread_" + self.__port.name,
                    daemon=True,
                )
                self.__read_thread.start()
                pass
            else:
                monSys = self.get_recent_ubx_message("MON-SYS")
                if monSys is not None:
                    if self.runTime <= monSys.runTime:
                        self.runTime = monSys.runTime
                    else:
                        self.logger.warn("Antenna rebooted. Reconfiguring the antenna")
                        # self.__save_boot_times(self.runTime)
                        self.runTime = 0
                        self.config()
            time.sleep(1)

    """
        This method is responsible for setting up the serial port based on port_name and baudrate. It also starts a receive_thread specific to port_name serial port and configures a reader to parse the data
    """

    def __setup_serial_port_and_reader(
        self,
        port_name: str,
        baudrate: Literal[
            1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
        ],
    ) -> None:
        try:
            self.__port = serial.Serial(port_name, baudrate)
            # protfilter=7 gives out UBX, NMEA and RTCM messages.
            self.__ubr = ubxreader.UBXReader(self.__port, protfilter=7, validate=0)

            # Start a separate thread for reading and parsing serial stream.
            self.__read_thread = threading.Thread(
                target=self.__receive_thread,
                name="receive_thread_" + port_name,
                daemon=True,
            )
            self.__read_thread.start()
            # Configurations to antenna to work in a specified mode.
            self.config()
            self.__service_constellations = self.__get_service_constellations()
            pass
        except serial.SerialException as se:
            self.logger.error(se.strerror)
            self.__port = None
            pass
        except Exception:
            self.logger.error("Exception occured while setting up the Serial Module.")
        pass

    def open(self) -> None:
        if self.__port is not None and self.__port.is_open:
            pass
        else:
            self.__port.open()

    def close(self) -> None:
        if self.__port is not None and self.__port.is_open:
            self.__port.close()

    """
        Reads data from serial port and calls respective message handlers methods.
    """

    def __receive_thread(self) -> None:
        while rclpy.ok():
            if self.__port is not None and self.__port.is_open:
                try:
                    (raw_data, parsed_data) = self.__ubr.read()
                    if isinstance(parsed_data, NMEAMessage):
                        self.__nmea_message_received(parsed_data)
                    elif isinstance(parsed_data, UBXMessage):
                        self.__ublox_message_received(parsed_data)
                    elif isinstance(parsed_data, RTCMMessage):
                        self.__rtcm_message_received(parsed_data)
                except Exception as e:
                    self.logger.warn(
                        "Port is open but unable to read from port. Error : {error}".format(
                            error=str(e)
                        )
                    )
                    break
            else:
                self.logger.warn("Port is not configured/open.")
                break

    """
        Message handler for Ublox message. Invokes ublox_message_found event.
    """

    def __ublox_message_received(self, message: UBXMessage) -> None:
        self.logger.debug(
            " <- [Ubx:{identity}] : {bytes}".format(
                identity=message.identity, bytes=str(message)
            )
        )
        if (
            (message.identity == "RXM-SPARTN" or message.identity == "RXM-RTCM")
            and message.msgUsed == 2
        ) or (message.identity != "RXM-SPARTN" and message.identity != "RXM-RTCM"):
            self.__recent_ubx_message[message.identity] = (time.time(), message)
        self.ublox_message_found(message)
        pass

    """
        Message handler for Nmea message. Invokes nmea_message_found event.
    """

    def __nmea_message_received(self, message: NMEAMessage) -> None:
        self.logger.debug(
            " <- [Nmea:{identity}] : {bytes}".format(
                identity=message.identity, bytes=str(message)
            )
        )
        if message.identity == "GNRMC" or message.identity == "GNGGA":
            self.__status.latitude = (
                round(float(message.lat), 6) if message.lat else self.__status.latitude
            )
            self.__status.longitude = (
                round(float(message.lon), 6) if message.lon else self.__status.latitude
            )
            if message.identity == "GNGGA":
                self.__quality = message.quality
                pass
        self.nmea_message_found(message)
        pass

    """
        Message handler for Rtcm message. Invokes rtcm_message_found event.
    """

    def __rtcm_message_received(self, message: RTCMMessage) -> None:
        self.logger.debug(
            " <- [Rtcm:{identity}] : {bytes}".format(
                identity=message.identity, bytes=str(message)
            )
        )
        self.rtcm_message_found(message)
        pass

    """
        Writes data to port if the port is open. Raises an exception if not.
    """

    def send(self, data: bytes) -> None:
        try:
            if self.__port is not None and self.__port.is_open:
                self.logger.debug(" -> {bytes}".format(bytes=data.hex(" ")))
                self.__port.write(data)
            else:
                self.logger.warn("Port is not configured/open.")
        except Exception as e:
            self.logger.error(
                "Exception occured while writing on to the port : {error}".format(
                    error=str(e)
                )
            )
            pass

    # still need to work on this method
    """
        Polls the respective messages for rover/base to update status.
    """

    def poll(self):
        if self.__port is not None:
            self.logger.debug(" -> {bytes}".format(bytes="Polling Messages"))
            for class_name, msg_name in self.__poll_messages:
                ubx = UBXMessage(class_name, msg_name, POLL)
                self.send(ubx.serialize())
        else:
            self.logger.warn("Port is not configured/open.")
        pass

    def poll_once(self, class_name: str, msg_name: str) -> UBXMessage:
        ubx = UBXMessage(class_name, msg_name, POLL)
        retry_count = 0
        polled_message: UBXMessage = None
        while polled_message is None and retry_count < 3:
            self.send(ubx.serialize())
            # delay to make sure antenna response is logged before another attempt.
            time.sleep(0.5)
            polled_message = self.get_recent_ubx_message(msg_name)
            retry_count = retry_count + 1
        return polled_message

    """
        Adds only if not present.
    """

    def add_to_poll(self, class_name: str, msg_name: str) -> None:
        self.__poll_messages.add((class_name, msg_name))
        pass

    """
        Adds only if not present and deletes immediately so it is removed if present or not present from the set. Update if better method is found.
    """

    def remove_from_poll(self, class_name: str, msg_name: str) -> None:
        self.__poll_messages.add((class_name, msg_name))
        self.__poll_messages.remove((class_name, msg_name))
        pass

    """
        Configures the antenna based on parameters. Retries the config for 3times before throwing the error.
        Refer ubxtypes_configdb.py at /pyubx2/ubxtypes_configdb.py for equivalent name strings for different keys
    """

    def config(self) -> bool:
        config_successful = False
        config_data = self.__get_config_set(
            mode_of_operation=self.__rtk_mode,
            use_corrections=self.__use_corrections,
            corrections_source=self.__corrections_source,
        )

        # Sets the configs only to RAM. will reset if the antenna is power cycled.
        ubx: UBXMessage = UBXMessage.config_set(1, 0, config_data)
        retry_count = 0
        while not config_successful and retry_count < 3:
            self.send(ubx.serialize())
            # delay to make sure antenna response is logged before another attempt.
            time.sleep(0.5)
            if self.get_recent_ubx_message("ACK-ACK") is not None:
                config_successful = True
            retry_count = retry_count + 1

        self.__config_status = config_successful
        if not self.__config_status:
            self.logger.error("Configuration failed.")
        else:
            self.logger.info("Configuration Successful.")
        return config_successful

    """
        returns the Rover/Base status. Currently returning only Rover status. Will improve this further.
    """

    def get_status(self) -> GnssSignalStatus:
        try:
            # lat and long information
            if (
                self.__status.latitude is None
                or self.__status.latitude == 0
                or self.__status.longitude is None
                or self.__status.longitude == 0
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

            # augmentations information
            augmentations_used = False
            if self.__use_corrections:
                if self.__corrections_source == "PointPerfect":
                    rxm_spartn = self.get_recent_ubx_message("RXM-SPARTN")
                    augmentations_used = (
                        True if rxm_spartn and rxm_spartn.msgUsed == 2 else False
                    )
                else:
                    rxm_rtcm = self.get_recent_ubx_message("RXM-RTCM")
                    augmentations_used = (
                        True if rxm_rtcm and rxm_rtcm.msgUsed == 2 else False
                    )

            if self.__rtk_mode == "Rover":
                rxm_rtcm = self.get_recent_ubx_message("RXM-RTCM")
                augmentations_used = (
                    True if rxm_rtcm and rxm_rtcm.msgUsed == 2 else False
                )
            self.__status.augmentations_used = augmentations_used

            # heading information
            if (
                nav_relposned is not None
                and nav_relposned.relPosValid == 1
                and nav_relposned.relPosHeadingValid == 1
            ):
                self.__status.heading = round(nav_relposned.relPosHeading, 2)
                self.__status.length = round(
                    float(nav_relposned.relPosLength * 0.01), 2
                )

            # accuracy information
            if (
                nav_hpposecef is not None
                and nav_hpposecef.invalidEcef == 0
                and nav_hpposllh is not None
                and nav_hpposllh.invalidLlh == 0
            ):
                self.__status.altitude = round(
                    float(nav_hpposllh.height * 0.001), 4
                )  # scaling and meters conversion
                self.__status.accuracy_2d = round(
                    float(nav_hpposllh.hAcc * 0.001), 4
                )  # scaling and meters conversion
                self.__status.accuracy_3d = round(
                    float(nav_hpposecef.pAcc * 0.001), 4
                )  # scaling and meters conversion

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
                pass

            # navsatstatus information
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

            # navsignal information
            if nav_sig is not None:
                nav_sat_info_list = []
                no_of_satellites = 0
                gnss_sat_count = {}
                for a in dir(nav_sig):
                    if (
                        a.startswith("gnss")
                        and getattr(nav_sig, "cno_" + a.split("_")[1]) > 0
                    ):
                        gnss_id = getattr(nav_sig, a)
                        gnss_sat_count[gnss_id] = (
                            gnss_sat_count.get(gnss_id)
                            if gnss_sat_count.get(gnss_id)
                            else 0
                        ) + 1
                        no_of_satellites += 1
                        pass
                for item in gnss_sat_count:
                    nav_sat_info = NavSatInfo()
                    nav_sat_info.gnss_id = item
                    nav_sat_info.count = gnss_sat_count[item]
                    nav_sat_info_list.append(nav_sat_info)
                pass

                self.__status.no_of_satellites = no_of_satellites
                self.__status.satellite_information = UserList(nav_sat_info_list)

        except Exception as ex:
            self.logger.error(str(ex))
            pass
        return self.__status

    """
        returns a Ubx message only if it arrived in the last 10sec.
    """

    def get_recent_ubx_message(self, msgId) -> UBXMessage:
        try:
            time_of_message, ubxmessage = self.__recent_ubx_message[msgId]
            if (
                time.time() - time_of_message < 10
            ):  # return only if received in less than 10sec
                return ubxmessage
            else:
                return None
        except:
            return None

    """
        returns the Quality string for status based on Nav_Pvt message.
    """

    def __get_quality_string(self, nav_pvt: UBXMessage) -> str:
        if nav_pvt is None:
            return ""
        quality = ""
        if self.__quality == 0:
            quality += "No Fix"
        elif self.__quality == 1:
            quality += "Autonomous Gnss Fix"
        elif self.__quality == 2:
            quality += "Differential Gnss Fix"
        elif self.__quality == 3:
            quality += "PPS"
        elif self.__quality == 4:
            quality += "RTK Fixed"
        elif self.__quality == 5:
            quality += "RTK Float"
        elif self.__quality == 6:
            quality += "Dead Reckoning Fix"
        elif self.__quality == 7:
            quality += "Manual"
        elif self.__quality == 8:
            quality += "Simulated"

        quality += "("
        # Fix type definition
        if nav_pvt.fixType == 0:
            quality += "NoFix"
        elif nav_pvt.fixType == 1:
            quality += "DR"
        elif nav_pvt.fixType == 2:
            quality += "2D"
        elif nav_pvt.fixType == 3:
            quality += "3D"
        elif nav_pvt.fixType == 4:
            quality += "GDR"
        elif nav_pvt.fixType == 5:
            quality += "TF"

        if nav_pvt.gnssFixOk == 1:
            quality += "/DGNSS"

        if nav_pvt.carrSoln == 1:
            quality += "/Float"
        elif nav_pvt.carrSoln == 2:
            quality += "/Fixed"

        quality += ")"
        return quality

    def __get_config_set(
        self,
        mode_of_operation: Literal["Disabled", "Heading_Base", "Rover"],
        use_corrections: bool = False,
        corrections_source: Literal["PointPerfect", "Ntrip"] = "PointPerfect",
    ) -> list:
        # Common configuration. Enabling Nmea, Ubx messages for both input and output.
        config_data = [
            ("CFG_UART1INPROT_NMEA", 1),
            ("CFG_UART1INPROT_UBX", 1),
            ("CFG_UART1OUTPROT_NMEA", 1),
            ("CFG_UART1OUTPROT_UBX", 1),
            ("CFG_MSGOUT_UBX_MON_SYS_UART1", 1),
            ("CFG_NAVSPG_DYNMODEL", 0),
            ("CFG_MSGOUT_UBX_NAV_SIG_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_COV_UART1", 1),
        ]
        config_data.extend(
            [
                ("CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1", 1),
                ("CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1", 1),
                ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1),
            ]
        )
        if mode_of_operation == "Disabled":
            pass
        elif mode_of_operation == "Heading_Base":
            # Enabling output RTCM and disabling input RTCM
            config_data.extend(
                [("CFG_UART1INPROT_RTCM3X", 0), ("CFG_UART1OUTPROT_RTCM3X", 1)]
            )

            # Common RTCM message types for Base (1074, 1084, 1094, 1124).
            config_data.extend(
                [
                    ("CFG_MSGOUT_RTCM_3X_TYPE1074_UART1", 0x1),
                    ("CFG_MSGOUT_RTCM_3X_TYPE1084_UART1", 0x1),
                    ("CFG_MSGOUT_RTCM_3X_TYPE1124_UART1", 0x1),
                    ("CFG_MSGOUT_RTCM_3X_TYPE1094_UART1", 0x1),
                ]
            )

            # 4072.0, 1230 is Enabled for Heading_Base. Also, TimeMode is set to disabled.
            config_data.extend(
                [
                    ("CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1", 0x1),
                    ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 0x1),
                    ("CFG_TMODE_MODE", 0x0),
                ]
            )
        elif mode_of_operation == "Rover":
            # rover related configurations
            config_data.extend(
                [
                    ("CFG_UART1INPROT_RTCM3X", 1),
                    ("CFG_MSGOUT_UBX_RXM_RTCM_UART1", 0x1),
                    ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
                ]
            )

        if use_corrections:
            if corrections_source == "PointPerfect":
                config_data.extend(
                    [
                        ("CFG_SPARTN_USE_SOURCE", 0),
                        ("CFG_UART1INPROT_SPARTN", 1),
                        ("CFG_MSGOUT_UBX_RXM_SPARTN_UART1", 1),
                        ("CFG_MSGOUT_UBX_RXM_COR_UART1", 1),
                    ]
                )
            else:
                config_data.extend(
                    [
                        ("CFG_UART1INPROT_RTCM3X", 1),
                        ("CFG_MSGOUT_UBX_RXM_RTCM_UART1", 0x1),
                        ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
                        ("CFG_MSGOUT_UBX_RXM_COR_UART1", 1),
                    ]
                )

        return config_data

    def __get_service_constellations(self) -> int:
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
        retry_count = 0
        while gnss_config_msg is None and retry_count < 3:
            self.send(gnss_config_poll.serialize())
            time.sleep(0.5)
            gnss_config_msg = self.get_recent_ubx_message("CFG-VALGET")
            retry_count = retry_count + 1
            pass
        if gnss_config_msg is None:
            self.logger.error("Antenna is not responding.")
        service_constellations: int = 0
        if gnss_config_msg.CFG_SIGNAL_GPS_ENA == 1:
            service_constellations = service_constellations | NavSatStatus.SERVICE_GPS
        if gnss_config_msg.CFG_SIGNAL_GLO_ENA == 1:
            service_constellations = (
                service_constellations | NavSatStatus.SERVICE_GLONASS
            )
        if gnss_config_msg.CFG_SIGNAL_BDS_ENA == 1:
            service_constellations = (
                service_constellations | NavSatStatus.SERVICE_COMPASS
            )
        if gnss_config_msg.CFG_SIGNAL_GAL_ENA == 1:
            service_constellations = (
                service_constellations | NavSatStatus.SERVICE_GALILEO
            )
        return service_constellations

    def __save_boot_times(self, time_in_sec: int):
        with open(self.__rtk_mode + "_boot_times.txt", "a") as log_file:
            log_file.write("\nAntenna Rebooted after sec: " + str(time_in_sec))
        pass
