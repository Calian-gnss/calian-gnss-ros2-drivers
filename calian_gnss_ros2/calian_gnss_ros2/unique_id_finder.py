"""Unique ID Finder — scans serial ports for connected Calian antennas and prints their unique IDs.

This is a one-shot utility node: it runs the scan, prints results, and exits.
"""

import serial
import concurrent.futures
from serial.tools.list_ports import comports

import rclpy
from rclpy.node import Node

from calian_gnss_ros2.serial_module import SerialUtilities
from calian_gnss_ros2.logging import setup_node_logging


# Standard baud rates to probe (low → high)
_BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800]
_EXTRACT_TIMEOUT_SEC = 3


class UniqueIdFinder(Node):
    """Scans all serial ports with 'Standard' in the description for antenna unique IDs."""

    def __init__(self) -> None:
        super().__init__("unique_id_finder")

        _, self.logger = setup_node_logging(self, "UniqueIdFinder")

        self.logger.info("Processing connected ports...")
        ports = comports()

        if not ports:
            self.logger.warn("No ports connected.")
            return

        with concurrent.futures.ThreadPoolExecutor() as executor:
            for port in ports:
                if "Standard" not in port.description:
                    continue

                for baudrate in _BAUD_RATES:
                    standard_port = None
                    try:
                        self.logger.debug(
                            f"Connecting to {port.device} at baud {baudrate}"
                        )
                        standard_port = serial.Serial(port.device, baudrate)
                        future = executor.submit(
                            SerialUtilities.extract_unique_id_of_port,
                            standard_port=standard_port,
                            timeout=_EXTRACT_TIMEOUT_SEC,
                        )
                        unique_id = future.result(_EXTRACT_TIMEOUT_SEC)
                        self.logger.info(
                            f"{port.device} : {unique_id.upper()} at baud {baudrate}"
                        )
                        break  # Found it — move to next port
                    except Exception as e:
                        self.logger.error(
                            f"Cannot get unique id of {port.device} at baud {baudrate}: {e}"
                        )
                    finally:
                        if standard_port is not None:
                            standard_port.close()

                self.logger.debug("Moving on to next port.")

        self.logger.info("All ports are processed.")


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------


def main():
    rclpy.init()
    node = UniqueIdFinder()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
