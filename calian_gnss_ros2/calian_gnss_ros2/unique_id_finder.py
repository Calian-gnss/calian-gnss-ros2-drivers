from calian_gnss_ros2.serial_module import SerialUtilities
import rclpy
import serial
from serial.tools.list_ports import comports
from rclpy.node import Node
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger
import concurrent.futures


class UniqueIdFinder(Node):
    def __init__(self) -> None:
        super().__init__("unique_id_finder")
        self.declare_parameter("save_logs", False)
        self.declare_parameter("log_level", LoggingLevel.Info)
        self.save_logs = (
            self.get_parameter("save_logs").get_parameter_value().bool_value
        )
        self.log_level: LoggingLevel = LoggingLevel(
            self.get_parameter("log_level").get_parameter_value().integer_value
        )
        self.baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800]
        # endregion
        internal_logger = Logger(self.get_logger())
        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger("unique_id_finder")
        self.logger.info("Processing connected ports.....")
        ports = comports()
        if len(ports) == 0:
            self.logger.warn("No ports connected.")
        else:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for port in ports:
                    if port.description.find("Standard") != -1:
                        for baudrate in self.baudrates:
                            standard_port = None
                            try:
                                self.logger.debug("Connecting to port " + port.device + " using baud : " + str(baudrate))
                                standard_port = serial.Serial(port.device, baudrate)
                                thread = executor.submit(
                                    SerialUtilities.extract_unique_id_of_port,
                                    standard_port=standard_port,
                                    timeout=3,
                                )
                                unique_id_of_port = thread.result(3)
                                self.logger.info(
                                    port.device + " : " + unique_id_of_port.upper() + " using baud : " + str(baudrate) 
                                )
                                break
                            except:
                                self.logger.error(
                                    "Cannot get unique id of the port " + port.device + " using baud : " + str(baudrate)
                                )
                                pass
                            finally:
                                if standard_port is not None:
                                    standard_port.close()
                            
                        self.logger.debug("Moving on to next port.")
                    pass
                pass
        self.logger.info("All ports are processed.")


def main():
    rclpy.init()
    unique_id_finder = UniqueIdFinder()
    unique_id_finder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
