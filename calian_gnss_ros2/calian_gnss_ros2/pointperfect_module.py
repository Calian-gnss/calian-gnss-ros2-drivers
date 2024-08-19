import os
from textwrap import wrap
import rclpy
from std_srvs.srv import Empty
from std_msgs.msg import Header
from rclpy.node import Node
import threading
from typing import Literal
import paho.mqtt.client as mqtt
from events import Event
import json
from calian_gnss_ros2.logging import Logger, LoggingLevel, SimplifiedLogger
from calian_gnss_ros2_msg.msg import CorrectionMessage

"""
    This class is responsible to get all the fields required to make an MQTT connection for the PointPerfect correction messages.

    Needs a valid subscription to Ublox/PointPerfect corrections.
"""


class PointPerfectConfiguration:
    """
    region: Select based on region of operation.
    configPath: Absolute path to the ucenter-config file downloaded from https://portal.thingstream.io/
    """

    def __init__(
        self, region: Literal["us", "kr", "eu", "au"], config_path: str
    ) -> None:
        self.__region = region
        self.Host: str = "pp.services.u-blox.com"
        self.Port: int = 8883
        self.__generate_configs_from_file(config_path)
        pass

    """
        Extracts configurations from PointPerfect configuration file. Config file <ucenter_config_file> should be the downloaded from https://portal.thingstream.io/. 
    """

    def __generate_configs_from_file(self, config_path: str):
        with open(config_path) as json_file:
            config_json = json.load(json_file)["MQTT"]
            self.keep_alive = config_json["Connectivity"]["KeepAliveInterval"]
            self.client_id = config_json["Connectivity"]["ClientID"]
            self.topics = self.__get_topics(config_json["Subscriptions"])
            self.cert_file = self.__generate_cert_file(
                "CERTIFICATE",
                config_json["Connectivity"]["ClientCredentials"]["Cert"],
                "pp_cert.crt",
            )
            self.ca_cert = self.__generate_cert_file(
                "CERTIFICATE",
                config_json["Connectivity"]["ClientCredentials"]["RootCA"],
                "pp_ca_cert.crt",
            )
            self.key_file = self.__generate_cert_file(
                "RSA PRIVATE KEY",
                config_json["Connectivity"]["ClientCredentials"]["Key"],
                "pp_key.pem",
            )
        pass

    """
        Extracts necessary topics from Subsciption topics.
    """

    def __get_topics(self, subs: dict) -> None:
        topics: list[(str, int)] = [(subs["Key"]["KeyTopics"][0], subs["Key"]["QoS"])]
        qos = subs["Data"]["QoS"]
        for topic in subs["Data"]["DataTopics"]:
            if self.__region in topic:
                topics.append((topic, qos))
        topics.pop()  # vomitting deserialized topic
        return topics

    """
        Generates the required cert/key files from the Configuration.
    """

    def __generate_cert_file(self, header: str, contents: str, file_name: str) -> str:
        directory = "src/calian_gnss_ros2/pointperfect_files"
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, file_name), "w") as file:
            lines = []
            pem_prefix = "-----BEGIN {}-----\n".format(header)
            lines.append(pem_prefix)
            for line in wrap(contents, 64):
                lines.append(line + "\n")
            pem_suffix = "-----END {}-----".format(header)
            lines.append(pem_suffix)
            file.writelines(lines)
            pass
        return os.path.join(directory, file_name)


class PointPerfectModule(Node):
    def __init__(self) -> None:
        super().__init__("pointperfect")

        # region Parameters declaration
        self.declare_parameter("config_path", "")
        self.declare_parameter("region", "")
        self.declare_parameter("corrections_frame_id", "corrections")
        self.declare_parameter("save_logs", False)
        self.declare_parameter("log_level", LoggingLevel.Info)
        # endregion

        # region Parameters Initialization
        self.config_path = (
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        self.region = self.get_parameter("region").get_parameter_value().string_value
        self._corrections_frame_id = (
            self.get_parameter("corrections_frame_id")
            .get_parameter_value()
            .string_value
        )
        self.save_logs = (
            self.get_parameter("save_logs").get_parameter_value().bool_value
        )
        self.log_level: LoggingLevel = LoggingLevel(
            self.get_parameter("log_level").get_parameter_value().integer_value
        )
        # endregion

        self.__pc = PointPerfectConfiguration(self.region, self.config_path)
        self.on_correction_message = self.create_publisher(
            CorrectionMessage, "corrections", 100
        )
        _ = self.create_service(Empty, "restart", self.reconnect)

        # Mqtt client initialization.
        self.__client = mqtt.Client(
            client_id=self.__pc.client_id, reconnect_on_failure=True, clean_session=True
        )
        self.__client.tls_set(
            ca_certs=self.__pc.ca_cert,
            certfile=self.__pc.cert_file,
            keyfile=self.__pc.key_file,
        )

        self.__client.on_message = self.__on_message
        self.__client.on_disconnect = self.__on_disconnect
        self.__client.on_connect = self.__on_connect

        internal_logger = Logger(self.get_logger())
        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger("PointPerfect")

        _ = threading.Thread(
            target=self.__connect, name="corrections_connect_thread", daemon=True
        ).start()
        pass

    """
        Default callback for incoming message received from MQTT connection. Invokes on_correction_message event
    """

    def __on_message(self, client, userdata, message) -> None:
        self.logger.info(
            "Received message '"
            + str(message.payload)
            + "' on topic '"
            + message.topic
            + "' with QoS "
            + str(message.qos)
        )
        self.on_correction_message.publish(
            CorrectionMessage(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self._corrections_frame_id,
                ),
                message=message.payload,
            )
        )

    """
        Gets connection message based on 'rc' value.
    """

    def __get_connection_message(self, rc: int) -> str:
        if rc == 0:
            return "Connection successful."
        elif rc == 1:
            return "Connection refused - incorrect protocol version."
        elif rc == 2:
            return "Connection refused - invalid client identifier."
        elif rc == 3:
            return "Connection refused - server unavailable."
        elif rc == 4:
            return "Connection refused - bad username or password."
        elif rc == 5:
            return "Connection refused - not authorised."
        elif rc >= 6:
            return "Currently Unused."

    """
        Callback for MQTT disconnect.
    """

    def __on_disconnect(self, client, userdata, rc) -> None:
        if rc != 0:
            self.logger.warn(
                "Unexpected disconnection."
                + self.__get_connection_message(rc)
                + ". Trying to reconnect.."
            )
            self.__client.reconnect()

    """
        Callback for MQTT connect. subscribe to topics after connect.
    """

    def __on_connect(self, client, userdata, flags, rc) -> None:
        self.logger.info(
            "Connection returned result: " + self.__get_connection_message(rc)
        )
        self.__client.subscribe(self.__pc.topics)

    """
        Connects to MQTT client.
    """

    def __connect(self) -> None:
        try:
            pc = self.__pc
            self.__client.connect(host=pc.Host, port=pc.Port, keepalive=pc.keep_alive)
            self.__client.loop_start()
        except Exception as ex:
            self.logger.error(
                "Exception occured while connecting to PointPerfect client." + str(ex)
            )

    """
        Disconnects from MQTT client.
    """

    def __disconnect(self) -> None:
        self.__client.loop_stop()
        self.__client.disconnect()

    def reconnect(self, request, response) -> None:
        try:
            self.__client.disconnect()
            self.__client.reconnect()
        except Exception as ex:
            self.logger.error(
                "Exception occured while reconnecting to PointPerfect client"
            )
        return response


def main():
    rclpy.init()
    pp_corrections_module = PointPerfectModule()
    try:
        rclpy.spin(pp_corrections_module)
    except KeyboardInterrupt:
        pass
    pp_corrections_module.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
