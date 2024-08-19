from enum import Enum, IntEnum
import os
import datetime

# from rclpy.impl import RcutilsLogger


class LoggingModules(Enum):
    Gps = 1
    Serial = 2
    PointPerfect = 3


class LoggingLevel(IntEnum):
    # As per python standards
    NotSet = 0
    Debug = 10
    Info = 20
    Warn = 30
    Error = 40
    Critical = 50


class Logger(object):
    _instance = None
    _internal_logger = None
    _log_level = LoggingLevel.NotSet
    _should_save_logs = False

    def __new__(cls, logger=None):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._internal_logger = logger
            # Put any initialization here.
        cls._log_directory = "src/calian_gnss_ros2/logs"
        os.makedirs(cls._log_directory, exist_ok=True)
        return cls._instance

    def setLevel(cls, level: LoggingLevel) -> bool:
        if isinstance(level, LoggingLevel):
            if cls._internal_logger is not None:
                cls._internal_logger.set_level(level)
            cls._log_level = level
            return True
        return False

    def toggle_logs(cls, save: bool):
        cls._should_save_logs = save

    def info(cls, message: str):
        if cls._internal_logger is not None:
            cls._internal_logger.info(message)

    def debug(cls, message: str):
        if cls._internal_logger is not None:
            cls._internal_logger.debug(message)

    def warn(cls, message: str):
        if cls._internal_logger is not None:
            cls._internal_logger.warn(message)

    def error(cls, message: str):
        if cls._internal_logger is not None:
            cls._internal_logger.error(message)

    def critical(cls, message: str):
        if cls._internal_logger is not None:
            cls._internal_logger.fatal(message)

    def log_to_file(cls, file_name: str, message: str):
        if cls._should_save_logs:
            with open(os.path.join(cls._log_directory, file_name), "a") as log_file:
                log_file.write("\n{message}".format(message=message))
                pass


class SimplifiedLogger:
    _name: str = None

    def __init__(self, name: str) -> None:
        self._name = name
        self._log_prefix = "[{log_name}] : ".format(log_name=self._name)
        self._logger = Logger()
        pass

    def _log(self, message: str):
        file_name = self._name + "_{day}.txt".format(
            day=datetime.datetime.now().strftime("%Y-%m-%d")
        )
        formatted_log_message = "{time}  {log_prefix}  {message}".format(
            time=(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")),
            log_prefix=self._log_prefix,
            message=message,
        )
        self._logger.log_to_file(file_name, formatted_log_message)

    def info(self, message: str):
        self._logger.info(self._log_prefix + message)
        self._log(message)

    def debug(self, message: str):
        self._logger.debug(self._log_prefix + message)
        self._log(message)

    def warn(self, message: str):
        self._logger.warn(self._log_prefix + message)
        self._log(message)

    def error(self, message: str):
        self._logger.error(self._log_prefix + message)
        self._log(message)

    def critical(self, message: str):
        self._logger.critical(self._log_prefix + message)
        self._log(message)
