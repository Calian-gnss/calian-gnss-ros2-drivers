"""Logging infrastructure for calian_gnss_ros2 nodes.

Provides a singleton Logger backed by the ROS 2 logger, a SimplifiedLogger
with per-module prefixes and optional file logging, and a helper
``setup_node_logging`` that eliminates the boilerplate repeated in every node.
"""

from enum import IntEnum
import os
import datetime
from typing import Tuple

from rclpy.node import Node


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_DEFAULT_LOG_DIRECTORY = "src/calian_gnss_ros2/logs"


# ---------------------------------------------------------------------------
# LoggingLevel
# ---------------------------------------------------------------------------


class LoggingLevel(IntEnum):
    """Standard Python / ROS 2 logging levels."""

    NotSet = 0
    Debug = 10
    Info = 20
    Warn = 30
    Error = 40
    Critical = 50


# ---------------------------------------------------------------------------
# Logger (singleton wrapper around a ROS 2 logger)
# ---------------------------------------------------------------------------


class Logger:
    """Singleton that delegates to a ROS 2 logger and optionally writes to files."""

    _instance = None
    _internal_logger = None
    _log_level: LoggingLevel = LoggingLevel.NotSet
    _should_save_logs: bool = False
    _log_directory: str = _DEFAULT_LOG_DIRECTORY

    def __new__(cls, logger=None):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._internal_logger = logger
        cls._log_directory = _DEFAULT_LOG_DIRECTORY
        os.makedirs(cls._log_directory, exist_ok=True)
        return cls._instance

    def set_level(self, level: LoggingLevel) -> bool:
        """Set the ROS logger severity and internal threshold."""
        if isinstance(level, LoggingLevel):
            if self._internal_logger is not None:
                self._internal_logger.set_level(level)
            self._log_level = level
            return True
        return False

    # Keep the old name as an alias so existing callers still work.
    setLevel = set_level

    def toggle_logs(self, save: bool) -> None:
        """Enable or disable file logging."""
        self._should_save_logs = save

    # ---- Convenience log methods -----------------------------------------

    def info(self, message: str) -> None:
        if self._internal_logger is not None:
            self._internal_logger.info(message)

    def debug(self, message: str) -> None:
        if self._internal_logger is not None:
            self._internal_logger.debug(message)

    def warn(self, message: str) -> None:
        if self._internal_logger is not None:
            self._internal_logger.warn(message)

    def error(self, message: str) -> None:
        if self._internal_logger is not None:
            self._internal_logger.error(message)

    def critical(self, message: str) -> None:
        if self._internal_logger is not None:
            self._internal_logger.fatal(message)

    # ---- File logging ----------------------------------------------------

    def log_to_file(self, file_name: str, message: str) -> None:
        """Append *message* to a daily log file when file logging is enabled."""
        if self._should_save_logs:
            with open(os.path.join(self._log_directory, file_name), "a") as log_file:
                log_file.write(f"\n{message}")


# ---------------------------------------------------------------------------
# SimplifiedLogger (per-module prefixed logger with file output)
# ---------------------------------------------------------------------------


class SimplifiedLogger:
    """Logger that prepends a module name and optionally writes to daily files."""

    def __init__(self, name: str) -> None:
        self._name = name
        self._log_prefix = f"[{name}] : "
        self._logger = Logger()

    def _log_to_file(self, message: str) -> None:
        """Write a timestamped message to the daily log file."""
        file_name = f"{self._name}_{datetime.datetime.now():%Y-%m-%d}.txt"
        formatted = (
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S.%f}  "
            f"{self._log_prefix}  {message}"
        )
        self._logger.log_to_file(file_name, formatted)

    def info(self, message: str) -> None:
        self._logger.info(self._log_prefix + message)
        self._log_to_file(message)

    def debug(self, message: str) -> None:
        self._logger.debug(self._log_prefix + message)
        self._log_to_file(message)

    def warn(self, message: str) -> None:
        self._logger.warn(self._log_prefix + message)
        self._log_to_file(message)

    def error(self, message: str) -> None:
        self._logger.error(self._log_prefix + message)
        self._log_to_file(message)

    def critical(self, message: str) -> None:
        self._logger.critical(self._log_prefix + message)
        self._log_to_file(message)

def setup_node_logging(
    node: Node, logger_name: str
) -> Tuple[Logger, SimplifiedLogger]:
    """Declare standard log parameters on *node* and return configured loggers.

        _, self.logger = setup_node_logging(self, "Rover_GPS")
    """
    node.declare_parameter("save_logs", False)
    node.declare_parameter("log_level", int(LoggingLevel.Info))

    save_logs: bool = (
        node.get_parameter("save_logs").get_parameter_value().bool_value
    )
    log_level = LoggingLevel(
        node.get_parameter("log_level").get_parameter_value().integer_value
    )

    internal_logger = Logger(node.get_logger())
    internal_logger.toggle_logs(save_logs)
    internal_logger.set_level(log_level)

    return internal_logger, SimplifiedLogger(logger_name)
