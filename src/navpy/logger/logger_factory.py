import logging
import os
from datetime import datetime
from logging.handlers import RotatingFileHandler

from navpy.args.logger_args import LoggerArgs
from navpy.logger.cache_logger import CacheLogger, IStatusLogger


LOG_DIR_ENV = "NAVPY_LOG_DIR"


def _resolve_log_path() -> str:
    """Return the process log directory, honoring an explicit run binding."""
    override = os.environ.get(LOG_DIR_ENV, "").strip()
    if override:
        return os.path.abspath(os.path.expanduser(override))
    return f'.logs/{datetime.today().date()}/{datetime.today().strftime("%H%M%S")}/'


def initialize_logger(args: LoggerArgs, sys_id, status_logger: IStatusLogger = None):
    # Create a custom logger
    logger = logging.getLogger(f'vehicle_logger_{sys_id}')

    # Ensure no other handlers are attached (prevents duplicate logs)
    if logger.hasHandlers():
        logger.handlers.clear()

    # Set the lowest level of the logger to DEBUG
    logger.setLevel(logging.DEBUG)

    # Create formatter
    log_formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')

    # Create file handler and set level to DEBUG
    log_path = _resolve_log_path()
    os.makedirs(log_path, exist_ok=True)
    log_file = os.path.join(log_path, f'uav_{sys_id}_navigation.log')

    # encoding="utf-8": log messages can contain non-cp1252 characters (e.g. the
    # '→' in DetectionCoordinator start_tracking lines). Without this the handler
    # uses the Windows locale codec (cp1252) and raises UnicodeEncodeError on emit.
    file_handler = RotatingFileHandler(
        log_file, maxBytes=5 * 1024 * 1024, backupCount=2, encoding="utf-8"
    )
    file_handler.setLevel(logging.DEBUG)  # Log DEBUG and above messages to the file
    file_handler.setFormatter(log_formatter)

    # Create console handler and set level to INFO
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)  # Only log INFO and above messages to the console
    console_handler.setFormatter(log_formatter)

    # Add handlers to the logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    # Ensure the logger is not propagating to the root logger
    logger.propagate = False

    cache_logger = CacheLogger(args, logger, status_logger)
    cache_logger.log_path = log_path

    return cache_logger
