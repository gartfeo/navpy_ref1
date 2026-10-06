"""Tests for backend rotating-file logging setup."""
import logging
from logging.handlers import RotatingFileHandler

from gcs.backend.main import _add_file_logging


def _count_handlers(target):
    root = logging.getLogger()
    return sum(
        1 for h in root.handlers
        if isinstance(h, RotatingFileHandler) and h.baseFilename == str(target)
    )


def test_creates_rotating_handler_and_dir(tmp_path):
    log_dir = tmp_path / "gcs"
    target = _add_file_logging(log_dir)
    try:
        assert target == log_dir / "backend.log"
        assert log_dir.exists()
        assert _count_handlers(target) == 1
        # a logged line is written to the file (set level explicitly so pytest's
        # log-level handling doesn't filter the INFO record before the handler)
        logger = logging.getLogger("gcs.backend.launch_controller")
        prev = logger.level
        logger.setLevel(logging.INFO)
        try:
            logger.info("[launch] test line")
        finally:
            logger.setLevel(prev)
        for h in logging.getLogger().handlers:
            if isinstance(h, RotatingFileHandler) and h.baseFilename == str(target):
                h.flush()
        assert "[launch] test line" in target.read_text(encoding="utf-8")
    finally:
        root = logging.getLogger()
        for h in list(root.handlers):
            if isinstance(h, RotatingFileHandler) and h.baseFilename == str(target):
                root.removeHandler(h)
                h.close()


def test_idempotent(tmp_path):
    log_dir = tmp_path / "gcs"
    target = _add_file_logging(log_dir)
    try:
        _add_file_logging(log_dir)  # second call must not add a duplicate
        assert _count_handlers(target) == 1
    finally:
        root = logging.getLogger()
        for h in list(root.handlers):
            if isinstance(h, RotatingFileHandler) and h.baseFilename == str(target):
                root.removeHandler(h)
                h.close()
