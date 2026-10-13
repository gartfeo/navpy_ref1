"""The output latch: a lost log line is recorded, while wall output stays the same."""

import logging

import pytest

from navpy.core.kernel.config.logger_args import LoggerArgsStub
from navpy.core.kernel.logging.cache_logger import CacheLogger
from navpy.core.kernel.logging.log_output_latch import (
    LatchingRotatingFileHandler,
    LogOutputFailed,
    OutputLatch,
    close_strictly,
)


class _FailingStream:
    def write(self, _text: str) -> None:
        raise OSError("disk full")

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def _logger(tmp_path, name: str) -> tuple[logging.Logger, LatchingRotatingFileHandler, OutputLatch]:
    latch = OutputLatch()
    handler = LatchingRotatingFileHandler(str(tmp_path / f"{name}.log"), encoding="utf-8", latch=latch)
    logger = logging.getLogger(f"latch_test_{name}")
    logger.handlers[:] = [handler]
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    return logger, handler, latch


def test_a_good_write_leaves_the_latch_clear(tmp_path):
    logger, handler, latch = _logger(tmp_path, "good")
    cache = CacheLogger(LoggerArgsStub(), logger, output_latch=latch)
    cache.info("fine")
    close_strictly(cache)
    handler.close()
    assert not latch.latched
    assert "fine" in (tmp_path / "good.log").read_text(encoding="utf-8")


def test_a_swallowed_emit_error_latches_and_strict_close_raises(tmp_path, capsys):
    logger, handler, latch = _logger(tmp_path, "bad")
    handler.stream.close()
    handler.stream = _FailingStream()
    cache = CacheLogger(LoggerArgsStub(), logger, output_latch=latch)
    cache.info("lost")
    cache.close()  # the wall close: the same swallowed error, no raise
    assert latch.latched
    assert "OSError: disk full" in capsys.readouterr().err  # logging's own report, unchanged
    with pytest.raises(LogOutputFailed):
        close_strictly(cache)
