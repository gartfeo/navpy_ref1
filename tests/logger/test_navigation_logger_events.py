"""Byte, cadence, flush, and lifecycle contracts for navigation logging."""

from __future__ import annotations

import tempfile
import threading
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional
from unittest.mock import Mock, patch

import pymap3d

from navpy.logger.navigation_event_recorder import NavigationEventRecorder
from navpy.logger.navigation_log_streams import (
    NavigationLogStreams,
    open_navigation_log_streams,
)
from navpy.logger.navigation_logger import NavigationLogger
from navpy.logger.log_events import LogEvent, STANDARD_EVENT_HEADER
from navpy.logger.log_schema import LEGACY_PRIMARY_FIELDS, PRIMARY_SCHEMA
from navpy.modules.common.models.location import Location


class _TrackingFile:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.flush_count = 0
        self.closed = False
        self.close_count = 0

    def write(self, line: str) -> None:
        self.lines.append(line)

    def flush(self) -> None:
        self.flush_count += 1

    def close(self) -> None:
        self.closed = True
        self.close_count += 1

    def value(self) -> str:
        return "".join(self.lines)

    def reset_flush_count(self) -> None:
        self.flush_count = 0


class _FlushFailingFile(_TrackingFile):
    def flush(self) -> None:
        self.flush_count += 1
        raise OSError("flush failed")


class _FirstFlushFailingFile(_TrackingFile):
    def flush(self) -> None:
        self.flush_count += 1
        if self.flush_count == 1:
            raise OSError("primary flush failed")


@dataclass
class _LoggerHarness:
    logger: NavigationLogger
    compact: Optional[_TrackingFile]
    debug: Optional[_TrackingFile]
    cache: Mock

    def close(self) -> None:
        self.logger.close()


def _make_logger(
    *,
    wall_time: Callable[[], float] = lambda: 1000.0,
    timestamp: Callable[[], str] = lambda: "12:00:00.000",
    time_source: Optional[Callable[[], float]] = None,
    cadence_interval: Optional[Callable[[float], float]] = None,
    compact_enabled: bool = True,
    debug_enabled: bool = True,
    headers: bool = True,
) -> _LoggerHarness:
    compact = _TrackingFile() if compact_enabled else None
    debug = _TrackingFile() if debug_enabled else None
    if headers and compact is not None:
        compact.write(PRIMARY_SCHEMA.header_line() + "\n")
        compact.flush()
    if headers and debug is not None:
        debug.write(STANDARD_EVENT_HEADER + "\n")
        debug.flush()
    streams = NavigationLogStreams(compact, debug)
    cache = Mock()
    logger = NavigationLogger(
        7,
        cache,
        time_source=time_source,
        cadence_interval=cadence_interval,
        wall_time=wall_time,
        timestamp=timestamp,
        streams=streams,
    )
    return _LoggerHarness(logger, compact, debug, cache)


def _log_minimal(
    logger: NavigationLogger,
    distance: float,
    *,
    rate_gate: Optional[bool] = None,
) -> None:
    logger.log(
        c_loc=Location(40.0, 44.0, 500.0, is_absolute=True),
        t_loc=Location(40.0, 44.0, 0.0, is_absolute=True),
        distance=distance,
        cmd_roll=0.0,
        cmd_pitch=0.0,
        yaw_error=0.0,
        pitch_error=0.0,
        actual_roll=0.0,
        actual_pitch=0.0,
        x_error=0,
        y_error=0,
        rate_gate=rate_gate,
    )


def _primary_rows(content: str) -> list[str]:
    return [
        line
        for line in content.splitlines()
        if line and not line.startswith("ts,") and "EVENT:" not in line
    ]


def _loc_from_ned(
    reference: Location,
    north: float,
    east: float,
    down: float,
) -> Location:
    lat, lng, alt = pymap3d.ned2geodetic(
        north,
        east,
        down,
        reference.lat,
        reference.lng,
        reference.alt,
    )
    return Location(lat, lng, alt, is_absolute=True)


class HeaderAndConfigTests(unittest.TestCase):
    def test_header_and_config_bytes_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = Mock()
            cache.log_path = tmp
            logger = NavigationLogger(
                1,
                cache,
                timestamp=lambda: "00:00:00.000",
            )
            logger.close()

            compact = Path(tmp, "uav_1_navigation_compact.csv").read_text()
            debug = Path(tmp, "uav_1_navigation_debug.csv").read_text()

        self.assertEqual(compact.splitlines(), [PRIMARY_SCHEMA.header_line()])
        self.assertEqual(debug.splitlines()[0], STANDARD_EVENT_HEADER)
        self.assertTrue(
            debug.splitlines()[1].startswith(
                "00:00:00.000,EVENT:CONFIG,"
            )
        )

    def test_config_payload_contains_interval_and_schema(self):
        harness = _make_logger(timestamp=lambda: "01:02:03.456")
        harness.close()

        config_line = harness.debug.value().splitlines()[1]
        self.assertIn("min_log_interval_s=0.1", config_line)
        self.assertIn(f"schema={PRIMARY_SCHEMA.name}", config_line)

    def test_debug_file_failure_preserves_compact_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = Mock()
            cache.log_path = tmp
            real_open = open

            def open_without_debug(path, *args, **kwargs):
                if str(path).endswith("_navigation_debug.csv"):
                    raise OSError("debug denied")
                return real_open(path, *args, **kwargs)

            with patch("builtins.open", open_without_debug):
                logger = NavigationLogger(
                    3,
                    cache,
                    timestamp=lambda: "09:08:07.006",
                )
            current = Location(40.0001, 44.0, 100.0, is_absolute=True)
            target = Location(40.0, 44.0, 100.0, is_absolute=True)
            logger.sample_snap(current, target)
            logger.write_summary_and_reset(algorithm="pn", kp=0.5)
            logger.close()

            compact = Path(tmp, "uav_3_navigation_compact.csv").read_text()
            debug_path = Path(tmp, "uav_3_navigation_debug.csv")
            debug_exists = debug_path.exists()

        self.assertIn("09:08:07.006,SNAP(PN),", compact)
        self.assertFalse(debug_exists)


class RateLimitTests(unittest.TestCase):
    def test_two_calls_within_interval_produce_one_row(self):
        now = {"value": 1000.0}
        harness = _make_logger(wall_time=lambda: now["value"])

        _log_minimal(harness.logger, 100.0)
        now["value"] += 0.05
        _log_minimal(harness.logger, 101.0)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 1)

    def test_two_calls_beyond_interval_produce_two_rows(self):
        now = {"value": 1000.0}
        harness = _make_logger(wall_time=lambda: now["value"])

        _log_minimal(harness.logger, 100.0)
        now["value"] += 0.2
        _log_minimal(harness.logger, 100.0)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 2)

    def test_cadence_adapter_changes_interval_without_scaling_clock(self):
        now = {"value": 5000.0}
        harness = _make_logger(
            wall_time=lambda: now["value"],
            cadence_interval=lambda period_s: period_s / 10.0,
        )

        _log_minimal(harness.logger, 100.0)
        now["value"] += 0.005
        _log_minimal(harness.logger, 101.0)
        now["value"] += 0.01
        _log_minimal(harness.logger, 102.0)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 2)

    def test_explicit_rate_gate_is_shared_with_caller(self):
        harness = _make_logger(wall_time=lambda: 1000.0)

        _log_minimal(harness.logger, 100.0, rate_gate=True)
        _log_minimal(harness.logger, 99.0, rate_gate=True)
        _log_minimal(harness.logger, 98.0, rate_gate=False)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 2)

    def test_distance_change_does_not_bypass_wall_interval(self):
        now = {"value": 1000.0}
        harness = _make_logger(wall_time=lambda: now["value"])

        _log_minimal(harness.logger, 100.0)
        now["value"] += 0.01
        _log_minimal(harness.logger, 500.0)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 1)

    def test_event_does_not_consume_primary_gate(self):
        now = {"value": 1000.0}
        harness = _make_logger(wall_time=lambda: now["value"])

        _log_minimal(harness.logger, 100.0)
        now["value"] += 0.05
        harness.logger.log_event(LogEvent.VALIDITY_FLIP, {"k": "v"})
        now["value"] += 0.05
        _log_minimal(harness.logger, 100.0)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 1)
        self.assertIn("EVENT:VALIDITY_FLIP", harness.debug.value())

    def test_summary_reset_reopens_primary_gate(self):
        harness = _make_logger(wall_time=lambda: 1000.0)

        _log_minimal(harness.logger, 100.0)
        harness.logger.write_summary_and_reset()
        _log_minimal(harness.logger, 99.0)
        harness.close()

        self.assertEqual(len(_primary_rows(harness.compact.value())), 2)


class FlushCadenceTests(unittest.TestCase):
    def test_each_accepted_primary_row_is_flushed_by_stream_worker(self):
        harness = _make_logger(headers=False)

        _log_minimal(harness.logger, 1.0, rate_gate=True)
        harness.logger.drain()
        self.assertEqual(harness.compact.flush_count, 1)
        harness.close()

    def test_non_snap_debug_events_flush_every_sixty_four_entries(self):
        debug = _TrackingFile()
        streams = NavigationLogStreams(None, debug)
        events = NavigationEventRecorder(
            streams,
            lambda: "12:00:00.000",
            NavigationLogger.DEBUG_EVENT_FLUSH_INTERVAL,
        )

        for index in range(NavigationLogger.DEBUG_EVENT_FLUSH_INTERVAL - 1):
            events.record(LogEvent.ABORT, {"reason": f"test-{index}"})
        streams.drain()
        self.assertEqual(debug.flush_count, 0)

        events.record(LogEvent.ABORT, {"reason": "flush"})
        streams.drain()
        self.assertEqual(debug.flush_count, 1)
        streams.close()

    def test_snap_flushes_debug_then_compact_immediately(self):
        compact = _TrackingFile()
        debug = _TrackingFile()
        streams = NavigationLogStreams(compact, debug)
        events = NavigationEventRecorder(
            streams,
            lambda: "12:00:00.000",
            NavigationLogger.DEBUG_EVENT_FLUSH_INTERVAL,
        )

        events.record(
            LogEvent.SNAP,
            {
                "algorithm": "pn",
                "snap": "3d=1.0(h=0.1; v=1.0)",
                "kp_str": "kp=1.00",
            },
        )

        self.assertEqual(debug.flush_count, 1)
        self.assertEqual(compact.flush_count, 1)
        self.assertIn("SNAP(PN)", compact.value())
        streams.close()

    def test_snap_components_flush_debug_immediately(self):
        debug = _TrackingFile()
        streams = NavigationLogStreams(None, debug)
        events = NavigationEventRecorder(
            streams,
            lambda: "12:00:00.000",
            NavigationLogger.DEBUG_EVENT_FLUSH_INTERVAL,
        )

        events.record(
            LogEvent.SNAP_COMPONENTS,
            {"algorithm": "VISION-NAV-PN", "lateral_m": "0.001000"},
        )

        self.assertEqual(debug.flush_count, 1)
        self.assertIn("EVENT:SNAP_COMPONENTS", debug.value())
        streams.close()


class StreamResourceSafetyTests(unittest.TestCase):
    def test_logger_constructor_closes_streams_when_writer_start_fails(self):
        compact = _TrackingFile()
        debug = _TrackingFile()
        streams = NavigationLogStreams(compact, debug)
        failure = RuntimeError("thread unavailable")

        with patch.object(threading.Thread, "start", side_effect=failure):
            with self.assertRaisesRegex(RuntimeError, "thread unavailable"):
                NavigationLogger(1, Mock(), streams=streams)

        self.assertTrue(compact.closed)
        self.assertTrue(debug.closed)

    def test_async_primary_error_surfaces_and_close_cleans_every_handle(self):
        compact = _FirstFlushFailingFile()
        debug = _TrackingFile()
        streams = NavigationLogStreams(compact, debug)
        streams.write_primary("row\n")

        with self.assertRaisesRegex(OSError, "primary flush failed"):
            streams.drain()
        with self.assertRaisesRegex(OSError, "primary flush failed"):
            streams.close()

        self.assertTrue(compact.closed)
        self.assertTrue(debug.closed)

    def test_close_attempts_every_handle_when_compact_flush_fails(self):
        compact = _FlushFailingFile()
        debug = _TrackingFile()
        streams = NavigationLogStreams(compact, debug)

        with self.assertRaisesRegex(OSError, "flush failed"):
            streams.close()

        self.assertTrue(compact.closed)
        self.assertTrue(debug.closed)
        self.assertFalse(streams.has_compact())
        streams.close()
        self.assertEqual(compact.close_count, 1)
        self.assertEqual(debug.close_count, 1)

    def test_header_flush_failure_closes_the_just_opened_handle(self):
        compact = _FlushFailingFile()
        debug = _TrackingFile()
        cache = Mock()
        with tempfile.TemporaryDirectory() as tmp:
            cache.log_path = tmp
            with patch("builtins.open", side_effect=(compact, debug)):
                streams = open_navigation_log_streams(4, cache)

        self.assertTrue(compact.closed)
        self.assertEqual(compact.close_count, 1)
        self.assertFalse(streams.has_compact())
        streams.close()
        self.assertTrue(debug.closed)


class SnapByteStabilityTests(unittest.TestCase):
    def test_snap_event_row_matches_legacy_format(self):
        compact = _TrackingFile()
        # NavigationLogStreams starts a NavigationStreamWorker daemon on first write;
        # close it so the worker thread does not outlive the test.
        streams = NavigationLogStreams(compact, _TrackingFile())
        self.addCleanup(streams.close)
        events = NavigationEventRecorder(
            streams,
            lambda: "23:45:01.234",
            NavigationLogger.DEBUG_EVENT_FLUSH_INTERVAL,
        )

        events.record(
            LogEvent.SNAP,
            {
                "algorithm": "pn",
                "snap": "3d=12.3(h=4.5; v=6.7)",
                "kp_str": "kp=1.23",
            },
        )

        self.assertEqual(
            compact.value().strip(),
            "23:45:01.234,SNAP(PN),3d=12.3(h=4.5; v=6.7),"
            "kp=1.23,,,,,,,",
        )

    def test_summary_writes_components_only_to_debug(self):
        harness = _make_logger(timestamp=lambda: "23:45:01.234")
        target = Location(40.0, 44.0, 100.0, is_absolute=True)
        start = _loc_from_ned(target, -10.0, 2.0, -3.0)
        end = _loc_from_ned(target, 10.0, 2.0, -3.0)

        harness.logger.sample_snap(start, target)
        harness.logger.sample_snap(end, target)
        returned = harness.logger.write_summary_and_reset(
            algorithm="vision-nav-pn",
            kp=1.23,
        )
        harness.close()

        compact = harness.compact.value()
        debug = harness.debug.value()
        self.assertIn("SNAP(VISION-NAV-PN)", compact)
        self.assertNotIn("EVENT:SNAP_COMPONENTS", compact)
        component_lines = [
            line
            for line in debug.splitlines()
            if ",EVENT:SNAP_COMPONENTS," in line
        ]
        self.assertEqual(len(component_lines), 1)
        cells = component_lines[0].split(",")
        payload = dict(
            part.split("=", 1)
            for part in cells[2].split(";")
            if "=" in part
        )
        self.assertAlmostEqual(float(payload["lateral_m"]), 2.0, delta=1e-5)
        self.assertAlmostEqual(float(payload["longitudinal_m"]), 0.0, delta=1e-5)
        self.assertAlmostEqual(float(payload["vertical_m"]), 3.0, delta=1e-5)
        self.assertEqual(len(cells), len(LEGACY_PRIMARY_FIELDS))
        self.assertAlmostEqual(returned.component_lateral, 2.0, delta=0.02)
        self.assertEqual(harness.logger.get_snap().dist, float("inf"))


class PrimaryRowByteStabilityTests(unittest.TestCase):
    def test_primary_row_marks_missing_detected_target_distance_unavailable(self):
        harness = _make_logger(timestamp=lambda: "12:34:56.789")

        harness.logger.log(
            c_loc=Location(40.0, 44.0, 500.0, is_absolute=True),
            t_loc=Location(40.0, 44.0, 0.0, is_absolute=True),
            distance=123.4,
            cmd_roll=5.4,
            cmd_pitch=-7.8,
            yaw_error=1.1,
            pitch_error=-2.2,
            actual_roll=3.3,
            actual_pitch=-4.4,
            x_error=42,
            y_error=-11,
        )
        harness.close()

        self.assertEqual(
            _primary_rows(harness.compact.value())[0],
            "12:34:56.789,123.4,inf,inf,5.4,-7.8,1.1,-2.2,"
            "3.3,-4.4,918,551,",
        )

    def test_missing_or_nonfinite_attitude_is_na_and_empty_csv(self):
        harness = _make_logger(timestamp=lambda: "12:34:56.789")

        harness.logger.log(
            c_loc=Location(40.0, 44.0, 500.0, is_absolute=True),
            t_loc=Location(40.0, 44.0, 0.0, is_absolute=True),
            distance=123.4,
            cmd_roll=5.4,
            cmd_pitch=-7.8,
            yaw_error=1.1,
            pitch_error=-2.2,
            actual_roll=None,
            actual_pitch=float("nan"),
            x_error=42,
            y_error=-11,
        )
        harness.close()

        row = _primary_rows(harness.compact.value())[0].split(",")
        self.assertEqual(row[8:10], ["", ""])
        status_text = " ".join(
            str(call.args[0])
            for call in harness.cache.info.call_args_list
            if call.args
        )
        self.assertIn("ar: N/A", status_text)
        self.assertIn("ap: N/A", status_text)

    def test_source_time_column_is_diagnostic_only(self):
        harness = _make_logger(
            timestamp=lambda: "12:34:56.789",
            time_source=lambda: 4321.5,
        )

        _log_minimal(harness.logger, 123.4)
        harness.close()

        row = _primary_rows(harness.compact.value())[0]
        self.assertTrue(row.startswith("12:34:56.789,"))
        self.assertEqual(row.split(",")[-1], "4321.500")

    def test_broken_source_time_blanks_column_without_failing(self):
        def exploding() -> float:
            raise RuntimeError("source clock gone")

        harness = _make_logger(
            timestamp=lambda: "00:00:01.000",
            time_source=exploding,
        )

        _log_minimal(harness.logger, 1.0)
        harness.close()

        self.assertEqual(
            _primary_rows(harness.compact.value())[0].split(",")[-1],
            "",
        )


if __name__ == "__main__":
    unittest.main()
