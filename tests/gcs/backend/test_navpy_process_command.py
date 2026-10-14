"""NavPy's command line: the wall launch is unchanged; lockstep differs in the stop only."""

from __future__ import annotations

import sys

from gcs.backend.navpy_process_launch import _build_navpy_command, lockstep_navpy_argv


def test_the_wall_command_line_is_unchanged():
    command = _build_navpy_command(
        2, "udpin:127.0.0.1:14560", detector_debug_show=True, vision_profile=" siyi_zr10 ",
        log_level="debug", navigation_speedup=4.0,
    )
    assert command == [
        sys.executable, "-m", "navpy.main", "-c", "udpin:127.0.0.1:14560", "-ss", "2",
        "-nt", "mav", "--detector-type", "sim", "--stop-on-stdin", "-gsu", "4.0",
        "-lsd", "Vehicle", "Network", "--vision-profile", "siyi_zr10", "--log-level", "DEBUG",
        "--detector-debug-show",
    ]


def test_a_lockstep_companion_stops_on_its_lifecycle_record_and_never_speeds_up():
    wall = _build_navpy_command(1, "lockstep:serial0.1", detector_debug_show=False,
                                vision_profile="siyi_zr10", log_level=None, navigation_speedup=0.0)
    lockstep = lockstep_navpy_argv(1, "lockstep:serial0.1", vision_profile="siyi_zr10")
    assert lockstep == [
        "--stop-on-lifecycle" if arg == "--stop-on-stdin" else arg for arg in wall[3:]
    ]
