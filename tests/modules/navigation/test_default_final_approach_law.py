"""The default final-approach law must satisfy the Pure Vision Approach Constraint.

AAS_DEL_CTRL 0/1 select the legacy geo Roll-L1 laws, whose lateral command
reads aircraft compass yaw/heading and NED ground velocity. An unconfigured
vehicle (no CLI override, no MAVLink param) and every place that seeds the
parameter default must therefore select vision-nav-pn (2).
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from navpy.args.navigation_args import (
    DEFAULT_NAVIGATION_ALGORITHM,
    NAVIGATION_ALGORITHM_PARAM_BY_VALUE,
    NavigationAlgorithm,
    NavigationArgs,
)
from navpy.args.pid_args import PIDArgs
from navpy.modules.navigation import navigation_composition, navigation_law_builders
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw


_REPO = Path(__file__).resolve().parents[3]


class _ForbiddenCommandInput(AssertionError):
    pass


def _forbidden(name):
    def read(*_args, **_kwargs):
        raise _ForbiddenCommandInput(f"final-approach composition read {name}")

    return property(read)


class _PureVisionGuardVehicle:
    """Vehicle double whose command-forbidden inputs fail loudly on read."""

    target_system = 1
    source_system = 255
    min_pitch = -60.0
    max_pitch = 30.0

    heading = _forbidden("heading")
    velocity = _forbidden("velocity")
    ground_speed_ned = _forbidden("ground_speed_ned")
    altitude = _forbidden("altitude")
    relative_altitude = _forbidden("relative_altitude")
    attitude = _forbidden("attitude")

    def __init__(self):
        self.set_attitude = Mock()

    def location(self, *_args, **_kwargs):
        raise _ForbiddenCommandInput("final-approach composition read location()")

    def get_param_or_default(self, _name, default):
        return default

    def get_parameter(self, _name, quiet=False):
        return None


def _unconfigured_navigation_args(vehicle):
    parser = argparse.ArgumentParser()
    NavigationArgs.add_args(parser)
    PIDArgs.add_args(parser, "pitch")
    return NavigationArgs(parser.parse_args([]), vehicle, Mock())


def test_unconfigured_vehicle_composes_vision_nav_not_legacy_roll_l1():
    vehicle = _PureVisionGuardVehicle()
    args = _unconfigured_navigation_args(vehicle)

    with patch.object(
        navigation_law_builders,
        "compose_roll_l1_law",
        side_effect=AssertionError("legacy Roll-L1 law built by default"),
    ), patch.object(navigation_composition, "NavigationLogger"):
        composition = navigation_composition.compose_navigation(
            vehicle,
            geo_ref=Mock(),
            zc_util=None,
            mission_planner=Mock(),
            logger=Mock(),
            args=args,
        )

    active = composition.mode_state.snapshot()
    assert active.spec.algorithm is NavigationAlgorithm.VISION_NAV_PN
    # Only the vision runtime exposes final-approach capabilities; the
    # legacy geo runtime does not.
    assert active.final_approach is not None
    assert isinstance(active.runtime._session._command_reset._ports.law, VisionNavLaw)


@pytest.mark.parametrize(
    "path, pattern",
    [
        ("scripts/lua/aas_params.lua", r'"DEL_CTRL",\s*(\d+)\)'),
        ("src/gcs/frontend/src/utils/aasParams.js", r"\bdel_ctrl:\s*(\d+)"),
    ],
)
def test_every_seeded_default_selects_pure_vision(path, pattern):
    expected = NAVIGATION_ALGORITHM_PARAM_BY_VALUE[
        NavigationAlgorithm.VISION_NAV_PN.value
    ]
    assert DEFAULT_NAVIGATION_ALGORITHM is NavigationAlgorithm.VISION_NAV_PN
    assert NavigationArgs.PARAMS["AAS_DEL_CTRL"] == expected

    match = re.search(pattern, (_REPO / path).read_text(encoding="utf-8"))
    assert match is not None, path
    assert int(match.group(1)) == expected, path
