"""Standalone pixel-PN harnesses complete on dock contact (physical_docking.md).

Regression: the harnesses waited on `poi_passed_override()` but never armed a
contact sensor, so after the visual pass was removed every run timed out.
"""

from __future__ import annotations

import ast
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from navpy.core.approach.navigation_final_approach import (
    FinalApproachNavigationService,
)
from navpy.core.approach.navigation_mode import (
    ActiveNavigationMode,
    NavigationFinalApproachCapabilities,
    NavigationModeState,
)
from navpy.core.kernel.location import Location
from navpy.core.ports.subscription import Subscription
from navpy.sim.world.dock_contact_feed import POSITION_MESSAGE
from scripts.pixel_pn_dock_contact import HarnessDockContact, recording_dispatch
from tests.detection_factory import make_detected_poi
from tests.modules.navigation.test_navigation import _runtime

ROOT = Path(__file__).resolve().parents[2]
POI = Location(40.0, 44.0, 120.0, is_absolute=True)
M_PER_DEG_LAT = 111_132.0


class _Vehicle:
    target_system = 7

    def __init__(self) -> None:
        self.callbacks = {}

    def on_message(self, name, callback):
        self.callbacks[name] = callback
        return Subscription(lambda: self.callbacks.pop(name))

    def position(self, boot_ms: int, north_m: float) -> None:
        self.callbacks[POSITION_MESSAGE](SimpleNamespace(
            time_boot_ms=boot_ms,
            lat=int(round((POI.lat + north_m / M_PER_DEG_LAT) * 1e7)),
            lon=int(round(POI.lng * 1e7)),
            alt=int(round(POI.alt * 1e3)),
            vx=2500, vy=0, vz=0,
        ))


def _service():
    runtime, _, _, _ = _runtime()
    mode_state = NavigationModeState()
    mode_state.install_initialized(
        ActiveNavigationMode(
            spec=Mock(),
            runtime=runtime,
            final_approach=NavigationFinalApproachCapabilities(
                status=runtime.session, confirmation=Mock(),
            ),
        ),
        expected_previous=None,
    )
    return FinalApproachNavigationService(mode_state)


def test_harness_completes_on_contact_with_its_poi():
    vehicle = _Vehicle()
    service = _service()
    contact = HarnessDockContact(vehicle, POI, service, Mock())
    contact.arm(make_detected_poi(obj_id=2, task_id=1))

    vehicle.position(10_000, -0.3)
    assert service.poi_passed_override() is False
    vehicle.position(10_025, 0.3)  # 25 ms at 25 m/s: through the dock
    assert service.poi_passed_override() is True

    contact.close()
    assert vehicle.callbacks == {}


def test_harness_flying_past_the_net_never_completes():
    vehicle = _Vehicle()
    service = _service()
    contact = HarnessDockContact(vehicle, POI, service, Mock())
    contact.arm(make_detected_poi(obj_id=2, task_id=1))
    for index in range(10):
        vehicle.callbacks[POSITION_MESSAGE](SimpleNamespace(
            time_boot_ms=10_000 + index * 25,
            lat=int(round((POI.lat + (-1.0 + index * 0.25) / M_PER_DEG_LAT) * 1e7)),
            lon=int(round((POI.lng + 0.6 / (111_320.0 * math.cos(math.radians(40.0)))) * 1e7)),
            alt=int(round(POI.alt * 1e3)),
            vx=1000, vy=0, vz=0,
        ))
    assert service.poi_passed_override() is False


def test_recording_dispatch_arms_once_after_the_first_recorded_frame():
    final_approach = Mock()
    final_approach.record_confirmed_detection.side_effect = [False, True]
    nav = Mock(return_value=True)
    contact = Mock()
    dispatch = recording_dispatch(final_approach, nav, contact)
    first, second, third = (make_detected_poi(obj_id=2, task_id=1) for _ in range(3))

    assert dispatch(first) is False
    contact.arm.assert_not_called()
    assert dispatch(second) is True
    assert dispatch(third) is True

    contact.arm.assert_called_once_with(second)
    assert final_approach.record_confirmed_detection.call_count == 2
    assert nav.call_count == 2


def _calls(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.func.id for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }


def test_both_pixel_pn_children_wire_the_contact_sensor():
    direct = _calls(ROOT / "scripts/direct_pixel_pn_child.py")
    siyi = _calls(ROOT / "scripts/siyi_pixel_pn_child.py")
    assert {"HarnessDockContact", "recording_dispatch"} <= direct
    assert "HarnessDockContact" in siyi
