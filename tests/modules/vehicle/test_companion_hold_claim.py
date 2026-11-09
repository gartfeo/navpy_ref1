"""NavPy's claim on its own recovery LOITER, in its companion heartbeat (companion_hold_claim.py;
docs/design/navpy_recovery.md step 10, item 9; owner ruling 2026-11-09, "Yes, guard it").

The autopilot's watchdog (scripts/lua/nav_watchdog.lua) guards a LOITER only while NavPy's heartbeat claims it; the
Lua side is tested in tests/scripts/test_nav_watchdog_lua.py.
"""
from __future__ import annotations

import re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

from navpy.app.flow.nav_watchdog_composition import nav_recovery
from navpy.app.nav_recovery_link import nav_recovery_link
from navpy.plugins.vehicle.ardupilot.companion_hold_claim import NAVPY_HOLD_CLAIM, CompanionHoldClaim
from navpy.plugins.vehicle.ardupilot.heartbeat_runtime import HeartbeatRuntime
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.vehicle_protocol_composition import build_protocol_parts

LUA = Path(__file__).resolve().parents[3] / "scripts" / "lua" / "nav_watchdog.lua"


class Transport:
    def __init__(self) -> None:
        self.mav = MagicMock()

    def call(self, action):
        return action(SimpleNamespace(mav=self.mav))


def _runtime(claim: CompanionHoldClaim | None = None, pacing=None) -> tuple[HeartbeatRuntime, Transport]:
    transport = Transport()
    runtime = HeartbeatRuntime(transport, SimpleNamespace(mav_type=18, target_system=7), HeartbeatState(7, 3.0),
                               SimpleNamespace(value=MagicMock()), 4.0, pacing, claim)
    return runtime, transport


def _custom_modes(transport: Transport) -> list[int]:
    return [c.args[3] for c in transport.mav.heartbeat_send.call_args_list]


def test_the_heartbeat_carries_the_claim_in_custom_mode():
    claim = CompanionHoldClaim()
    runtime, transport = _runtime(claim)
    runtime._send()
    claim.set(True)
    runtime._send()
    claim.set(False)
    runtime._send()
    assert _custom_modes(transport) == [0, NAVPY_HOLD_CLAIM, 0]


def test_without_a_claim_the_heartbeat_claims_nothing():
    runtime, transport = _runtime()
    runtime._send()
    assert _custom_modes(transport) == [0]


def test_a_change_of_claim_wakes_the_heartbeat_once():
    """Sent at once, not a period later: the claim must reach the autopilot before NavPy can die holding."""
    pacing = Mock()
    claim = CompanionHoldClaim()
    _runtime(claim, pacing)
    claim.set(False)
    claim.set(True)
    claim.set(True)
    claim.set(False)
    assert pacing.wake.call_count == 2


def test_the_vehicle_heartbeat_sends_its_buss_claim():
    claim = CompanionHoldClaim()
    link = SimpleNamespace(transport=Transport(), identity=SimpleNamespace(target_system=7, mav_type=18),
                           logger_ref=SimpleNamespace(value=MagicMock()),
                           active_bus=SimpleNamespace(companion_hold=claim), bus_lease=MagicMock())
    state = SimpleNamespace(messages=MagicMock(), mission_inbox=MagicMock(), heartbeat_state=HeartbeatState(7, 3.0),
                            callbacks=MagicMock(), parameter_repository=ParameterRepository(), packet_loss=MagicMock())
    request = SimpleNamespace(link=SimpleNamespace(target_system=7),
                              startup=SimpleNamespace(stop_requested=None, heartbeat_hz=4.0))
    runtime = build_protocol_parts(request, SimpleNamespace(link=link, state=state)).heartbeat_runtime
    claim.set(True)
    runtime._send()
    assert _custom_modes(link.transport) == [NAVPY_HOLD_CLAIM]


def test_nav_recovery_claims_through_the_buss_claim():
    claim = CompanionHoldClaim()
    link = nav_recovery_link(MavCommandFence(), Mock(), claim)
    step = nav_recovery(Mock(), Mock(), Mock(), link)
    step.recovery._ports.claim_hold(True)
    assert claim.custom_mode == NAVPY_HOLD_CLAIM


def test_the_lua_watchdog_reads_the_same_bit():
    text = LUA.read_text(encoding="utf-8")
    assert int(re.search(r"^local HOLD_CLAIM = (\d+)", text, re.MULTILINE).group(1)) == NAVPY_HOLD_CLAIM
