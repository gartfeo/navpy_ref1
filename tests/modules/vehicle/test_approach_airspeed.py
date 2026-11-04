"""Eligible airspeed (hitl_final_approach.md section 9, step 20).

The source (plugins/vehicle/ardupilot/airspeed_eligibility.py) admits only the own autopilot's selected pitot
sensor after an accepted 20 Hz claim, the pinned parameters and 60 healthy seconds at 18 Hz or more; any fault after
that revokes for good. The law reads only that source with integration timing (app/approach/law_airspeed.py), never
VFR_HUD, and the write guard sends nothing while the source is not admitted.

The live part at the end runs against pinned SITL when NAVPY_STEP20_SITL names its MAVLink endpoint; it covers the
four source states with the real firmware's flags and bits.
"""
from __future__ import annotations

import json
import math
import os
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20 import all as A
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_TYPE_GCS,
)

from navpy.app.approach.law_airspeed import (
    AirspeedIneligible,
    LawAirspeed,
    parse_airspeed_record,
    print_airspeed_record,
)
from navpy.core.perception.pipeline.real_detector_config import DetectorDependencies, DetectorPipelineConfig
from navpy.core.perception.pipeline.real_detector_runtime_composition import build_association
from navpy.plugins.vehicle.ardupilot import airspeed_attach
from navpy.plugins.vehicle.ardupilot.airspeed_attach import attach
from navpy.plugins.vehicle.ardupilot.airspeed_eligibility import AirspeedEligibility
from navpy.plugins.vehicle.ardupilot.airspeed_profile import (
    ADMITTED,
    ADMIT_WINDOW_S,
    DIFFERENTIAL_PRESSURE,
    PENDING,
    REVOKED,
    SET_MESSAGE_INTERVAL,
    SIMULATED_PROFILES,
    profile_for_mode,
)
from navpy.plugins.vehicle.ardupilot.inbound_router import (
    OWN_AUTOPILOT_ONLY_TYPES,
    InboundMessageRouter,
)
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState, PacketLossTracker
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.message_subscriptions import CallbackRegistry
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity

PROFILE = SIMULATED_PROFILES["pc-sitl"]
USING, UNHEALTHY = A.AIRSPEED_SENSOR_USING, A.AIRSPEED_SENSOR_UNHEALTHY
BITS = DIFFERENTIAL_PRESSURE
HZ = 20.0
CAS = 22.0
HUD = 25.0                     # a plausible VFR_HUD airspeed: never what the law reads
OWN = 1


def _ready(events: list | None = None) -> AirspeedEligibility:
    sink = [] if events is None else events
    source = AirspeedEligibility(PROFILE, sink.append)
    source.claim_sent(0.0)
    source.observe_ack(A.MAV_RESULT_ACCEPTED, 0.0)
    source.pins_read({"ARSPD_TYPE": 2.0, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 0.0}, 0.0)
    return source


def _feed(source: AirspeedEligibility, start_s: float, seconds: float, hz: float = HZ, flags: int = USING,
          status_bits: int = BITS, cas: float = CAS) -> float:
    """Good AIRSPEED at ``hz`` and SYS_STATUS at 10 Hz from ``start_s``; returns the last packet's time."""
    n = int(round(seconds * hz))
    t = start_s
    for i in range(n + 1):
        t = start_s + i / hz
        source.observe_airspeed(0, cas, flags, t)
        if i % max(1, int(round(hz / 10))) == 0:
            source.observe_status(status_bits, status_bits, status_bits, t)
    return t


def _admitted(events: list | None = None) -> tuple[AirspeedEligibility, float]:
    source = _ready(events)
    t = _feed(source, 1.0, ADMIT_WINDOW_S + 0.1)
    assert source.state == ADMITTED
    return source, t


def _revokes(events: list) -> list[str]:
    return [e["reason"] for e in events if e["event"] == "revoked"]


# -- admission ---------------------------------------------------------------------------------------------------
def test_sixty_healthy_seconds_at_twenty_hz_admit_and_the_law_reads_the_pitot():
    source = _ready()
    t = _feed(source, 1.0, ADMIT_WINDOW_S - 1.0)
    assert source.state == PENDING and source.read(t) is None
    t = _feed(source, t + 1 / HZ, 1.2)
    assert source.state == ADMITTED
    assert source.read(t) == CAS


def test_below_eighteen_hz_never_admits():
    source = _ready()
    t = _feed(source, 1.0, 120.0, hz=17.0)
    assert source.state == PENDING and source.read(t) is None


def test_the_window_opens_only_after_the_claim_ack_and_the_pins():
    source = AirspeedEligibility(PROFILE)
    source.claim_sent(0.0)
    _feed(source, 1.0, 70.0)
    assert source.state == PENDING
    source.observe_ack(A.MAV_RESULT_ACCEPTED, 71.0)
    source.pins_read({"ARSPD_TYPE": 2, "ARSPD_USE": 1, "ARSPD_PRIMARY": 0}, 71.0)
    t = _feed(source, 71.05, 59.0)
    assert source.state == PENDING          # the 70 s before the ack and pins do not count
    _feed(source, t + 1 / HZ, 1.5)
    assert source.state == ADMITTED


@pytest.mark.parametrize("result", [A.MAV_RESULT_DENIED, A.MAV_RESULT_UNSUPPORTED, A.MAV_RESULT_FAILED])
def test_a_refused_claim_revokes_for_good(result):
    source = AirspeedEligibility(PROFILE)
    source.claim_sent(0.0)
    source.observe_ack(result, 0.1)
    source.observe_ack(A.MAV_RESULT_ACCEPTED, 0.2)
    _feed(source, 1.0, 70.0)
    assert source.revoked == "claim_refused" and source.read(71.0) is None


def test_no_accepted_ack_by_the_deadline_revokes():
    source = AirspeedEligibility(PROFILE)
    source.claim_sent(0.0)
    source.ack_deadline(3.0)
    assert source.revoked == "claim_unacknowledged"


@pytest.mark.parametrize("values, reason", [
    ({"ARSPD_TYPE": 2.0, "ARSPD_USE": 0.0, "ARSPD_PRIMARY": 0.0}, "pin_mismatch"),
    ({"ARSPD_TYPE": 100.0, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 0.0}, "pin_mismatch"),   # the serial board's type
    ({"ARSPD_TYPE": 2.0, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 1.0}, "pin_mismatch"),
    ({"ARSPD_TYPE": 2.0, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": None}, "pin_unreadable"),
    ({"ARSPD_TYPE": 2.5, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 0.0}, "pin_mismatch"),
    ({"ARSPD_TYPE": float("nan"), "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 0.0}, "pin_mismatch"),
])
def test_pins_that_do_not_hold_revoke(values, reason):
    source = AirspeedEligibility(PROFILE)
    source.claim_sent(0.0)
    source.observe_ack(A.MAV_RESULT_ACCEPTED, 0.0)
    source.pins_read(values, 0.0)
    assert source.revoked == reason


@pytest.mark.parametrize("bad", [
    {"flags": USING | UNHEALTHY}, {"flags": 0}, {"status_bits": 0}, {"cas": float("nan")},
])
def test_a_fault_before_admission_restarts_the_window(bad):
    events: list = []
    source = _ready(events)
    t = _feed(source, 1.0, 30.0)
    t = _feed(source, t + 1 / HZ, 0.5, **bad)
    t = _feed(source, t + 1 / HZ, ADMIT_WINDOW_S - 1.0)
    assert source.state == PENDING            # 30 good seconds before the fault are forgotten
    assert any(e["event"] == "window_restart" for e in events)
    _feed(source, t + 1 / HZ, 1.5)
    assert source.state == ADMITTED


def test_a_gap_before_admission_restarts_the_window_from_the_packet_after_it():
    source = _ready()
    t = _feed(source, 1.0, 30.0)
    t = _feed(source, t + 0.5, ADMIT_WINDOW_S - 0.5)
    assert source.state == PENDING
    _feed(source, t + 1 / HZ, 0.6)
    assert source.state == ADMITTED



def test_airspeed_without_fresh_status_never_counts_toward_the_window():
    """Every counted packet needs fresh SYS_STATUS: 60 s of pitot alone, then a first status, admits nothing."""
    source = _ready()
    t = 1.0
    for i in range(int(ADMIT_WINDOW_S * HZ) + 20):
        t = 1.0 + i / HZ
        source.observe_airspeed(0, CAS, USING, t)
    source.observe_status(BITS, BITS, BITS, t + 0.01)
    source.observe_airspeed(0, CAS, USING, t + 0.02)
    assert source.state == PENDING
    t = _feed(source, t + 0.05, ADMIT_WINDOW_S - 1.0)
    assert source.state == PENDING
    _feed(source, t + 1 / HZ, 1.2)
    assert source.state == ADMITTED

# -- the four source states, after admission: revoke, no readmission, no law input ----------------------------------
@pytest.mark.parametrize("bad, reason", [
    ({"flags": USING | UNHEALTHY}, "sensor_unhealthy"),           # healthy -> unhealthy
    ({"flags": 0}, "sensor_not_used"),                            # disabled / not used (ARSPD_USE 0)
    ({"status_bits": 0}, "status_bad"),                           # SYS_STATUS no longer present/enabled/healthy
    ({"cas": float("inf")}, "sensor_value_invalid"),
])
def test_a_fault_after_admission_revokes_for_good(bad, reason):
    events: list = []
    source, t = _admitted(events)
    t = _feed(source, t + 1 / HZ, 0.2, **bad)
    t = _feed(source, t + 1 / HZ, 2 * ADMIT_WINDOW_S)
    assert source.state == REVOKED and _revokes(events) == [reason]
    assert source.read(t) is None


def test_stale_status_revokes_at_the_read():
    events: list = []
    source, t = _admitted(events)
    for i in range(1, 12):                              # AIRSPEED keeps coming, SYS_STATUS stops
        source.observe_airspeed(0, CAS, USING, t + i / HZ)
    assert source.read(t + 11 / HZ) is None
    assert _revokes(events) == ["status_stale"]
    _feed(source, t + 1.0, 2 * ADMIT_WINDOW_S)
    assert source.state == REVOKED


def test_stale_status_on_arrival_revokes():
    events: list = []
    source, t = _admitted(events)
    source.observe_status(BITS, BITS, BITS, t + 0.8)
    assert _revokes(events) == ["status_stale"]


@pytest.mark.parametrize("gap_s", [0.25, 5.0])
def test_a_stale_pitot_revokes_at_the_read_or_on_the_next_packet(gap_s):
    events: list = []
    source, t = _admitted(events)
    assert source.read(t + gap_s) is None
    source.observe_airspeed(0, CAS, USING, t + gap_s)
    assert source.state == REVOKED and _revokes(events) == ["sensor_stale"]


def test_another_sensor_in_use_revokes():
    events: list = []
    source, t = _admitted(events)
    source.observe_airspeed(1, CAS, 0, t + 0.01)
    assert source.state == ADMITTED                       # a second sensor not in use is only counted
    source.observe_airspeed(1, CAS, USING, t + 0.02)
    assert _revokes(events) == ["source_changed"]


def test_a_changed_pin_revokes():
    events: list = []
    source, t = _admitted(events)
    source.observe_param("ARSPD_PRIMARY", 0.0, t)
    assert source.state == ADMITTED
    source.observe_param("ARSPD_USE", 0.0, t)
    assert _revokes(events) == ["pin_changed"]


# -- the law-input guard: VFR_HUD is never read, no write without an admitted source --------------------------------
class _Actuator:
    def __init__(self) -> None:
        self.writes: list[tuple] = []

    def issue(self, roll_deg, pitch_deg, throttle):
        self.writes.append((roll_deg, pitch_deg, throttle))


def test_the_guard_sends_nothing_before_admission_or_after_a_revoke(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr("navpy.app.approach.law_airspeed.rt.perf_counter", lambda: clock.now)
    source = _ready()
    actuator = _Actuator()
    guarded = LawAirspeed(source).guard(actuator)
    with pytest.raises(AirspeedIneligible):
        guarded.issue(1.0, 2.0, None)
    clock.now = _feed(source, 1.0, ADMIT_WINDOW_S + 0.1)
    guarded.issue(1.0, 2.0, None)
    assert actuator.writes == [(1.0, 2.0, None)]
    source.observe_airspeed(0, CAS, USING | UNHEALTHY, clock.now + 0.05)
    clock.now = _feed(source, clock.now + 0.1, 70.0)
    for _ in range(5):
        with pytest.raises(AirspeedIneligible):
            guarded.issue(1.0, 2.0, None)
    assert actuator.writes == [(1.0, 2.0, None)] and guarded.refused == 6



def test_the_guard_holds_the_source_through_the_write_so_no_revoke_falls_between(monkeypatch):
    """The check and the send are one step: during the send the bus thread cannot revoke (it would wait), and a
    revoke that lands first leaves the send unsent."""
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr("navpy.app.approach.law_airspeed.rt.perf_counter", lambda: clock.now)
    source, clock.now = _admitted()
    held: list[bool] = []

    class _Probe(_Actuator):
        def issue(self, roll_deg, pitch_deg, throttle):
            free = source._lock.acquire(blocking=False)
            if free:
                source._lock.release()
            held.append(not free)
            super().issue(roll_deg, pitch_deg, throttle)

    actuator = _Probe()
    guarded = LawAirspeed(source).guard(actuator)
    guarded.issue(1.0, 2.0, None)
    assert held == [True] and actuator.writes == [(1.0, 2.0, None)]
    source.observe_airspeed(0, CAS, USING | UNHEALTHY, clock.now + 0.01)
    with pytest.raises(AirspeedIneligible):
        guarded.issue(1.0, 2.0, None)
    assert actuator.writes == [(1.0, 2.0, None)]

def test_a_plausible_hud_value_is_never_the_law_input():
    """Integration timing takes the eligible reader only: the vehicle's VFR_HUD airspeed is not read."""
    vehicle = SimpleNamespace(air_speed=HUD, attitude_sample=None, location=lambda _: None,
                              attitude_history=lambda: None, link_identity=None)
    mount = SimpleNamespace(capture_frame_state=lambda: None, gimbal_capture_history=lambda: None)
    deps = DetectorDependencies(vehicle, mount, MagicMock())
    source = _ready()
    reader = LawAirspeed(source).read
    integration = SimpleNamespace(config=SimpleNamespace(run_id="r"))
    timing = object()
    with pytest.raises(ValueError, match="go together"):
        DetectorPipelineConfig(integration_timing=timing)
    with pytest.raises(ValueError, match="go together"):
        DetectorPipelineConfig(law_airspeed=reader)
    pipeline = DetectorPipelineConfig(integration_timing=timing, law_airspeed=reader)
    builder = build_association(deps, pipeline, integration)
    assert builder._air_speed_reader is reader
    assert builder._air_speed_reader() is None              # pending: no airspeed, although HUD shows 25 m/s
    plain = build_association(deps, DetectorPipelineConfig(), None)
    assert plain._air_speed_reader() == HUD                  # outside HITL only


@pytest.mark.parametrize("mode", ["flight", "qualification", "hil", ""])
def test_flight_refuses_the_simulated_source_profile(mode):
    vehicle = MagicMock()
    with pytest.raises(ValueError, match="simulated pitot profile"):
        profile_for_mode(mode)
    with pytest.raises(ValueError):
        LawAirspeed.attach(vehicle, mode)
    assert vehicle.method_calls == []                        # nothing claimed or subscribed


def test_records_round_trip(capsys):
    print_airspeed_record({"event": "revoked", "reason": "sensor_unhealthy", "t_s": 1.5})
    line = capsys.readouterr().out.strip()
    assert parse_airspeed_record(line) == {"event": "revoked", "reason": "sensor_unhealthy", "t_s": 1.5}
    assert parse_airspeed_record("NAVPY_AIRSPEED_V1 [1]") is None
    assert parse_airspeed_record("other") is None


# -- attach: claim, ack, pins, own-autopilot callbacks ------------------------------------------------------------
class _Msg:
    def __init__(self, system: int = OWN, component: int = MAV_COMP_ID_AUTOPILOT1, **fields) -> None:
        self._system, self._component = system, component
        self.__dict__.update(fields)

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


class _Vehicle:
    target_system = OWN

    def __init__(self, ack: int | None = A.MAV_RESULT_ACCEPTED, pins: dict | None = None) -> None:
        self.callbacks: dict[str, list] = {}
        self.commands: list[tuple] = []
        self._ack = ack
        self._pins = {"ARSPD_TYPE": 2.0, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 0.0} if pins is None else pins

    def on_message(self, name, callback):
        self.callbacks.setdefault(name, []).append(callback)

    def deliver(self, name, message):
        for callback in self.callbacks.get(name, []):
            callback(message)

    def send_command_long(self, command, **parameters):
        self.commands.append((command, parameters))
        if self._ack is not None:
            self.deliver("COMMAND_ACK", _Msg(system=OWN + 1, command=command, result=A.MAV_RESULT_DENIED))
            self.deliver("COMMAND_ACK", _Msg(command=command, result=self._ack))

    def get_parameter_fresh(self, name, timeout=2.0):
        value = self._pins[name]
        if isinstance(value, Exception):
            raise value
        return value


def test_attach_claims_twenty_hz_and_reads_the_pins():
    vehicle = _Vehicle()
    events: list = []
    source = attach(vehicle, PROFILE, events.append)
    assert vehicle.commands == [(SET_MESSAGE_INTERVAL, {"p1": 295, "p2": 50_000})]
    assert source.state == PENDING and source.claim_acknowledged       # the peer's refusal was not heard
    assert [e["event"] for e in events] == ["claim", "claim_ack", "pins"]


def test_attach_without_an_ack_revokes(monkeypatch):
    monkeypatch.setattr(airspeed_attach, "CLAIM_ACK_TIMEOUT_S", 0.01)
    assert attach(_Vehicle(ack=None), PROFILE, lambda r: None).revoked == "claim_unacknowledged"


def test_attach_with_a_refused_ack_or_an_unreadable_pin_revokes():
    assert attach(_Vehicle(ack=A.MAV_RESULT_DENIED), PROFILE, lambda r: None).revoked == "claim_refused"
    pins = {"ARSPD_TYPE": 2.0, "ARSPD_USE": RuntimeError("timeout"), "ARSPD_PRIMARY": 0.0}
    assert attach(_Vehicle(pins=pins), PROFILE, lambda r: None).revoked == "pin_unreadable"


def test_attach_hears_only_the_own_autopilot(monkeypatch):
    vehicle = _Vehicle()
    source = attach(vehicle, PROFILE, lambda r: None)
    unhealthy = dict(id=0, airspeed=CAS, flags=USING | UNHEALTHY)
    vehicle.deliver("AIRSPEED", _Msg(system=OWN + 1, **unhealthy))
    vehicle.deliver("AIRSPEED", _Msg(component=MAV_COMP_ID_ONBOARD_COMPUTER, **unhealthy))
    vehicle.deliver("PARAM_VALUE", _Msg(system=OWN + 1, param_id="ARSPD_USE", param_value=0.0))
    assert source.counts["airspeed"] == 0 and source.state == PENDING
    vehicle.deliver("AIRSPEED", _Msg(id=0, airspeed=CAS, flags=USING))
    vehicle.deliver("SYS_STATUS", _Msg(onboard_control_sensors_present=BITS, onboard_control_sensors_enabled=BITS,
                                        onboard_control_sensors_health=BITS))
    assert source.counts["airspeed"] == 1 and source.counts["status"] == 1
    vehicle.deliver("PARAM_VALUE", _Msg(param_id="ARSPD_USE", param_value=0.0))
    assert source.revoked == "pin_changed"


# -- the inbound router admits AIRSPEED and SYS_STATUS only from the own autopilot ---------------------------------
def _router() -> tuple[InboundMessageRouter, CallbackRegistry]:
    callbacks = CallbackRegistry()
    router = InboundMessageRouter(
        VehicleIdentity(OWN, OWN, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_TYPE_GCS),
        MessageStore({"AIRSPEED": 4, "SYS_STATUS": 4}), callbacks, ParameterRepository(),
        HeartbeatState(OWN, 3.0), PacketLossTracker(), MissionInbox(), SimpleNamespace(value=MagicMock()),
    )
    return router, callbacks


def _packet(name: str, system: int, component: int):
    mav = A.MAVLink(None, srcSystem=system, srcComponent=component)
    if name == "AIRSPEED":
        message = mav.airspeed_encode(0, CAS, 2500, 3.0, USING)
    else:
        message = mav.sys_status_encode(BITS, BITS, BITS, 0, 12000, -1, -1, 0, 0, 0, 0, 0, 0)
    return mav.parse_char(message.pack(mav))


@pytest.mark.parametrize("name", ["AIRSPEED", "SYS_STATUS"])
def test_the_router_admits_only_the_own_autopilot(name):
    assert name in OWN_AUTOPILOT_ONLY_TYPES
    router, callbacks = _router()
    heard: list = []
    callbacks.subscribe(name, heard.append)
    router.ingest(_packet(name, OWN + 1, MAV_COMP_ID_AUTOPILOT1))
    router.ingest(_packet(name, OWN, MAV_COMP_ID_ONBOARD_COMPUTER))
    router.ingest(_packet(name, OWN, MAV_COMP_ID_AUTOPILOT1))
    assert len(heard) == 1 and heard[0].get_srcSystem() == OWN


# -- live: pinned SITL, the real firmware's flags and bits ---------------------------------------------------------
LIVE = os.environ.get("NAVPY_STEP20_SITL", "")
live = pytest.mark.skipif(not LIVE, reason="set NAVPY_STEP20_SITL to the pinned SITL's MAVLink endpoint")


@live
def test_live_pinned_sitl_source_states():
    from tests.modules.vehicle.approach_airspeed_live import run_live_states

    result = run_live_states(LIVE)
    out = os.environ.get("NAVPY_STEP20_RESULT", "")
    if out:
        with open(out, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=1, default=str)
    assert result["admitted"], result
    for state in ("unhealthy", "not_used", "stale_status"):
        assert result[state]["revoked"], (state, result)
        assert result[state]["reads_after_revoke"] == 0, (state, result)
    assert result["hud"]["hud_airspeed"] is not None and math.isfinite(result["hud"]["hud_airspeed"])
    assert result["hud"]["law_read"] is None, result
