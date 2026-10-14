"""Doubles for the 9e backend participant runs.

:class:`AutopilotSitl` plays the relay side of one serial-step SITL whose
serial ports carry the reference autopilot's bytes: frame k's egress is
what the autopilot wrote in loop k (after the ingress of reply k-1), so it
answers one tick later, as the firmware does at best. Frames and replies
strictly alternate, as ``lockstep_relay.py`` copies them.

:func:`setup_script` is the scripted API actor: the operator's setup calls
for the slice fixture, through the lockstep ports on the operator's baton.
"""

from __future__ import annotations

import hashlib
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator, RunResult
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner
from navpy.sim.lockstep.coordinator_types import Manifest, RoleSpec, WallBudgets
from navpy.sim.lockstep.operator_baton import (
    BatonClock,
    LockstepApi,
    LockstepEventStream,
    OperatorBaton,
)
from navpy.sim.lockstep.mavlink_wire import StreamDecoder
from navpy.sim.lockstep.operator_tail import LockstepLogTail
from navpy.sim.lockstep.part_wire import Done, RequestId
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.step_wire import Frame, PortState, pack_frame, unpack_reply
from scripts.eval_gcs_demo_command_bounds import command_bounds_from_snapshot
from scripts.eval_gcs_demo_config import aas_params_for, build_settings
from scripts.eval_gcs_demo_constants import FINAL_APPROACH_ROLL_LIMIT_DEG
from scripts.eval_gcs_demo_lifecycle import connect_vehicles
from scripts.eval_gcs_demo_mission_upload import upload_scenario_missions
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_scenario import DemoScenario, load_scenario
from scripts.eval_gcs_demo_stack_ops import require_write_results, wait_for_vehicles
from tests.modules.lockstep.coordinator_doubles import PYTHON_PATH, REPO, AllowAll, HostDelays
from tests.modules.lockstep.lockstep_autopilot import SYS_ID, ReferenceAutopilot

SLICE_SCENARIO = REPO / "scripts" / "eval_lockstep_slice_scenario.json"
NAVPY, BACKEND = 1, 2
#: The toy companion's readiness payload (what a configured restart validates).
READY_PAYLOAD = {"mission_items": 6, "targ_wps": 4, "nav_last_wp": 5}
SERIAL0 = "lockstep:serial0"
BACKEND_HOOKS = "gcs.backend.lockstep.participant:backend"
SERIAL1 = "lockstep:serial1"
#: The command-bound parameters the operator reads by name under lockstep.
COMMAND_BOUND_NAMES = ("PTCH_LIM_MIN_DEG", "PTCH_LIM_MAX_DEG", "AAS_DEL_THR", "TRIM_THROTTLE")
BOUND_VALUES = {"PTCH_LIM_MIN_DEG": -25.0, "PTCH_LIM_MAX_DEG": 20.0, "AAS_DEL_THR": -1.0,
                "TRIM_THROTTLE": 45.0}


class AutopilotSitl:
    def __init__(self, autopilot: ReferenceAutopilot, *, period_us: int = 20_000,
                 quiet_from: int = 3, delays: HostDelays | None = None) -> None:
        self.autopilot = autopilot
        self._period_us = period_us
        self._quiet_from = quiet_from
        self._delays = delays or HostDelays(0)
        self._awaiting = False
        self._ingress: tuple[bytes, ...] = (b"", b"")
        self.k = 0
        self.rows: list[tuple[int, int, str]] = []
        self.serial1: list[tuple[int, bytes, bytes]] = []
        self.snapshot_rows: list[tuple[int, int, str]] | None = None
        self.stopped = False

    def recv_frame(self) -> bytes:
        if self.stopped:
            raise ConnectionError("relay closed")
        if self._awaiting:
            raise ConnectionError("the relay alternates: frame read before its reply")
        self._delays("frame")
        self.k += 1
        source_us = 1_000_000 + self.k * self._period_us
        egress = self.autopilot.step(source_us, list(self._ingress))
        frame = Frame((1, 0), self.k, source_us, 0, self.k >= self._quiet_from,
                      tuple(PortState(0, 0, 0, data) for data in egress))
        raw = pack_frame(frame)
        self.rows.append((self.k, source_us, hashlib.sha256(raw).hexdigest()))
        self.serial1.append((self.k, self._ingress[1], egress[1]))
        self._awaiting = True
        return raw

    def send_reply(self, reply: bytes) -> None:
        self._ingress = unpack_reply(reply, expected_tick=self.k)
        self._awaiting = False
        self._delays("reply")

    def snapshot(self, parked_tick: int) -> None:
        assert self.rows[-1][0] == parked_tick and self._awaiting
        self.snapshot_rows = list(self.rows)

    def stop(self) -> None:
        self.stopped = True


def stack_context(operator: OperatorBaton, scenario: DemoScenario) -> StackContext:
    return StackContext(
        api=LockstepApi(operator, BACKEND, error=RegressionError), sys_ids=(SYS_ID,),
        backend_tail=LockstepLogTail(operator, BACKEND), clock=BatonClock(operator), scenario=scenario,
        event_stream_factory=lambda _url: LockstepEventStream(operator, BACKEND),
        navigation_tail=lambda _sys_id: LockstepLogTail(operator, NAVPY), monitor_device=SERIAL1,
    )


def _received(events: LockstepEventStream) -> list[str]:
    """Every delivery queued so far (a zero wait never parks)."""
    received = []
    while True:
        try:
            received.append(events.receive(0.0))
        except TimeoutError:
            return received


def setup_script(scenario: DemoScenario) -> Callable[[OperatorBaton], dict]:
    """Connect, read the command bounds by name, write the AAS profile, upload."""

    def script(operator: OperatorBaton) -> dict:
        operator.wait_ready(BACKEND)
        context = stack_context(operator, scenario)
        api = context.api
        with context.event_stream_factory(api.websocket_url) as events:
            connect_vehicles(context)
            wait_for_vehicles(context, 30.0)
            names = ",".join(COMMAND_BOUND_NAMES)
            snapshot = api.get_json(f"/api/vehicles/{SYS_ID}/params/named?names={names}")
            bounds = command_bounds_from_snapshot(
                SYS_ID, snapshot, roll_limit_deg=FINAL_APPROACH_ROLL_LIMIT_DEG)
            aas = aas_params_for(scenario, SYS_ID, SYS_ID)
            require_write_results(api.put_json(f"/api/vehicles/{SYS_ID}/params", {"params": aas}),
                                  aas, "AAS")
            upload_scenario_missions(context)
            received = _received(events)
        return {"events": received, "bounds": repr(bounds), "aas": aas}

    return script


def manifest(scenario: DemoScenario, *, mission_budget_us: int = 0,
             companions: bool = False) -> Manifest:
    """No NavPy, so no terminal tick: the mission phase ends on its budget at
    the first mission tick, and finalization lasts until the operator returns.

    With ``companions``, the toy rover (NavPy-like, on ``lockstep:serial0``)
    is the backend's companion for the scenario's sysid, through its
    launcher port.
    """
    config: dict = {"settings": build_settings(scenario)}
    roles = {}
    if companions:
        config["companions"] = {str(SYS_ID): {"role": "navpy", "ordinal": NAVPY,
                                              "device": SERIAL0}}
        roles["navpy"] = RoleSpec("tests.modules.lockstep.toy_participants:rover", (SERIAL0,),
                                  {"ready_after_s": 0.06, "ready_payload": READY_PAYLOAD})
    roles["backend"] = RoleSpec(BACKEND_HOOKS, (SERIAL1,), config)
    return Manifest(
        seed=1, epoch_us=1_760_000_000_000_000, max_frame_gap_us=25_000, drain_period_us=20_000,
        mission_budget_us=mission_budget_us, roles=roles, initial=(("backend", BACKEND),),
        companion_roles=("navpy",) if companions else (),
    )


@dataclass
class BackendRun:
    coordinator: LockstepCoordinator
    sitl: AutopilotSitl
    operator: OperatorBaton
    #: Every ``DONE`` the backend sent, decoded, in tick order.
    dones: list[Done] = field(default_factory=list)

    def run(self) -> RunResult:
        return self.coordinator.run()

    def sent(self) -> list[tuple[int, str, object]]:
        """``(tick, type, message)`` of every message the backend wrote on serial1."""
        decoder = StreamDecoder()
        return [(done.k, message.get_type(), message) for done in self.dones
                for device, data in done.ports if device == SERIAL1
                for message in decoder.feed(data)]

    def response(self, request_id: RequestId) -> tuple[int, object]:
        return next((done.k, r.payload) for done in self.dones for r in done.responses
                    if r.id == request_id)

    def events(self, kind: str) -> list[tuple[int, dict]]:
        return [(done.k, event.payload) for done in self.dones for event in done.events
                if event.kind == kind]


def build(run_dir: Path, *, delay_seed: int = 0, script=None, autopilot=None,
          companions: bool = False) -> BackendRun:
    """``script(scenario)`` makes the operator's script (default: the setup calls)."""
    scenario = load_scenario(SLICE_SCENARIO)
    plan = manifest(scenario, companions=companions)
    delays = HostDelays(delay_seed)
    sitl = AutopilotSitl(autopilot or ReferenceAutopilot(parameters=BOUND_VALUES), delays=delays)
    run_dir.mkdir(parents=True, exist_ok=True)
    roster = Roster(plan, LocalSpawner(run_dir / "host", PYTHON_PATH), AllowAll(),
                    HostSidecar(run_dir / PROCESSES_HOST_FILE),
                    WallBudgets(register_s=120, done_s=120, exit_s=60))
    ids = RequestIds(0)
    operator = OperatorBaton(ids, (script or setup_script)(scenario))
    coordinator = LockstepCoordinator(plan, sitl, sitl, operator, roster, ids, run_dir / "host",
                                      delay=delays)
    run = BackendRun(coordinator, sitl, operator)
    receive = roster.receive

    def capture(member, k):
        done, raw = receive(member, k)
        run.dones.append(done)
        return done, raw

    roster.receive = capture
    return run


__all__ = ["AutopilotSitl", "BACKEND", "BackendRun", "SERIAL1", "build", "manifest", "setup_script"]
