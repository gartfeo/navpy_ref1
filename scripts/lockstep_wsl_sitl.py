"""Serial-step SITLs in one WSL root, each behind its relay (steps 9f-9h, 10d).

:class:`WslFleet` launches N SITLs for the coordinator
(``lockstep_fleet.md``, Fleet barrier, Launch): the run's root once
(``create_root``), every instance (``prepare_instance``), relays 1..N, then
SITLs 1..N back to back, SITL i as instance ``3*slot + i-1`` behind relay
port ``lockstep_relay_port(slot, i-1)``. Its :attr:`WslFleet.sitls` are the
coordinator's ``Sitl`` s (frames in, replies out; snapshot, stop). Every WSL
process is recorded in the run's process file and, under the registry lock,
as a child of the run's eval-band slot before it runs
(``lockstep_wsl_spawn.spawn``); a stop, of any one SITL or after any
failure, stops every recorded process by ownership. :class:`HostDelays`
injects seeded coordinator host delays (acceptance (c)); the same seed drives
the relays' delays and the firmware's helper-thread jitter.

The grid constants are the lockstep manifest's: ``MAX_FRAME_GAP_US`` (Δ_max)
is set from a recorded serial-step frame sequence
(``tests/fixtures/lockstep/serial_step_frames.csv``, checked by
``test_grid_check_recording.py``); ``BOOT_BUDGET_US`` from the 10d recording
of a 3-SITL boot (``tests/fixtures/lockstep/fleet_boot_latches.json``).
"""

from __future__ import annotations

import math
import random
import secrets
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry_lockstep as lockstep
from navpy.sim.lockstep.coordinator import StepLink
from navpy.sim.lockstep.coordinator_deadline import LAUNCH, ExchangeDeadline
from scripts import lockstep_relay, lockstep_wsl, lockstep_wsl_spawn

ROOT = Path(__file__).resolve().parents[1]
#: Δ_max: the largest ``source_us`` step between two serial-step frames. The
#: SITL steps its physics in whole 833 us frames (1200 Hz) and runs one
#: scheduler loop per frame exchange at 50 Hz, so a frame advances 19992 or
#: 20825 us (24 or 25 physics steps); the recording shows only those two.
#: Δ_max is the larger one: a 26-step gap or a stall fails the grid check.
MAX_FRAME_GAP_US = 20_825
#: Δ_drain: the synthetic drain tick period (the scheduler period).
DRAIN_PERIOD_US = 20_000
#: The latest ``helpers_quiet`` latch of the 10d recording, sim time after
#: frame 1 (``tests/fixtures/lockstep/fleet_boot_latches.json``): every SITL
#: of every recorded 3-SITL run (plain, host delays with helper jitter, two
#: slots) latched at frame 1. The firmware's helper gate holds each helper
#: thread before every frame, so the latch tick is a firmware fact, not a
#: host timing (step 7).
BOOT_LATCH_RECORDED_US = 0
#: Sim time allowed beyond the recorded latch: 250 scheduler loops, room for
#: a helper-jitter seed or a firmware rebuild that moves a helper's first
#: gate wait by a few loops. A boot that is still not quiet after it is
#: broken, and ends as ``CoordinatorFault("boot_barrier")`` within seconds of wall time.
BOOT_BUDGET_MARGIN_US = 5_000_000
#: The fleet manifest's ``boot_budget_us`` (``lockstep_fleet.md``, Boot barrier).
BOOT_BUDGET_US = BOOT_LATCH_RECORDED_US + BOOT_BUDGET_MARGIN_US
#: Wall budget for one SITL frame (its exchange deadline) and the relay link.
STEP_DEADLINE_US = 30_000_000
#: Wall time a reply may take from the coordinator's send to the firmware,
#: beyond the relay's injected delay: one loopback TCP hop to the relay and
#: its copy onto the SITL's socket, with room for a loaded host's scheduling.
REPLY_TRANSIT_ALLOWANCE_S = 0.5
#: The coordinator's exchange margin (``coordinator_deadline``): it ends each
#: tick's exchange this long before the earliest firmware reply deadline, so
#: the reply still arrives in time: the relay's largest injected delay plus
#: the transit allowance.
EXCHANGE_MARGIN_S = lockstep_relay.MAX_DELAY_S + REPLY_TRANSIT_ALLOWANCE_S
CONNECT_TIMEOUT_S = 30.0
#: Upper bound of one injected coordinator host delay (seconds).
MAX_DELAY_S = 0.004


class HostDelays:
    """Seeded coordinator host delays at named points; seed 0 is none."""

    def __init__(self, seed: int) -> None:
        self._random = random.Random(seed) if seed else None

    def __call__(self, _point: str = "") -> None:
        if self._random is not None:
            time.sleep(self._random.random() * MAX_DELAY_S)


@dataclass(frozen=True)
class FleetLaunch:
    """What a :class:`WslFleet` launches: its vehicles ``(sys_id, home)`` in
    SITL order, the run seed (the boot id), the host delay seed (relays and
    helper jitter; 0 is none), the root's inputs, read once per run and
    hashed in the input manifest (``lockstep_wsl.merged_defaults``,
    ``terrain_tiles``), and each SITL's noise seed by sysid
    (``sitl_models.sitl_rng_seeds``; a SITL without one keeps the fixed noise)."""

    vehicles: tuple[tuple[int, str], ...]
    seed: int
    delay_seed: int
    defaults: str
    tiles: Mapping[str, bytes]
    rng_seeds: Mapping[int, int] = field(default_factory=dict)


class FleetSitl:
    """SITL i of a :class:`WslFleet`: its link (once launched) and its host side."""

    def __init__(self, fleet: "WslFleet") -> None:
        self._fleet = fleet
        self.link: StepLink | None = None
        self.parked_tick: int | None = None

    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        return self.link.recv_frame(timeout_s)

    def send_reply(self, reply: bytes, timeout_s: float = math.inf) -> None:
        self.link.send_reply(reply, timeout_s)

    def snapshot(self, parked_tick: int) -> None:
        """Serial-step firmware writes no step CSV and BIN is off (owner Q6):
        the journal's frame hashes are the SITL's record; the parked tick is noted."""
        self.parked_tick = parked_tick

    def stop(self) -> None:
        """Stopping one SITL stops the fleet: every recorded process, by ownership."""
        self._fleet.stop()


class WslFleet:
    """N serial-step SITLs behind N relays in one WSL root, launched by the coordinator."""

    def __init__(self, run_dir: Path, slot: int, owner: dict, launch: FleetLaunch) -> None:
        if not 0 < len(launch.vehicles) <= ip.VEHICLES_PER_CHAT:
            raise ValueError(f"a fleet has 1..{ip.VEHICLES_PER_CHAT} SITLs, not {len(launch.vehicles)}")
        self._record = run_dir / lockstep_wsl.PROCESSES_FILE
        self._slot = slot
        self._owner = owner
        self._launch = launch
        self._run_name = f"{run_dir.name}-{secrets.token_hex(4)}"
        self.root: str | None = None
        #: Every wsl.exe from its start (spawn's held list); stop() closes them.
        self._children: list[lockstep_wsl_spawn.WslChild] = []
        self.sitls = tuple(FleetSitl(self) for _ in launch.vehicles)
        self._linux_stopped = False  # the WSL processes, by ownership (stop_owned)

    def instance(self, index: int) -> int:
        """SITL ``index`` (0-based)'s ``-I``: slot-derived, so no SITL ports are shared."""
        return ip.VEHICLES_PER_CHAT * self._slot + index

    def launch(self, deadline: ExchangeDeadline) -> None:
        """The coordinator's launch hook, after the initial participants registered.

        The root and every instance first, then relays 1..N (each connected),
        then SITLs 1..N. SITL 1's ``go`` anchors frame 1's exchange deadline
        and opens it; SITLs 2..N launch on that deadline (``LAUNCH``), since
        SITL 1's firmware deadline runs from its ``go``."""
        spec = self._launch
        self.root = lockstep_wsl.create_root(self._run_name, spec.defaults)
        dirs = [lockstep_wsl.prepare_instance(self.root, self.instance(index), spec.tiles)
                for index in range(len(spec.vehicles))]
        relay = lockstep_wsl.wsl_path(ROOT / "scripts" / "lockstep_relay.py")
        for index, (sitl, inst_dir) in enumerate(zip(self.sitls, dirs)):
            port = ip.lockstep_relay_port(self._slot, index)
            args = (f"python3 {relay} --dir {inst_dir} --port {port}"
                    + (f" --delay-seed {spec.delay_seed}" if spec.delay_seed else ""))
            lockstep_wsl_spawn.spawn(f"relay{index + 1}", args, inst_dir, self._record,
                                     held=self._children, admit=self._admit,
                                     ready="LOCKSTEP_RELAY_READY")
            sitl.link = StepLink(port, timeout_s=CONNECT_TIMEOUT_S)

        def first_go() -> None:
            deadline.anchor()
            deadline.open(1)

        def on_deadline(own_s: float) -> float:
            return deadline.budget(LAUNCH, own_s)

        for index, ((sys_id, home), inst_dir) in enumerate(zip(spec.vehicles, dirs)):
            # The boot id comes from the run seed, never the slot, so journals
            # compare across slots (the firmware makes it distinct per sysid).
            script = (lockstep_wsl.sitl_environment(spec.seed, STEP_DEADLINE_US, spec.delay_seed,
                                                    spec.rng_seeds.get(sys_id))
                      + " " + lockstep_wsl.sitl_command(self.root, home, self.instance(index), sys_id))
            try:
                # SITL 1 opens frame 1's deadline at its go; every wait of a
                # later SITL's spawn draws on it; on expiry spawn only releases (stop() reaps).
                lockstep_wsl_spawn.spawn(f"sitl{index + 1}", script, inst_dir, self._record,
                                         held=self._children, admit=self._admit,
                                         on_go=None if index else first_go,
                                         wait_budget=on_deadline if index else None)
            except Exception as error:
                deadline.raise_if_spent(LAUNCH, error)  # a wait cut by the deadline is its fault
                raise
            deadline.check(LAUNCH)

    def _admit(self, owned: lockstep_wsl.OwnedProcess) -> bool:
        return lockstep.record_child(self._slot, self._owner, {
            "kind": "wsl", "role": owned.role, "wsl_pid": owned.wsl_pid,
            "start_ticks": owned.start_ticks, "cwd": owned.cwd, "root": self.root,
        })

    def stop(self) -> None:
        """Stop every recorded Linux process by ownership (relays and SITLs),
        then close each child's host resources (``wsl.exe`` reaped, output
        sidecar finished). Idempotent and retryable: a failed ownership stop
        is tried again, and a child whose close failed stays held for the
        next call. The first failure is raised."""
        failures: list[BaseException] = []
        for sitl in self.sitls:
            if sitl.link is not None:
                try:
                    sitl.link.close()
                except Exception as error:  # noqa: BLE001 - raised after the ownership stop
                    failures.append(error)
                sitl.link = None
        if self.root is not None and not self._linux_stopped:
            try:
                lockstep_wsl.stop_owned(self._record, self.root)
                self._linux_stopped = True
            except BaseException as error:  # noqa: BLE001 - raised below; the next stop retries
                failures.append(error)
        if self._linux_stopped:  # a launcher only exits once its child is stopped
            for child in self._children:
                try:
                    child.close()
                except Exception as error:  # noqa: BLE001 - raised after the rest are tried
                    failures.append(error)
            self._children = [child for child in self._children if not child.closed]
        if failures:
            raise failures[0]


def exchange_deadline() -> ExchangeDeadline:
    """The serial-step firmware's exchange deadline rule: every lockstep run's roster draws on it."""
    return ExchangeDeadline(STEP_DEADLINE_US / 1_000_000, EXCHANGE_MARGIN_S)


__all__ = [
    "BOOT_BUDGET_MARGIN_US", "BOOT_BUDGET_US", "BOOT_LATCH_RECORDED_US", "CONNECT_TIMEOUT_S",
    "DRAIN_PERIOD_US", "EXCHANGE_MARGIN_S", "FleetLaunch", "FleetSitl", "HostDelays", "MAX_DELAY_S",
    "MAX_FRAME_GAP_US", "REPLY_TRANSIT_ALLOWANCE_S", "STEP_DEADLINE_US", "WslFleet",
    "exchange_deadline",
]
