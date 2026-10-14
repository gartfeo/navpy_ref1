"""One serial-step SITL in its own WSL root, behind its relay (steps 9f, 9g).

:class:`WslSitl` is the coordinator's ``SitlLink`` (frames in, replies out)
and ``SitlHost`` (snapshot, stop). Every WSL process is recorded in the
run's process file and, under the registry lock, as a child of the run's
eval-band slot before it runs (``lockstep_wsl_spawn.spawn``). :class:`HostDelays`
injects seeded coordinator host delays (acceptance (c)); the same seed drives
the relay's delays and the firmware's helper-thread jitter.

The grid constants are the lockstep manifest's: ``MAX_FRAME_GAP_US`` (Δ_max)
is set from a recorded serial-step frame sequence
(``tests/fixtures/lockstep/serial_step_frames.csv``, checked by
``test_grid_check_recording.py``).
"""

from __future__ import annotations

import math
import random
import secrets
import time
from collections.abc import Mapping
from pathlib import Path

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry_lockstep as lockstep
from navpy.sim.lockstep.coordinator import StepLink
from navpy.sim.lockstep.coordinator_deadline import ExchangeDeadline
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


class WslSitl:
    """One serial-step SITL behind its relay: the coordinator's ``SitlLink`` and ``SitlHost``."""

    def __init__(self, run_dir: Path, slot: int, owner: dict, *, seed: int, delay_seed: int,
                 defaults: str, tiles: Mapping[str, bytes]) -> None:
        # lockstep_wsl.merged_defaults() and terrain_tiles(), hashed in the input manifest
        self._defaults = defaults
        self._tiles = tiles
        self._record = run_dir / lockstep_wsl.PROCESSES_FILE
        self._slot = slot
        self._owner = owner
        self._seed = seed
        self._delay_seed = delay_seed
        self._instance = ip.VEHICLES_PER_CHAT * slot  # slot-derived: no shared SITL ports
        self._run_name = f"{run_dir.name}-{secrets.token_hex(4)}"
        self.root: str | None = None
        self._link: StepLink | None = None
        # Every wsl.exe from its start (spawn's held list); stop() closes them.
        self._children: list[lockstep_wsl_spawn.WslChild] = []
        self.parked_tick: int | None = None
        self._linux_stopped = False  # the WSL processes, by ownership (stop_owned)

    def start(self, home: str, sys_id: int) -> None:
        self.root = root = lockstep_wsl.prepare_root(self._run_name, self._instance, self._defaults,
                                                         self._tiles)
        inst_dir = f"{root}/{self._instance}"
        port = ip.lockstep_relay_port(self._slot, 0)
        relay_args = (
            f"python3 {lockstep_wsl.wsl_path(ROOT / 'scripts' / 'lockstep_relay.py')} "
            f"--dir {inst_dir} --port {port}"
            + (f" --delay-seed {self._delay_seed}" if self._delay_seed else "")
        )
        lockstep_wsl_spawn.spawn("relay", relay_args, inst_dir, self._record, held=self._children,
                                 admit=self._admit, ready="LOCKSTEP_RELAY_READY")
        self._link = StepLink(port, timeout_s=CONNECT_TIMEOUT_S)
        # The boot id comes from the run seed, never the slot, so journals
        # compare across slots. The host delay seed also seeds the firmware's
        # helper-thread jitter (acceptance (c)).
        sitl = (lockstep_wsl.sitl_environment(self._seed, STEP_DEADLINE_US, self._delay_seed) + " "
                + lockstep_wsl.sitl_command(root, home, self._instance, sys_id))
        lockstep_wsl_spawn.spawn("sitl", sitl, inst_dir, self._record, held=self._children,
                                 admit=self._admit)

    def _admit(self, owned: lockstep_wsl.OwnedProcess) -> bool:
        return lockstep.record_child(self._slot, self._owner, {
            "kind": "wsl", "role": owned.role, "wsl_pid": owned.wsl_pid,
            "start_ticks": owned.start_ticks, "cwd": owned.cwd, "root": self.root,
        })

    # -- SitlLink ------------------------------------------------------------------
    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        return self._link.recv_frame(timeout_s)

    def send_reply(self, reply: bytes, timeout_s: float = math.inf) -> None:
        self._link.send_reply(reply, timeout_s)

    # -- SitlHost ------------------------------------------------------------------
    def snapshot(self, parked_tick: int) -> None:
        """Serial-step firmware writes no step CSV and BIN is off (owner Q6):
        the journal's frame hashes are the SITL's record; the parked tick is noted."""
        self.parked_tick = parked_tick

    def stop(self) -> None:
        """Stop the Linux processes by ownership, then close each child's host
        resources (``wsl.exe`` reaped, output sidecar finished). Retryable: a
        failed ownership stop is tried again, and a child whose close failed
        stays held for the next call. The first failure is raised."""
        if self._link is not None:
            self._link.close()
            self._link = None
        if self.root is None:
            return
        failures: list[BaseException] = []
        if not self._linux_stopped:
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
    """The serial-step firmware's exchange deadline rule, for a fleet launch (step 10d)."""
    return ExchangeDeadline(STEP_DEADLINE_US / 1_000_000, EXCHANGE_MARGIN_S)


__all__ = [
    "CONNECT_TIMEOUT_S", "DRAIN_PERIOD_US", "EXCHANGE_MARGIN_S", "HostDelays", "MAX_DELAY_S",
    "MAX_FRAME_GAP_US", "REPLY_TRANSIT_ALLOWANCE_S", "STEP_DEADLINE_US", "WslSitl",
    "exchange_deadline",
]
