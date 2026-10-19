"""One serial-step SITL in lockstep with a scripted operator (step 7).

Infrastructure check, not a mission eval: two runs of the same script must
give the same journal digest, also with seeded host delays (relay and
coordinator) and seeded firmware helper-thread jitter, and with zero
out-of-loop clock advances. Artifacts per run directory:

- ``lockstep_journal.csv``: per tick, ``source_us``, frame and reply sha256
  and the hash chain;
- ``lockstep_steps.json``: the operator's script steps with their ticks;
- ``lockstep_digest.txt``: the final chain;
- ``lockstep_sensors.csv``: serial1's baro and airspeed samples per tick
  (``SCALED_PRESSURE``, ``VFR_HUD``), the step-2 proof of
  ``deterministic_dev_sim.md`` (``--rng-seed`` sets ``NAVPY_STEP_RNG_SEED``);
- ``lockstep_stats_host.json``: wall cost per tick and process records.

Usage: python scripts/eval_lockstep_single.py --run-dir DIR [--ticks N]
       [--host-delay-seed N] [--helper-jitter-seed N] [--seed N] [--rng-seed N]
"""

from __future__ import annotations

import argparse
import json
import os
import secrets
import statistics
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as registry  # noqa: E402
from navpy.sim.lockstep.coordinator import StepLink, run  # noqa: E402
from navpy.sim.lockstep.mavlink_wire import StreamDecoder  # noqa: E402
from navpy.sim.lockstep.scripted_operator import ScriptedOperator  # noqa: E402
from navpy.sim.lockstep.step_wire import Frame  # noqa: E402
from scripts import lockstep_wsl, lockstep_wsl_spawn  # noqa: E402
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario  # noqa: E402
from scripts.eval_gcs_demo_sitl_root import terrain_tiles  # noqa: E402

SYSID = 1
#: Seconds of sim time flown after the operator's script finished.
FLY_AFTER_SCRIPT_S = 5.0
TICK_HZ = 50
#: Wall budget for one SITL frame (its exchange deadline) and the relay link.
STEP_DEADLINE_US = 30_000_000
CONNECT_TIMEOUT_S = 30.0
#: The noisy sensors serial1 reports: baro and airspeed pressure, and the airspeed.
SENSOR_FIELDS = {"SCALED_PRESSURE": ("press_abs", "press_diff", "temperature"), "VFR_HUD": ("airspeed",)}


class SensorTap:
    """The operator, plus a read-only decode of serial1's egress: each sensor field per tick."""

    def __init__(self, operator: ScriptedOperator) -> None:
        self._operator = operator
        self._decoder = StreamDecoder()
        self.rows: list[tuple[int, int, str, float]] = []

    def on_tick(self, frame: Frame) -> tuple[bytes, bytes]:
        for message in self._decoder.feed(frame.ports[1].egress):
            for name in SENSOR_FIELDS.get(message.get_type(), ()):
                self.rows.append((frame.tick, frame.source_us, f"{message.get_type()}.{name}",
                                  float(getattr(message, name))))
        return self._operator.on_tick(frame)

    def write_csv(self, path: Path) -> None:
        lines = ["tick,source_us,field,value", *(f"{t},{us},{f},{v!r}" for t, us, f, v in self.rows)]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def claim_slot(owner: str) -> int:
    """An eval-band slot this process keeps alive (its pid is the run's owner)."""
    if registry.find_for_owner(owner, label="lockstep") is not None:
        raise RuntimeError("a lockstep run from this directory is already active")
    slot = int(registry.claim(label="lockstep", clone=str(ROOT), owner=owner,
                              lo=ip.EVAL_CHAT_MIN, launcher_pid=os.getpid())["chat_index"])
    registry.record_pids(slot, sitl=True, sitl_pid=os.getpid())
    return slot


def release_slot(slot: int, owner: str) -> None:
    """Release only an entry that is still this run's."""
    entry = registry.get(slot)
    if entry and entry.get("owner") == owner and entry.get("sitl_pid") == os.getpid():
        registry.release(slot)


def run_once(run_dir: Path, ticks: int, host_delay_seed: int, jitter_seed: int, seed: int,
             rng_seed: int = 0) -> dict:
    """``rng_seed`` 0 leaves ``NAVPY_STEP_RNG_SEED`` unset (the firmware's fixed noise)."""
    run_dir.mkdir(parents=True, exist_ok=False)
    owner = registry.owner_for(str(ROOT))
    slot = claim_slot(owner)
    record = run_dir / lockstep_wsl.PROCESSES_FILE
    instance = ip.VEHICLES_PER_CHAT * slot  # slot-derived: no shared SITL ports
    run_name = f"{run_dir.name}-{secrets.token_hex(4)}"
    home = load_scenario(CERTIFIED_SCENARIO_PATH).home
    operator = ScriptedOperator(SYSID, home)
    tap = SensorTap(operator)
    state: dict = {"root": None, "link": None}
    children: list[lockstep_wsl_spawn.WslChild] = []  # held: their output drains to *_host
    failure: BaseException | None = None
    try:
        state["root"] = root = lockstep_wsl.create_root(run_name, lockstep_wsl.merged_defaults())
        inst_dir = lockstep_wsl.prepare_instance(root, instance, terrain_tiles())
        relay_args = (
            f"python3 {lockstep_wsl.wsl_path(ROOT / 'scripts' / 'lockstep_relay.py')} "
            f"--dir {inst_dir} --port {ip.lockstep_relay_port(slot, 0)}"
            + (f" --delay-seed {host_delay_seed}" if host_delay_seed else "")
        )
        lockstep_wsl_spawn.spawn("relay", relay_args, inst_dir, record, held=children,
                                 ready="LOCKSTEP_RELAY_READY")
        state["link"] = link = StepLink(ip.lockstep_relay_port(slot, 0), timeout_s=CONNECT_TIMEOUT_S)
        sitl_script = (
            # The boot id comes from the run seed, never the slot, so journals
            # compare across slots.
            lockstep_wsl.sitl_environment(seed, STEP_DEADLINE_US, jitter_seed, rng_seed or None) + " "
            + lockstep_wsl.sitl_command(root, home, instance, SYSID)
        )
        lockstep_wsl_spawn.spawn("sitl", sitl_script, inst_dir, record, held=children)
        finished_at: list[int] = []

        def stop_when() -> bool:
            if operator.done and not finished_at:
                finished_at.append(operator.tick)
            return bool(finished_at) and operator.tick >= finished_at[0] + FLY_AFTER_SCRIPT_S * TICK_HZ

        started = time.perf_counter()
        journal, wall = run(link, tap, ticks=ticks,
                            host_delay_seed=host_delay_seed, stop_when=stop_when)
        elapsed = time.perf_counter() - started
        if not operator.done:
            raise RuntimeError(f"the operator script did not finish within {ticks} ticks")
    except BaseException as error:
        failure = error
    # Each cleanup stage runs whatever the others do; the first fault wins.
    stopped: list[str] = []
    for stage in (
        lambda: state["link"] and state["link"].close(),
        operator.close,
        lambda: stopped.extend(lockstep_wsl.stop_owned(record, state["root"]) if state["root"] else []),
        *(child.close for child in children),
        lambda: release_slot(slot, owner),
    ):
        try:
            stage()
        except BaseException as error:
            failure = failure or error
    if failure is not None:
        raise failure
    root = state["root"]
    journal.write_csv(run_dir / "lockstep_journal.csv")
    (run_dir / "lockstep_steps.json").write_text(
        json.dumps([asdict(step) for step in operator.steps], indent=2) + "\n", encoding="utf-8")
    (run_dir / "lockstep_digest.txt").write_text(journal.digest + "\n", encoding="utf-8")
    tap.write_csv(run_dir / "lockstep_sensors.csv")
    (run_dir / "lockstep_statustext.json").write_text(
        json.dumps([asdict(step) for step in operator.statustexts], indent=2) + "\n",
        encoding="utf-8")
    sim_s = (journal.rows[-1].source_us - journal.rows[0].source_us) / 1e6
    stats = {
        "ticks": len(journal.rows),
        "sim_s": sim_s,
        "wall_s": elapsed,
        "real_time_factor": sim_s / elapsed,
        "wall_ms_per_tick_median": statistics.median(wall) * 1e3,
        "wall_ms_per_tick_p95": sorted(wall)[int(len(wall) * 0.95)] * 1e3,
        "host_delay_seed": host_delay_seed,
        "helper_jitter_seed": jitter_seed,
        "seed": seed,
        "rng_seed": rng_seed,
        "slot": slot,
        "root": root,
        "stopped": stopped,
    }
    (run_dir / "lockstep_stats_host.json").write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
    return {"digest": journal.digest, "steps": [asdict(s) for s in operator.steps], **stats}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--ticks", type=int, default=12_000)
    parser.add_argument("--host-delay-seed", type=int, default=0)
    parser.add_argument("--helper-jitter-seed", type=int, default=0)
    parser.add_argument("--seed", type=int, default=1, help="the run seed (boot id)")
    parser.add_argument("--rng-seed", type=int, default=0,
                        help="NAVPY_STEP_RNG_SEED for the SITL's noise; 0 leaves it unset")
    args = parser.parse_args()
    result = run_once(args.run_dir, args.ticks, args.host_delay_seed,
                      args.helper_jitter_seed, args.seed, args.rng_seed)
    print(json.dumps({key: result[key] for key in (
        "digest", "ticks", "sim_s", "real_time_factor", "wall_ms_per_tick_median", "steps")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
