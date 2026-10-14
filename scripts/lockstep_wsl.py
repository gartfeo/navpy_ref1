"""WSL side of a lockstep run: the run root, spawned processes, ownership.

Each run gets its own WSL root (``~/navpy-eval-roots/lockstep-<run>``) with a
copy of the serial-step firmware (checked against ``lockstep_firmware.lock``),
pre-boot defaults, ``aas_params.lua`` and the wall eval's pinned SITL terrain
tiles only. Every WSL process (relay, SITL) is recorded the moment it is
spawned, by WSL pid, process start time
and cwd under the root, in ``<run>/lockstep_processes_host.json``, and is
stopped only through those records (``docs/design/lockstep_contracts.md``,
Ownership). Spawning, and holding each child's ``wsl.exe`` handle and output,
is ``lockstep_wsl_spawn``.
"""

from __future__ import annotations

import json
import shlex
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from scripts.eval_gcs_demo_sitl_root import (
    SITL_DEFAULT_OVERRIDES,
    SITL_LUA_DIR,
    SITL_ROOTS,
    SITL_START_TIME_UTC,
    STOCK_DEFAULTS,
    merge_defaults,
    sha256_bytes,
)

ROOT = Path(__file__).resolve().parents[1]
PROC_HELPER = ROOT / "scripts" / "lockstep_wsl_proc.py"
FIRMWARE_LOCK = ROOT / "scripts" / "lockstep_firmware.lock.json"
PROCESSES_FILE = "lockstep_processes_host.json"
#: Lockstep-only defaults on top of the wall eval's (step 3): no BIN log,
#: scripting for aas_params.lua only (contract: Firmware helpers). Terrain is
#: on, as in the wall eval (the firmware default, stated): the SITL's ground
#: and ``TERRAIN_REPORT`` come from the pinned tiles :func:`prepare_root`
#: seeds; SITL reads them on its main thread (contract: Firmware helpers).
LOCKSTEP_DEFAULT_OVERRIDES = {
    **SITL_DEFAULT_OVERRIDES,
    "LOG_BACKEND_TYPE": 0.0,
    "TERRAIN_ENABLE": 1.0,
    "SCR_ENABLE": 1.0,
    "SIM_BAUDLIMIT_EN": 0.0,
}


@dataclass(frozen=True)
class OwnedProcess:
    role: str
    wsl_pid: int
    start_ticks: int  # /proc/<pid>/stat field 22
    cwd: str


def wsl(script: str, *, timeout: float = 60.0) -> str:
    result = subprocess.run(
        ["wsl.exe", "-e", "bash", "-c", script],
        capture_output=True, text=True, timeout=timeout,
    )
    if result.returncode != 0:
        raise RuntimeError(f"WSL command failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout


def wsl_path(path: Path) -> str:
    return wsl(f"wslpath -a {shlex.quote(str(path))}").strip()


def load_firmware_lock() -> dict:
    return json.loads(FIRMWARE_LOCK.read_text(encoding="utf-8"))


def merged_defaults() -> str:
    """The ``defaults.parm`` a lockstep SITL consumes: WSL's stock ``plane.parm``
    with :data:`LOCKSTEP_DEFAULT_OVERRIDES` merged in. Read once per run, so the
    input manifest hashes exactly what :func:`prepare_root` writes."""
    home = wsl("printf %s \"$HOME\"")
    stock = wsl(f"cat {shlex.quote(home + '/' + STOCK_DEFAULTS)}")
    return merge_defaults(stock, LOCKSTEP_DEFAULT_OVERRIDES)


def prepare_root(run_name: str, instance: int, defaults: str, tiles: Mapping[str, bytes]) -> str:
    """Create the run's WSL root and its instance directory with ``defaults``
    (:func:`merged_defaults`) as its ``defaults.parm`` and ``tiles``
    (``eval_gcs_demo_sitl_root.terrain_tiles``, the wall eval's pinned SITL
    terrain) in its ``terrain/``, where SITL looks (relative to its cwd);
    return the root. Both are read once per run and hashed in the input
    manifest, so it hashes exactly what is written here."""
    lock = load_firmware_lock()
    home = wsl("printf %s \"$HOME\"")
    root = f"{home}/{SITL_ROOTS}/lockstep-{run_name}"
    lua = (SITL_LUA_DIR / "aas_params.lua").read_bytes().replace(b"\r\n", b"\n")
    q = shlex.quote
    inst = f"{root}/{instance}"
    script = (
        f"set -e; test ! -e {q(root)}; mkdir -p {q(inst)}/scripts {q(root)}/bin; "
        f"cp {q(lock['binary_path'])} {q(root)}/bin/arduplane; "
        f"test \"$(sha256sum {q(root)}/bin/arduplane | cut -d' ' -f1)\" = {q(lock['binary_sha256'])}; "
        f"cat > {q(root)}/defaults.parm; "
    )
    subprocess.run(["wsl.exe", "-e", "bash", "-c", script], input=defaults.encode(),
                   check=True, capture_output=True, timeout=60)
    subprocess.run(["wsl.exe", "-e", "bash", "-c", f"cat > {q(inst)}/scripts/aas_params.lua"],
                   input=lua, check=True, capture_output=True, timeout=60)
    for name, data in sorted(tiles.items()):
        tile = f"{inst}/terrain/{name}"
        subprocess.run(
            ["wsl.exe", "-e", "bash", "-c",
             f"set -e; mkdir -p {q(inst)}/terrain; cat > {q(tile)}; "
             f"test \"$(sha256sum {q(tile)} | cut -d' ' -f1)\" = {q(sha256_bytes(data))}"],
            input=data, check=True, capture_output=True, timeout=60)
    return root


def proc(*args: object) -> str:
    """Run ``lockstep_wsl_proc.py`` in WSL (identity, stop, alive by record)."""
    helper = shlex.quote(wsl_path(PROC_HELPER))
    return wsl("python3 " + helper + " " + " ".join(shlex.quote(str(a)) for a in args)).strip()


def stop_owned(record: Path, root: str) -> list[str]:
    """Stop every recorded process that is still the recorded one; report each."""
    if not record.exists():
        return []
    owned_list = [OwnedProcess(**raw) for raw in json.loads(record.read_text())]
    for owned in owned_list:
        if not owned.cwd.startswith(root + "/") and owned.cwd != root:
            raise RuntimeError(f"recorded process outside the run root: {owned}")
    # Every record is tried whatever the others do; the failures are raised together.
    reports, failures = [], []
    for owned in owned_list:
        try:
            result = proc("stop", owned.wsl_pid, owned.start_ticks, owned.cwd)
        except Exception as error:  # noqa: BLE001 - reported after the rest are tried
            failures.append(f"{owned.role} {owned.wsl_pid}: {type(error).__name__}: {error}")
            continue
        reports.append(f"{owned.role} {owned.wsl_pid}: {result}")
    if failures:
        raise RuntimeError(f"could not stop {'; '.join(failures)} (stopped: {reports})")
    return reports


def stop_recorded(wsl_pid: int, start_ticks: int, cwd: str, root: str) -> str:
    """Stop one recorded WSL process if it is still the recorded one (``gcs_stop``)."""
    if not cwd.startswith(root + "/") and cwd != root:
        raise RuntimeError(f"recorded process outside the run root: {wsl_pid} {cwd}")
    return proc("stop", wsl_pid, start_ticks, cwd)


def recorded_alive(wsl_pid: int, start_ticks: int, cwd: str, root: str) -> bool:
    """A fresh probe: does the pid still name the recorded process?"""
    del root
    return proc("alive", wsl_pid, start_ticks, cwd) == "alive"


def sitl_command(root: str, home: str, instance: int, sysid: int) -> str:
    q = shlex.quote
    return (
        f"setarch -R {q(root)}/bin/arduplane -S --model plane --speedup 1 --slave 0 "
        f"--defaults {q(root)}/defaults.parm --serial0=sim:navpystep "
        f"--serial1=sim:navpystep --serial2=none --start-time {SITL_START_TIME_UTC} "
        f"--home {q(home)} --sysid {sysid} -I{instance}"
    )


def sitl_environment(boot_id: int, deadline_us: int, jitter_seed: int) -> str:
    """``env -i`` prefix: the SITL sees only its serial-step settings."""
    variables = {
        "PATH": "/usr/bin:/bin",
        "NAVPY_SERIAL_STEP": "1",
        "NAVPY_STEP_BOOT_ID": str(boot_id),
        "NAVPY_STEP_DEADLINE_US": str(deadline_us),
    }
    if jitter_seed:
        variables["NAVPY_STEP_HELPER_JITTER_SEED"] = str(jitter_seed)
    return "env -i " + " ".join(f"{key}={shlex.quote(value)}" for key, value in variables.items())


__all__ = [
    "LOCKSTEP_DEFAULT_OVERRIDES",
    "OwnedProcess",
    "PROCESSES_FILE",
    "load_firmware_lock",
    "merged_defaults",
    "prepare_root",
    "proc",
    "recorded_alive",
    "sitl_command",
    "sitl_environment",
    "stop_owned",
    "stop_recorded",
    "wsl",
    "wsl_path",
]
