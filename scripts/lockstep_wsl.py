"""WSL side of a lockstep run: the run root, spawned processes, ownership.

Each run gets its own WSL root (``~/navpy-eval-roots/lockstep-<run>``) with a
copy of the serial-step firmware (checked against ``lockstep_firmware.lock``),
pre-boot defaults and ``aas_params.lua`` only. Every WSL process (relay,
SITL) is recorded the moment it is spawned, by WSL pid, process start time
and cwd under the root, in ``<run>/lockstep_processes_host.json``, and is
stopped only through those records (``docs/design/lockstep_contracts.md``,
Ownership).
"""

from __future__ import annotations

import json
import shlex
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path

from scripts.eval_gcs_demo_sitl_root import (
    SITL_DEFAULT_OVERRIDES,
    SITL_LUA_DIR,
    SITL_ROOTS,
    SITL_START_TIME_UTC,
    STOCK_DEFAULTS,
    merge_defaults,
)

ROOT = Path(__file__).resolve().parents[1]
PROC_HELPER = ROOT / "scripts" / "lockstep_wsl_proc.py"
FIRMWARE_LOCK = ROOT / "scripts" / "lockstep_firmware.lock.json"
PROCESSES_FILE = "lockstep_processes_host.json"
#: Lockstep-only defaults on top of the wall eval's (step 3): no BIN log, no
#: terrain IO thread, scripting for aas_params.lua only (contract: Firmware
#: helpers).
LOCKSTEP_DEFAULT_OVERRIDES = {
    **SITL_DEFAULT_OVERRIDES,
    "LOG_BACKEND_TYPE": 0.0,
    "TERRAIN_ENABLE": 0.0,
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


def prepare_root(run_name: str, instance: int) -> str:
    """Create the run's WSL root and its instance directory; return the root."""
    lock = load_firmware_lock()
    home = wsl("printf %s \"$HOME\"")
    root = f"{home}/{SITL_ROOTS}/lockstep-{run_name}"
    stock = wsl(f"cat {shlex.quote(home + '/' + STOCK_DEFAULTS)}")
    defaults = merge_defaults(stock, LOCKSTEP_DEFAULT_OVERRIDES)
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
    return root


def _proc(*args: object) -> str:
    helper = shlex.quote(wsl_path(PROC_HELPER))
    return wsl("python3 " + helper + " " + " ".join(shlex.quote(str(a)) for a in args)).strip()


def spawn(role: str, script: str, cwd: str, record: Path) -> tuple[subprocess.Popen, OwnedProcess]:
    """Start ``script`` in WSL under ``cwd``; it runs only once it is recorded.

    The child stops itself before it runs anything. The WSL-side bash
    publishes its pid only once it is stopped, then continues it on a ``go``
    line from us, written after the pid, start time and cwd are recorded
    atomically. Anything else (we fail, are interrupted or die: stdin closes)
    makes that bash kill the child, so no unrecorded process survives. The
    child runs as bash's child, not by an ``exec`` of the session's bash: a
    SITL that replaced the top-level bash of a ``wsl.exe`` session exited
    early.
    """
    launcher = (
        f"cd {shlex.quote(cwd)} && {{ {{ kill -STOP $BASHPID; exec {script}; }} & c=$!; "
        f"while [ \"$(cut -d' ' -f3 /proc/$c/stat)\" != T ]; do sleep 0.001; done; "
        f"echo LOCKSTEP_PID $c; "
        f"if read -r go && [ \"$go\" = go ]; then kill -CONT $c; wait $c; "
        f"else kill -KILL $c; fi; }}"
    )
    process = subprocess.Popen(
        ["wsl.exe", "-e", "bash", "-c", launcher],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE,
    )
    try:
        first = process.stdout.readline().decode().split()
        if len(first) != 2 or first[0] != "LOCKSTEP_PID":
            raise RuntimeError(f"{role} did not report its pid: {first}")
        pid = int(first[1])
        ticks, proc_cwd = _proc("identity", pid).split(" ", 1)
        owned = OwnedProcess(role, pid, int(ticks), proc_cwd)
        records = json.loads(record.read_text()) if record.exists() else []
        records.append(asdict(owned))
        tmp = record.with_suffix(".tmp")
        tmp.write_text(json.dumps(records, indent=2), encoding="utf-8")
        tmp.replace(record)  # atomic: a crash never leaves a half-written record
        process.stdin.write(b"go\n")
        process.stdin.flush()
    except BaseException:
        process.stdin.close()  # EOF: the WSL bash kills the stopped child
        process.wait(timeout=30)
        raise
    return process, owned


def stop_owned(record: Path, root: str) -> list[str]:
    """Stop every recorded process that is still the recorded one; report each."""
    if not record.exists():
        return []
    owned_list = [OwnedProcess(**raw) for raw in json.loads(record.read_text())]
    for owned in owned_list:
        if not owned.cwd.startswith(root + "/") and owned.cwd != root:
            raise RuntimeError(f"recorded process outside the run root: {owned}")
    return [
        f"{owned.role} {owned.wsl_pid}: "
        + _proc("stop", owned.wsl_pid, owned.start_ticks, owned.cwd)
        for owned in owned_list
    ]


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
    "prepare_root",
    "sitl_command",
    "sitl_environment",
    "spawn",
    "stop_owned",
    "wsl",
    "wsl_path",
]
