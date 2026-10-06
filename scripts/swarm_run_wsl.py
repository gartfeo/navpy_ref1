"""WSL command construction and chat-scoped process cleanup."""

from __future__ import annotations

import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

from gcs.backend import instance_ports as ip


WSL_BASH_ARGV = ("wsl", "--exec", "bash", "-lc")


def resolve_chat(
    explicit: int | None,
    *,
    eval_mode: bool,
    root: Path,
    registry: Any,
) -> int:
    """Resolve or reserve this directory's chat in the appropriate band."""
    if explicit is not None:
        return explicit
    owner = registry.owner_for(str(root))
    label = "sitl-eval" if eval_mode else "sitl"
    entry = registry.find_for_owner(owner, label=label)
    if entry is not None:
        return int(entry["chat_index"])
    if eval_mode:
        return int(
            registry.claim(
                label=label,
                clone=str(root),
                owner=owner,
                lo=ip.EVAL_CHAT_MIN,
            )["chat_index"]
        )
    return int(
        registry.claim(
            label=label,
            clone=str(root),
            owner=owner,
            hi=ip.interactive_chat_hi(),
        )["chat_index"]
    )


def wsl_argv(script: str) -> list[str]:
    """Build argv that gives a Bash script to WSL without outer-shell parsing."""
    return [*WSL_BASH_ARGV, script]


def run_wsl(command: str) -> None:
    """Run a Bash command inside WSL."""
    subprocess.run(wsl_argv(command), capture_output=True)


def ensure_fork_supports_companion_udp() -> None:
    """Reject an ArduPilot fork predating the companion-UDP transport gate."""
    probe = subprocess.run(
        wsl_argv(
            "grep -q COMPANION_UDP "
            "~/ardupilot/Tools/autotest/run_swarm.sh"
        ),
        capture_output=True,
    )
    if probe.returncode != 0:
        sys.exit(
            "swarm_run: ~/ardupilot/Tools/autotest/run_swarm.sh does not know "
            "COMPANION_UDP — update the WSL fork before launching this branch."
        )


def ensure_defaults_file(defaults_path: str) -> None:
    """Reject a pre-boot parameter file before the launch machinery starts.

    ``run_swarm.sh`` does check the file itself -- "Defaults not found" at its
    own preflight -- and the SITL binary panics if it cannot open one, so a bad
    path fails loudly either way.  The value of checking here is ORDER: this
    runs before a registry slot is claimed and before SITL or the launcher
    start, so a bad path costs nothing to recover from.  It can also name the
    cause, which the launcher's message cannot.  (It is not free of processes
    itself -- the readability probe below runs a WSL command.)

    One cause worth naming is a Windows shell rewriting the path: MSYS turns
    ``/home/gart/x.parm`` into ``C:/Program Files/Git/home/gart/x.parm`` unless
    MSYS_NO_PATHCONV=1 is set.  Observed here, though how often it is the
    cause elsewhere is not something this code has measured.

    An absolute path is required because the launcher changes directory before
    reading it.  Commas are rejected because ArduPilot splits its ``--defaults``
    argument on them, so a comma in a filename silently becomes two paths.
    """
    if ":" in defaults_path:
        sys.exit(
            f"swarm_run: --defaults {defaults_path!r} looks like a Windows "
            "path. A Windows shell rewrites a leading-slash path (MSYS turns "
            "/home/... into C:/Program Files/Git/home/...); prefix the command "
            "with MSYS_NO_PATHCONV=1, or pass /mnt/c/... for a file on the "
            "Windows side."
        )
    if not defaults_path.startswith("/"):
        sys.exit(
            f"swarm_run: --defaults {defaults_path!r} is not an absolute path. "
            "run_swarm.sh changes directory before reading it, so a relative "
            "path will not resolve as you expect."
        )
    if "," in defaults_path:
        sys.exit(
            f"swarm_run: --defaults {defaults_path!r} contains a comma, which "
            "ArduPilot treats as a separator between defaults files."
        )
    probe = subprocess.run(
        wsl_argv(f"test -f {shlex.quote(defaults_path)} && "
                 f"test -r {shlex.quote(defaults_path)}"),
        capture_output=True,
    )
    if probe.returncode != 0:
        sys.exit(
            f"swarm_run: --defaults {defaults_path!r} is not a readable file "
            "inside WSL. run_swarm.sh would refuse it a few steps later, after "
            "a slot has been claimed."
        )


def sysid_process_pattern(sys_id: int) -> str:
    """Return a pkill-safe pattern for one ArduPilot system ID.

    Both spellings ArduPilot accepts are matched. Recognising only
    ``--sysid N`` let an aircraft started as ``--sysid=N`` survive teardown and
    then co-stream on an id the next run believes it owns exclusively.
    """
    return f"[-]-sysid[[:space:]=]+{sys_id}([[:space:]]|$)"


def router_process_pattern(chat: int) -> str:
    """Return a pkill-safe pattern for one chat's MAVLink router."""
    return f"[m]avlink-routerd.*:{ip.monitor_port(chat)}([[:space:]]|$)"


def cleanup(chat: int, *, wsl_run: Callable[[str], None] = run_wsl) -> None:
    """Terminate only one chat's SITL instances and router."""
    patterns = [sysid_process_pattern(sysid) for sysid in ip.sysids_for_chat(chat)]
    patterns.append(router_process_pattern(chat))
    terminate = "; ".join(
        f"pkill -f -- '{pattern}' || true" for pattern in patterns
    )
    kill = "; ".join(
        f"pkill -KILL -f -- '{pattern}' || true" for pattern in patterns
    )
    wsl_run(f"{terminate}; sleep 2; {kill}")


def ensure_experimental_paths(root: str | None, binary: str | None) -> None:
    """Validate explicit experimental paths before reserving a machine slot."""
    if root is None and binary is None:
        return
    # Reuse command validation, including absolute paths and shell quoting.
    try:
        launch_command(1, 0, "", 1, 0, firmware_root=root, binary_path=binary)
    except ValueError as error:
        sys.exit(f"swarm_run: {error}")
    executable = binary or f"{root.rstrip('/')}/build/sitl/bin/arduplane"
    probe = subprocess.run(wsl_argv(
        f"test -d {shlex.quote(root)} && test -x {shlex.quote(executable)}"
    ), capture_output=True)
    if probe.returncode != 0:
        sys.exit("swarm_run: experimental root or executable is unavailable")


def launch_command(
    speedup: float,
    offset: int,
    win_ports: str,
    instances: int,
    distance: int,
    *,
    keep_instance_state: bool = False,
    home_coords: str | None = None,
    defaults_path: str | None = None,
    firmware_root: str | None = None,
    binary_path: str | None = None,
) -> str:
    """Build one inline-environment invocation of ``run_swarm.sh``.

    ``defaults_path`` overrides the pre-boot parameter file; see the inline
    note below for why that is the only lever for allocation-time values.

    ``home_coords`` pins ``HOME_COORDS`` ("lat,lon,alt,heading") so the swarm
    starts at a fixed place.  Left unset, the start point is whatever the
    launcher's default resolves to, which has been observed to shift with the
    slot index; an experiment that varies wind relative to the vehicle's ground
    track needs the start geometry held fixed across runs, so the caller states
    it explicitly rather than inheriting it.
    """
    # ArduPilot's sim_vehicle.py accepts only an integer launch speed.  Start
    # fractional requests at its 1x floor; the supervisor then verifies and,
    # when needed, persists the exact AP_Float SIM_SPEEDUP before relaunch.
    launch_speedup = max(1, int(speedup))
    environment = [
        f"SPEEDUP={launch_speedup}",
        f"SWARM_OFFSET={offset}",
        f"ROUTER_WIN_PORTS={win_ports}",
        "COMPANION_UDP=1",
    ]
    if binary_path is not None and firmware_root is None:
        raise ValueError("an explicit SITL binary requires an isolated firmware root")
    for value in (firmware_root, binary_path):
        if value is not None and (not value.startswith("/") or "\x00" in value):
            raise ValueError("SITL root and binary must be absolute WSL paths")
    if firmware_root is not None:
        environment.append(f"ARDUPILOT_DIR={shlex.quote(firmware_root)}")
        executable = binary_path or f"{firmware_root.rstrip('/')}/build/sitl/bin/arduplane"
        environment.append(f"BIN={shlex.quote(executable)}")
    if keep_instance_state:
        environment.append("CLONE_FROM_TEMPLATE=0")
    if home_coords:
        # Quoted, not interpolated raw: this string is handed to `bash -lc`
        # (WSL_BASH_ARGV), so an unquoted value carrying `;` or a backtick
        # would run as a second command rather than set a variable.
        environment.append(f"HOME_COORDS={shlex.quote(home_coords)}")
    if defaults_path:
        # `run_swarm.sh` reads DEFAULTS with a `${DEFAULTS:-...}` fallback to
        # the shared `models/plane.parm`.  Overriding it per launch supplies
        # PRE-BOOT parameters -- the only point at which buffer-allocation and
        # sensor-calibration values (EK3_HGT_DELAY, SIM_BARO_*, GPS1_DELAY_MS)
        # can still take effect -- without editing that shared template, which
        # every concurrent session on this machine boots from.  A WSL-side
        # path: run_swarm.sh runs there, so a Windows path must be given as
        # /mnt/c/...
        environment.append(f"DEFAULTS={shlex.quote(defaults_path)}")
    return (
        " ".join(environment)
        + " ~/ardupilot/Tools/autotest/run_swarm.sh pi "
        f"-n {instances} -d {distance}"
    )
