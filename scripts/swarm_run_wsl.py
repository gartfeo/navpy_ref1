"""WSL command construction and chat-scoped process cleanup."""

from __future__ import annotations

import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

from gcs.backend import instance_ports as ip


WSL_BASH_ARGV = ("wsl", "--exec", "bash", "-lc")
# Discard-port sink for run_swarm.sh `pi` endpoints that must not leave the host.
LOOPBACK_SINK = "127.0.0.1:9"


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


def run_wsl(command: str) -> subprocess.CompletedProcess:
    """Run a Bash command inside WSL."""
    return subprocess.run(wsl_argv(command), capture_output=True)


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


REAP_HELPER = Path(__file__).with_name("wall_reap_wsl.py")


def cleanup_command(chat: int, sysids: list[int]) -> str:
    """The WSL command that reaps one wall launch's SITLs and router.

    ``wall_reap_wsl.py`` kills a process only when its command line names one
    of ``sysids`` (or the chat's router) AND its environment holds this chat's
    ``SWARM_OFFSET``, which :func:`launch_command` sets. A sysid alone is no
    owner: lockstep SITLs run ``--sysid 1..3`` on any slot (todo 51). The
    helper's source goes inline, so it works from any checkout.
    """
    arguments = ["reap", ip.VEHICLES_PER_CHAT * chat, ip.monitor_port(chat), *sysids]
    return (f"python3 -c {shlex.quote(REAP_HELPER.read_text(encoding='utf-8'))} "
            + " ".join(str(argument) for argument in arguments))


def cleanup(
    chat: int,
    *,
    sysids: list[int] | None = None,
    wsl_run: Callable[[str], subprocess.CompletedProcess] = run_wsl,
) -> None:
    """Terminate only one chat's SITL instances and router.

    ``sysids`` defaults to the chat's own three; a fleet launched past three
    aircraft passes every sysid it launched. A failed reap raises, so a stop
    does not release the slot and a launch does not start over its orphans.
    """
    result = wsl_run(cleanup_command(chat, ip.sysids_for_chat(chat) if sysids is None else sysids))
    if result.returncode != 0:
        detail = (result.stderr or b"").decode("utf-8", "replace").strip()[-500:]
        raise RuntimeError(f"wall cleanup of chat {chat} failed ({result.returncode}): {detail}")


def ensure_experimental_paths(
    root: str | None,
    binary: str | None,
    instance_seed: str | None = None,
) -> None:
    """Validate explicit experimental paths before reserving a machine slot."""
    if root is None and binary is None and instance_seed is None:
        return
    # Reuse command validation, including absolute paths and shell quoting.
    try:
        launch_command(1, 0, "", 1, 0, firmware_root=root, binary_path=binary,
                       instance_seed=instance_seed)
    except ValueError as error:
        sys.exit(f"swarm_run: {error}")
    executable = binary or f"{root.rstrip('/')}/build/sitl/bin/arduplane"
    checks = f"test -d {shlex.quote(root)} && test -x {shlex.quote(executable)}"
    if instance_seed is not None:
        checks += f" && test -d {shlex.quote(instance_seed)}"
    probe = subprocess.run(wsl_argv(checks), capture_output=True)
    if probe.returncode != 0:
        sys.exit("swarm_run: experimental root, executable or instance seed is unavailable")


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
    loopback_only: bool = False,
    instance_seed: str | None = None,
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

    ``instance_seed`` names a directory each instance directory is copied from
    when it does not exist yet, so a run boots from a fresh eeprom with known
    scripts and terrain instead of the shared templates 1-3, whose mutable
    eeprom ``run_swarm.sh`` clones.  An existing instance directory is kept, so
    a relaunch on the same root keeps what its first boot persisted.
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
    if instance_seed is not None and firmware_root is None:
        raise ValueError("an instance seed requires an isolated firmware root")
    for value in (firmware_root, binary_path, instance_seed):
        if value is not None and (not value.startswith("/") or "\x00" in value):
            raise ValueError("SITL root, binary and instance seed must be absolute WSL paths")
    if firmware_root is not None:
        environment.append(f"ARDUPILOT_DIR={shlex.quote(firmware_root)}")
        executable = binary_path or f"{firmware_root.rstrip('/')}/build/sitl/bin/arduplane"
        environment.append(f"BIN={shlex.quote(executable)}")
    if loopback_only:
        # run_swarm.sh `pi` defaults point serial2 and extra router endpoints at
        # LAN hosts; an empty value would fall back to those defaults, so name
        # loopback sinks instead.
        environment += [f"PI_TARGETS_CSV={LOOPBACK_SINK}",
                        f"PI_ROUTER_EXTRA_CSV={LOOPBACK_SINK}"]
    seed_prefix = ""
    if instance_seed is not None:
        root = firmware_root.rstrip("/")
        directories = " ".join(
            shlex.quote(f"{root}/{offset + index}") for index in range(1, instances + 1)
        )
        seed_prefix = (
            f'for d in {directories}; do [ -e "$d" ] || '
            f'cp -a {shlex.quote(instance_seed)} "$d" || exit 1; done; '
        )
    if keep_instance_state or instance_seed is not None:
        # A seeded instance must never be overwritten by a template eeprom.
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
        seed_prefix
        + " ".join(environment)
        + " ~/ardupilot/Tools/autotest/run_swarm.sh pi "
        f"-n {instances} -d {distance}"
    )
