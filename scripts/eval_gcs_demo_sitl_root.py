"""One run's isolated SITL root in WSL, used instead of the shared templates.

``run_swarm.sh`` clones the eeprom of templates ``~/ardupilot/1-3``, which other
sessions keep rewriting and which carries calibration, statistics, a stored
mission and fence, and a different partial terrain tile per template. A run's
root holds a fresh-eeprom instance seed (deployed Lua and the pinned terrain
tile), the pre-boot defaults and a binary wrapper that pins the SITL clock.
"""

from __future__ import annotations

import hashlib
import math
import re
import shlex
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from scripts.eval_gcs_demo_constants import (
    FINAL_APPROACH_ROLL_LIMIT_DEG,
    ROOT,
    SIM_SPEEDUP,
)
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.swarm_run_wsl import wsl_argv


#: SITL RTC and GPS epoch (2026-01-01T00:00:00Z). Without it SITL starts at the
#: host's wall clock, so GPS time, SYSTEM_TIME and log stamps differ per run.
SITL_START_TIME_UTC = 1_767_225_600
#: Under the WSL home directory.
FIRMWARE_BINARY = "ardupilot/build/sitl/bin/arduplane"
STOCK_DEFAULTS = "ardupilot/Tools/autotest/models/plane.parm"
SITL_ROOTS = "navpy-eval-roots"
#: Read by every launch besides the root's own files.
WSL_HOST_FILES = (FIRMWARE_BINARY, STOCK_DEFAULTS, "ardupilot/Tools/autotest/run_swarm.sh")
WSL_LOGIN_PROFILES = (".profile", ".bashrc", ".bash_profile", ".bash_login")

SITL_LUA_DIR = ROOT / "scripts" / "lua"
SITL_TERRAIN_DIR = ROOT / ".terrain" / "sitl"

#: Values the shared template eeproms carried over stock ``plane.parm`` on
#: 2026-10-13, which the demo has been flown with. A fresh eeprom drops them,
#: so the run states them. Left out on purpose: calibration (INS_*, AHRS_TRIM,
#: ARSPD_OFFSET, BARO*_GND_PRESS), statistics (STAT_*), the stored mission and
#: fence (MIS_TOTAL, FENCE_TOTAL; the fence is disabled), SIM_SPEEDUP (the
#: launch flag sets it), RTL_ALTITUDE (the mission upload sets it per vehicle)
#: and AAS_* (registered by Lua after boot, so written by the AAS PUT).
SITL_DEFAULT_OVERRIDES: Mapping[str, float] = {
    "AIRSPEED_MAX": 40.0, "AIRSPEED_MIN": 18.0,
    "ARMING_CHECK": 0.0,
    "FLTMODE_CH": 5.0, "FLTMODE1": 5.0, "FLTMODE2": 5.0,
    "FLTMODE3": 7.0, "FLTMODE4": 7.0, "FLTMODE5": 8.0,
    "FLTMODE6": 8.0,
    "NAVL1_PERIOD": 18.0,
    "PTCH2SRV_RMAX_DN": 75.0, "PTCH2SRV_RMAX_UP": 75.0, "PTCH2SRV_TCONST": 0.2875,
    "PTCH_LIM_MAX_DEG": 35.0,
    "PTCH_LIM_MIN_DEG": -85.0,
    "RC6_OPTION": 16.0,
    "RC8_OPTION": 153.0,
    "RLL2SRV_RMAX": 75.0, "RLL2SRV_TCONST": 0.2875,
    "ROLL_LIMIT_DEG": FINAL_APPROACH_ROLL_LIMIT_DEG,
    # Scripting debug output shifts the MAVLink sequence; stated, not inherited.
    "SCR_DEBUG_OPTS": 0.0,
    "SERVO5_FUNCTION": 94.0,
    # serial0 feeds the companion and serial1 the GCS; without these rates a
    # fresh eeprom streams almost nothing to NavPy. EXTRA1 and RAW_SENS stay
    # off on serial0: NavPy reads none of their messages and requests its
    # ATTITUDE itself (pose_streams.py). They were 28 of the 149 packets per
    # sim second each NavPy read (todo 66).
    "SR0_ADSB": 5.0, "SR0_EXT_STAT": 2.0, "SR0_EXTRA1": 0.0,
    "SR0_EXTRA2": 4.0, "SR0_EXTRA3": 2.0, "SR0_POSITION": 2.0,
    "SR0_RAW_CTRL": 1.0, "SR0_RAW_SENS": 0.0, "SR0_RC_CHAN": 2.0,
    "SR1_EXT_STAT": 2.0, "SR1_EXTRA1": 4.0, "SR1_EXTRA2": 4.0,
    "SR1_EXTRA3": 2.0, "SR1_POSITION": 2.0, "SR1_RAW_SENS": 2.0,
    "SR1_RC_CHAN": 2.0,
    "TECS_SPDWEIGHT": 1.5,
    "THR_FAILSAFE": 0.0,
    "TRIM_THROTTLE": 55.0,
    "WP_LOITER_RAD": 500.0,
}

_PARAM_LINE = re.compile(r"^\s*([A-Z][A-Z0-9_]*)[\s,]+(\S+)\s*$")


@dataclass(frozen=True)
class SitlRoot:
    """WSL paths of one run's isolated SITL root."""

    root: str

    @property
    def binary(self) -> str:
        return f"{self.root}/bin/arduplane"

    @property
    def defaults(self) -> str:
        return f"{self.root}/defaults.parm"

    @property
    def seed(self) -> str:
        return f"{self.root}/seed"

    def launch_arguments(self) -> list[str]:
        """``gcs_launch`` flags that boot every instance from this root only."""
        return [
            "--sitl-root", self.root,
            "--sitl-binary", self.binary,
            "--sitl-defaults", self.defaults,
            "--sitl-instance-seed", self.seed,
            "--single-boot",
            "--loopback-only",
        ]


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def merge_defaults(stock: str, overrides: Mapping[str, float]) -> str:
    """Stock parameter lines with each override replacing or appending its value."""
    pending = dict(overrides)
    lines: list[str] = []
    for line in stock.splitlines():
        match = _PARAM_LINE.match(line)
        if match is not None and match.group(1) in pending:
            name = match.group(1)
            lines.append(f"{name} {_param_value(pending.pop(name))}")
        else:
            lines.append(line)
    lines.extend(f"{name} {_param_value(value)}" for name, value in pending.items())
    return "\n".join(lines) + "\n"


def _param_value(value: float) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"SITL default must be a finite number, got {value!r}")
    return format(float(value), ".15g")


def binary_wrapper(firmware: str, start_time_utc: int) -> str:
    """A launcher-compatible executable that pins the SITL clock epoch."""
    if not firmware.startswith("/") or "'" in firmware:
        raise ValueError(f"firmware path must be absolute and unquoted: {firmware!r}")
    return f"#!/bin/sh\nexec '{firmware}' --start-time {int(start_time_utc)} \"$@\"\n"


def lua_scripts() -> dict[str, bytes]:
    """Deployed Lua with LF endings, as SITL would load it."""
    scripts = {
        path.name: path.read_bytes().replace(b"\r\n", b"\n")
        for path in sorted(SITL_LUA_DIR.glob("*.lua"))
    }
    if not scripts:
        raise RegressionError(f"input fault: no Lua scripts in {SITL_LUA_DIR}")
    return scripts


def terrain_tiles() -> dict[str, bytes]:
    tiles = {path.name: path.read_bytes() for path in sorted(SITL_TERRAIN_DIR.glob("*.DAT"))}
    if not tiles:
        raise RegressionError(f"input fault: no SITL terrain tiles in {SITL_TERRAIN_DIR}")
    return tiles


def stage_sitl_root(directory: Path, stock_defaults: str, firmware: str) -> dict[str, str]:
    """Write the root's files into *directory*; return their sha256 by relative path."""
    files: dict[str, bytes] = {
        "bin/arduplane": binary_wrapper(firmware, SITL_START_TIME_UTC).encode("ascii"),
        "defaults.parm": merge_defaults(stock_defaults, SITL_DEFAULT_OVERRIDES).encode("ascii"),
    }
    files.update({f"seed/scripts/{name}": data for name, data in lua_scripts().items()})
    files.update({f"seed/terrain/{name}": data for name, data in terrain_tiles().items()})
    for relative, data in files.items():
        target = directory / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    return {relative: sha256_bytes(data) for relative, data in sorted(files.items())}


def _wsl_output(script: str) -> str:
    result = subprocess.run(wsl_argv(script), capture_output=True, timeout=120)
    if result.returncode != 0:
        raise RegressionError(
            f"WSL input step failed ({result.returncode}): "
            f"{result.stderr.decode('utf-8', 'replace').strip()}"
        )
    return result.stdout.decode("utf-8")


def _wsl_path(path: Path) -> str:
    result = subprocess.run(
        ["wsl", "--exec", "wslpath", "-a", str(path.resolve())],
        capture_output=True,
        timeout=60,
    )
    if result.returncode != 0:
        raise RegressionError(f"wslpath failed for {path}")
    return result.stdout.decode("utf-8").strip()


def _sha_lines(text: str) -> dict[str, str]:
    hashes: dict[str, str] = {}
    for line in text.splitlines():
        digest, _, name = line.partition("  ")
        hashes[name.removeprefix("./")] = digest
    return hashes


def root_name(run_dir: Path) -> str:
    """Unique per run directory, readable in a WSL listing."""
    resolved = str(run_dir.resolve())
    label = re.sub(r"[^A-Za-z0-9._-]", "-", run_dir.name)[:48] or "run"
    return f"{label}-{sha256_bytes(resolved.encode('utf-8'))[:8]}"


def _host_hashes(home: str) -> dict[str, JsonValue]:
    """WSL files the launch reads outside the root; an absent file is null.

    ``bash -lc`` loads the login profiles (the user's and ``/etc``'s), so they
    can change the launcher's environment (e.g. ``RUN_ROUTER``) behind the
    run's back.
    """
    paths = [f"{home}/{name}" for name in (*WSL_HOST_FILES, *WSL_LOGIN_PROFILES)]
    script = "; ".join(
        f"if [ -f {shlex.quote(path)} ]; then sha256sum {shlex.quote(path)}; "
        f"else printf 'absent  %s\n' {shlex.quote(path)}; fi"
        for path in paths
    )
    # System-wide login files: listed by what exists, keyed by absolute path.
    script += '; for f in /etc/profile /etc/bash.bashrc /etc/profile.d/*.sh; do [ -f "$f" ] && sha256sum "$f"; done; true'
    hashes = _sha_lines(_wsl_output(script))
    files: dict[str, JsonValue] = {
        path.removeprefix(f"{home}/"): (None if hashes[path] == "absent" else hashes[path])
        for path in paths
    }
    files.update({path: digest for path, digest in sorted(hashes.items()) if path.startswith("/etc/")})
    return files


def prepare_sitl_root(run_dir: Path) -> tuple[SitlRoot, dict[str, JsonValue], str]:
    """Create the run's WSL root; return it, its manifest section and its host path."""
    home = _wsl_output('printf %s "$HOME"').strip()
    if not home.startswith("/"):
        raise RegressionError(f"unexpected WSL home {home!r}")
    firmware = f"{home}/{FIRMWARE_BINARY}"
    stock = _wsl_output(f"cat {shlex.quote(f'{home}/{STOCK_DEFAULTS}')}")
    staged = run_dir / "sitl_root"
    if staged.exists():
        raise RegressionError(f"run directory already holds a SITL root: {staged}")
    files = stage_sitl_root(staged, stock, firmware)
    root = SitlRoot(f"{home}/{SITL_ROOTS}/{root_name(run_dir)}")
    quoted = shlex.quote(root.root)
    copied = _sha_lines(_wsl_output(
        "set -e; "
        f"test ! -e {quoted}; mkdir -p {shlex.quote(f'{home}/{SITL_ROOTS}')}; "
        f"cp -r {shlex.quote(_wsl_path(staged))} {quoted}; "
        f"chmod 755 {shlex.quote(root.binary)}; "
        f"cd {quoted}; find . -type f -print0 | sort -z | xargs -0 sha256sum"
    ))
    if copied != files:
        raise RegressionError("SITL root copy in WSL does not match the staged files")
    section: dict[str, JsonValue] = {
        "host_files": _host_hashes(home),
        "start_time_utc": SITL_START_TIME_UTC,
        "speedup": SIM_SPEEDUP,
        "files": dict(files),
    }
    return root, section, root.root


__all__ = [
    "SITL_DEFAULT_OVERRIDES",
    "SITL_LUA_DIR",
    "SITL_START_TIME_UTC",
    "SITL_TERRAIN_DIR",
    "SitlRoot",
    "binary_wrapper",
    "merge_defaults",
    "prepare_sitl_root",
    "root_name",
    "sha256_bytes",
    "sha256_file",
    "stage_sitl_root",
]
