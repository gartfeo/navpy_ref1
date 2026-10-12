"""WSL filesystem operations restricted to this opt-in prototype's instances."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
from simtime_navigation_protocol import WINDOW_SIZE
from simtime_firmware_contract import (
    GUIDANCE_COUNT_ENV, GUIDANCE_CSV, GUIDANCE_MODE_CLOSED_LOOP, GUIDANCE_MODE_ENV, GUIDANCE_SOCKET, GUIDANCE_START_US_ENV, POSE_CAPTURE_ENV, POSE_CSV, RENDER_POSE_ENV, STEP_CSV, STEP_MODE_ENV, STEP_SOCKET, STEP_START_US_ENV,
)


def owned_directory(root: Path, vehicle: int) -> Path:
    root = root.resolve(strict=True)
    if not 1 <= vehicle <= 255 or not (root / "ArduPlane/NavPyStepProbe.cpp").is_file():
        raise ValueError("not a prototype firmware root/vehicle")
    directory = root / str(vehicle)
    if directory.is_symlink():
        raise ValueError("instance directory must not be a symlink")
    directory.mkdir(exist_ok=True)
    if directory.resolve().parent != root:
        raise ValueError("instance escaped firmware root")
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            if (process / "cwd").resolve(strict=True) == directory:
                raise RuntimeError(f"instance still has process {process.name}")
        except (FileNotFoundError, PermissionError):
            pass
    return directory


def prepare(root: Path, vehicle: int, case: Path, mode: str) -> dict:
    directory = owned_directory(root, vehicle)
    for name in (STEP_SOCKET, STEP_CSV, GUIDANCE_SOCKET, GUIDANCE_CSV, POSE_CSV):
        if (directory / name).exists() or (directory / name).is_symlink():
            raise RuntimeError(f"uncollected artifact: {name}")
    storage = directory / "eeprom.bin"
    if storage.exists():
        if not storage.is_file() or storage.is_symlink():
            raise ValueError("unexpected instance storage")
        shutil.copyfile(storage, case / "previous-eeprom.bin")
        storage.unlink()  # only this inactive owned instance; next boot is fresh
    wrapper = case / "launch-probe.sh"
    binary = root / "build/sitl/bin/arduplane"
    with wrapper.open("x", encoding="utf-8", newline="\n") as output:
        output.write("#!/bin/sh\n")
        output.write(f"unset {STEP_MODE_ENV} {STEP_START_US_ENV}\n")
        output.write(f"unset {GUIDANCE_MODE_ENV} {GUIDANCE_START_US_ENV} {GUIDANCE_COUNT_ENV}\n")
        output.write(f"unset {POSE_CAPTURE_ENV}\n")
        output.write(f"unset {RENDER_POSE_ENV}\n")
        if mode.startswith("navigation"):
            output.write(f"export {GUIDANCE_MODE_ENV}={GUIDANCE_MODE_CLOSED_LOOP}\n")
            output.write(f"export {GUIDANCE_START_US_ENV}=40000000\n")
            output.write(f"export {GUIDANCE_COUNT_ENV}={WINDOW_SIZE}\n")
            if mode.startswith("navigation-pose"):
                output.write(f"export {POSE_CAPTURE_ENV}=1\n")
            if mode in ("navigation-pose-rounded", "navigation-pose-precast"):
                output.write(f"export {RENDER_POSE_ENV}={mode.rsplit('-', 1)[1]}\n")
        elif mode != "off":
            probe_mode = "observe" if mode == "observe" else "handshake"
            output.write(f"export {STEP_MODE_ENV}={probe_mode}\n")
            output.write(f"export {STEP_START_US_ENV}=40000000\n")
        output.write(f"exec {shlex.quote(str(binary))} \"$@\"\n")
    wrapper.chmod(0o700)
    result = {"directory": str(directory), "wrapper": str(wrapper)}
    if mode.startswith("navigation"):
        defaults = case / "navigation-defaults.parm"
        base = (root / "Tools/autotest/models/plane.parm").read_text()
        disarmed = 0 if mode == "navigation-unready" else 1
        with defaults.open("x") as output:
            output.write(base + f"\nLOG_DISARMED {disarmed}\nLOG_FILE_DSRMROT 0\n")
        result["defaults"] = str(defaults)
    return result


def collect(root: Path, vehicle: int, case: Path, mode: str = "observe") -> dict:
    directory = owned_directory(root, vehicle)
    source = directory / (GUIDANCE_CSV if mode.startswith("navigation") else STEP_CSV)
    if not source.exists():
        return {"collected": False}
    sources = [source]
    pose = directory / POSE_CSV
    if mode.startswith("navigation-pose"):
        sources.append(pose)
    elif pose.exists() or pose.is_symlink():
        raise ValueError("unexpected pose evidence in non-capture run")
    for item in sources:
        if item.is_symlink() or not item.is_file():
            raise ValueError("missing or unexpected evidence file")
        target = case / item.name
        with target.open("xb") as output:
            output.write(item.read_bytes())
        if target.read_bytes() != item.read_bytes():
            raise RuntimeError("evidence copy mismatch")
    for item in sources:
        item.unlink()  # originals removed only after every copy was verified
    return {"collected": True}


def stop_peer(case: Path) -> dict:
    pidfile = case / "peer.pid"
    if not pidfile.exists():
        return {"peer_running": False}
    pid = int(pidfile.read_text())
    proc = Path(f"/proc/{pid}")
    try:
        argv = (proc / "cmdline").read_bytes().split(b"\0")
    except FileNotFoundError:
        return {"peer_running": False}
    if not any(arg.endswith((b"/simtime_step_peer.py", b"/simtime_navigation_peer.py")) for arg in argv):
        raise RuntimeError("peer PID was reused")
    if str(case / "peer.json").encode() not in argv:
        raise RuntimeError("peer PID belongs to a different case")
    os.kill(pid, signal.SIGTERM)
    return {"peer_stop_requested": pid}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("prepare", "collect", "stop-peer"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--vehicle", type=int, required=True)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--mode", default="observe")
    args = parser.parse_args()
    if args.operation == "prepare":
        result = prepare(args.root, args.vehicle, args.case, args.mode)
    elif args.operation == "collect":
        result = collect(args.root, args.vehicle, args.case, args.mode)
    else:
        result = stop_peer(args.case)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
