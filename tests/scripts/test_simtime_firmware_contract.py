"""The harness spells firmware-defined names exactly as the lockstep firmware binary does."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

from scripts.simtime_firmware_contract import FIRMWARE_NAMES, GUIDANCE_SOCKET

ROOT = Path(__file__).resolve().parents[2]
BINARY_ENV = "NAVPY_LOCKSTEP_FIRMWARE_BINARY"
DEFAULT_LINUX_BINARY = "/home/gart/navpy-simtime-step-handshake/build/sitl/bin/arduplane"
DEFAULT_UNC_BINARY = ("\\\\wsl.localhost\\Ubuntu-24.04\\home\\gart\\navpy-simtime-step-handshake"
                      "\\build\\sitl\\bin\\arduplane")
# GCC inlines `strcpy(sun_path, "navpy-guidance.sock")` as a 16-byte pooled
# load plus an immediate tail, so only its first 16 bytes are in the binary
# (NavPyGuidanceStep.cpp:127). Every other name must appear whole.
SPLIT_LITERAL_PREFIX_BYTES = {GUIDANCE_SOCKET: 16}


def _present(name: str, binary: bytes) -> bool:
    raw = name.encode()
    if raw in binary:
        return True
    prefix_bytes = SPLIT_LITERAL_PREFIX_BYTES.get(name)
    return prefix_bytes is not None and raw[:prefix_bytes] in binary


def _read_binary() -> tuple[bytes | None, str]:
    """Read the binary from a Windows/UNC path, else through wsl.exe (Linux path)."""
    configured = os.environ.get(BINARY_ENV)
    if configured and not configured.startswith("/"):
        # An explicit Windows path is checked as given, never replaced by a default.
        path = Path(configured)
        return (path.read_bytes() if path.is_file() else None), configured
    if not configured and Path(DEFAULT_UNC_BINARY).is_file():
        return Path(DEFAULT_UNC_BINARY).read_bytes(), DEFAULT_UNC_BINARY
    linux_path = configured or DEFAULT_LINUX_BINARY
    try:
        result = subprocess.run(["wsl.exe", "--exec", "cat", linux_path],
                                capture_output=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return None, linux_path
    return (result.stdout if result.returncode == 0 and result.stdout else None), linux_path


@pytest.fixture(scope="module")
def firmware_binary() -> bytes:
    data, where = _read_binary()
    if data is None and os.environ.get(BINARY_ENV):
        pytest.fail(f"{BINARY_ENV} names an unreadable firmware binary: {where}")
    if data is None:
        pytest.skip(f"lockstep firmware binary not readable ({where}); set {BINARY_ENV} to a "
                    "Windows path or an absolute WSL path (host-specific)")
    return data


@pytest.mark.parametrize("name", FIRMWARE_NAMES)
def test_firmware_binary_contains_name(name: str, firmware_binary: bytes) -> None:
    assert _present(name, firmware_binary), f"{name!r} is not in the firmware binary"


def test_presence_check_rejects_a_renamed_name() -> None:
    assert not _present("NAVPY_NAVIGATION_MODE", b"NAVPY_GUIDANCE_MODE\0")
    assert not _present("navpy-navigation.sock", b"navpy-guidance.s\0")
    assert _present(GUIDANCE_SOCKET, b"navpy-guidance.s\0")


def test_presence_check_needs_the_whole_name_outside_the_split_literal() -> None:
    assert not _present("NAVPY_GUIDANCE_COUNT", b"NAVPY_GUIDANCE_COMPLETE count=%u\0")


def test_entry_scripts_run_directly_without_pythonpath(tmp_path: Path) -> None:
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    for script in ("eval_simtime_navigation.py", "eval_lockstep_fleet.py", "run_lockstep_matrix.py"):
        result = subprocess.run([sys.executable, str(ROOT / "scripts" / script), "--help"],
                                cwd=tmp_path, env=env, capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, result.stderr
