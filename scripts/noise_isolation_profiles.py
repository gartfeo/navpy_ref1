"""Explicit investigation defaults; no change to normal evaluation defaults."""

from pathlib import Path
import json
import subprocess

from eval_preboot_params import parse_parm
from eval_preboot_profile import combined_text, _read_template
from eval_simtime_step import ROOT, wsl_path

PROFILES = {
    "stock": {"SIM_NOISE_OFF": 0, "SIM_RATE_HZ": 1200},
    "noise-off": {"SIM_NOISE_OFF": 8191, "SIM_RATE_HZ": 1200},
    "noise-off-1000": {"SIM_NOISE_OFF": 8191, "SIM_RATE_HZ": 1000},
}
PROFILE_ORDER = ("stock", "noise-off-1000", "noise-off")


def cells() -> list[dict]:
    return [dict(instances=1, speedup=speed, delayed=delay, noise_profile=profile)
            for speed in (10., 1.) for delay in (False, True) for profile in PROFILE_ORDER]


def prepare_profile(case: Path, firmware: str, name: str) -> None:
    expected = PROFILES[name]
    template = firmware + "/Tools/autotest/models/plane.parm"
    base = _read_template(template)
    (case / "firmware-template.parm").write_text(base, encoding="utf-8", newline="\n")
    delta = case / "noise-profile.parm"
    values = dict(expected, LOG_DISARMED=1, LOG_FILE_DSRMROT=0)
    delta.write_text("".join(f"{key} {value}\n" for key, value in values.items()),
                     encoding="utf-8", newline="\n")
    combined = combined_text(delta, template=template)
    if parse_parm(combined) != dict(parse_parm(base), **values):
        raise ValueError("firmware template changed while composing profile")
    (case / "navigation-defaults.parm").write_text(combined, encoding="utf-8", newline="\n")


def log_operation(operation: str, firmware: str, vehicle: int, case: Path) -> dict:
    command = ["wsl.exe", "--exec", "timeout", "--kill-after=2s", "25s", "python3",
               wsl_path(ROOT / "scripts/noise_log_artifacts.py"),
               operation, "--root", firmware, "--vehicle", str(vehicle), "--case", wsl_path(case)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise RuntimeError(result.stderr or result.stdout)
    return json.loads(result.stdout)
