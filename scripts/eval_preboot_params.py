"""Confirm that a pre-boot parameter file actually took effect.

Supplying DEFAULTS to ``run_swarm.sh`` selects the file the SITL binary loads,
but it does not by itself decide the value the vehicle flies: ArduPilot applies
a defaults-file entry only when that parameter is not already configured in
storage (``AP_Param.cpp``), and ``run_swarm.sh`` clones each instance's eeprom
from a template.  A parameter saved in that template therefore outranks the
file, silently.

That failure is invisible in the result: the swarm launches, the mission flies,
and the numbers look ordinary while answering a question about a value that was
never in force.  So the requested overrides are read back from the live vehicle
before anything is measured.

Only the entries that DIFFER from the launcher's own template are checked.  The
template is what an unmodified launch would already have applied, so re-reading
all ~85 of its parameters would cost round trips without testing anything the
caller asked for.
"""
from __future__ import annotations

import math
import subprocess
import time
from typing import Any, Callable

import swarm_run_wsl as wsl
from eval_navigation_vehicle_config import message_from_target

LAUNCHER_TEMPLATE = "~/ardupilot/Tools/autotest/models/plane.parm"

from eval_param_float32 import _f32, _INTEGER_LIMITS, as_stored, same_value, strtof


def _read_wsl_file(path: str) -> str:
    probe = subprocess.run(wsl.wsl_argv(f"cat {path}"), capture_output=True)
    if probe.returncode != 0:
        raise RuntimeError(f"cannot read {path!r} inside WSL")
    return probe.stdout.decode("utf-8", errors="replace")


from scripts.eval_param_file import _FIRMWARE_ROW_BYTES, parse_parm


def overrides(defaults_path: str, template: str = LAUNCHER_TEMPLATE) -> dict[str, float]:
    """Return the entries that differ from what the launcher would apply anyway."""
    supplied = parse_parm(_read_wsl_file(shell_quote(defaults_path)))
    baseline = parse_parm(_read_wsl_file(template))
    return {
        name: value
        for name, value in supplied.items()
        # Absent from the template is an override: absence is no evidence that
        # storage already holds the requested value.
        #
        # Exact equality is right because both sides are already float32 --
        # the value the vehicle would store, converted once.  Two entries that
        # compare equal here really do store identically, so skipping one is
        # correct rather than a silent drop.
        if name not in baseline or baseline[name] != value
    }


def shell_quote(path: str) -> str:
    import shlex

    return shlex.quote(path)


def read_param_typed(
    master: Any, name: str, *, timeout_s: float = 5.0
) -> tuple[float, int] | None:
    """Read one parameter with the storage class the vehicle reports.

    The value alone is not enough to judge a match: an integer parameter
    rounds the request on the way in, so 119.99999 legitimately reads back as
    120.  PARAM_VALUE carries param_type for exactly this reason
    (GCS_Param.cpp sends mav_param_type of the stored class).
    """
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode("ascii"), -1
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is None or not message_from_target(
            message, master.target_system
        ):
            continue
        raw = getattr(message, "param_id", "")
        if isinstance(raw, bytes):
            raw = raw.decode("ascii", errors="ignore")
        if str(raw).strip(chr(0)) != name:
            continue
        try:
            return float(message.param_value), int(
                getattr(message, "param_type", 0)
            )
        except (TypeError, ValueError):
            return None
    return None


# Written after boot by paths OUTSIDE the sim-parameter loop, so they do not
# appear in sim_parameters(): the SIM_CPA stage configures its target
# (eval_sim_cpa_config.derive_params) and the terminal step-down sets the clock
# (pixel_pn_terminal_speed).  Values are irrelevant here -- only the names.
LATER_WRITERS: tuple[tuple[str, float], ...] = (
    ("SIM_CPA_LAT_HI", 0.0),
    ("SIM_CPA_LAT_LO", 0.0),
    ("SIM_CPA_LNG_HI", 0.0),
    ("SIM_CPA_LNG_LO", 0.0),
    ("SIM_CPA_ALT_CM", 0.0),
    ("SIM_CPA_ENABLE", 0.0),
    ("SIM_SPEEDUP", 0.0),
)


def reject_postboot_conflicts(
    wanted: dict[str, float], later: tuple[tuple[str, float], ...]
) -> None:
    """Raise if a value proven at boot is written again after it.

    verify() reads the vehicle before the harness configures it, which proves
    what the BOOT applied -- not what flies.  prepare_aircraft writes the sim
    parameters afterwards, and a later write of the same name simply replaces
    the pre-boot value, leaving a run that reports a clean pre-boot check and
    flies something else.

    That is worth refusing rather than warning about: the whole reason a value
    goes in the defaults file is that it must be set before boot, so writing it
    again afterwards is a mistake in the request, not a preference.
    """
    clashes = sorted(name for name, _ in later if name in wanted)
    if clashes:
        raise RuntimeError(
            "pre-boot parameters would be overwritten after boot ("
            + ", ".join(clashes)
            + "). verify() proves the boot value and the later write then "
            "replaces it, so the proof no longer describes the flight -- "
            "whatever value that write carries."
        )


def verify(master: Any, sys_ids: list[int], wanted: dict[str, float]) -> None:
    """Raise unless every requested override is live on every vehicle.

    This proves what BOOT applied.  Anything written later replaces it; see
    reject_postboot_conflicts.
    """
    if not wanted:
        return
    mismatches = []
    for sys_id in sys_ids:
        master.target_system = sys_id
        master.target_component = 1
        for name, value in sorted(wanted.items()):
            reading = read_param_typed(master, name)
            if reading is None:
                mismatches.append(f"{name} unreadable on {sys_id}")
                continue
            actual, param_type = reading
            limits = _INTEGER_LIMITS.get(param_type)
            # Against the CONVERTED request, which is what the vehicle
            # receives: 127.0000001 arrives as 127.0 and is a fine AP_Int8
            # request, where the arithmetic value of the text is not.
            stored = _f32(value)
            if limits is not None and not limits[0] <= stored <= limits[1]:
                # The vehicle silently clamps, so a readback CAN match and
                # the check would pass -- certifying a flight that did not
                # test the requested value.  At the AP_Int32 upper bound the
                # C conversion is undefined as well, because constrain_float
                # converts INT32_MAX to a float32 that is one too large.
                arrived = ("" if stored == value
                           else f" (arrives as {stored:.0f} in a float32 "
                                "parameter field)")
                mismatches.append(
                    f"{name} asks {value:g} on {sys_id}{arrived}, outside "
                    f"the [{limits[0]}, {limits[1]}] this parameter holds"
                )
                continue
            if not same_value(actual, value, param_type):
                mismatches.append(
                    f"{name} is {actual:g} on {sys_id}, asked {value:g}"
                )
    if mismatches:
        raise RuntimeError(
            "pre-boot parameters did not take effect ("
            + "; ".join(mismatches)
            + "). A value saved in the instance eeprom outranks a defaults "
            "file, so the flight would not have tested what was requested."
        )


def verify_defaults(
    master: Any, sys_ids: list[int], defaults: str,
    owned: dict[str, float] | None,
    later_writers: Callable[[], tuple[tuple[str, float], ...]],
) -> None:
    """Verify both file overrides and profile-owned values before flight.

    The template may share a profile value while persisted EEPROM overrides
    it. Profile-owned values therefore require readback even when they do not
    differ from the template. Reject later writes before checking readback.
    """
    requested = overrides(defaults)
    requested.update(owned or {})
    reject_postboot_conflicts(requested, later_writers())
    verify(master, sys_ids, requested)
