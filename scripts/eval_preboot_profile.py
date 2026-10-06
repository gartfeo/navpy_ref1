"""Build the pre-boot defaults file the evaluation bench actually boots.

``run_swarm.sh`` takes ONE ``DEFAULTS`` path and its ``[[ -f ]]`` preflight
rejects a comma list, so a profile cannot simply be layered on top of the
launcher's ``models/plane.parm`` at the command line -- naming a file REPLACES
that template outright, taking the whole airframe tuning with it.

So the file is built here: the template, then this repo's delta
(``eval_preboot_defaults.parm``).  The template is read from WSL at launch
rather than copied into the repo, so ArduPilot stays the source of truth for
everything the delta does not mention.

A name the delta sets is DROPPED from the template half rather than repeated.
ArduPilot has no single answer for a name given twice in one defaults file --
``set_float`` runs per row so RAM holds the last, while the override lookup
returns the first (AP_Param.cpp:2560) -- and none of the delta's names is in
the template today, so the collision would arrive silently on an upstream
change rather than announce itself.

The result is written beside the run's own evidence, not into a shared home
directory: concurrent sessions cannot collide on it, and the exact bytes the
aircraft booted stay archived with the numbers they produced.
"""
from __future__ import annotations

import re
import subprocess
import textwrap
from pathlib import Path
from typing import Any

import eval_preboot_params as preboot
from scripts.eval_certificate_process import (
    normalize_wsl_path, run_text, wsl_executable,
)

PROFILE = Path(__file__).resolve().parent / "eval_preboot_defaults.parm"

# Long enough for a cold WSL to answer, short enough that a broken install
# fails the launch instead of hanging it.
WSL_TIMEOUT_S = 30.0


def _wsl() -> str:
    executable = wsl_executable()
    if executable is None:
        raise RuntimeError("wsl.exe is not on PATH, so SITL cannot be launched")
    return executable


def _read_template(path: str) -> str:
    """Read the launcher template out of WSL, under a bound, as UTF-8.

    No shell: ``normalize_wsl_path`` expands the constant's leading "~" on its
    own, so there is no command string for a path to be quoted into.  The read
    is bounded because an unbounded one would sit forever with no message --
    before the span is claimed, so nothing leaks, but the launch never starts.

    Decoded here rather than through ``run_text`` because that asks
    ``subprocess`` for text, which uses the console codepage: cp1252 on this
    host.  A 62-byte UTF-8 comment comes back as 122 bytes once re-encoded,
    and the firmware row-length guard then rejects a template that was fine.
    """
    executable = _wsl()
    resolved, reason = normalize_wsl_path(
        path, executable=executable, timeout_s=WSL_TIMEOUT_S
    )
    if resolved is None:
        raise RuntimeError(f"cannot resolve {path!r} inside WSL: {reason}")
    try:
        probe = subprocess.run(
            [executable, "--exec", "cat", "--", resolved],
            capture_output=True,
            timeout=WSL_TIMEOUT_S,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"cannot read {resolved!r} inside WSL ({exc})") from exc
    if probe.returncode != 0:
        detail = probe.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"cannot read {resolved!r} inside WSL: {detail}")
    return probe.stdout.decode("utf-8", errors="replace")


def _row_name(raw: str) -> str | None:
    """The parameter a template row sets, as ARDUPILOT would read it.

    Deliberately not the same as stripping a "#" comment: the firmware honours
    "#" only at line[0] (AP_Param.cpp:2263) and otherwise tokenises on
    ", =\\t\\r\\n", so "GPS1_DELAY_MS #comment" is a PARAMETER row to it, with
    strtof("#comment") giving 0.  Reading that as a comment here would leave
    the row in place next to the profile's own, which is the duplicate this
    module exists to prevent.
    """
    line = raw.strip("\r\n")
    if not line or line[0] == "#":
        return None
    parts = [part for part in re.split(r"[,= \t]+", line.strip()) if part]
    return parts[0].upper() if len(parts) >= 2 else None


def _names(text: str) -> set[str]:
    return set(preboot.parse_parm(text))


def combined_text(
    profile: Path = PROFILE, template: str = preboot.LAUNCHER_TEMPLATE
) -> str:
    """Return the launcher template with the profile's overrides applied."""
    profile_text = profile.read_text(encoding="utf-8")
    overridden = _names(profile_text)
    if not overridden:
        # An empty profile would boot the plain template while every caller
        # believed the bench configuration was in force.
        raise RuntimeError(f"{profile} sets no parameters")
    template_text = _read_template(template)
    # A comment or blank line has no name, so it is kept: the template's own
    # annotations survive into the file the aircraft boots.
    kept = [
        raw for raw in template_text.splitlines()
        if _row_name(raw) not in overridden
    ]
    merged = "\n".join(
        [
            "# GENERATED by eval_preboot_profile.py -- do not edit.",
            *["# " + line for line in textwrap.wrap(
                f"{template}, then {profile.name}.".encode("ascii", "backslashreplace").decode("ascii"),
                width=preboot._FIRMWARE_ROW_BYTES - 2, break_on_hyphens=False)],
            *kept,
            "",
            profile_text.rstrip("\n"),
            "",
        ]
    )
    # Parses the bytes that will actually be booted: catches an overlong row,
    # an unreadable value, and a duplicate the template brought with it.
    preboot.parse_parm(merged)
    return merged


def materialise(
    destination: Path,
    *,
    profile: Path = PROFILE,
    template: str = preboot.LAUNCHER_TEMPLATE,
) -> str:
    """Write the combined defaults file and return its path as WSL sees it."""
    destination.write_text(combined_text(profile, template), encoding="utf-8")
    executable = _wsl()
    wsl_path, reason = run_text(
        [executable, "--exec", "wslpath", "-a", str(destination)],
        timeout_s=WSL_TIMEOUT_S,
    )
    if not wsl_path:
        raise RuntimeError(
            f"cannot express {destination} as a WSL path: "
            f"{reason or 'wslpath returned nothing'}"
        )
    return wsl_path


def owned() -> dict[str, float]:
    """Every parameter the shipped profile sets, whatever the template says.

    ``overrides()`` reports only what DIFFERS from the launcher template,
    which is right for an arbitrary file: the rest is what an unmodified
    launch would apply anyway.  It is not enough for this profile.  The
    cloned eeprom outranks the defaults file AND the template alike, so a
    profile value the template happens to share would fly unverified -- and
    the whole point of shipping the profile is that these ten values are
    proven on the vehicle, not assumed.
    """
    return preboot.parse_parm(PROFILE.read_text(encoding="utf-8"))


def defaults_for(args: Any, case_dir: Path) -> tuple[str | None, dict[str, float]]:
    """Decide which pre-boot defaults file this case boots, and build it.

    Returns the path and the parameters that must be proven on the vehicle
    whatever the template holds -- empty when the caller supplied their own
    file, because then the profile is not in force.

    The profile is the default because the alternative is not "no
    configuration" but a DIFFERENT one: the assumed GPS lag and
    EK3_HGT_DELAY are both wrong for SITL, pushing the published position
    the same way by different amounts, and the leftover mismatch between
    the channels displaces it vertically at closest approach.
    A run without the profile measures that, not the navigation law.

    ``--sitl-defaults`` still means exactly what it always did -- a complete
    replacement for the launcher template -- so a caller who names a file gets
    that file and nothing added to it.

    Resolved per case, never cached: each repetition writes its own copy next
    to its own results, so the bytes an aircraft booted stay with the numbers
    it produced.
    """
    chosen = getattr(args, "sitl_defaults", None)
    if chosen:
        return chosen, {}
    if getattr(args, "no_preboot_profile", False):
        return None, {}
    return materialise(case_dir / "preboot.parm"), owned()
