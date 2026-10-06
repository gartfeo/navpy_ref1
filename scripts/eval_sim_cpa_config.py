"""Per-case SIM_CPA module configuration for the single-UAV driver.

Derives the module's decimal-chunked POI parameters from the case's
resolved POI and pushes them with exact-echo checks -- the shared
``set_param`` helper accepts echoes within 1%, which cannot certify chunk
identity (a one-unit LAT_LO error is a centimetre of POI).  The SCPC
rows in the BIN remain the configuration authority; the echoes only gate
whether the flight proceeds with the module armed.

Policy (R1/R2): the operator may
disable the cross-check (``--sim-cpa off``, or the legacy
``--sitl-param SIM_CPA_ENABLE=0`` spelling) but may never redirect its
POI -- any other explicit ``SIM_CPA_*`` override is rejected before
launch.  A binary without the parameter set is recorded as unsupported
and the flight continues stream-scored; coverage is enforced by A/B
eligibility, not by crashing the case.

Deliberately NOT wired into the shared ``eval_sim_parameters`` tuple:
the three-UAV and SIYI harnesses must keep their behavior, including the
manual chunk-push flow used for the 2026-09-03 module validation.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any, Callable

from pymavlink import mavutil

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_sim_cpa_block import (  # noqa: E402
    CONFIG_CONFIGURED, CONFIG_DISABLED, CONFIG_FAILED, CONFIG_UNSUPPORTED,
    default_configuration,
)
from eval_sim_parameters import parse_sitl_param  # noqa: E402

MODE_AUTO = "auto"
MODE_OFF = "off"
SIM_CPA_MODES = (MODE_AUTO, MODE_OFF)

ENABLE_PARAM = "SIM_CPA_ENABLE"
CONFIG_RECORD_NAME = "sim_cpa_config.json"

# Module firmware limits (SIM_CPA.cpp @Range): chunk magnitudes stay below
# 2^24 so the REAL32 parameter transport is exact.
_MAX_ALT_CM = 1_000_000


def poi_ints(
    lat_deg: float, lon_deg: float, abs_alt_m: float
) -> tuple[int, int, int]:
    """The integer POI the module scores against, from the case POI."""
    return (
        round(lat_deg * 1e7),
        round(lon_deg * 1e7),
        round(abs_alt_m * 100),
    )


def chunk(value_e7: int) -> tuple[int, int]:
    """Decimal HI/LO split, truncated toward zero: value = HI*10000 + LO."""
    hi = int(value_e7 / 10000)
    return hi, value_e7 - hi * 10000


def derive_params(
    lat_deg: float, lon_deg: float, abs_alt_m: float
) -> list[tuple[str, int]]:
    """The full push list, chunks first and ENABLE last (module contract)."""
    lat_e7, lng_e7, alt_cm = poi_ints(lat_deg, lon_deg, abs_alt_m)
    if abs(lat_e7) > 90 * 10**7 or abs(lng_e7) > 180 * 10**7:
        raise ValueError(
            f"POI out of coordinate range: lat_e7={lat_e7} lng_e7={lng_e7}"
        )
    if abs(alt_cm) > _MAX_ALT_CM:
        raise ValueError(
            f"POI altitude outside the module's exact range: {alt_cm}cm"
        )
    lat_hi, lat_lo = chunk(lat_e7)
    lng_hi, lng_lo = chunk(lng_e7)
    return [
        ("SIM_CPA_LAT_HI", lat_hi),
        ("SIM_CPA_LAT_LO", lat_lo),
        ("SIM_CPA_LNG_HI", lng_hi),
        ("SIM_CPA_LNG_LO", lng_lo),
        ("SIM_CPA_ALT_CM", alt_cm),
        (ENABLE_PARAM, 1),
    ]


def resolve_mode(
    sim_cpa_arg: str | None,
    raw_sitl_params: list[str] | None,
) -> str:
    """Effective mode from the flag and the legacy ENABLE=0 spelling.

    Any explicit ``SIM_CPA_*`` override other than exactly ENABLE=0 is a
    redirected cross-check and is rejected.  ENABLE=0 normalizes to
    ``off`` for compatibility with the pre-integration invocation; asking
    for that AND ``--sim-cpa auto`` in the same run is a contradiction,
    not a precedence question.
    """
    legacy_off = False
    for raw in raw_sitl_params or []:
        name, value = parse_sitl_param(raw)
        if not name.startswith("SIM_CPA_"):
            continue
        if name == ENABLE_PARAM and value == 0.0:
            legacy_off = True
            continue
        raise ValueError(
            f"--sitl-param {name} would redirect the SIM_CPA cross-check; "
            "the module POI always comes from the case POI "
            "(disable with --sim-cpa off if the module must stay dark)"
        )
    if sim_cpa_arg is None:
        return MODE_OFF if legacy_off else MODE_AUTO
    if sim_cpa_arg not in SIM_CPA_MODES:
        raise ValueError(f"--sim-cpa must be one of {SIM_CPA_MODES}")
    if sim_cpa_arg == MODE_AUTO and legacy_off:
        raise ValueError(
            "--sim-cpa auto conflicts with --sitl-param SIM_CPA_ENABLE=0"
        )
    return sim_cpa_arg


def set_param_exact(
    master: Any,
    name: str,
    value: int,
    *,
    timeout_s: float = 5.0,
) -> bool:
    """PARAM_SET with a bit-exact REAL32 echo requirement.

    Every SIM_CPA value is an integer below 2^24, so ``float(value)`` is
    exactly representable and the echo must equal it exactly -- anything
    else means the transport or the firmware altered the chunk.
    """
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        message = master.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is None:
            continue
        if getattr(message, "get_srcSystem", lambda: None)() not in (
            None, master.target_system
        ):
            continue
        raw = getattr(message, "param_id", "")
        if isinstance(raw, bytes):
            raw = raw.decode("ascii", errors="ignore")
        if str(raw).strip("\x00") != name:
            continue
        try:
            echoed = float(getattr(message, "param_value"))
        except (TypeError, ValueError):
            return False
        return echoed == float(value)
    return False


def configure_sim_cpa(
    master: Any,
    *,
    mode: str,
    lat_deg: float,
    lon_deg: float,
    abs_alt_m: float,
    case_dir: Path | None = None,
    read_param: Callable[..., Any] | None = None,
    set_exact: Callable[..., bool] | None = None,
) -> dict[str, Any]:
    """Probe and configure the module; returns the configuration record.

    Best-effort by design: every failure is recorded, none raises.  The
    sequence stops at the first failed push so a partially-configured
    module can never be armed (ENABLE is last).
    """
    if read_param is None:
        from eval_navigation_vehicle_config import read_param as _read_param
        read_param = _read_param
    if set_exact is None:
        set_exact = set_param_exact
    record = default_configuration()
    record["mode"] = mode
    lat_e7, lng_e7, alt_cm = poi_ints(lat_deg, lon_deg, abs_alt_m)
    record["expected_poi"] = {
        "lat_e7": lat_e7, "lng_e7": lng_e7, "alt_cm": alt_cm,
    }
    try:
        probe = read_param(master, ENABLE_PARAM)
        if probe is None:
            record["status"] = CONFIG_UNSUPPORTED
            record["error"] = f"{ENABLE_PARAM} not served by this binary"
            return record
        if mode == MODE_OFF:
            record["requested_params"] = [[ENABLE_PARAM, 0]]
            if set_exact(master, ENABLE_PARAM, 0):
                record["acknowledged_params"] = [[ENABLE_PARAM, 0]]
                record["status"] = CONFIG_DISABLED
            else:
                record["status"] = CONFIG_FAILED
                record["failed_param"] = ENABLE_PARAM
                record["error"] = "disable push was not exactly echoed"
            return record
        push_list: list[tuple[str, int]] = []
        if float(probe) != 0.0:
            # A stale template EEPROM can boot with the module armed on an
            # old POI; disarm before touching chunks so no interval is
            # ever scored against a half-written configuration.
            push_list.append((ENABLE_PARAM, 0))
        push_list.extend(derive_params(lat_deg, lon_deg, abs_alt_m))
        record["requested_params"] = [list(pair) for pair in push_list]
        acknowledged: list[list[Any]] = []
        for name, value in push_list:
            if not set_exact(master, name, value):
                record["acknowledged_params"] = acknowledged
                record["status"] = CONFIG_FAILED
                record["failed_param"] = name
                record["error"] = (
                    f"{name}={value} was not exactly echoed; later "
                    "parameters were not attempted"
                )
                return record
            acknowledged.append([name, value])
        record["acknowledged_params"] = acknowledged
        record["status"] = CONFIG_CONFIGURED
        return record
    except Exception as error:  # never fail the flight for the cross-check
        record["status"] = CONFIG_FAILED
        record["error"] = f"{type(error).__name__}: {error}"
        return record
    finally:
        if case_dir is not None:
            try:
                (case_dir / CONFIG_RECORD_NAME).write_text(
                    json.dumps(record, indent=2), encoding="utf-8"
                )
            except OSError:
                pass


__all__ = [
    "CONFIG_RECORD_NAME",
    "ENABLE_PARAM",
    "MODE_AUTO",
    "MODE_OFF",
    "SIM_CPA_MODES",
    "chunk",
    "configure_sim_cpa",
    "derive_params",
    "resolve_mode",
    "set_param_exact",
    "poi_ints",
]
