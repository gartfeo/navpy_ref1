"""Unreal's pose stamp log (-NavPyPoseStampLog=1, NavPyVehicleStateReceiver): its received samples, each tick's
applied pose with its stamp, the vehicle move cost, and the check that each applied pose is the samples
interpolated at its stamp (tools/unreal/pose_replay.py --unreal-log)."""
from __future__ import annotations

import bisect
import math
import re
import statistics
from dataclasses import dataclass
from typing import Iterable, Optional


def quantiles(values: list[float]) -> Optional[dict]:
    if not values:
        return None
    ordered = sorted(values)
    return {"n": len(ordered), "median": round(statistics.median(ordered), 5),
            "p95": round(ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))], 5), "max": round(ordered[-1], 5)}


# --- Unreal log (-NavPyPoseStampLog=1) -----------------------------------------------------------------------------
_STAMP = r"\[(\d{4})\.(\d\d)\.(\d\d)-(\d\d)\.(\d\d)\.(\d\d):(\d{3})\]"
_VEC = r"loc=([-\d.]+),([-\d.]+),([-\d.]+) quat=([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)"
_SAMPLE = re.compile(r"NavPyPoseSample board_us=(\d+) seq=(\d+) " + _VEC)
_APPLIED = re.compile(_STAMP + r".*NavPyPoseApplied board_us=(\d+) seq=(\d+) valid=(\d) exact=(\d) interp=(\d) " + _VEC)
_MOVE = re.compile(r"NavPyMoveCost moves=(\d+) mean_ms=([\d.]+) max_ms=([\d.]+)")


@dataclass(frozen=True)
class LoggedPose:
    board_time_us: int
    location: tuple[float, float, float]
    quat: tuple[float, float, float, float]          # x, y, z, w
    log_s: float = 0.0
    valid: bool = True
    exact: bool = False
    interpolated: bool = False


def _log_seconds(groups: tuple[str, ...]) -> float:
    _y, _mo, d, h, mi, s, ms = (int(g) for g in groups)
    return ((d * 24 + h) * 60 + mi) * 60 + s + ms / 1000.0


def parse_unreal_log(lines: Iterable[str]) -> tuple[list[LoggedPose], list[LoggedPose], list[dict]]:
    samples, applied, moves = [], [], []
    for line in lines:
        if (m := _SAMPLE.search(line)) is not None:
            v = [float(x) for x in m.groups()[2:]]
            samples.append(LoggedPose(int(m.group(1)), (v[0], v[1], v[2]), (v[3], v[4], v[5], v[6])))
        elif (m := _APPLIED.search(line)) is not None:
            g = m.groups()
            v = [float(x) for x in g[12:]]
            applied.append(LoggedPose(int(g[7]), (v[0], v[1], v[2]), (v[3], v[4], v[5], v[6]), _log_seconds(g[:7]),
                                      g[9] == "1", g[10] == "1", g[11] == "1"))
        elif (m := _MOVE.search(line)) is not None:
            moves.append({"moves": int(m.group(1)), "mean_ms": float(m.group(2)), "max_ms": float(m.group(3))})
    return samples, applied, moves


def _slerp(a: tuple, b: tuple, alpha: float) -> tuple:
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-x for x in b), -dot
    if dot > 0.9995:
        q = tuple(x + alpha * (y - x) for x, y in zip(a, b))
    else:
        theta = math.acos(min(1.0, dot))
        wa, wb = math.sin((1 - alpha) * theta) / math.sin(theta), math.sin(alpha * theta) / math.sin(theta)
        q = tuple(wa * x + wb * y for x, y in zip(a, b))
    norm = math.sqrt(sum(x * x for x in q))
    return tuple(x / norm for x in q)


def _angle_deg(a: tuple, b: tuple) -> float:
    """Rotation between two unit quaternions; |a - b| = 2 sin(angle / 4) is well conditioned near zero."""
    if sum(x * y for x, y in zip(a, b)) < 0:
        b = tuple(-x for x in b)
    return math.degrees(4.0 * math.asin(min(1.0, math.dist(a, b) / 2.0)))


def interpolate_samples(samples: list[LoggedPose], board_time_us: int) -> Optional[tuple[tuple, tuple]]:
    times = [s.board_time_us for s in samples]
    i = bisect.bisect_right(times, board_time_us)
    if i == 0:
        return None
    a = samples[i - 1]
    if a.board_time_us == board_time_us:
        return a.location, a.quat
    if i == len(samples):
        return None
    b = samples[i]
    alpha = (board_time_us - a.board_time_us) / (b.board_time_us - a.board_time_us)
    return tuple(x + alpha * (y - x) for x, y in zip(a.location, b.location)), _slerp(a.quat, b.quat, alpha)


def stamp_check(samples: list[LoggedPose], applied: list[LoggedPose]) -> Optional[dict]:
    """Each valid applied pose against the received samples interpolated at its stamp."""
    ordered = sorted({s.board_time_us: s for s in samples}.values(), key=lambda s: s.board_time_us)
    position_cm, angle_deg = [], []
    for pose in applied:
        if not pose.valid:
            continue
        expected = interpolate_samples(ordered, pose.board_time_us)
        if expected is None:
            continue
        position_cm.append(math.dist(expected[0], pose.location))
        angle_deg.append(_angle_deg(expected[1], pose.quat))
    if not position_cm:
        return None
    return {"checked": len(position_cm), "position_cm": quantiles(position_cm), "attitude_deg": quantiles(angle_deg)}
