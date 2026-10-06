"""Matrix argument parsing and case generation."""

from __future__ import annotations

import argparse
import math
from collections.abc import Iterable, Sequence

from eval_navigation_models import MatrixCase


def parse_float_list(raw: str) -> list[float]:
    values = [float(part.strip()) for part in raw.split(",") if part.strip()]
    if not values:
        raise argparse.ArgumentTypeError("at least one numeric value is required")
    if not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError("values must be finite")
    return values


def parse_int_list(raw: str) -> list[int]:
    values = [int(part.strip()) for part in raw.split(",") if part.strip()]
    if not values:
        raise argparse.ArgumentTypeError("at least one integer is required")
    return values


def parse_poi_alts(raw: str) -> list[int]:
    result: list[int] = []
    for token in (part.strip() for part in raw.split(",")):
        if not token:
            continue
        try:
            value = float(token)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"invalid --poi-alts value {token!r}"
            ) from exc
        if not math.isfinite(value):
            raise argparse.ArgumentTypeError("--poi-alts values must be finite")
        if value < 0:
            raise argparse.ArgumentTypeError(
                "--poi-alts values must be non-negative"
            )
        if not math.isclose(value, round(value), abs_tol=1e-9):
            raise argparse.ArgumentTypeError(
                "--poi-alts must be whole numbers "
                "(NavPy AAS_TARG_ALT is integer)"
            )
        altitude = int(round(value))
        if altitude not in result:
            result.append(altitude)
    if not result:
        raise argparse.ArgumentTypeError("at least one POI altitude is required")
    return result


def parse_navigation_speedups(raw: str) -> list[float] | None:
    if raw.strip().lower() == "match":
        return None
    values = parse_float_list(raw)
    if any(value <= 0 for value in values):
        raise argparse.ArgumentTypeError("navigation speedups must be > 0")
    return values


def matrix_cases(
    poi_alts: Sequence[int],
    speedups: Sequence[float],
    navigation_speedups: Sequence[float] | None,
    winds: Sequence[float],
    directions: Sequence[int],
) -> Iterable[MatrixCase]:
    for poi_altitude in poi_alts:
        for speedup in speedups:
            nav_rates = navigation_speedups or [float(speedup)]
            for navigation_speedup in nav_rates:
                for wind in winds:
                    if abs(wind) < 1e-9:
                        yield MatrixCase(
                            poi_altitude,
                            speedup,
                            float(navigation_speedup),
                            0.0,
                            0,
                        )
                        continue
                    for direction in directions:
                        yield MatrixCase(
                            poi_altitude,
                            speedup,
                            float(navigation_speedup),
                            wind,
                            direction,
                        )
