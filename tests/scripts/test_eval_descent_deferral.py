from __future__ import annotations

import csv
from pathlib import Path

import pytest

from scripts.eval_descent_deferral import DESCENT_FRACTIONS, deferral


def _write(path: Path, rows: list[tuple[float, float]]) -> Path:
    """Write a navigation compact log, including the trailing SNAP summary row."""
    log = path / "uav_121_navigation_compact.csv"
    with log.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["ts", "dist", "h_dist", "v_dist", "cmd_p"])
        for horizontal, vertical in rows:
            writer.writerow(["12:00:00.000", horizontal + vertical, horizontal,
                             vertical, -10.0])
        writer.writerow(["12:00:01.000", "SNAP(VISION-NAV-PN)", "", "", ""])
    return log


def _constant_glide(steps: int = 100) -> list[tuple[float, float]]:
    """Height comes off exactly in step with ground covered."""
    return [
        (888.0 * (1 - n / steps), 340.0 * (1 - n / steps))
        for n in range(steps + 1)
    ]


def _deferred(steps: int = 100) -> list[tuple[float, float]]:
    """Level until 60% of the leg is flown, then all the descent at once."""
    rows = []
    for n in range(steps + 1):
        flown = n / steps
        horizontal = 888.0 * (1 - flown)
        vertical = 340.0 if flown < 0.6 else 340.0 * (1 - (flown - 0.6) / 0.4)
        rows.append((horizontal, vertical))
    return rows


def test_constant_glide_sits_on_the_diagonal(tmp_path: Path) -> None:
    result = deferral(_write(tmp_path, _constant_glide()))

    for fraction in DESCENT_FRACTIONS:
        assert result.leg_fraction_at[fraction] == pytest.approx(fraction, abs=0.02)
    assert result.worst_deferral == pytest.approx(0.0, abs=0.02)


def test_deferred_descent_is_far_above_the_diagonal(tmp_path: Path) -> None:
    result = deferral(_write(tmp_path, _deferred()))

    assert result.leg_fraction_at[0.1] == pytest.approx(0.64, abs=0.03)
    assert result.leg_fraction_at[0.5] == pytest.approx(0.80, abs=0.03)
    assert result.worst_deferral > 0.3


def test_entry_geometry_is_reported(tmp_path: Path) -> None:
    result = deferral(_write(tmp_path, _constant_glide()))

    assert result.entry_horizontal_m == pytest.approx(888.0)
    assert result.entry_vertical_m == pytest.approx(340.0)


def test_metric_is_dimensionless_across_geometries(tmp_path: Path) -> None:
    """The point of the metric: a 300 m leg is comparable to an 888 m one."""
    (tmp_path / "a").mkdir(parents=True, exist_ok=True)
    (tmp_path / "b").mkdir(parents=True, exist_ok=True)
    long_leg = deferral(_write(tmp_path / "a", _deferred()))
    # Case B from the matched-geometry design: 300 m leg, 115 m drop, same
    # 21 degree chord angle, so only the scale differs.
    short = [(300.0 * h / 888.0, 115.0 * v / 340.0) for h, v in _deferred()]
    short_leg = deferral(_write(tmp_path / "b", short))

    for fraction in DESCENT_FRACTIONS:
        assert short_leg.leg_fraction_at[fraction] == pytest.approx(
            long_leg.leg_fraction_at[fraction], abs=0.02
        )


def test_level_flight_never_reaches_the_checkpoints(tmp_path: Path) -> None:
    rows = [(888.0 * (1 - n / 50), 340.0) for n in range(51)]

    result = deferral(_write(tmp_path, rows))

    assert result.leg_fraction_at == {}


def test_no_height_to_lose_is_refused(tmp_path: Path) -> None:
    rows = [(888.0 * (1 - n / 50), 0.0) for n in range(51)]

    with pytest.raises(ValueError, match="no height to lose"):
        deferral(_write(tmp_path, rows))


def test_too_few_rows_is_refused(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="insufficient navigation rows"):
        deferral(_write(tmp_path, [(888.0, 340.0)]))
