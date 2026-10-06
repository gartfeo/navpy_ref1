"""A starved aircraft must fail loudly, not quietly produce a plausible number.

A SITL that is not keeping up still integrates its physics correctly in
simulator time: attitudes and positions stay entirely plausible while its
seconds stop being seconds. Nothing downstream reveals it -- the approach error
it yields looks like a navigation result. So the certification has to be a raise,
and it has to cover every aircraft, because the launcher covers only three.
"""

from __future__ import annotations

import itertools

import pytest

from scripts.eval_fleet_clock import measure_fleet_rates, verify_fleet_clock


class FakeMessage:
    def __init__(self, sys_id: int, boot_s: float) -> None:
        self._sys_id = sys_id
        self.time_boot_ms = boot_s * 1000.0

    def get_srcSystem(self) -> int:
        return self._sys_id


class FakeLink:
    """Replays scripted samples; the wall clock is supplied, never slept on."""

    def __init__(self, samples: list[tuple[int, float]]) -> None:
        self._samples = list(samples)

    def recv_match(self, *, type: str, blocking: bool, timeout: float):  # noqa: A002
        if not self._samples:
            return None
        sys_id, boot_s = self._samples.pop(0)
        return FakeMessage(sys_id, boot_s)


def _rates(samples, sys_ids, monkeypatch, *, span_s=1.0):
    """Drive the measurement with a wall clock that ticks once per sample."""
    ticks = itertools.count()
    monkeypatch.setattr(
        "scripts.eval_fleet_clock.time.monotonic", lambda: float(next(ticks))
    )
    return measure_fleet_rates(
        FakeLink(samples), sys_ids, span_s=span_s, timeout_s=50.0
    )


def test_an_aircraft_that_never_reports_is_a_failure_not_a_missing_datum(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    rates = _rates([(1, 0.0), (1, 1.0), (1, 2.0)], [1, 2], monkeypatch)

    assert rates[2] is None


def test_a_fleet_is_refused_when_any_member_is_off_rate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """One starved member makes its cell mean something else than the rest."""
    ticks = itertools.count()
    monkeypatch.setattr(
        "scripts.eval_fleet_clock.time.monotonic", lambda: float(next(ticks))
    )
    link = FakeLink([(1, 0.0), (2, 0.0), (1, 4.0), (2, 0.2), (1, 8.0), (2, 0.4)])

    with pytest.raises(RuntimeError, match="not running at"):
        verify_fleet_clock(link, [1, 2], 1.0, span_s=1.0, timeout_s=50.0)


def test_the_refusal_names_the_aircraft_that_failed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A fleet of 36 is unactionable if the error does not say which one."""
    ticks = itertools.count()
    monkeypatch.setattr(
        "scripts.eval_fleet_clock.time.monotonic", lambda: float(next(ticks))
    )
    link = FakeLink([(7, 0.0), (8, 0.0), (7, 1.0), (8, 1.0), (7, 2.0)])

    with pytest.raises(RuntimeError) as caught:
        verify_fleet_clock(link, [7, 8, 9], 1.0, span_s=1.0, timeout_s=50.0)

    assert "9=no samples" in str(caught.value)
