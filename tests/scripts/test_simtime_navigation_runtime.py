"""Replay real rendering/navigation with perturbed host scheduling."""

from dataclasses import replace

import pytest

from navpy.modules.common.models.location import Location
from scripts.simtime_navigation_protocol import ATTITUDE, Snapshot
from tests.scripts.test_simtime_navigation_protocol import packet


def replay(delays: tuple[float, ...], fraction: tuple[int, int] = (4, 5)) -> list[bytes]:
    from scripts.simtime_navigation_runtime import SynchronousNavigation
    wall = [100.0]
    engine = SynchronousNavigation(Location(40.01, 44., 1300., is_absolute=True),
                                 speedup=10, camera_fraction=fraction,
                                 wall_now=lambda: wall[0])
    outputs = []
    try:
        for index in range(30):
            snap = Snapshot.decode(packet(index + 1))
            snap = replace(snap, truth=replace(snap.truth, latitude=40. + index * .000004))
            receipt = wall[0]
            wall[0] += delays[index % len(delays)]
            command, _ = engine.advance(snap, receipt)
            outputs.append(command.reply(snap.identity))
            wall[0] += .002
        assert engine.seed_step == 1
        assert engine.command_count > 20
    finally:
        engine.close()
    return outputs


@pytest.mark.parametrize("fraction", [(1, 1), (4, 5)])
def test_actual_camera_and_law_are_repeatable_under_host_delay(fraction: tuple[int, int]) -> None:
    assert replay((0.,), fraction) == replay((0., .002, 0., .006, .001), fraction)


def test_expired_wall_receipt_invalidates_instead_of_becoming_a_different_flight() -> None:
    with pytest.raises(ValueError, match="receipt liveness"):
        replay((0., 0., .15))


def test_missing_camera_tick_reissues_the_previous_command() -> None:
    from scripts.simtime_navigation_protocol import COMMAND, HEADER
    output = replay((0.,))
    # Capture ticks 0,2,3,4,5,7: tick6 must reissue tick5's command.
    previous = COMMAND.unpack_from(output[5], HEADER.size)
    held = COMMAND.unpack_from(output[6], HEADER.size)
    assert previous[2] == held[2] == ATTITUDE
    assert previous[3:] == held[3:]
    assert previous[:2] != held[:2]


def test_drain_stops_admission_and_does_not_compute_discarded_commands() -> None:
    from scripts.simtime_navigation_runtime import SynchronousNavigation
    engine = SynchronousNavigation(Location(40.01, 44., 1300., is_absolute=True),
                                 speedup=10, wall_now=lambda: 100.)
    snapshot = Snapshot.decode(packet())
    try:
        engine.advance(snapshot, 100.)
        before = engine.command_count
        assert engine.drain()["draining"]
        assert engine.command_count == before
        with pytest.raises(ValueError, match="drained"):
            engine.advance(snapshot, 100.)
    finally:
        engine.close()


def test_diagnostic_telemetry_and_roll_provider_do_not_change_commands(monkeypatch) -> None:
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.navigation.nav.vision_nav.diagnostics import TerminalDiagnosticSnapshot
    from scripts import simtime_navigation_runtime as adapter
    baseline = replay((0.,))
    compose = adapter.compose_terminal_runtime
    samples = []
    class Reader:
        def read(self):
            samples.append(True)
            return TerminalDiagnosticSnapshot(
                attitude=Attitude(35., 170., 27.),
                location=Location(-35., -120., 9000., is_absolute=True))
    def with_telemetry(**kwargs):
        kwargs['aircraft_roll_deg'] = lambda: 27.
        kwargs['diagnostic_reader'] = Reader()
        return compose(**kwargs)
    monkeypatch.setattr(adapter, 'compose_terminal_runtime', with_telemetry)
    assert replay((0., .002, .004)) == baseline
    assert samples, 'diagnostic independence must exercise the actual diagnostic reader'


def test_allowed_pitch_input_changes_the_replayed_commands(monkeypatch) -> None:
    baseline = replay((0.,))
    decode = Snapshot.decode
    def changed(packet_bytes):
        snapshot = decode(packet_bytes)
        return replace(snapshot, observation=replace(snapshot.observation, pitch_rad=.08))
    monkeypatch.setattr(Snapshot, 'decode', staticmethod(changed))
    assert replay((0.,)) != baseline


def test_missing_throttle_configuration_cannot_silently_delegate_to_tecs() -> None:
    from scripts.simtime_navigation_runtime import SynchronousNavigation
    engine = SynchronousNavigation(Location(40.01, 44., 1300., is_absolute=True),
                                 speedup=10, wall_now=lambda: 100.)
    snapshot = Snapshot.decode(packet())
    snapshot = replace(snapshot, limits=(*snapshot.limits[:4], -1.0))
    try:
        with pytest.raises(ValueError, match="throttle configuration"):
            engine.advance(snapshot, 100.)
    finally:
        engine.close()
