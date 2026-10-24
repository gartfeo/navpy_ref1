"""The air load report (``radio_budget.md`` item 3): framed bytes per sending node and window."""

from __future__ import annotations

from scripts.eval_gcs_demo_air_load import air_load

GRID = {"grid": {"sitl_ports": [["lockstep:serial0.1", "sitl1.serial1"],
                                ["lockstep:serial0.2", "sitl2.serial1"]]}}
TICK_US = 20_000
CLOSING = {"mavlink_counts": {"sitl1.serial1": {"router_skipped_bytes": {"skipped_bytes": 6}},
                              "router.backend": {"router_skipped_bytes": {"skipped_bytes": 0}}},
           "held": {"sitl2/1/sitl2.serial1": {"bytes": 15, "sha256": "x"}}}


def _emitted(origin: str, device: str, size: int) -> list:
    return ["emitted", [origin, 1, device, 1], [0, size], "0" * 64, 0, [1, 1], [-1, -1]]


def _tick(k: int, *packets: list, lifecycle: tuple = ()) -> dict:
    return {"k": k, "t_us": k * TICK_US, "packets": list(packets), "lifecycle": list(lifecycle)}


def _parked(t_us: int, *packets: list) -> dict:
    return {"parked": {"tick": 125, "t_us": t_us, "packets": list(packets)}}


def _records(*ending: dict, parked_us: int = 2_500_000) -> list[dict]:
    spawn = {"kind": "spawn"}
    return [
        GRID,
        _tick(1, _emitted("sitl1", "sitl1.serial1", 100), _emitted(4, "lockstep:router", 30)),
        _tick(25, _emitted("sitl1", "sitl1.serial1", 100)),
        # The spawn tick: its bytes were written before the spawn, so they are setup.
        _tick(50, _emitted("sitl1", "sitl1.serial1", 7), lifecycle=(spawn,)),
        _tick(60, _emitted("sitl2", "sitl2.serial1", 40),
              _emitted("sitl2", "lockstep:serial0.2", 500),  # a NavPy link: not the air
              ["copy", ["router", 1, "sitl1.serial1", 1], ["sitl2", 1, "sitl2.serial1", 1]]),
        _tick(100, _emitted("sitl1", "sitl1.serial1", 20)),
        _tick(124),
        _parked(parked_us, _emitted("sitl1", "sitl1.serial1", 5)),  # frame k_F+1, never replied to
        *ending,
        CLOSING,
    ]


def test_setup_and_mission_are_apart_and_only_the_radio_senders_count():
    report = air_load(_records())
    setup, mission = report["setup"], report["mission"]
    assert setup["seconds"] == 1.0  # SITL boot to the spawn tick
    assert setup["nodes"]["sitl1"] == {"bytes": 207, "mean_bytes_per_s": 207.0, "max_bytes_per_s": 207.0}
    assert setup["nodes"]["backend"]["bytes"] == 30 and setup["nodes"]["total"]["bytes"] == 237
    assert mission["seconds"] == 1.5  # to the parked frame's own sim time
    assert {name: node["bytes"] for name, node in mission["nodes"].items()} == {
        "backend": 0, "sitl1": 25, "sitl2": 40, "total": 65}


def test_a_partial_last_window_is_scaled_by_its_length():
    mission = air_load(_records())["mission"]["nodes"]
    # Windows (1.0 s, 2.0 s] and (2.0 s, 2.5 s]: sitl1 has 20 B, then the parked 5 B in half a second.
    assert mission["sitl1"]["max_bytes_per_s"] == 20.0
    assert mission["sitl1"]["mean_bytes_per_s"] == 16.7  # 25 B over 1.5 s
    assert mission["total"]["max_bytes_per_s"] == 60.0


def test_a_parked_frame_on_a_window_boundary_without_drain_ticks_keeps_its_bytes():
    """Codex P1: the parked frame has its own time, so its bytes end the last window."""
    mission = air_load(_records(parked_us=3_000_000))["mission"]
    assert mission["seconds"] == 2.0
    assert mission["nodes"]["sitl1"]["bytes"] == 25
    assert mission["nodes"]["sitl1"]["max_bytes_per_s"] == 20.0  # (2.0 s, 3.0 s] holds the parked 5 B


def test_drain_ticks_after_the_parked_frame_extend_the_measurement():
    drain = {"k": 125, "phase": "drain", "t_us": 2_520_000,
             "packets": [_emitted(4, "lockstep:router", 12)]}
    mission = air_load(_records(drain))["mission"]
    assert mission["seconds"] == 1.52
    assert mission["nodes"]["backend"]["bytes"] == 12 and mission["nodes"]["sitl1"]["bytes"] == 25


def test_skipped_and_held_bytes_are_reported_apart_and_loss_is_named():
    """The journal's grid names the radio's loss (``radio_link``); without it the load is offered."""
    lossy = _records()
    lossy[0] = {"grid": {**GRID["grid"], "radio_loss": {"model": "radio_byte_loss", "loss_pct": 0.3}}}
    report = air_load(lossy)
    assert report["skipped_bytes"] == {"backend": 0, "sitl1": 6, "sitl2": 0}
    assert report["held_bytes"] == {"backend": 0, "sitl1": 0, "sitl2": 15}
    assert report["measures"] == "observed after radio byte loss"
    assert air_load(_records())["measures"] == "offered load"
    clean = _records()
    clean[0] = {"grid": {**GRID["grid"], "radio_loss": {"model": "radio_byte_loss", "loss_pct": 0.0}}}
    assert air_load(clean)["measures"] == "offered load"


def test_a_run_without_a_spawn_is_all_setup():
    records = [r for r in _records() if not r.get("lifecycle")]
    report = air_load(records)
    assert report["mission"] == {"seconds": 0.0, "nodes": {}}
    assert report["setup"]["seconds"] == 2.5
    assert report["setup"]["nodes"]["sitl1"]["bytes"] == 225
