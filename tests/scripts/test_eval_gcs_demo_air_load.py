"""The air load report (``radio_budget.md`` item 3): framed bytes per sending node and window."""

from __future__ import annotations

from scripts.eval_gcs_demo_air_load import air_load

GRID = {"grid": {"sitl_ports": [["lockstep:serial0.1", "sitl1.serial1"],
                                ["lockstep:serial0.2", "sitl2.serial1"]]}}
TICK_US = 20_000


def _emitted(origin: str, device: str, size: int) -> list:
    return ["emitted", [origin, 1, device, 1], [0, size], "0" * 64, 0, [1, 1], [-1, -1]]


def _tick(k: int, *packets: list, lifecycle: tuple = ()) -> dict:
    return {"k": k, "t_us": k * TICK_US, "packets": list(packets), "lifecycle": list(lifecycle)}


def _records() -> list[dict]:
    spawn = {"kind": "spawn"}
    return [
        GRID,
        _tick(0, _emitted("sitl1", "sitl1.serial1", 100), _emitted(4, "lockstep:router", 30)),
        _tick(25, _emitted("sitl1", "sitl1.serial1", 100)),
        _tick(50, _emitted("sitl1", "sitl1.serial1", 7), lifecycle=(spawn,)),  # the mission starts here
        _tick(60, _emitted("sitl2", "sitl2.serial1", 40),
              _emitted("sitl2", "lockstep:serial0.2", 500),  # a NavPy link: not the air
              ["copy", ["router", 1, "sitl1.serial1", 1], ["sitl2", 1, "sitl2.serial1", 1]]),
        _tick(100, _emitted("sitl1", "sitl1.serial1", 20)),
        _tick(123),
        _tick(124),  # the last tick: the mission covers [50, 125) ticks, 1.5 s
        {"mavlink_counts": {"sitl1.serial1": {"router_skipped_bytes": {"skipped_bytes": 6}},
                            "router.backend": {"router_skipped_bytes": {"skipped_bytes": 0}}},
         "held": {"sitl2/1/sitl2.serial1": {"bytes": 15, "sha256": "x"}}},
    ]


def test_setup_and_mission_are_apart_and_only_the_radio_senders_count():
    report = air_load(_records(), uart_loss=False)
    setup, mission = report["setup"], report["mission"]
    assert setup["seconds"] == 1.0
    assert setup["nodes"]["sitl1"] == {"bytes": 200, "mean_bytes_per_s": 200.0, "max_bytes_per_s": 200.0}
    assert setup["nodes"]["backend"]["bytes"] == 30 and setup["nodes"]["total"]["bytes"] == 230
    assert mission["seconds"] == 1.5
    assert {name: node["bytes"] for name, node in mission["nodes"].items()} == {
        "backend": 0, "sitl1": 27, "sitl2": 40, "total": 67}


def test_a_partial_last_window_is_scaled_by_its_length():
    mission = air_load(_records(), uart_loss=False)["mission"]["nodes"]
    # Windows [1.0 s, 2.0 s) and [2.0 s, 2.5 s): sitl1 has 7 B, then 20 B in half a second.
    assert mission["sitl1"]["max_bytes_per_s"] == 40.0
    assert mission["sitl1"]["mean_bytes_per_s"] == 18.0  # 27 B over 1.5 s
    assert mission["total"]["max_bytes_per_s"] == 47.0


def test_skipped_and_held_bytes_are_reported_apart_and_loss_is_named():
    report = air_load(_records(), uart_loss=True)
    assert report["skipped_bytes"] == {"backend": 0, "sitl1": 6, "sitl2": 0}
    assert report["held_bytes"] == {"backend": 0, "sitl1": 0, "sitl2": 15}
    assert report["measures"] == "observed after SITL byte loss"
    assert air_load(_records(), uart_loss=False)["measures"] == "offered load"


def test_a_run_without_a_spawn_is_all_setup():
    records = [r for r in _records() if not r.get("lifecycle")]
    report = air_load(records, uart_loss=False)
    assert report["mission"] == {"seconds": 0.0, "nodes": {}}
    assert report["setup"]["nodes"]["sitl1"]["bytes"] == 220
