"""HITL JSBSim bridge end to end (tools/tuning/hitl_bridge/bridge.py, frame_loop.py) with a fake link and plant."""
import csv
import io
import itertools
import json
import math
import socket
import threading
import time

from pymavlink.dialects.v20 import common as mavlink2

import pytest

from tests.tools.tuning.hitl_data import SYNTHETIC_GROUND_REST, synthetic_rail
from tests.tools.tuning.test_hitl_bridge_link import FakeTransport, actuator_frame, make_link
from tests.tools.tuning.test_hitl_bridge_physics import FakePlant, rest_state
from tools.tuning.hitl_bridge.bridge import (DEFAULT_CATAPULT_DELAY_S, DEFAULT_CATAPULT_DURATION_S,
                                             DEFAULT_CATAPULT_FORCE_N, DEFAULT_INTEGRATION_DT_S,
                                             DEFAULT_LAUNCHER_HEIGHT_M, DEFAULT_LAUNCHER_PITCH_DEG, EXIT_OK,
                                             LAUNCH_RAIL, build_parser, launch_option, main, plant_options,
                                             wind_option)
from tools.tuning.hitl_bridge.catapult import Catapult
from tools.tuning.hitl_bridge.frame_loop import LOG_COLUMNS, Bridge, LinkLog, run_bridge
from tools.tuning.hitl_bridge.board_clock import BoardClockStepper
from tools.tuning.hitl_bridge.physics import Origin
from tools.tuning.hitl_bridge.plant_options import DEFAULT_SERVO_SPEED_S, StartPose, launcher_pose
from tools.tuning.hitl_bridge.rail import RailGeometry, RailLaunch, spec_stroke
from tools.tuning.private_data import PRIVATE_DIR_ENV

LOOP_US = 5000


def serve(reads, plant=None, catapult=None, said=None):
    """Run the bridge over scripted transport reads until they are used up."""
    transport = FakeTransport(reads=reads)
    link = make_link(transport, [])
    plant = plant or FakePlant()
    stepper = BoardClockStepper(plant, catapult or Catapult(0.0, 0.0, 0.0))
    stream = io.StringIO()
    ticks = itertools.count()
    run_bridge(link, Bridge(stepper), LinkLog(stream), lambda: next(ticks) * 0.001,
               should_stop=lambda: not transport.reads, say=(said if said is not None else []).append)
    replies = mavlink2.MAVLink(None).parse_buffer(bytes(transport.written)) or []
    rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
    return replies, rows, plant, link


def test_every_frame_gets_a_state_at_its_board_time():
    times = [1_000_000 + k * LOOP_US for k in range(5)]
    replies, rows, plant, _ = serve([b"".join(actuator_frame(t) for t in times)])
    assert [m.get_type() for m in replies] == ["HIL_STATE_QUATERNION"] * 5
    assert [m.time_usec for m in replies] == times
    # FakePlant reports its step count as latitude: the reply carries the state after
    # the physics reached the frame's board time
    assert [m.lat for m in replies] == [round(6 * k * 1e7) for k in range(5)]
    assert [r["event"] for r in rows] == ["first", "step", "step", "step", "step"]
    assert [int(r["substeps"]) for r in rows] == [0, 6, 6, 6, 6]
    assert [int(r["board_dt_us"]) for r in rows] == [0] + [LOOP_US] * 4
    assert all(r["sent"] == "1" and float(r["pc_tx_s"]) > float(r["pc_rx_s"]) for r in rows)
    assert list(rows[0].keys()) == list(LOG_COLUMNS)
    assert rows[0]["zacc_mg"] == "-1000"


def test_commands_and_catapult_appear_in_the_log():
    frames = [actuator_frame(k * LOOP_US, armed=k >= 2, pwm=(1600, 1800, 2000)) for k in range(8)]
    # armed at 10 ms: force from 20.2 ms to 30.3 ms, i.e. on substeps 25..36
    replies, rows, plant, _ = serve([b"".join(frames)], catapult=Catapult(350.0, 0.0101, 0.0102))
    assert [i for i, (_, f) in enumerate(plant.steps) if f] == list(range(25, 37))
    assert [r["armed"] for r in rows] == ["0", "0"] + ["1"] * 6
    assert [float(r["catapult_n"]) for r in rows] == [0, 0, 0, 0, 0, 350.0, 350.0, 0]
    assert (rows[3]["aileron"], rows[3]["elevator"], rows[3]["throttle"]) == ("0.2000", "-0.4000", "1.0000")


class PitchedPlant(FakePlant):
    """A plant standing in the air at the launcher's 10 deg: its own specific force is free fall."""

    def state(self):
        return rest_state(pitch_rad=math.radians(10.0), fz_m_s2=0.0, lat_deg=self.steps_since_reset)


def test_on_the_launcher_the_board_reads_rest_and_the_launchers_support_until_the_stroke():
    frames = [actuator_frame(k * LOOP_US, armed=k >= 2) for k in range(8)]
    # armed at 10 ms, stroke from 20.2 ms: frames 0-4 are answered from the launcher
    replies, rows, plant, _ = serve([b"".join(frames)], plant=PitchedPlant(),
                                    catapult=Catapult(350.0, 0.0101, 0.0102, holds=True))
    assert [r["on_launcher"] for r in rows] == ["1"] * 5 + ["0"] * 3
    assert [(m.xacc, m.zacc) for m in replies[:5]] == [(174, -985)] * 5     # g sin 10, -g cos 10 in mG
    assert [(m.xacc, m.zacc) for m in replies[5:]] == [(0, 0)] * 3          # the plant's own reading
    assert [m.lat for m in replies[:6]] == [0] * 5 + [50_000_000]            # first plant steps: the stroke
    assert float(rows[5]["catapult_n"]) == 350.0


def test_board_reboot_puts_the_aircraft_back_at_the_origin():
    frames = [actuator_frame(t) for t in (50_000_000, 50_005_000, 1_200_000, 1_205_000)]
    said = []
    replies, rows, plant, _ = serve([b"".join(frames)], said=said)
    assert [r["event"] for r in rows] == ["first", "step", "reinit", "step"]
    assert plant.resets == 2
    assert [m.lat for m in replies] == [0, 60_000_000, 0, 60_000_000]
    assert len(said) == 1 and "board reboot" in said[0]


def test_a_link_pause_with_lost_frames_continues_the_flight_and_is_reported():
    frames = [actuator_frame(t) for t in (1_000_000, 1_005_000, 1_105_000, 1_110_000)]
    said = []
    replies, rows, plant, _ = serve([b"".join(frames)], said=said)
    assert [r["event"] for r in rows] == ["first", "step", "gap", "step"]
    assert plant.resets == 1 and [int(r["lost_frames"]) for r in rows] == [0, 0, 19, 0]
    assert [m.lat for m in replies] == [0, 60_000_000, 1_260_000_000, 1_320_000_000]   # 6, 126, 132 steps
    # contract v2: lost frames are estimated from board time (no frame counter); frame = the bridge's index
    assert said == ["bridge: GAP of 100.0 ms board time before frame 3 (about 19 frames lost): stepped across "
                    "with the held outputs"]


def test_frames_after_a_reconnect_continue_the_same_flight():
    reads = [actuator_frame(0) + actuator_frame(LOOP_US), OSError("USB glitch"),
             actuator_frame(2 * LOOP_US), actuator_frame(3 * LOOP_US)]
    replies, rows, plant, link = serve(reads)
    assert [m.time_usec for m in replies] == [0, LOOP_US, 2 * LOOP_US, 3 * LOOP_US]
    assert [r["event"] for r in rows] == ["first", "step", "step", "step"]
    assert plant.resets == 1 and len(plant.steps) == 18
    assert (link.stats.connects, link.stats.disconnects) == (2, 1)


ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.16523, ground_alt_msl_m=584.0, heading_deg=353.0)


def test_cli_names_every_value_with_the_documented_defaults():
    # Changed behaviour: the default launch is now the measured catapult rail; the earlier simple
    # launcher stays selectable as --launch launcher with its own defaults. The rail and the rest pose on
    # the skids come from the private launch profile (here the synthetic one, hitl_data.py).
    args = build_parser().parse_args(["--device", "COM11", "--model-dir", "Peron15"])
    assert (args.origin_lat, args.origin_lon, args.origin_alt, args.heading) == (-35.363261, 149.16523, 584.0, 353.0)
    assert (args.catapult_force_n, args.catapult_duration_s, args.catapult_delay_s) == (
        DEFAULT_CATAPULT_FORCE_N, DEFAULT_CATAPULT_DURATION_S, DEFAULT_CATAPULT_DELAY_S)
    assert args.integration_dt_s == DEFAULT_INTEGRATION_DT_S == 1.0 / 1200.0
    start, launch = launch_option(args, ORIGIN)
    assert plant_options(args, start).servo_speed_s == DEFAULT_SERVO_SPEED_S == 0.14       # SITL's SIM_SERVO_SPEED
    assert (wind_option(args).speed_m_s, wind_option(args).from_deg) == (0.0, 0.0)     # calm
    assert args.launch == LAUNCH_RAIL and isinstance(launch, RailLaunch) and launch.delay_s == DEFAULT_CATAPULT_DELAY_S
    spec = synthetic_rail()
    assert launch.stroke == spec_stroke(spec) and launch.stroke.release_speed_m_s == spec.release_speed_m_s == 11.0
    assert (launch.geometry.elevation_deg, launch.geometry.height_m, launch.geometry.pitch_deg) == (
        spec.elevation_deg, spec.height_m, spec.carriage_pitch_deg) == (12.0, 0.8, 14.0)   # 2 deg above the rail
    assert start == launch.release_pose() and start.velocity_ned_m_s != (0.0, 0.0, 0.0)
    faster = build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--rail-release-speed-m-s", "20",
                                        "--rail-elevation-deg", "11", "--rail-height-m", "1.5"])
    _, fast = launch_option(faster, ORIGIN)
    assert (fast.stroke.release_speed_m_s, fast.geometry.elevation_deg, fast.geometry.height_m) == (20.0, 11.0, 1.5)
    assert fast.geometry.pitch_deg == spec.carriage_pitch_deg               # the carriage keeps its pitch
    old = build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--launch", "launcher"])
    start, launch = launch_option(old, ORIGIN)
    assert start == launcher_pose(DEFAULT_LAUNCHER_PITCH_DEG, DEFAULT_LAUNCHER_HEIGHT_M) == launcher_pose(10.0, 1.0)
    assert launch == Catapult(350.0, 0.5, 3.0, holds=True)
    ground = build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--launch", "ground"])
    rest = StartPose(height_agl_ft=SYNTHETIC_GROUND_REST["height_agl_ft"], pitch_deg=SYNTHETIC_GROUND_REST["pitch_deg"],
                     on_ground=True)
    assert launch_option(ground, ORIGIN) == (rest, Catapult(350.0, 0.5, 3.0, holds=False))
    args = build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--servo-speed-s", "0",
                                      "--wind-speed-m-s", "4", "--wind-from-deg", "250"])
    assert (plant_options(args, rest).servo_speed_s, wind_option(args).speed_m_s,
            wind_option(args).from_deg) == (0.0, 4.0, 250.0)


def test_the_launch_profile_is_read_from_the_private_folder_or_the_given_file(tmp_path, monkeypatch):
    """The rail and the rest pose are the aircraft's private data: without the profile the rail and the
    ground launches are refused before the physics starts; the simple launcher needs no profile."""
    monkeypatch.setenv(PRIVATE_DIR_ENV, str(tmp_path / "empty"))
    plants = []
    for launch in ("rail", "ground"):
        code = main(["--device", "tcp:127.0.0.1:9", "--model-dir", str(tmp_path), "--launch", launch,
                     "--log-csv", str(tmp_path / "link.csv")], make_plant=lambda *a: plants.append(a) or FakePlant())
        assert code == 2 and plants == []
    with pytest.raises(FileNotFoundError, match="launch data is private"):
        launch_option(build_parser().parse_args(["--device", "COM11", "--model-dir", "P"]), ORIGIN)
    launcher = build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--launch", "launcher"])
    assert launch_option(launcher, ORIGIN)[1] == Catapult(350.0, 0.5, 3.0, holds=True)
    other = tmp_path / "other.json"
    other.write_text(json.dumps({"rail": {**synthetic_rail().__dict__, "accel_table_g": [1.0, 2.0, 2.0],
                                          "release_speed_m_s": 9.0}}))
    _, rail = launch_option(build_parser().parse_args(["--device", "COM11", "--model-dir", "P",
                                                       "--launch-file", str(other)]), ORIGIN)
    assert rail.stroke.release_speed_m_s == 9.0
    other.write_text(json.dumps({"rail": {"travel_m": 2.0}}))
    with pytest.raises(ValueError, match="rail.accel_table_bin_s is missing"):
        launch_option(build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--launch-file",
                                                 str(other)]), ORIGIN)
    with pytest.raises(ValueError, match="no 'ground_rest' section"):
        launch_option(build_parser().parse_args(["--device", "COM11", "--model-dir", "P", "--launch", "ground",
                                                 "--launch-file", str(other)]), ORIGIN)


def test_a_rail_release_speed_that_cannot_launch_is_refused_before_the_physics_starts(tmp_path):
    plants = []
    code = main(["--device", "tcp:127.0.0.1:9", "--model-dir", str(tmp_path), "--rail-release-speed-m-s", "0",
                 "--log-csv", str(tmp_path / "link.csv")], make_plant=lambda *a: plants.append(a) or FakePlant())
    assert code == 2 and plants == []


def test_cli_refuses_bad_inputs_without_writing_a_log(tmp_path):
    log = tmp_path / "link.csv"
    assert main(["--device", "tcp:127.0.0.1:9", "--model-dir", str(tmp_path / "NoModel"),
                 "--log-csv", str(log)]) == 2
    plants = []

    def make_plant(*args):
        plants.append(FakePlant())
        return plants[-1]
    assert main(["--device", "tcp:9", "--model-dir", str(tmp_path), "--log-csv", str(log)],
                make_plant=make_plant) == 2              # malformed device string
    assert not log.exists()
    assert plants[0].closed                               # the physics started first is stopped again


def _free_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def test_the_physics_is_up_before_the_board_port_listens(tmp_path):
    port = _free_port()
    seen = []

    def make_plant(model_dir, origin, dt_s, options):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.settimeout(1.0)
            seen.append(probe.connect_ex(("127.0.0.1", port)))   # refused: not listening yet
        return FakePlant(dt_s)
    code = main(["--device", f"tcpin:{port}", "--model-dir", str(tmp_path), "--client-grace-s", "0.2",
                 "--log-csv", str(tmp_path / "link.csv")], make_plant=make_plant)
    assert code == EXIT_OK and seen and seen[0] != 0


def test_a_bridge_that_never_gets_a_client_ends_after_the_grace_time(tmp_path):
    plants = []

    def make_plant(model_dir, origin, dt_s, options):
        plants.append(FakePlant(dt_s))
        return plants[-1]
    start = time.monotonic()
    code = main(["--device", f"tcpin:{_free_port()}", "--model-dir", str(tmp_path), "--client-grace-s", "0.3",
                 "--log-csv", str(tmp_path / "link.csv")], make_plant=make_plant)
    assert code == EXIT_OK and plants[0].closed
    assert 0.3 <= time.monotonic() - start < 5.0


class _StopAfter(FakePlant):
    """Ends the run like Ctrl-C once the physics has taken max_steps steps."""

    def __init__(self, dt_s: float, max_steps: int, ground_alt_msl_m: float | None = None) -> None:
        super().__init__(dt_s)
        self.max_steps = max_steps
        self.ground_alt_msl_m = ground_alt_msl_m

    def state(self):
        """On the ground at the run's origin when it has one: a state under the ground is a crash, and a
        crashed plant is not stepped again (board_clock.crash_reason), so the run would never stop."""
        state = super().state()
        if self.ground_alt_msl_m is None:
            return state
        return rest_state(lat_deg=state.lat_deg, alt_msl_m=self.ground_alt_msl_m)

    def step(self, command, catapult_force_n, wind) -> None:
        if len(self.steps) == self.max_steps:
            raise KeyboardInterrupt
        super().step(command, catapult_force_n, wind)


def test_cli_runs_over_tcp_and_writes_the_log_and_run_options(tmp_path):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    received = bytearray()

    def board():
        conn, _ = server.accept()
        conn.sendall(b"".join(actuator_frame(k * LOOP_US) for k in range(3)))
        conn.settimeout(5)                                  # guard: the bridge closes on its stop
        try:
            while chunk := conn.recv(4096):
                received.extend(chunk)
        finally:
            conn.close()

    thread = threading.Thread(target=board)
    thread.start()
    plants = []

    def make_plant(model_dir, origin, dt_s, options):
        plants.append((model_dir, origin))
        return _StopAfter(dt_s, max_steps=6, ground_alt_msl_m=origin.ground_alt_msl_m)   # frame 3: steps 7..12

    log = tmp_path / "run" / "link.csv"
    try:
        # on the skids the physics steps from the first frame (a launcher would hold the
        # never-armed aircraft, and _StopAfter would never stop the run)
        code = main(["--device", f"tcp:127.0.0.1:{server.getsockname()[1]}", "--model-dir", str(tmp_path / "M"),
                     "--origin-alt", "1300", "--log-csv", str(log), "--launch", "ground"], make_plant=make_plant)
    finally:
        thread.join(timeout=10)
        server.close()
    assert code == 0
    assert plants[0][1].ground_alt_msl_m == 1300.0
    rows = list(csv.DictReader(log.open()))
    assert [r["event"] for r in rows] == ["first", "step"]
    replies = mavlink2.MAVLink(None).parse_buffer(bytes(received)) or []
    assert [m.time_usec for m in replies] == [0, LOOP_US]
    options = json.loads(log.with_suffix(".json").read_text())
    assert options["origin_alt"] == 1300.0 and options["device"].startswith("tcp:")
