"""The catapult rail of the HITL bridge (tools/tuning/hitl_bridge/rail.py): the push integrated, the
accelerometer on the rail, the hold until the release and the hand-over to the plant.

On a synthetic rail (hitl_data.py): the real launcher's numbers are private, and test_hitl_private_data.py
checks them against their measurements."""
import math

import pytest

from tests.tools.tuning.test_hitl_bridge_link import actuator_frame
from tests.tools.tuning.test_hitl_bridge_physics import FakePlant, _rotate, commands_as_given, rest_state
from tests.tools.tuning.test_hitl_bridge_run import serve
from tools.tuning.hitl_bridge.environment import CALM, US_PER_S, Wind
from tools.tuning.hitl_bridge.board_clock import REINIT, BoardClockStepper
from tools.tuning.hitl_bridge.physics import NEUTRAL_COMMAND, Origin, PlantState
from tools.tuning.hitl_bridge.plant_options import FT_PER_M, offset_lat_lon
from tests.tools.tuning.hitl_data import SYNTHETIC_LAUNCH_PARAMS, synthetic_rail
from tools.tuning.hitl_bridge.rail import RailGeometry, RailLaunch, RailSpec, rail_state, rail_stroke, spec_stroke
from tools.tuning.hitl_bridge.state_fields import STANDARD_GRAVITY_M_S2, euler_to_quaternion, on_launcher

G = STANDARD_GRAVITY_M_S2
ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.16523, ground_alt_msl_m=584.0, heading_deg=353.0)
LOOP_US = 5000                      # 200 Hz board frames

SPEC = synthetic_rail()
TRAVEL_M, RELEASE_SPEED_M_S = SPEC.travel_m, SPEC.release_speed_m_s
RAIL_ELEVATION_DEG, RAIL_HEIGHT_M, CARRIAGE_PITCH_DEG = SPEC.elevation_deg, SPEC.height_m, SPEC.carriage_pitch_deg
TKOFF_THR_MINACC = SYNTHETIC_LAUNCH_PARAMS["TKOFF_THR_MINACC"]
# An independent midpoint integration of the stroke's acceleration at FINE_STEP_S steps agrees with its
# closed form to these (the bin edges fall inside a fine step: at most a*FINE_STEP_S per edge).
FINE_STEP_S = 1e-5
INTEGRATION_TOL_M = 1e-3
INTEGRATION_TOL_M_S = 1e-3
# EAS/TAS of the plant's air at the release: below 1 at an origin above sea level
PLANT_EAS_PER_TAS = 0.97


def release_state(launch: RailLaunch) -> PlantState:
    """The plant's state at its initial conditions, the release pose, as the JSBSim program reports it."""
    pose = launch.release_pose()
    lat, lon = offset_lat_lon(ORIGIN.lat_deg, ORIGIN.lon_deg, pose.north_m, pose.east_m)
    vn, ve, vd = pose.velocity_ned_m_s
    tas = math.sqrt(vn * vn + ve * ve + vd * vd)
    return PlantState(lat_deg=lat, lon_deg=lon, alt_msl_m=ORIGIN.ground_alt_msl_m + pose.height_agl_ft / FT_PER_M,
                      roll_rad=0.0, pitch_rad=math.radians(pose.pitch_deg), yaw_rad=math.radians(ORIGIN.heading_deg),
                      p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0, vn_m_s=vn, ve_m_s=ve, vd_m_s=vd,
                      ias_m_s=tas * PLANT_EAS_PER_TAS, tas_m_s=tas, fx_m_s2=-0.8, fy_m_s2=0.0, fz_m_s2=-1.0)


def rail(delay_s: float = 0.0, speed_m_s: float = RELEASE_SPEED_M_S, spec: RailSpec = SPEC) -> RailLaunch:
    geometry = RailGeometry(ORIGIN, spec.elevation_deg, spec.height_m, spec.carriage_pitch_deg)
    return RailLaunch(geometry, spec_stroke(spec, speed_m_s), delay_s)


class ReleasePlant(FakePlant):
    """Stands at the rail's release state until it is stepped, then reports its step count as latitude."""

    def __init__(self, release: PlantState) -> None:
        super().__init__()
        self.release = release

    def state(self) -> PlantState:
        return self.release if self.steps_since_reset == 0 else rest_state(lat_deg=self.steps_since_reset)


# --- the push ------------------------------------------------------------------------------
def test_the_push_releases_at_the_specs_speed_after_its_travel():
    stroke = spec_stroke(SPEC)
    assert stroke.at(stroke.duration_s) == (TRAVEL_M, RELEASE_SPEED_M_S, 0.0) == (2.5, 11.0, 0.0)
    s, v, _ = stroke.at(stroke.duration_s * (1 - 1e-12))          # the push reaches them continuously
    assert (s, v) == pytest.approx((TRAVEL_M, RELEASE_SPEED_M_S), rel=1e-9)
    # the table's shape, scaled to the release speed: k^2 on each bin's acceleration, 1/k on its length
    k = SPEC.accel_table_bin_s / stroke.bin_s
    assert stroke.accel_m_s2[:len(SPEC.accel_table_g)] == pytest.approx([a * G * k * k for a in SPEC.accel_table_g])
    assert min(stroke.accel_m_s2) <= RELEASE_SPEED_M_S / stroke.duration_s <= max(stroke.accel_m_s2)
    assert stroke.at(-0.1) == stroke.at(0.0) == (0.0, 0.0, 0.0)
    assert spec_stroke(SPEC, RELEASE_SPEED_M_S) == stroke


def test_an_independent_integration_of_the_push_reaches_the_same_travel_and_speed():
    stroke = spec_stroke(SPEC)
    s = v = t = 0.0
    while t < stroke.duration_s:
        dt = min(FINE_STEP_S, stroke.duration_s - t)
        a = stroke.at(t + dt / 2)[2]
        s, v, t = s + v * dt + 0.5 * a * dt * dt, v + a * dt, t + dt
    assert s == pytest.approx(TRAVEL_M, abs=INTEGRATION_TOL_M)
    assert v == pytest.approx(RELEASE_SPEED_M_S, abs=INTEGRATION_TOL_M_S)


def test_a_higher_release_speed_is_the_same_rail_with_a_stronger_shorter_push():
    base, fast = spec_stroke(SPEC), spec_stroke(SPEC, 20.0)
    k = 20.0 / RELEASE_SPEED_M_S
    assert fast.at(fast.duration_s) == (TRAVEL_M, 20.0, 0.0)
    assert fast.duration_s == pytest.approx(base.duration_s / k, rel=1e-12)
    assert max(fast.accel_m_s2) == pytest.approx(max(base.accel_m_s2) * k * k, rel=1e-12)
    s_base, v_base, _ = base.at(0.2)
    s_fast, v_fast, _ = fast.at(0.2 / k)                          # the same point of the push
    assert (s_fast, v_fast) == pytest.approx((s_base, k * v_base), rel=1e-12)
    with pytest.raises(ValueError, match="positive"):
        spec_stroke(SPEC, 0.0)
    with pytest.raises(ValueError, match="short of"):                # a push that stops before the arrest
        rail_stroke(RELEASE_SPEED_M_S, (1.0, 0.0), SPEC.accel_table_bin_s, TRAVEL_M)


# --- the aircraft on the rail ------------------------------------------------------------------
def test_on_the_rail_the_accelerometer_reads_the_rail_acceleration_plus_the_gravity_reaction():
    launch = rail()
    release = release_state(launch)
    a = 7.5 * G
    state = rail_state(release, launch.geometry, 0.4, 10.0, a, CALM)
    theta, el = math.radians(CARRIAGE_PITCH_DEG), math.radians(RAIL_ELEVATION_DEG)
    # wings level: along the body x axis the rail acceleration's share plus g sin(pitch), none sideways
    assert (state.fx_m_s2, state.fy_m_s2, state.fz_m_s2) == pytest.approx(
        (a * math.cos(theta - el) + G * math.sin(theta), 0.0, a * math.sin(theta - el) - G * math.cos(theta)), abs=1e-9)
    # turned back to NED it is the rail acceleration minus gravity: what an accelerometer measures
    u = launch.geometry.direction_ned()
    f_ned = _rotate(euler_to_quaternion(state.roll_rad, state.pitch_rad, state.yaw_rad),
                    [state.fx_m_s2, state.fy_m_s2, state.fz_m_s2])
    assert f_ned == pytest.approx([a * u[0], a * u[1], a * u[2] - G], abs=1e-9)
    assert (state.vn_m_s, state.ve_m_s, state.vd_m_s) == pytest.approx([10.0 * c for c in u])
    assert (state.p_rad_s, state.q_rad_s, state.r_rad_s) == (0.0, 0.0, 0.0)
    rest = rail_state(release, launch.geometry, 0.0, 0.0, 0.0, CALM)       # at rest: the carriage's support
    held = on_launcher(release)
    assert (rest.fx_m_s2, rest.fy_m_s2, rest.fz_m_s2) == pytest.approx((held.fx_m_s2, held.fy_m_s2, held.fz_m_s2))


def test_the_rail_runs_from_its_start_to_the_plants_release_point_in_the_relative_wind():
    launch = rail()
    release = release_state(launch)
    start = rail_state(release, launch.geometry, 0.0, 0.0, 0.0, CALM)
    assert (start.lat_deg, start.lon_deg, start.alt_msl_m) == (ORIGIN.lat_deg, ORIGIN.lon_deg,
                                                              ORIGIN.ground_alt_msl_m + RAIL_HEIGHT_M)
    end = rail_state(release, launch.geometry, 1.0, RELEASE_SPEED_M_S, 0.0, CALM)
    assert (end.lat_deg, end.lon_deg, end.alt_msl_m) == pytest.approx((release.lat_deg, release.lon_deg,
                                                                      release.alt_msl_m), abs=1e-12)
    assert (end.ias_m_s, end.tas_m_s) == pytest.approx((release.ias_m_s, release.tas_m_s))
    half = rail_state(release, launch.geometry, 0.5, 8.0, 0.0, CALM)
    assert half.alt_msl_m == pytest.approx(start.alt_msl_m + 0.5 * TRAVEL_M * math.sin(math.radians(RAIL_ELEVATION_DEG)))
    headwind = Wind(speed_m_s=5.0, from_deg=ORIGIN.heading_deg)        # blowing down the rail
    moving = rail_state(release, launch.geometry, 0.5, 8.0, 0.0, headwind)
    wind_n, wind_e = headwind.air_velocity_ne_m_s()
    assert moving.tas_m_s == pytest.approx(math.dist((moving.vn_m_s, moving.ve_m_s, moving.vd_m_s), (wind_n, wind_e, 0)))
    assert moving.ias_m_s == pytest.approx(moving.tas_m_s * PLANT_EAS_PER_TAS)
    assert rail_state(release, launch.geometry, 0.0, 0.0, 0.0, headwind).tas_m_s == pytest.approx(5.0)


def test_the_board_sees_the_launch_acceleration_along_its_body_axis():
    """TKOFF_THR_MINACC compares TECS VXdot = AccX - g sin(pitch) (takeoff.cpp via AP_TECS): at rest it reads
    0 here (no AHRS trim), and on the rail the push's share along the body x axis, a cos(pitch - elevation),
    so the push crosses TKOFF_THR_MINACC within its first bins (the real launcher's detection time against
    its logs: test_hitl_private_data.py). The board's own 20 Hz accel filter and 50 Hz TECS tick come on top."""
    launch = rail()
    release = release_state(launch)
    launch.note(0, armed=True)
    times_s = [k * LOOP_US / US_PER_S for k in range(20)]
    states = [launch.held_state(release, t * US_PER_S, CALM) for t in times_s]
    vxdot = [s.fx_m_s2 - G * math.sin(s.pitch_rad) for s in states]
    along_body = math.cos(math.radians(CARRIAGE_PITCH_DEG - RAIL_ELEVATION_DEG))
    assert vxdot == pytest.approx([launch.stroke.at(t)[2] * along_body for t in times_s], abs=1e-9)
    assert vxdot[0] == pytest.approx(0.0, abs=1e-12)
    first = next(t for t, x in zip(times_s, vxdot) if x >= TKOFF_THR_MINACC)
    assert 0.0 < first <= 2 * launch.stroke.bin_s


def test_offsets_from_the_origin_follow_the_wgs84_ellipsoid():
    """Against the standard series for the length of a degree on WGS84 (independent of the radii
    offset_lat_lon uses), which is good to about 1e-6; 1 km offsets at the origin and at 45 deg."""
    series_agreement = 1e-6
    for lat_deg in (ORIGIN.lat_deg, 45.0):
        phi = math.radians(lat_deg)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi) - 0.0023 * math.cos(6 * phi)
        m_per_deg_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi) + 0.118 * math.cos(5 * phi)
        lat, lon = offset_lat_lon(lat_deg, 10.0, 1000.0, -1000.0)
        assert (lat - lat_deg) * m_per_deg_lat == pytest.approx(1000.0, rel=series_agreement)
        assert (lon - 10.0) * m_per_deg_lon == pytest.approx(-1000.0, rel=series_agreement)
    assert offset_lat_lon(ORIGIN.lat_deg, ORIGIN.lon_deg, 0.0, 0.0) == (ORIGIN.lat_deg, ORIGIN.lon_deg)


def test_the_plant_starts_at_the_rail_end_with_the_release_velocity():
    pose = rail().release_pose()
    el, hdg = math.radians(RAIL_ELEVATION_DEG), math.radians(ORIGIN.heading_deg)
    assert (pose.north_m, pose.east_m) == pytest.approx((TRAVEL_M * math.cos(el) * math.cos(hdg),
                                                        TRAVEL_M * math.cos(el) * math.sin(hdg)))
    rise_m = pose.height_agl_ft / FT_PER_M - RAIL_HEIGHT_M
    assert rise_m == pytest.approx(TRAVEL_M * math.sin(el))
    vn, ve, vd = pose.velocity_ned_m_s
    assert math.sqrt(vn * vn + ve * ve + vd * vd) == pytest.approx(RELEASE_SPEED_M_S)
    assert math.degrees(math.atan2(-vd, math.hypot(vn, ve))) == pytest.approx(RAIL_ELEVATION_DEG)
    assert (pose.pitch_deg, pose.on_ground) == (CARRIAGE_PITCH_DEG, False)


# --- the hold and the hand-over ------------------------------------------------------------------
def test_the_rail_holds_the_plant_until_the_release_and_the_board_flies_the_rail_meanwhile():
    launch = rail(delay_s=0.0102)
    plant = ReleasePlant(release_state(launch))
    stepper = BoardClockStepper(plant, launch, mixer=commands_as_given)
    times = [k * LOOP_US for k in range(90)]
    results, states = [], []
    for k, t in enumerate(times):                                     # armed from 10 ms: stroke at 20.2 ms
        results.append(stepper.on_actuators(t, NEUTRAL_COMMAND, armed=k >= 2))
        states.append(stepper.board_state(t))
    release_us = 10_000 + 10_200 + launch.stroke.duration_s * US_PER_S
    dt_us = plant.dt_s * US_PER_S
    first_step = math.ceil(release_us / dt_us)        # the plant's first step starts at the release or after it
    held = [r.on_launcher for r in results]
    assert held == [math.floor(t / dt_us + 1e-6) <= first_step for t in times]     # frames before that step's end
    assert True in held and False in held
    assert all(r.substeps == 0 for r, t in zip(results, times) if t < release_us)
    assert {force for _, force in plant.steps} == {0.0}                # the rail pushes, not the plant
    on_rail = [s for s, h in zip(states, held) if h]
    assert all(s.vn_m_s == 0.0 and s.alt_msl_m == ORIGIN.ground_alt_msl_m + RAIL_HEIGHT_M for s in on_rail[:5])
    speeds = [math.hypot(s.vn_m_s, s.ve_m_s, s.vd_m_s) for s in on_rail]
    assert speeds == sorted(speeds) and 0.0 < speeds[-1] <= RELEASE_SPEED_M_S
    assert [s.lat_deg for s in on_rail] == sorted(s.lat_deg for s in on_rail)      # heading 353: northwards
    assert states[held.index(False)].lat_deg > 0                                     # then the plant's own


def test_from_the_release_until_the_plants_first_step_the_board_reads_the_release_state():
    launch = rail()
    release = release_state(launch)
    launch.note(1_000_000, armed=True)
    released_us = 1_000_000 + math.ceil(launch.stroke.duration_s * US_PER_S)     # the first whole microsecond
    assert launch.held_state(release, released_us, CALM) is release
    assert not launch.holding(released_us) and launch.holding(released_us - 1)
    assert launch.force_at(released_us - 1) == 0.0


def test_a_board_reboot_puts_the_aircraft_back_on_the_carriage():
    launch = rail()
    plant = ReleasePlant(release_state(launch))
    stepper = BoardClockStepper(plant, launch, mixer=commands_as_given)
    assert launch.stroke.duration_s < 0.5
    for k in range(100):                                              # armed at once: released before 0.5 s
        stepper.on_actuators(k * LOOP_US, NEUTRAL_COMMAND, armed=True)
    assert not stepper.on_launcher and plant.steps
    assert stepper.on_actuators(0, NEUTRAL_COMMAND, armed=False).event == REINIT
    assert stepper.on_launcher and launch.armed_at_us is None and plant.resets == 2
    state = stepper.board_state(0)
    assert (state.vn_m_s, state.alt_msl_m) == (0.0, ORIGIN.ground_alt_msl_m + RAIL_HEIGHT_M)


def test_through_the_frame_loop_the_board_reads_the_carriage_then_the_push():
    launch = rail(delay_s=0.0102)
    frames = [actuator_frame(k * LOOP_US, armed=k >= 2) for k in range(30)]
    replies, rows, _, _ = serve([b"".join(frames)], plant=ReleasePlant(release_state(launch)), catapult=launch)
    theta = math.radians(CARRIAGE_PITCH_DEG)
    at_rest = (round(math.sin(theta) * 1000), round(-math.cos(theta) * 1000))
    assert [(m.xacc, m.zacc) for m in replies[:5]] == [at_rest] * 5      # armed, before the stroke at 20.2 ms
    peak = max(launch.stroke.accel_m_s2) * math.cos(theta - math.radians(RAIL_ELEVATION_DEG)) / G + math.sin(theta)
    assert max(m.xacc for m in replies) == pytest.approx(1000 * peak, abs=1)    # the push's strongest bin
    assert [m.vx for m in replies] == sorted(m.vx for m in replies) and replies[-1].vx > 0
    assert all(r["on_launcher"] == "1" and float(r["catapult_n"]) == 0.0 for r in rows)
