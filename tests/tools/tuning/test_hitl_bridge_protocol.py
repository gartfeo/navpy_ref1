"""Lines of the JSBSim program (tools/tuning/hitl_bridge/jsbsim_protocol.py): output, replies, batches."""
import pytest

from tools.tuning.hitl_bridge import jsbsim_files as jf
from tools.tuning.hitl_bridge import jsbsim_protocol as jproto
from tools.tuning.hitl_bridge.jsbsim_plant import FT_TO_M, N_PER_LBF
from tools.tuning.hitl_bridge.environment import CALM, Wind
from tools.tuning.hitl_bridge.physics import Origin, SurfaceCommand

ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.16523, ground_alt_msl_m=584.0, heading_deg=353.0)
MASS_SLUGS = 0.4                   # synthetic aircraft
REST_FBZ_LBS = -12.804             # -32.01 ft/s^2 (JSBSim's local g in the WSL check) * mass, at rest
# The first datagram of /usr/games/JSBSim (FlightGear 2020.3.18) on the bridge's run folder,
# copied verbatim (scratchpad hitl/bridge_fix/labels_capture.py, 2026-10-12).
REAL_LABELS_LINE = (b"<LABELS> ,Time,sim-time-sec,frame,frame-mod,dlat-udeg,dlon-udeg,h-sl-meters,phi-rad,theta-rad,"
                    b"psi-rad,p-rad sec,q-rad sec,r-rad sec,v-north-fps,v-east-fps,v-down-fps,vc-fps,vtrue-fps,"
                    b"fbx-total-lbs,fby-total-lbs,fbz-total-lbs,mass-slugs\n")


def jsbsim_line(values) -> bytes:
    """A SOCKET output line as FGfdmSocket writes it: setw(12), setprecision(7), comma separated."""
    return (",".join(f"{value:12.7g}" for value in values) + "\n").encode()


def test_expected_labels_are_the_real_programs_printable_names():
    # The real program writes "p-rad sec" for velocities/p-rad_sec (GetPrintableName: "_" -> " ")
    assert jproto.parse_labels(REAL_LABELS_LINE) == jproto.expected_labels()
    assert jproto.printable_name("velocities/p-rad_sec") == "p-rad sec"
    assert jproto.printable_name("hitl/frame-mod") == "frame-mod"


def test_a_packed_state_line_is_read_in_si_units_with_the_precise_position():
    values = {jf.SIM_TIME_S: 1234.567, jf.FRAME: 1481480, jf.FRAME_MOD: 481480, jf.DLAT_UDEG: -1234.567,
              jf.DLON_UDEG: 89.01234, "position/h-sl-meters": 612.3456, "attitude/phi-rad": 0.1,
              "attitude/theta-rad": -0.2, "attitude/psi-rad": 3.0, "velocities/p-rad_sec": 0.01,
              "velocities/q-rad_sec": -0.02, "velocities/r-rad_sec": 0.03, "velocities/v-north-fps": 80.0,
              "velocities/v-east-fps": -10.0, "velocities/v-down-fps": 2.0, "velocities/vc-fps": 78.0,
              "velocities/vtrue-fps": 79.0, "forces/fbx-total-lbs": 3.2, "forces/fby-total-lbs": -0.5,
              "forces/fbz-total-lbs": REST_FBZ_LBS, "inertia/mass-slugs": MASS_SLUGS}
    line = jsbsim_line([1234.567] + [values[name] for name in jf.STATE_PROPERTIES])
    assert b"-1234.567" in line and b"   481480," in line         # 7 significant digits, as JSBSim prints
    sample = jproto.parse_state_line(line, ORIGIN)
    assert (sample.sim_time_s, sample.frame, sample.frame_mod) == (1234.567, 1481480, 481480)
    s = sample.state
    assert s.lat_deg == pytest.approx(-35.363261 - 1234.567e-6, abs=1e-12)
    assert s.lon_deg == pytest.approx(149.16523 + 89.01234e-6, abs=1e-12)
    assert (s.alt_msl_m, s.roll_rad, s.pitch_rad, s.yaw_rad) == (612.3456, 0.1, -0.2, 3.0)
    assert (s.p_rad_s, s.q_rad_s, s.r_rad_s) == (0.01, -0.02, 0.03)
    assert (s.vn_m_s, s.ve_m_s, s.vd_m_s) == pytest.approx((80 * FT_TO_M, -10 * FT_TO_M, 2 * FT_TO_M))
    assert (s.ias_m_s, s.tas_m_s) == pytest.approx((78 * FT_TO_M, 79 * FT_TO_M))
    # specific force = total body force / mass (the property choice of the WSL check)
    assert s.fz_m_s2 == pytest.approx(REST_FBZ_LBS / MASS_SLUGS * FT_TO_M) == pytest.approx(-9.757, abs=1e-3)
    assert s.fx_m_s2 == pytest.approx(3.2 / MASS_SLUGS * FT_TO_M)


def test_a_line_with_missing_or_unreadable_columns_is_refused():
    good = jsbsim_line([0.005] + [1.0] * len(jf.STATE_PROPERTIES))
    with pytest.raises(jf.JSBSimError, match="values"):
        jproto.parse_state_line(good.rsplit(b",", 1)[0] + b"\n", ORIGIN)
    with pytest.raises(jf.JSBSimError, match="unreadable"):
        jproto.parse_state_line(b"nan-ish,x\n", ORIGIN)


def test_replies_are_split_into_lines_without_prompts_across_reads():
    reader = jproto.ReplyReader()
    assert reader.feed(b"Connected to JSBSim server\nJSBSim> set succ") == ["Connected to JSBSim server"]
    assert reader.feed(b"essful\nJSBSim> Iterations performed\nJSBSim> ") == ["set successful",
                                                                              "Iterations performed"]
    assert reader.feed(b"Unknown property\n") == ["Unknown property"]


def test_expect_replies_skips_the_banner_and_stops_at_a_refusal_or_the_deadline():
    chunks = [b"Connected to JSBSim server\nJSBSim> set successful\nJSBSim> ", b"Iterations performed\nJSBSim> "]
    jproto.expect_replies(lambda timeout: chunks.pop(0), jproto.ReplyReader(),
                          [jproto.REPLY_SET_OK, jproto.REPLY_ITERATE_OK], deadline=10.0, clock=lambda: 0.0)
    assert chunks == []
    with pytest.raises(jf.JSBSimError, match="Unknown property"):
        jproto.expect_replies(lambda timeout: b"Unknown property\nJSBSim> ", jproto.ReplyReader(),
                              [jproto.REPLY_SET_OK], deadline=10.0, clock=lambda: 0.0)
    ticks = iter([0.0, 1.0, 2.5])
    with pytest.raises(jf.JSBSimError, match="did not answer"):
        jproto.expect_replies(lambda timeout: b"", jproto.ReplyReader(), [jproto.REPLY_SET_OK],
                              deadline=2.0, clock=lambda: next(ticks))


def test_one_batch_sets_the_commands_the_catapult_in_lbs_and_the_wind_in_fps_then_iterates():
    data = jproto.iterate_command(jf.DIRECT_COMMAND_PROPERTIES, SurfaceCommand(aileron=-0.25, elevator=0.5,
                                                                               throttle=1.0), 350.0,
                                  Wind(speed_m_s=3.048, from_deg=180.0), 6)
    assert data.decode().splitlines() == [
        "set fcs/aileron-cmd-norm -0.25", "set fcs/elevator-cmd-norm 0.5", "set fcs/throttle-cmd-norm 1",
        f"set external_reactions/catapult/magnitude {350.0 / N_PER_LBF:.9g}",
        "set atmosphere/wind-north-fps 10", f"set atmosphere/wind-east-fps {-10.0 * 1.2246467991473532e-16:.9g}",
        "iterate 6"]                                    # from the south: the air moves north
    # 350 N / 4.4482216152605 N/lbf = 78.68313... lbf (the earlier literal 78.6830282 was a typo)
    assert data.endswith(b"\n") and f"{350.0 / N_PER_LBF:.9g}" == "78.6831301"
    lagged = jproto.iterate_command(jf.LAGGED_COMMAND_PROPERTIES, SurfaceCommand(0.0, 0.0, 0.5), 0.0, CALM, 3)
    assert lagged.decode().splitlines() == ["set hitl/aileron-cmd-raw 0", "set hitl/elevator-cmd-raw 0",
                                            "set hitl/throttle-cmd-raw 0.5", "set external_reactions/catapult/magnitude 0",
                                            "set atmosphere/wind-north-fps 0", "set atmosphere/wind-east-fps 0",
                                            "iterate 3"]
    assert jproto.batch_replies(jf.LAGGED_COMMAND_PROPERTIES) == ["set successful"] * 6 + ["Iterations performed"]


def test_frame_relation_and_the_printed_time_tolerance():
    assert jproto.frame_relation(7, 7) == jproto.TARGET
    assert jproto.frame_relation(1, 7) == jproto.BEHIND and jproto.frame_relation(8, 7) == jproto.AHEAD
    assert jproto.frame_relation(999_999, 1_000_001) == jproto.BEHIND       # across the modulo
    assert jproto.frame_relation(1, 1_000_001) == jproto.TARGET
    assert jproto.printed_time_tolerance_s(0.005833333) == pytest.approx(5e-10)
    assert jproto.printed_time_tolerance_s(1234.567) == pytest.approx(5e-4)
    assert jproto.printed_time_tolerance_s(0.0) == 0.0
