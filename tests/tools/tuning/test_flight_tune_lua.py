"""Tests for the in-flight tuner tools/tuning/lua/flight_tune.lua.

The script runs under plain Lua 5.3 against tests/lua/flight_tune_harness.lua, which
mocks the ArduPilot bindings and flies a simulated fixed wing at 200 Hz. Every script
call runs under ArduPilot's instruction limit (SCR_VM_I_COUNT 10000).
"""
import json
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

from tools.tuning import design
from tools.tuning.ap_controller import Gains

REPO_ROOT = Path(__file__).resolve().parents[3]
HARNESS = REPO_ROOT / "tests" / "lua" / "flight_tune_harness.lua"
SCRIPT = REPO_ROOT / "tools" / "tuning" / "lua" / "flight_tune.lua"
VM_I_COUNT = 10000


def _resolve_lua():
    override = os.environ.get("LUA5_3")
    if override:
        return [override], False
    for cand in ("lua5.3", "lua"):
        exe = shutil.which(cand)
        if exe:
            return [exe], False
    if sys.platform == "win32" and shutil.which("wsl"):
        return [shutil.which("wsl"), "lua5.3"], True
    return None, False


LUA_ARGV, LUA_VIA_WSL = _resolve_lua()
pytestmark = pytest.mark.skipif(LUA_ARGV is None, reason="lua5.3 not available (set LUA5_3 or install lua5.3 / wsl lua5.3)")


def _lua_path(p: Path) -> str:
    s = str(p).replace("\\", "/")
    if LUA_VIA_WSL and len(s) > 2 and s[1] == ":":
        return f"/mnt/{s[0].lower()}{s[2:]}"
    return s


PROBE = REPO_ROOT / "tools" / "tuning" / "lua" / "timing_probe.lua"


def run(scenario: str, export: bool = False, script: Path = SCRIPT) -> dict:
    """Load the script on the simulated aircraft, run a scenario snippet, return the dump."""
    driver = textwrap.dedent(f"""
        local h = dofile('{_lua_path(HARNESS)}')
        local X = h.load_script('{_lua_path(script)}', {str(export).lower()})
        local S = h.state()
        S.params.FTUN_ENABLE = 1
        local function wait_ns(on, limit)
            local stop = S.t + limit
            while S.t < stop and S.ns.enabled ~= on do h.run(0.005) end
        end
    """) + textwrap.dedent(scenario) + "\nif not DUMPED then h.dump() end\n"
    env = dict(os.environ, MSYS_NO_PATHCONV="1")
    proc = subprocess.run(LUA_ARGV + ["-e", driver], capture_output=True, text=True, timeout=300, env=env)
    assert proc.returncode == 0, f"lua exited {proc.returncode}\n{proc.stderr}\n{proc.stdout[-2000:]}"
    out = json.loads(proc.stdout.strip().splitlines()[-1])
    assert out["max_instr"] <= VM_I_COUNT, out.get("first_overrun")
    assert not out.get("script_error"), out.get("script_error")     # nil fields are left out of the dump
    assert not out["long_msgs"], out["long_msgs"]      # STATUSTEXT holds 50 characters
    return out


def msgs(out):
    return out["msgs"]


def has(out, text):
    return any(text in m for m in out["msgs"])


START = """
h.set_switch(0)
h.run(1)
h.set_switch(2)
"""

BASE_ROLL = {"RLL_RATE_FF": 0.15, "RLL_RATE_P": 0.05, "RLL_RATE_I": 0.05, "RLL_RATE_D": 0.0}
BASE_PITCH = {"PTCH_RATE_FF": 0.6, "PTCH_RATE_P": 0.08, "PTCH_RATE_I": 0.1, "PTCH_RATE_D": 0.0}


def assert_gains(out, base):
    for k, v in base.items():
        assert out["params"][k] == pytest.approx(v), k


# ---------- ArduPilot's Lua build ----------
AP_MAXVARS = 100     # ArduPilot libraries/AP_Scripting/lua/src/lparser.c (stock Lua 5.3: 200)
AP_MAXUPVAL = 127    # lfunc.h


def _luac_listing() -> str:
    luac = [a.replace("lua5.3", "luac5.3") if a.endswith("lua5.3") else a for a in LUA_ARGV]
    if luac == LUA_ARGV:
        pytest.skip("luac5.3 not found next to lua5.3")
    proc = subprocess.run(luac + ["-l", "-l", "-p", _lua_path(SCRIPT)], capture_output=True, text=True,
                          env=dict(os.environ, MSYS_NO_PATHCONV="1"))
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


def test_script_fits_ardupilot_lua_limits():
    # ArduPilot compiles scripts with at most 100 active locals and 127 upvalues per
    # function; stock lua5.3 allows 200 locals, so the harness alone would not notice.
    max_active, max_up = 0, 0
    lines = _luac_listing().splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("locals ("):
            n = int(line.split("(")[1].split(")")[0])
            spans = []
            for j in range(i + 1, i + 1 + n):
                # "<index> <name> <startpc> <endpc>"; names like "(for index)" contain spaces
                start, end = lines[j].split()[-2:]
                spans.append((int(start), int(end)))
            points = sorted({p for sp in spans for p in sp})
            for pc in points:
                max_active = max(max_active, sum(1 for a, b in spans if a <= pc < b))
            i += n
        elif line.startswith("upvalues ("):
            max_up = max(max_up, int(line.split("(")[1].split(")")[0]))
        i += 1
    assert max_active <= AP_MAXVARS, max_active
    assert max_up <= AP_MAXUPVAL, max_up


# ---------- inert when off ----------
def test_inert_when_disabled_even_with_switch_high():
    out = run("""
S.params.FTUN_ENABLE = 0
h.set_switch(0); h.run(1); h.set_switch(2); h.run(30)
""")
    assert out["ns_enable_true"] == 0 and out["ns_calls"] == 0
    assert out["param_writes"] == []
    assert not any("start" in m for m in msgs(out))


def test_inert_with_switch_low():
    out = run("h.set_switch(0); h.run(30)")
    assert out["ns_enable_true"] == 0 and out["ns_calls"] == 0 and out["param_writes"] == []


def test_switch_high_at_boot_needs_a_movement_to_start():
    out = run("h.set_switch(2); h.run(20)")
    assert out["ns_enable_true"] == 0 and not has(out, "start")


# ---------- refusals ----------
@pytest.mark.parametrize("setup, text", [
    ("S.mode = 5", "use LOITER, CRUISE or FBWB"),                 # FBWA: parachute takeover must cancel us
    ("S.params.MTUNE_BTMSK = 3", "remove tune_pitch_roll.lua"),
    ("S.params.CHUTE_RC_CHAN = 9", "switch is the parachute channel"),
    ("S.aux[107] = 2", "AUTOTUNE switch is on"),
    ("S.params.RLL_RATE_NTF = 1", "RLL_RATE_NTF must be 0"),
    ("S.flying = false", "not flying"),
    ("S.params.SERVO_AUTO_TRIM = 1", "set SERVO_AUTO_TRIM 0"),
    ("S.params.PTCH_RATE_D_FF = 0.01", "PTCH_RATE_D_FF must be 0"),
])
def test_refuses_to_start(setup, text):
    out = run(setup + "\n" + START + "h.run(20)")
    assert has(out, text), msgs(out)
    assert out["ns_enable_true"] == 0 and out["param_writes"] == []


# ---------- full tune ----------
HARNESS_DT = 0.005
HARNESS_DELAY_STEPS = 4          # flight_tune_harness.lua: delayed() keeps tau/DT + 1 values
TRUE_PLANT = {"roll": {"K": 4.0, "a": 6.0, "b": 40.0},
              "pitch": {"K": 1.0, "z": 3.0, "wn": 9.0, "zeta": 0.5, "b": 40.0}}


def _harness_rate_loop(axis, P, I, D, fltd_hz, sc=1.0):
    """Exact discrete rate-loop response of the simulator in flight_tune_harness.lua.

    rate_ctl computes out_k from the rate of the previous step, the plant uses out
    four steps old, and the lag and rate states are integrated semi-implicitly, as in
    sim_step. Independent of the Lua tuner's model and margin code.
    """
    T = HARNESS_DT
    w = np.logspace(-1, np.log10(np.pi / T * 0.999), 20000)
    zi = np.exp(-1j * w * T)                       # z^-1
    pl = TRUE_PLANT[axis]
    lag = T * pl["b"] * zi ** HARNESS_DELAY_STEPS / (1 - (1 - T * pl["b"]) * zi)
    if axis == "roll":
        g = lag * T * pl["a"] * pl["K"] / (1 - (1 - T * pl["a"]) * zi)
    else:
        wn, zeta = pl["wn"], pl["zeta"]
        den = (1 - zi + 2 * zeta * wn * T * zi) + T * T * wn * wn * zi / (1 - zi)
        x2 = T * wn * wn * lag / den
        x1 = T * x2 / (1 - zi)
        g = pl["K"] * (x1 + x2 / pl["z"])
    alpha_d = T / (T + 1 / (2 * np.pi * fltd_hz))
    c = sc * sc * (P + I * T / (1 - zi) + D * alpha_d * (1 - zi) / (T * (1 - (1 - alpha_d) * zi)))
    return (c * zi * g)[None, :], w


@pytest.fixture(scope="module")
def full_tune():
    return run(START + """
S.params.FTUN_AXES = 3
h.run(400)
""")


def test_full_tune_keeps_and_saves_better_gains_for_both_axes(full_tune):
    out = full_tune
    assert has(out, "roll new gains saved") and has(out, "pitch new gains saved"), msgs(out)
    assert has(out, "done")
    assert set(out["saved"]) == {"RLL_RATE_FF", "RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D",
                                 "PTCH_RATE_FF", "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D"}
    for row in out["rows"]["FTDC"]:
        assert row["Keep"] == 1
        assert row["MAEb"] < 0.8 * row["MAEa"]
    assert not out["ns_enabled"]                 # control handed back
    assert out["ns_max_rate"] <= 180


def test_each_gain_set_is_logged_once_per_axis(full_tune):
    rows = full_tune["rows"]["FTGN"]
    assert sorted((r["Ax"], r["Set"]) for r in rows) == [(1, 0), (1, 1), (2, 0), (2, 1)]


def test_full_tune_identifies_the_simulated_plant(full_tune):
    roll, pitch = full_tune["rows"]["FTMD"]
    assert roll["K"] == pytest.approx(4.0, rel=0.05)
    assert roll["P1"] == pytest.approx(6.0, rel=0.1)       # roll pole a
    assert roll["P2"] == pytest.approx(40.0, rel=0.15)     # lag b
    assert pitch["K"] == pytest.approx(1.0, rel=0.05)
    assert pitch["P1"] == pytest.approx(3.0, rel=0.1)      # zero z
    assert pitch["P2"] == pytest.approx(9.0, rel=0.1)      # wn
    assert pitch["P3"] == pytest.approx(0.5, abs=0.05)     # zeta
    for row in (roll, pitch):
        assert row["Rms"] < 0.1


FF_GRID = (1.0, 0.85, 0.7, 0.55)     # flight_tune.lua K.FF_GRID


def test_full_tune_ff_comes_from_steady_state_gain(full_tune):
    # FF * scaler / EAS2TAS = 1 / K at the test speed (24 m/s = SCALING_SPEED, scaler 1),
    # times one of the searched fractions; roll (TCONST 0.5, fast roll pole) keeps it all
    assert full_tune["saved"]["RLL_RATE_FF"] == pytest.approx(1 / 4.0, rel=0.05)
    frac = full_tune["saved"]["PTCH_RATE_FF"] / (1 / 1.0)
    assert min(abs(frac - f) for f in FF_GRID) < 0.05, frac


@pytest.mark.parametrize("axis, prefix", [("roll", "RLL_RATE_"), ("pitch", "PTCH_RATE_")])
def test_designed_gains_keep_margins_on_the_exact_simulator_loop(full_tune, axis, prefix):
    # independent check of the saved gains on the simulator's exact discrete loop at
    # the test speed (24 m/s, scaler 1); the tuner designed on its fitted model for
    # FTUN_PM 45 deg and FTUN_GM 6 dB at three speeds
    s = full_tune["saved"]
    L, w = _harness_rate_loop(axis, s[prefix + "P"], s[prefix + "I"], s[prefix + "D"], fltd_hz=10.0)
    pm, gm = design._pm_gm(L, w)
    assert pm[0] >= 42 and gm[0] >= 5.5, (pm[0], gm[0])


def test_exact_loop_check_detects_bad_gains():
    # the same check fails for gains far above the stable range, so it is not vacuous
    L, w = _harness_rate_loop("roll", 5.0, 1.0, 0.05, fltd_hz=10.0)
    pm, gm = design._pm_gm(L, w)
    assert gm[0] < 0 or pm[0] < 0


def test_full_tune_prediction_agrees_with_measured_checks(full_tune):
    preds = full_tune["rows"]["FTPR"]
    assert len(preds) == 2
    for pred, dc in zip(preds, full_tune["rows"]["FTDC"]):
        assert dc["MAEb"] == pytest.approx(pred["MAEb"], rel=0.5, abs=0.2)


@pytest.mark.parametrize("late_s", [0.01, 0.04])
def test_late_script_calls_keep_the_model_and_safe_gains(full_tune, late_s):
    # The old timer-based script needed exact timing. Here each sample is time-stamped,
    # output and rate are read together, and the Fourier sums integrate exactly between
    # samples, so calls that start up to late_s late (every call) still give the same
    # model and gains that keep their margins on the exact simulator loop.
    out = run(START + f"""
math.randomseed(7)
S.jitter_s = {late_s}
S.params.FTUN_AXES = 3
h.run(500)
""")
    assert has(out, "roll new gains saved") and has(out, "pitch new gains saved"), msgs(out)
    for calm, late in zip(full_tune["rows"]["FTMD"], out["rows"]["FTMD"]):
        assert late["K"] == pytest.approx(calm["K"], rel=0.05)
        assert late["P1"] == pytest.approx(calm["P1"], rel=0.1)
        assert late["P2"] == pytest.approx(calm["P2"], rel=0.15)
        assert late["Rms"] < 0.1
    for axis, prefix in (("roll", "RLL_RATE_"), ("pitch", "PTCH_RATE_")):
        g = out["saved"]
        assert g[prefix + "FF"] == pytest.approx(full_tune["saved"][prefix + "FF"], rel=0.05)
        L, w = _harness_rate_loop(axis, g[prefix + "P"], g[prefix + "I"], g[prefix + "D"], fltd_hz=10.0)
        pm, gm = design._pm_gm(L, w)
        assert pm[0] >= 42 and gm[0] >= 5.5, (axis, pm[0], gm[0])


# ---------- stops ----------
INTO_NEW_GAINS_CHECK = START + """
S.params.FTUN_AXES = 1
assert(h.run_until("check 2/4", 300), "no check 2")
wait_ns(true, 10)
h.run(1.0)
BEFORE = S.params.RLL_RATE_P
"""


def _after_stop(extra=""):
    return """
local t_stop = S.t
h.run(0.03)
NS_AFTER = S.ns.enabled
h.run(5)
""" + extra + """
h.dump({ns_after = NS_AFTER, p_during = BEFORE})
DUMPED = true
"""


@pytest.mark.parametrize("action, reason", [
    ("S.norm[1] = 0.5", "stick input"),
    ("S.norm[2] = -0.3", "stick input"),
    ("h.set_switch(0)", "switch off"),
    ("S.rc_valid = false", "RC lost"),
    ("S.batt_fs = true", "battery failsafe"),
    ("S.alt = 50", "height"),
    ("S.asp = 18", "airspeed"),
    ("S.params.CHUTE_RC_CHAN = 7; S.pwm[7] = 2000", "parachute switch"),
])
def test_stop_releases_control_at_once_and_restores_old_gains(action, reason):
    out = run(INTO_NEW_GAINS_CHECK + action + _after_stop())
    assert has(out, "stopped: " + reason), msgs(out)
    assert out["extra"]["p_during"] != pytest.approx(BASE_ROLL["RLL_RATE_P"])   # new gains were flying
    assert out["extra"]["ns_after"] is False
    assert_gains(out, BASE_ROLL)
    assert not out["saved"]


def test_mode_change_by_parachute_script_cancels_control_and_tuner_stays_off():
    # chute_deploy.lua switches to FBWA; ArduPlane's Mode::enter clears script control
    out = run(START + """
S.params.FTUN_AXES = 1
assert(h.run_until("test 2/4", 200))
wait_ns(true, 10)
h.run(1.0)
S.mode = 5; S.ns.enabled = false
local enables = S.ns_enable_true
h.run(30)
h.dump({enables_before = enables})
DUMPED = true
""")
    assert has(out, "stopped: mode changed")
    assert out["ns_enable_true"] == out["extra"]["enables_before"]
    assert_gains(out, BASE_ROLL)


def test_parachute_switch_latches_until_reboot():
    out = run(START + """
S.params.FTUN_AXES = 1
S.params.CHUTE_RC_CHAN = 7
assert(h.run_until("test 2/4", 200))
S.pwm[7] = 2000
h.run(2)
S.pwm[7] = 1000
h.set_switch(0); h.run(1); h.set_switch(2)
local enables = S.ns_enable_true
h.run(30)
h.dump({enables_before = enables})
DUMPED = true
""")
    assert has(out, "stopped: parachute switch")
    assert out["ns_enable_true"] == out["extra"]["enables_before"]


def test_no_real_improvement_keeps_old_gains():
    out = run(START + """
S.params.FTUN_AXES = 1
S.params.FTUN_IMPROVE = 0.99
h.run(200)
""")
    assert has(out, "roll kept old"), msgs(out)
    assert_gains(out, BASE_ROLL)
    assert not out["saved"]
    assert out["rows"]["FTDC"][0]["Keep"] == 0


def test_script_delay_drops_the_record_and_continues():
    out = run(START + """
S.params.FTUN_AXES = 1
assert(h.run_until("test 2/4", 200))
wait_ns(true, 10)
h.run(2.5)
S.stall_until = S.t + 0.15
h.run(200)
""")
    assert has(out, "roll test dropped: script delayed"), msgs(out)
    assert has(out, "roll new gains saved")


def test_saturated_test_is_dropped_and_rate_reduced():
    out = run(START + """
S.params.FTUN_AXES = 1
S.params.RLL_RATE_FF = 0.7
h.run(300)
""")
    assert has(out, "roll test dropped: saturated"), msgs(out)


def test_model_ff_far_from_current_stops_the_axis():
    out = run(START + """
S.params.FTUN_AXES = 2
S.params.PTCH_RATE_FF = 0.25
h.run(200)
""")
    assert has(out, "suspect"), msgs(out)
    assert not out["saved"]
    assert out["params"]["PTCH_RATE_FF"] == pytest.approx(0.25)


def test_internal_error_releases_control_and_restores_gains():
    out = run(INTO_NEW_GAINS_CHECK + """
local real = logger.write
logger.write = function() error("boom") end
h.run(0.03)
NS_AFTER = S.ns.enabled
logger.write = real
h.run(2)
h.dump({ns_after = NS_AFTER, p_during = BEFORE})
DUMPED = true
""")
    assert has(out, "internal error"), msgs(out)
    assert out["extra"]["ns_after"] is False
    assert_gains(out, BASE_ROLL)
    assert out["script_dead"]                    # stopped on purpose: no reschedule after an error


# ---------- calculation pieces ----------
def test_margins_of_delayed_integrator_match_theory():
    # L = K e^(-s tau) / s: PM = 90 - K tau (deg); GM at w = pi / (2 tau)
    out = run("""
local K, tau = 10, 0.02
local lm, ph = {}, {}
for k, w in ipairs(X.DES_W) do lm[k] = math.log(K / w); ph[k] = -math.pi / 2 - w * tau end
local pm, gm, wc = X.margins(lm, ph)
h.dump({pm = pm, gm = gm, wc = wc})
DUMPED = true
""", export=True)
    e = out["extra"]
    assert e["pm"] == pytest.approx(90 - np.degrees(10 * 0.02), abs=0.5)
    assert e["wc"] == pytest.approx(10, rel=0.02)
    assert e["gm"] == pytest.approx(20 * np.log10((np.pi / (2 * 0.02)) / 10), abs=0.3)


def test_margins_agree_with_python_design_module():
    # same loop as test_phase_and_gain_margin_of_delayed_integrator in Python, plus a lag
    out = run("""
local lm, ph = {}, {}
for k, w in ipairs(X.DES_W) do
    lm[k] = math.log(12 / w) - 0.5 * math.log(1 + (w / 30) ^ 2)
    ph[k] = -math.pi / 2 - w * 0.015 - math.atan(w / 30)
end
local pm, gm = X.margins(lm, ph)
h.dump({pm = pm, gm = gm})
DUMPED = true
""", export=True)
    w = np.logspace(-1, np.log10(600.0), 8000)
    L = (12 * np.exp(-1j * w * 0.015) / (1j * w) / (1 + 1j * w / 30))[None, :]
    pm, gm = design._pm_gm(L, w)
    assert out["extra"]["pm"] == pytest.approx(pm[0], abs=1.0)
    assert out["extra"]["gm"] == pytest.approx(gm[0], abs=0.3)


def test_timing_probe_reports_call_gaps_and_touches_nothing():
    out = run("""
math.randomseed(3)
S.jitter_s = 0.02
h.run(25)
""", script=PROBE)
    reports = [m for m in out["msgs"] if m.startswith("FTIM: n ")]
    assert len(reports) == 2, out["msgs"]
    fields = reports[0].split()
    med, mx = float(fields[4]), float(fields[10])
    assert 20 <= med <= 40 and mx <= 45, reports[0]
    assert out["ns_calls"] == 0 and out["param_writes"] == []


# ---------- review fixes (Codex, 2026-10-11) ----------
@pytest.mark.parametrize("when", ["between tests", "while computing"])
@pytest.mark.parametrize("action, reason", [
    ("S.norm[1] = 0.4", "stick input"),
    ("S.mode = 7", "mode changed"),                # LOITER to CRUISE: both allowed, still a takeover
    ("S.params.RLL2SRV_TCONST = 0.4", "RLL2SRV_TCONST changed"),
])
def test_takeover_or_changed_setting_stops_outside_manoeuvres(when, action, reason):
    wait = 'wait_ns(false, 20)' if when == "between tests" else 'assert(h.run_until("roll test 4/4", 300)); wait_ns(true, 20); wait_ns(false, 20); h.run(0.5)'
    out = run(START + f"""
S.params.FTUN_AXES = 1
assert(h.run_until("test 2/4", 200))
{wait}
local enables = S.ns_enable_true
{action}
h.run(10)
h.dump({{enables_before = enables}})
DUMPED = true
""")
    assert has(out, "stopped: " + reason), msgs(out)
    assert out["ns_enable_true"] == out["extra"]["enables_before"]
    assert not out["saved"]


def test_failed_gain_write_stops_and_restores():
    out = run(START + """
S.params.FTUN_AXES = 1
S.fail_set.RLL_RATE_P = true
h.run(300)
""")
    assert has(out, "roll gain write failed"), msgs(out)
    assert_gains(out, BASE_ROLL)
    assert not out["saved"]


def test_elevon_saturation_after_mixing_drops_the_test():
    # each axis output stays below 44 deg, but a trim offset clips one elevon at 45
    out = run(START + """
S.params.FTUN_AXES = 1
S.elevon_offset = 40
h.run(120)
""")
    assert has(out, "roll test dropped: saturated"), msgs(out)


@pytest.mark.parametrize("delay_s, steps", [(0.0, 2), (0.010, 4), (0.020, 6)])
def test_step_simulation_delay_is_exact(delay_s, steps):
    # model with an instant plant: the rate first moves when the output computed at
    # step 1 arrives after the measured delay plus one loop period (design_new loop_dt)
    out = run(f"""
local dz = {{x = {{math.log(2), math.log(1e4), math.log(1e4), {delay_s}}}, axis = 1, e2t = 1, v_test = 20,
            loop_dt = 0.005, s = {{}}, ctl = {{tc = 0.5, rmax_up = 0, rmax_dn = 0, fltt = 0, flte = 0, fltd = 0, imax = 0.6}}}}
dz.s[2] = {{v = 20, sc = 1}}
local first
for k = 1, 10 do
    local m = X.stepsim_new(dz, {{FF = 1, P = 0, I = 0, D = 0}}, 2, 10, 200, {{{{k * 0.005, 1}}}})
    while not X.stepsim_run(m) do end
    if not first and m.rate ~= 0 then first = k end
end
h.dump({{first = first}})
DUMPED = true
""", export=True)
    assert out["extra"]["first"] == steps
