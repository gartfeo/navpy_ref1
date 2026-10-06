import json
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
HARNESS_LUA = REPO_ROOT / "tests" / "lua" / "chute_deploy_harness.lua"
SCRIPT_LUA = REPO_ROOT / "scripts" / "lua" / "chute_deploy.lua"


def _resolve_lua():
    override = os.environ.get("LUA5_3")
    if override:
        return [override], False
    for cand in ("lua5.3", "lua"):
        exe = shutil.which(cand)
        if exe:
            return [exe], False
    if sys.platform == "win32":
        wsl = shutil.which("wsl")
        if wsl:
            return [wsl, "lua5.3"], True
    return None, False


LUA_ARGV, LUA_VIA_WSL = _resolve_lua()
pytestmark = pytest.mark.skipif(
    LUA_ARGV is None,
    reason="lua5.3 not available (set LUA5_3 env or install lua5.3 / wsl lua5.3)",
)


def _to_lua_path(p: Path) -> str:
    if not LUA_VIA_WSL:
        return str(p)
    s = str(p).replace("\\", "/")
    if len(s) > 2 and s[1] == ":":
        return f"/mnt/{s[0].lower()}{s[2:]}"
    return s


def _lua_str(s: str) -> str:
    """Quote a string for embedding inside a Lua single-quoted literal."""
    return "'" + s.replace("\\", "\\\\").replace("'", "\\'") + "'"


def run_scenario(scenario_lua: str) -> dict:
    """Run a scenario Lua snippet against the harness; return parsed JSON output."""
    driver = textwrap.dedent(f"""
        local h = dofile({_lua_str(_to_lua_path(HARNESS_LUA))})
        h.load_script({_lua_str(_to_lua_path(SCRIPT_LUA))})
        {scenario_lua}
        h.dump()
    """)
    proc = subprocess.run(
        LUA_ARGV + ["-e", driver],
        capture_output=True,
        text=True,
        timeout=30,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"lua exited {proc.returncode}\nSTDERR:\n{proc.stderr}\nSTDOUT:\n{proc.stdout}"
        )
    if not proc.stdout.strip():
        raise AssertionError(f"empty stdout from lua\nSTDERR:\n{proc.stderr}")
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as e:
        raise AssertionError(
            f"invalid JSON from lua harness: {e}\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
        ) from e


def msgs(out: dict) -> list[str]:
    return [m["msg"] for m in out["messages"]]


def base_airframe() -> str:
    return textwrap.dedent("""
        h.state.servo_funcs[9] = 94  -- chute output on SERVO9
        h.state.servo_funcs[3] = 70  -- throttle output on SERVO3
        h.state.armed = true
        h.state.hagl = 100
        h.state.pitch = 0            -- radians
        h.state.airspeed = 20
    """)


def trigger_snippet() -> str:
    return textwrap.dedent("""
        h.tick(2)                    -- drain edge detector with low switch
        h.state.rc_pwms[7] = 1900    -- low -> high trigger
        h.tick(1)
    """)


def fast_deploy_snippet() -> str:
    return base_airframe() + textwrap.dedent("""
        h.state.params.CHUTE_CLIMB_MS = 150
        h.tick(2)
        h.state.rc_pwms[7] = 1900
        h.tick(5)                    -- enter PITCHUP, miss pitch angle, emit deployed log
    """)


def test_script_loads_and_starts():
    """Harness can install mocks, dofile chute_deploy.lua, and start() returns update + TICK_MS=50."""
    out = run_scenario(textwrap.dedent("""
        assert(h.update ~= nil, 'update fn not captured')
        assert(h.tick_ms == 50, 'expected TICK_MS=50, got ' .. tostring(h.tick_ms))
    """))
    assert any("Chute deploy script loaded" in m for m in msgs(out))


def test_safe_inhibits_with_missing_servo():
    """With no SERVO*_FUNCTION=94, SAFE state must emit the inhibit message."""
    out = run_scenario(textwrap.dedent("""
        h.tick(120)
    """))
    assert any("Chute inhibited: No SERVOx_FUNCTION=94" in m for m in msgs(out)), msgs(out)


def test_trigger_blocked_below_min_alt():
    """CHUTE_MIN_ALT remains the pre-trigger altitude gate."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.hagl = 50
    """) + trigger_snippet() + textwrap.dedent("""
        h.tick(3)
    """))
    messages = msgs(out)
    assert any("altitude 50 < CHUTE_MIN_ALT 80" in m for m in messages), messages
    assert not any("Chute: PITCHUP" in m for m in messages), messages
    assert not any("Chute deployed" in m for m in messages), messages


def test_valid_trigger_enters_pitchup_with_full_throttle():
    """A valid trigger enters setup, switches to FBWA, and commands pitch/throttle max."""
    out = run_scenario(base_airframe() + trigger_snippet())
    messages = msgs(out)
    assert any("Chute: PITCHUP (full throttle)" in m for m in messages), messages
    assert not any("Chute deployed" in m for m in messages), messages
    assert any(a["mode"] == 5 and a["ok"] for a in out["mode_attempts"]), out["mode_attempts"]
    assert any(o["chan"] == 2 and o["pwm"] == 1900 for o in out["rc_overrides"]), out["rc_overrides"]
    assert any(o["chan"] == 3 and o["pwm"] == 1900 for o in out["rc_overrides"]), out["rc_overrides"]


def test_pitchup_deploys_after_pitch_response_window_when_angle_not_reached():
    """Failure to reach pitch angle deploys only after the response window expires."""
    out = run_scenario(fast_deploy_snippet())
    messages = msgs(out)
    assert any("Chute deployed (pitch_timeout)" in m for m in messages), messages
    assert any(w["chan"] == 8 and w["pwm"] == 2000 for w in out["servo_writes"]), out["servo_writes"]
    assert any(w["chan"] == 2 and w["pwm"] == 1100 for w in out["servo_writes"]), out["servo_writes"]


def test_pitchup_does_not_deploy_before_pitch_response_window_expires():
    """The script gives the aircraft time to reach the target pitch."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.params.CHUTE_CLIMB_MS = 300
    """) + trigger_snippet() + textwrap.dedent("""
        h.tick(3)
    """))
    messages = msgs(out)
    assert any("Chute: PITCHUP (full throttle)" in m for m in messages), messages
    assert not any("Chute deployed" in m for m in messages), messages


def test_pitchup_deploys_on_airspeed_cue():
    """Airspeed cue is honored after pitch angle starts the throttle-cut slowdown phase."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.airspeed = 10
    """) + trigger_snippet() + textwrap.dedent("""
        h.tick(2)
        h.state.pitch = math.rad(20)
        h.tick(1)                    -- pitch threshold -> SLOWDOWN, timer restart, throttle cut
        h.tick(2)                    -- fire on slowdown tick, log on DEPLOYED tick
    """))
    messages = msgs(out)
    assert any("Chute: SLOWDOWN (pitch hold, throttle cut)" in m for m in messages), messages
    assert any("Chute deployed (airspeed)" in m for m in messages), messages
    assert any(w["chan"] == 2 and w["pwm"] == 1100 for w in out["servo_writes"]), out["servo_writes"]
    assert any(o["chan"] == 2 and o["pwm"] == 1900 for o in out["rc_overrides"]), out["rc_overrides"]
    assert any(o["chan"] == 3 and o["pwm"] == 0 for o in out["rc_overrides"]), out["rc_overrides"]


def test_pitchup_waits_for_pitch_angle_before_using_airspeed_cue():
    """Low airspeed alone does not deploy before reaching the pitch threshold."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.airspeed = 10
    """) + trigger_snippet() + textwrap.dedent("""
        h.tick(3)
    """))
    messages = msgs(out)
    assert any("Chute: PITCHUP (full throttle)" in m for m in messages), messages
    assert not any("Chute deployed" in m for m in messages), messages


def test_pitchup_pitch_angle_enters_slowdown_before_timeout():
    """Reaching CHUTE_PITCH_DEG cuts throttle while holding pitch-up."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.params.CHUTE_SETUP_MS = 150
        h.state.airspeed = 20
    """) + trigger_snippet() + textwrap.dedent("""
        h.state.pitch = math.rad(20)
        h.tick(1)
    """))
    messages = msgs(out)
    assert any("Chute: PITCHUP (full throttle)" in m for m in messages), messages
    assert any("Chute: SLOWDOWN (pitch hold, throttle cut)" in m for m in messages), messages
    assert any(o["chan"] == 2 and o["pwm"] == 1900 for o in out["rc_overrides"]), out["rc_overrides"]
    assert any(o["chan"] == 3 and o["pwm"] == 0 for o in out["rc_overrides"]), out["rc_overrides"]
    assert not any("Chute deployed" in m for m in messages), messages


def test_slowdown_restarts_timer_at_pitch_threshold():
    """The post-cut hold timer starts at pitch threshold, not at initial trigger."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.params.CHUTE_SETUP_MS = 150
        h.state.airspeed = 20
        h.tick(2)
        h.state.rc_pwms[7] = 1900
        h.tick(2)                    -- 100ms into PITCHUP
        h.state.pitch = math.rad(20)
        h.tick(1)                    -- pitch threshold -> SLOWDOWN, timer resets
        h.tick(2)                    -- only 100ms into SLOWDOWN: no deploy yet
    """))
    messages = msgs(out)
    assert any("Chute: SLOWDOWN (pitch hold, throttle cut)" in m for m in messages), messages
    assert not any("Chute deployed" in m for m in messages), messages

    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.params.CHUTE_SETUP_MS = 150
        h.state.airspeed = 20
        h.tick(2)
        h.state.rc_pwms[7] = 1900
        h.tick(2)
        h.state.pitch = math.rad(20)
        h.tick(1)
        h.tick(4)
    """))
    assert any("Chute deployed (post_cut_hold)" in m for m in msgs(out)), msgs(out)


def test_pitchup_deploys_when_setup_controls_unavailable():
    """Missing pitch/throttle setup controls cannot block emergency deployment."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.params.RCMAP_PITCH = nil
    """) + trigger_snippet() + textwrap.dedent("""
        h.tick(1)
    """))
    messages = msgs(out)
    assert any("setup unavailable (RCMAP_PITCH missing) - deploying" in m for m in messages), messages
    assert any("Chute deployed (setup_unavailable: RCMAP_PITCH missing)" in m for m in messages), messages


def test_pitchup_deploys_when_fbwa_setup_fails():
    """FBWA setup failure cannot block emergency deployment."""
    out = run_scenario(base_airframe() + textwrap.dedent("""
        h.state.mode_fail_for[5] = true
    """) + trigger_snippet() + textwrap.dedent("""
        h.tick(1)
    """))
    messages = msgs(out)
    assert any("cannot enter FBWA setup - deploying" in m for m in messages), messages
    assert any("Chute deployed (setup_fbwa_failed)" in m for m in messages), messages


def test_pitchup_deploys_when_rc_lost_during_setup():
    """RC loss after a valid trigger cannot leave setup waiting until timeout."""
    out = run_scenario(base_airframe() + trigger_snippet() + textwrap.dedent("""
        h.state.valid_rc = false
        h.tick(1)
        h.tick(1)
    """))
    messages = msgs(out)
    assert any("setup_unavailable: RC lost during setup" in m for m in messages), messages


def test_recovery_releases_on_consent():
    """chute LOW + throttle zero + valid RC + 2s hold -> SAFE + FBWA + audit log."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.tick(45)
    """))
    messages = msgs(out)
    assert any("Chute: post-deploy lock released by operator -> FBWA" in m for m in messages), messages
    assert out["state_summary"]["mode"] == 5


def test_recovery_blocked_when_chute_high():
    """Chute switch high must not satisfy the release gate."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1900
        h.state.rc_pwms[3] = 1100
        h.tick(45)
    """))
    assert not any("post-deploy lock released" in m for m in msgs(out)), msgs(out)


def test_recovery_blocked_when_throttle_not_zero():
    """Throttle stick above RCx_MIN+epsilon must not satisfy the release gate."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1500
        h.tick(45)
    """))
    assert not any("post-deploy lock released" in m for m in msgs(out)), msgs(out)


def test_recovery_blocked_without_valid_rc():
    """rc:has_valid_input()==false must refuse release."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.state.valid_rc = false
        h.tick(45)
    """))
    assert not any("post-deploy lock released" in m for m in msgs(out)), msgs(out)


def test_recovery_blocked_when_held_only_1s():
    """Recovery hold timer resets if the gates stop being true."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.tick(20)
        h.state.rc_pwms[7] = 1900
        h.tick(25)
    """))
    assert not any("post-deploy lock released" in m for m in msgs(out)), msgs(out)


def test_recovery_retries_when_set_mode_fails():
    """If FBWA set_mode fails, lock stays asserted and recovery retries."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.state.mode_fail_for[5] = true
        h.tick(45)
        h.state.mode_fail_for[5] = nil
        h.tick(1)
    """))
    messages = msgs(out)
    assert any("Chute: set_mode(FBWA) failed; retrying" in m for m in messages), messages
    assert any("Chute: post-deploy lock released by operator -> FBWA" in m for m in messages), messages


def test_recovery_blocked_when_rcmap_throttle_nil():
    """RCMAP_THROTTLE missing -> fail closed, no release."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.params.RCMAP_THROTTLE = nil
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.tick(45)
    """))
    messages = msgs(out)
    assert any("recovery blocked - RCMAP_THROTTLE missing" in m for m in messages), messages
    assert not any("post-deploy lock released" in m for m in messages), messages


def test_recovery_blocked_when_rcx_min_nil():
    """RC<throttle>_MIN missing -> fail closed, no release."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.params.RC3_MIN = nil
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.tick(45)
    """))
    messages = msgs(out)
    assert any("recovery blocked - RC3_MIN missing" in m for m in messages), messages
    assert not any("post-deploy lock released" in m for m in messages), messages


def test_retrigger_requires_low_then_high_after_release():
    """After recovery, a stale high switch does not retrigger; fresh low->high does."""
    out = run_scenario(fast_deploy_snippet() + textwrap.dedent("""
        h.state.rc_pwms[7] = 1100
        h.state.rc_pwms[3] = 1100
        h.tick(41)                   -- release exactly on this tick

        h.state.pitch = 0
        h.state.airspeed = 20
        h.state.rc_pwms[7] = 1900    -- stale high immediately after edge reset
        h.tick(1)

        h.state.rc_pwms[7] = 1100
        h.tick(1)
        h.state.rc_pwms[7] = 1900    -- fresh edge
        h.tick(1)
    """))
    pitchup_count = sum(1 for m in msgs(out) if "Chute: PITCHUP" in m)
    assert pitchup_count == 2, msgs(out)
