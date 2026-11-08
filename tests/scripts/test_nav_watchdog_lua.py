"""The autopilot's NavPy watchdog, scripts/lua/nav_watchdog.lua (docs/design/navpy_recovery.md, step 7).

The script runs under plain Lua 5.3 against tests/lua/nav_watchdog_harness.lua, every call under ArduPilot's
instruction limit. Heartbeats are encoded by pymavlink, so the script's CRC check is tested against an independent
encoder.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav2

REPO_ROOT = Path(__file__).resolve().parents[2]
HARNESS = REPO_ROOT / "tests" / "lua" / "nav_watchdog_harness.lua"
SCRIPT = REPO_ROOT / "scripts" / "lua" / "nav_watchdog.lua"
AAS_PARAMS = REPO_ROOT / "scripts" / "lua" / "aas_params.lua"
VM_I_COUNT = 10000
GUIDED, AUTO, RTL, LOITER, FBWA = 15, 10, 11, 12, 5
NAVPY_COMPONENT = mav2.MAV_COMP_ID_ONBOARD_COMPUTER


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


def heartbeat(
    *, sysid: int = 1, compid: int = NAVPY_COMPONENT, mav_type: int = mav2.MAV_TYPE_ONBOARD_CONTROLLER,
    mavlink1: bool = False, corrupt: bool = False, claimed_len: int | None = None,
) -> str:
    """A HEARTBEAT as the mavlink_message_t ArduPilot hands a script, in hex: checksum, then the wire header and
    payload (the C struct's field order). ``claimed_len`` overwrites the length byte and pads the payload as the
    full C struct does."""
    link = mav2.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    wire = bytearray(link.heartbeat_encode(mav_type, mav2.MAV_AUTOPILOT_INVALID, 0, 0, mav2.MAV_STATE_ACTIVE)
                     .pack(link, force_mavlink1=mavlink1))
    assert wire[0] == (0xFE if mavlink1 else 0xFD)
    if corrupt:
        wire[-3] ^= 0x01                               # a payload bit flipped in transit
    if claimed_len is not None:
        wire[1] = claimed_len
        wire[-2:-2] = bytes(claimed_len)              # the struct's payload buffer behind the real payload
    if mavlink1:
        # v1 wire: magic len seq sysid compid msgid payload crc -> struct: crc magic len 0 0 seq sysid compid msgid(3)
        header = bytes([wire[0], wire[1], 0, 0, *wire[2:5]]) + bytes([wire[5], 0, 0])
        payload = bytes(wire[6:-2])
    else:
        header, payload = bytes(wire[:10]), bytes(wire[10:-2])
    return (bytes(wire[-2:]) + header + payload).hex()


NAVPY = heartbeat()


def run(scenario: str) -> dict:
    driver = textwrap.dedent(f"""
        local h = dofile('{_lua_path(HARNESS)}')
        local S = h.state
        h.load('{_lua_path(SCRIPT)}')
        local NAVPY = '{NAVPY}'
    """) + textwrap.dedent(scenario) + "\nh.dump()\n"
    proc = subprocess.run(LUA_ARGV + ["-e", driver], capture_output=True, text=True, timeout=120,
                          env=dict(os.environ, MSYS_NO_PATHCONV="1"))
    assert proc.returncode == 0, f"lua exited {proc.returncode}\n{proc.stderr}\n{proc.stdout[-2000:]}"
    out = json.loads(proc.stdout)
    assert out["max_instr"] <= VM_I_COUNT
    assert "script_error" not in out, out.get("script_error")
    assert all(len(m["msg"]) <= 50 for m in out["msgs"]), out["msgs"]     # STATUSTEXT holds 50 characters
    return out


def modes(out: dict) -> list[int]:
    return [m["mode"] for m in out["modes"] if m["ok"]]


def texts(out: dict) -> list[str]:
    return [m["msg"] for m in out["msgs"]]


# NavPy alive in GUIDED for 3 s, heartbeat every 200 ms, then it stops.
ALIVE = "h.run(3000, NAVPY, 200)\n"


def test_loads_receives_heartbeats_and_owns_its_aas_params():
    out = run("h.run(100)")
    state = out["state"]
    assert state["init_args"] == [10, 1] and state["registered"] == [0]       # (queue length, message ids)
    assert state["table_added"] == [88, "AAS_", 30]
    assert [(p["index"], p["name"]) for p in state["added_params"]] == [
        (20, "NWD_ENABLE"), (21, "NWD_TMO"), (22, "NWD_RTL"), (23, "NWD_COMP"),
    ]
    assert "NavPy watchdog loaded" in texts(out)


def test_aas_params_lua_leaves_the_watchdog_indices_free():
    used = {int(line.split(",")[1]) for line in AAS_PARAMS.read_text(encoding="utf-8").splitlines()
            if "add_param(PARAM_TABLE_KEY" in line}
    assert used.isdisjoint({20, 21, 22, 23})


def test_reports_its_state_once_a_second():
    out = run("h.run(3000, NAVPY, 200)")
    floats = [(f["name"], f["value"], f["t"]) for f in out["floats"]]
    assert [f[1] for f in floats] == [0, 0, 0] and all(f[0] == "NAVWD" for f in floats)
    assert [f[2] for f in floats] == [100, 1100, 2100]


def test_navpy_alive_in_guided_is_left_alone():
    out = run("h.run(60000, NAVPY, 1000)\nh.run(2000)")      # 1 Hz heartbeat, then gone
    assert modes(out) == [LOITER] and out["modes"][0]["t"] == 61500


def test_no_navpy_heartbeat_for_the_timeout_holds_in_loiter():
    out = run(ALIVE + """
        h.run(1400)
        assert(#h.out.modes == 0, 'held before the timeout')
        h.run(200)
        h.run(1000)
    """)
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] - 3000 == 1500                    # last heartbeat at 3000 ms
    assert "NavPy watchdog: no NavPy heartbeat, LOITER" in texts(out)
    assert out["floats"][-1]["value"] == 1


def test_navpy_back_in_the_hold_resumes_auto():
    out = run(ALIVE + "h.run(5000)\nh.run(400, NAVPY, 200)")
    assert modes(out) == [LOITER, AUTO]
    assert "NavPy watchdog: NavPy back, AUTO" in texts(out)


def test_a_hold_without_navpy_for_the_recovery_time_gives_rtl_and_stays():
    out = run(ALIVE + """
        h.run(1500)
        h.run(29900)
        assert(#h.out.modes == 1, 'RTL before the hold time')
        h.run(200)
        h.run(3000, NAVPY, 200)              -- NavPy back after RTL: not undone
    """)
    assert modes(out) == [LOITER, RTL]
    assert out["modes"][1]["t"] - out["modes"][0]["t"] == 30000
    assert out["floats"][-1]["value"] == 2


@pytest.mark.parametrize("pilot_mode", [FBWA, RTL, AUTO])
def test_a_pilot_or_failsafe_mode_in_the_hold_wins(pilot_mode):
    out = run(ALIVE + f"""
        h.run(2000)
        S.mode = {pilot_mode}
        h.run(40000)
        h.run(3000, NAVPY, 200)
    """)
    assert modes(out) == [LOITER]
    assert "NavPy watchdog: mode changed, standing down" in texts(out)


def test_rtl_left_by_the_pilot_stands_down():
    out = run(ALIVE + """
        h.run(32000)
        S.mode = 15                          -- the operator puts it back in GUIDED, NavPy still dead
        h.run(5000)
    """)
    assert modes(out) == [LOITER, RTL]
    assert out["floats"][-1]["value"] == 0


def test_guided_without_navpy_is_not_navpys():
    out = run("h.run(10000)\nS.mode = 12\nS.mode = 15\nh.run(10000)")
    assert out["modes"] == []


def test_guided_entered_after_navpy_died_is_not_navpys():
    out = run("S.mode = 10\n" + ALIVE + "h.run(3000)\nS.mode = 15\nh.run(5000)")
    assert out["modes"] == []


def test_disarmed_is_left_alone():
    out = run("S.armed = false\n" + ALIVE + "h.run(5000)")
    assert out["modes"] == []


@pytest.mark.parametrize("frame", [
    heartbeat(sysid=2), heartbeat(compid=1), heartbeat(mav_type=mav2.MAV_TYPE_GCS), heartbeat(corrupt=True),
], ids=["other-system", "autopilot-component", "gcs-type", "bad-crc"])
def test_only_navpys_own_heartbeat_counts(frame):
    out = run(ALIVE + f"h.run(5000, '{frame}', 200)")
    assert modes(out) == [LOITER]                 # the other frames did not keep NavPy alive


def test_a_mavlink1_heartbeat_counts():
    """Codex step 7, finding 6: MAVLink 1 heartbeats make NavPy alive, so their loss holds."""
    out = run(f"h.run(3000, '{heartbeat(mavlink1=True)}', 200)\nh.run(2000)")
    assert modes(out) == [LOITER]


def test_a_refused_loiter_is_retried_and_reported_once():
    out = run("S.mode_refused[12] = true\n" + ALIVE + """
        h.run(2000)
        S.mode_refused[12] = nil
        h.run(200)
    """)
    assert [m["ok"] for m in out["modes"]][-1] is True and modes(out) == [LOITER]
    assert len(out["modes"]) > 2
    assert texts(out).count("NavPy watchdog: LOITER refused, retrying") == 1


def test_a_refused_auto_keeps_the_hold_and_its_rtl_deadline():
    out = run("S.mode_refused[10] = true\n" + ALIVE + "h.run(1600)\nh.run(31000, NAVPY, 200)")
    assert modes(out) == [LOITER, RTL]
    assert "NavPy watchdog: AUTO refused, retrying" in texts(out)


def test_disabled_does_nothing_and_reports_off():
    out = run("S.params.AAS_NWD_ENABLE = 0\n" + ALIVE + "h.run(5000)")
    assert out["modes"] == []
    assert out["floats"][-1]["value"] == -1 and "NavPy watchdog disabled" in texts(out)


def test_an_error_in_a_step_is_reported_and_the_loop_runs_on():
    out = run(ALIVE + """
        S.get_mode_error = 'boom'
        h.run(1000)
        S.get_mode_error = nil
        h.run(1000)
    """)
    assert any(t.startswith("NavPy watchdog error:") for t in texts(out))
    assert modes(out) == [LOITER] and out["calls"] == 50          # still called every tick
    assert len(out["floats"]) == 5                                  # and still reporting


def test_a_full_receive_queue_fits_the_instruction_limit():
    """A full queue of frames that each need the CRC (none is NavPy's: the worst case), then one of NavPy's."""
    other = heartbeat(corrupt=True)
    out = run(ALIVE + f"for _ = 1, 25 do h.push('{other}') end\nh.run(100)\n"
              "for _ = 1, 25 do h.push(NAVPY) end\nh.run(100)")
    assert out["max_instr"] <= VM_I_COUNT and out["modes"] == []


# ---------- ArduPilot's Lua build ----------
AP_MAXVARS = 100     # ArduPilot libraries/AP_Scripting/lua/src/lparser.c (stock Lua 5.3: 200)
AP_MAXUPVAL = 127    # lfunc.h


def test_script_fits_ardupilot_lua_limits():
    *prefix, lua = LUA_ARGV
    if not lua.endswith("lua5.3"):
        pytest.skip("luac5.3 not found next to lua")
    proc = subprocess.run(prefix + [lua[: -len("lua5.3")] + "luac5.3", "-l", "-l", "-p", _lua_path(SCRIPT)],
                          capture_output=True, text=True, env=dict(os.environ, MSYS_NO_PATHCONV="1"))
    assert proc.returncode == 0, proc.stderr
    lines = proc.stdout.splitlines()
    max_active = max_up = 0
    for i, line in enumerate(lines):
        if line.startswith("locals ("):
            n = int(line.split("(")[1].split(")")[0])
            spans = [tuple(int(x) for x in lines[j].split()[-2:]) for j in range(i + 1, i + 1 + n)]
            for pc in {p for span in spans for p in span}:
                max_active = max(max_active, sum(1 for a, b in spans if a <= pc < b))
        elif line.startswith("upvalues ("):
            max_up = max(max_up, int(line.split("(")[1].split(")")[0]))
    assert max_active <= AP_MAXVARS and max_up <= AP_MAXUPVAL, (max_active, max_up)


# ---------- Codex review of step 7 ----------


def _hold_then(scenario: str) -> str:
    return ALIVE + "h.run(1600)\nassert(S.mode == 12, 'not held')\n" + scenario


def test_finding1_a_pilot_mode_between_the_read_and_the_loiter_write_wins():
    """The autopilot's main loop runs between two binding calls: the pilot selects FBWA after the script read
    GUIDED, before its LOITER write."""
    out = run(ALIVE + """
        h.run(1400)
        S.on_armed = function() if S.time_ms >= 4500 then S.mode = 5; S.on_armed = nil end end
        h.run(5000)
    """)
    assert out["modes"] == [] and out["state"]["mode"] == FBWA


def test_finding1_a_pilot_mode_between_the_read_and_the_rtl_write_wins():
    out = run(_hold_then("""
        S.on_param = function(name) if name == 'AAS_NWD_RTL' and S.time_ms >= 34500 then S.mode = 5; S.on_param = nil end end
        h.run(35000)
    """))
    assert modes(out) == [LOITER] and out["state"]["mode"] == FBWA
    assert "NavPy watchdog: mode changed, standing down" in texts(out)


def test_finding1_a_pilot_mode_between_the_read_and_the_auto_write_wins():
    out = run(_hold_then("""
        S.on_param = function(name) if name == 'AAS_NWD_TMO' and S.time_ms >= 4800 then S.mode = 5; S.on_param = nil end end
        h.run(1000, NAVPY, 200)
    """))
    assert modes(out) == [LOITER] and out["state"]["mode"] == FBWA


def test_the_timeout_is_aas_nwd_tmo_after_the_last_heartbeat():
    """A fixed timeout in aircraft time (Demo's ruling on Codex round 2): NavPy paces its heartbeat in aircraft
    time (heartbeat_pacing.py), so the autopilot needs no estimate of its cadence."""
    out = run("S.params.AAS_NWD_TMO = 2.5\nh.run(10000, NAVPY, 1000)\nh.run(4000)")
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] - 10000 == 2500


def test_with_a_2_5s_timeout_one_lost_1hz_beat_does_not_hold():
    """The owner's option (b), 1 Hz and 2.5 s: one beat lost now and then is not a dead NavPy."""
    out = run("""
        S.params.AAS_NWD_TMO = 2.5
        for i = 1, 30 do
            h.run(1000)
            if i % 5 ~= 0 then h.push(NAVPY) end
        end
        h.run(3000)
    """)
    assert modes(out) == [LOITER]                 # only once NavPy is gone for good


# ---------- Codex round 2 on step 7 (the cadence estimator, dropped) ----------


def test_r2_finding1_a_duplicate_burst_does_not_change_the_timeout():
    """Frames received a millisecond apart are heartbeats, not a cadence: the hold still comes AAS_NWD_TMO after
    the last one."""
    out = run("""
        h.run(10000, NAVPY, 1000)
        for i = 1, 5 do h.push(NAVPY, 10000 + i) end
        h.run(10000, NAVPY, 1000)
        h.run(3000)
    """)
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] - 20000 == 1500


def test_r2_finding2_a_long_outage_does_not_lengthen_the_timeout():
    """One heartbeat, a 60 s gap, one more (GUIDED from then), then NavPy is gone: held AAS_NWD_TMO later."""
    out = run("""
        S.mode = 10
        h.push(NAVPY); h.run(100)
        h.run(60000)
        S.mode = 15
        h.push(NAVPY); h.run(100)
        h.run(3000)
    """)
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] - 60100 == 1500


def test_r2_finding3_one_heartbeat_then_loss_still_holds():
    out = run("h.push(NAVPY)\nh.run(3000)")
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] == 1500


def test_each_mode_write_comes_right_after_its_guard_read():
    """The known limit (design doc, step 7): stock-firmware Lua has no compare-and-set, so the guard read is the
    last binding call before each write; only the autopilot's main loop can run between them."""
    out = run(ALIVE + """
        h.run(1600)                          -- LOITER
        h.run(400, NAVPY, 200)               -- AUTO
        S.mode = 15
        h.run(1000, NAVPY, 200)
        h.run(33000)                         -- LOITER, then RTL
    """)
    assert modes(out) == [LOITER, AUTO, LOITER, RTL]
    assert {m["after"] for m in out["modes"]} == {"vehicle:get_mode"}


def test_finding3_a_stale_queued_heartbeat_does_not_end_the_hold():
    """A heartbeat the autopilot received before the hold, read by the script only now, is not NavPy back."""
    out = run(_hold_then("""
        h.push(NAVPY, 4400)                  -- received before the hold (4500), processed late
        h.run(2000)
    """))
    assert modes(out) == [LOITER]


def test_finding3_a_heartbeat_received_long_ago_is_not_fresh():
    out = run(ALIVE + """
        h.run(1400)
        h.push(NAVPY, 3100)                  -- received 1.3 s ago, processed now
        h.run(200)
    """)
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] - 3100 == 1500


def test_finding4_frames_claiming_a_long_payload_fit_the_instruction_limit():
    long = heartbeat(claimed_len=255)
    out = run(ALIVE + f"for _ = 1, 10 do h.push('{long}') end\nh.run(100)\nh.run(1500)")
    assert modes(out) == [LOITER]                 # they did not count as NavPy, and the script lives


def test_finding5_disarming_ends_the_hold():
    out = run(_hold_then("""
        S.armed = false
        h.run(200)
        h.run(1000, NAVPY, 200)              -- NavPy back while disarmed: no AUTO
        S.armed = true
        h.run(31000)                         -- re-armed: no RTL from the old hold
    """))
    assert modes(out) == [LOITER]
    assert "NavPy watchdog: disarmed, standing down" in texts(out)


# ---------- Owner ruling 2026-11-08: 4 Hz companion heartbeat, AAS_NWD_TMO 1.5 s ----------

FLY_4HZ = """
local next_beat = 250
local function fly(ms, lost)          -- NavPy's 4 Hz heartbeat, each received at its own time unless lost(t)
    local stop = S.time_ms + ms
    while S.time_ms < stop do
        while next_beat <= S.time_ms + 100 do
            if not (lost and lost(next_beat)) then h.push(NAVPY, next_beat) end
            next_beat = next_beat + 250
        end
        h.run(100)
    end
end
"""


def test_at_4hz_five_lost_beats_in_a_row_do_not_hold():
    """1.25 s without a heartbeat (five lost) is under the 1.5 s timeout."""
    out = run(FLY_4HZ + """
        fly(5000)
        fly(10000, function(t) return t > 6000 and t <= 7250 end)
        fly(5000)
    """)
    assert out["modes"] == []


def test_at_4hz_a_dead_navpy_holds_1_5s_after_its_last_heartbeat():
    out = run(FLY_4HZ + "fly(5000)\nh.run(2000)")
    assert modes(out) == [LOITER]
    assert out["modes"][0]["t"] - 5000 == 1500
