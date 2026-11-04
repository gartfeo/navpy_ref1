"""Step 17's owner checks (tools/tuning/approach_owners.py, approach_tables.py): exactly one owner each for the
autopilot, bridge, emulator and publisher, the runner's own; conflicting listeners and processes, in WSL and on
Windows, navpy's and aas-hitl's, are refused; the logs show one source of each. Synthetic tables only."""
from __future__ import annotations

import json
from dataclasses import replace

import pytest

from tools.tuning import approach_owners as owners
from tools.tuning.approach_procs import Owned, ProcSpec, RunnerError
from tools.tuning.approach_tables import (
    WINDOWS,
    WSL,
    Proc,
    Sock,
    Tables,
    parse_netstat,
    parse_proc_net,
    parse_windows_procs,
)

SELF = 10
EMU_ARGS = ("-s", "-m", "tools.hitl.zr10_emulator", "--vehicle-state", "172.0.0.1:17113", "--exit-on-stdin-eof")


class Handle:
    def __init__(self, pid: int, windows_pid: int | None = None) -> None:
        self.pid, self.windows_pid = pid, windows_pid


class Group:
    """The runner's owned roles: WSL leaders by pid, Unreal by its Windows pid, the emulator's own Windows Python."""

    def __init__(self) -> None:
        roles = [("backing", Handle(100)), ("router", Handle(200)), ("bridge", Handle(300)),
                 ("emulator", Handle(400, windows_pid=5001)), ("publisher", Handle(500)), ("navpy", Handle(700))]
        self.owned = [Owned(ProcSpec(role, (), None, 0.0, kind="interop" if role == "emulator" else "posix"),
                            handle, None, "t") for role, handle in roles]
        self.owned.append(Owned(ProcSpec("unreal", (), None, 0.0, kind="windows"), Handle(6000), None, "t"))


def chain_tables() -> list[Tables]:
    """A running chain as the tables show it, including the parts that are not owners."""
    wsl_procs = (
        Proc(WSL, SELF, 1, ("python", "-s", "-m", "tools.tuning.sitl_driver", "approach")),
        Proc(WSL, 100, SELF, ("python", "-s", "-m", "tools.tuning.hitl", "sitl-backing"), 100),
        Proc(WSL, 101, 100, ("/fw/arduplane", "-I240"), 101),        # SITL in its own group: owned by ancestry
        Proc(WSL, 200, SELF, ("python", "-m", "tools.tuning.mav_router"), 200),
        Proc(WSL, 300, SELF, ("python", "-s", "-m", "tools.tuning.hitl_bridge", "--device", "tcp:127.0.0.1:8168"), 300),
        Proc(WSL, 400, SELF, ("/mnt/c/venv/Scripts/python.exe", *EMU_ARGS), 400),      # the interop proxy
        Proc(WSL, 500, SELF, ("nice", "-n", "10", "/usr/bin/python3", "-E", "-m", "aas_hitl.video.gstreamer_encoder"),
             500),
        Proc(WSL, 501, 500, ("/opt/mediamtx", "cfg.yml"), 500),
        Proc(WSL, 700, SELF, ("python", "-m", "navpy.main"), 700),
    )
    wsl_socks = (
        Sock(WSL, "tcp", 8160, "listen", 101), Sock(WSL, "tcp", 8168, "listen", 101),
        Sock(WSL, "tcp", 8160, "established", 101, 40000),           # SITL's end of the router's connection
        Sock(WSL, "tcp", 40000, "established", 200, 8160),           # the router: the expected serial0 client
        Sock(WSL, "tcp", 40001, "established", 300, 8168),           # the bridge on HIL
        Sock(WSL, "udp", 9642, "bound", 200),
        Sock(WSL, "tcp", 18110, "listen", 500), Sock(WSL, "tcp", 18554, "listen", 501),
    )
    windows_procs = (
        Proc(WINDOWS, 5000, 4000, ("C:\\venv\\Scripts\\python.exe", *EMU_ARGS)),   # the venv launcher
        Proc(WINDOWS, 5001, 5000, ("C:\\Python311\\python.exe", *EMU_ARGS)),       # the emulator itself
        Proc(WINDOWS, 6000, 4000, ("UnrealEditor.exe", "A.uproject", "-game")),
        Proc(WINDOWS, 9000, 4, ("C:\\Program Files\\WSL\\wslrelay.exe", "--mode", "1")),
        Proc(WINDOWS, 9001, 4, ("--mode", "2", "--vm-id", "{0ca2}"), name="wslrelay.exe"),  # as Win32_Process shows it
    )
    windows_socks = (
        Sock(WINDOWS, "udp", 37260, "bound", 5001), Sock(WINDOWS, "udp", 17113, "bound", 5001),
        Sock(WINDOWS, "udp", 17110, "bound", 6000), Sock(WINDOWS, "udp", 17212, "bound", 6000),
        Sock(WINDOWS, "tcp", 51000, "established", 6000, 18110),     # Unreal's frames to the publisher
        Sock(WINDOWS, "tcp", 8160, "listen", 9000), Sock(WINDOWS, "tcp", 18110, "listen", 9000),   # WSL mirrors
        Sock(WINDOWS, "tcp", 8168, "listen", 9001), Sock(WINDOWS, "tcp", 18554, "listen", 9001),
    )
    return [Tables(wsl_procs, wsl_socks), Tables(windows_procs, windows_socks)]


def census(tables, running=True):
    return owners.census(Group(), lambda: tables, SELF, running)


def add(tables, host, procs=(), socks=()):
    i = 0 if host == WSL else 1
    tables[i] = Tables(tables[i].procs + tuple(procs), tables[i].socks + tuple(socks))
    return tables


# ---- programs --------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("argv, roles", [
    (("python", "-m", "tools.tuning.hitl_bridge"), ("bridge",)),
    (("python.exe", "-s", "-m", "tools.hitl.zr10_emulator", "--siyi", "x"), ("emulator",)),
    (("python3", "-E", "-s", "-m", "aas_hitl.video.gstreamer_encoder"), ("publisher",)),
    (("py", "-3.11", "-m", "aas_hitl", "run", "--count", "1"), ("autopilot", "bridge")),
    (("py", "-m", "aas_hitl", "sitl", "start"), ("autopilot",)),
    (("py", "-m", "aas_hitl", "sitl", "mission"), ("autopilot",)),
    (("py", "-m", "aas_hitl", "bridge", "--uav", "1"), ("bridge",)),
    (("py", "-m", "aas_hitl", "zr10", "emulate", "--config", "h.toml"), ("emulator",)),
    (("py", "-m", "aas_hitl.zr10"), ("emulator",)),
    (("py", "-m", "aas_hitl", "video", "publish"), ("publisher",)),
    (("py", "-m", "aas_hitl", "video", "check"), ()),                # read-only
    (("py", "-m", "aas_hitl", "sitl", "status"), ()),
    (("grep", "tools.hitl.zr10_emulator"), ()),                      # names it, does not run it
    (("python", "-m", "pytest", "-m", "tools.hitl.zr10_emulator"), ()),
    (("nice", "-n", "10", "/usr/bin/python3", "-E", "-m", "aas_hitl.video.gstreamer_encoder"), ("publisher",)),
    (("C:\\Py\\python.exe", "-W", "ignore", "-m", "tools.tuning.hitl_bridge"), ("bridge",)),
    (("python", "run.py", "-m", "tools.tuning.hitl_bridge"), ()),          # a script's own arguments
    (("python", "-c", "import x", "-m", "aas_hitl.zr10"), ()),
    (("C:\Py\pythonw.exe", "-m", "tools.hitl.zr10_emulator"), ("emulator",)),
    (("pyw", "-3.11", "-m", "aas_hitl", "zr10", "emulate"), ("emulator",)),
    # Found live: a shell whose command text names the emulator (Windows command lines split on spaces).
    (("C:\\Git\\bin\\bash.exe", "-c", "source", "s.sh", "&&", "py", "-3.11", "-m", "aas_hitl.zr10"), ()),
])
def test_a_program_plays_the_role_its_module_implements(argv, roles):
    assert owners.program_roles(argv) == roles


# ---- tables ----------------------------------------------------------------------------------------------------
PROC_NET_TCP = """  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode
   0: 0100007F:1FE0 00000000:0000 0A 00000000:00000000 00:00000000 00000000  1000        0 111 1
   1: 0100007F:9C40 0100007F:1FE0 01 00000000:00000000 00:00000000 00000000  1000        0 222 1
   2: 0100007F:9C41 0100007F:1FE0 06 00000000:00000000 00:00000000 00000000     0        0 0 3
   3: 0100007F:9C42 0100007F:1FE8 02 00000000:00000000 00:00000000 00000000  1000        0 333 1
"""
PROC_NET_UDP = """  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref
  10: 00000000:9154 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 444 2
"""


def test_proc_net_rows_give_port_state_and_owner_and_time_wait_holds_nothing():
    tcp = parse_proc_net(PROC_NET_TCP, "tcp", {111: 1, 222: 2})
    assert tcp == [Sock(WSL, "tcp", 8160, "listen", 1), Sock(WSL, "tcp", 40000, "established", 2, 8160),
                   Sock(WSL, "tcp", 40002, "established", None, 8168)]    # SYN_SENT; its process unreadable
    assert parse_proc_net(PROC_NET_UDP, "udp", {444: 5}) == [Sock(WSL, "udp", 37204, "bound", 5)]


NETSTAT = """
Active Connections

  Proto  Local Address          Foreign Address        State           PID
  TCP    0.0.0.0:18554          0.0.0.0:0              LISTENING       77
  TCP    172.0.0.1:51000        172.0.0.2:18110        ESTABLISHED     6000
  TCP    127.0.0.1:51001        127.0.0.1:51002        TIME_WAIT       0
  TCP    [::]:8160              [::]:0                 LISTENING       78
  UDP    172.0.0.1:37260        *:*                                    5001
  UDP    [::]:17113             *:*                                    79
"""


def test_netstat_rows_give_port_state_and_pid():
    assert parse_netstat(NETSTAT) == [
        Sock(WINDOWS, "tcp", 18554, "listen", 77), Sock(WINDOWS, "tcp", 51000, "established", 6000, 18110),
        Sock(WINDOWS, "tcp", 8160, "listen", 78), Sock(WINDOWS, "udp", 37260, "bound", 5001),
        Sock(WINDOWS, "udp", 17113, "bound", 79)]


def test_the_windows_process_list_is_one_object_or_a_list_and_a_missing_command_line_is_empty():
    one = parse_windows_procs(json.dumps({"ProcessId": 4, "ParentProcessId": 0, "Name": "System", "CommandLine": None}))
    line = '"C:\\Program Files\\Py\\python.exe" -s -m tools.hitl.zr10_emulator'
    many = parse_windows_procs(json.dumps([{"ProcessId": 5001, "ParentProcessId": 5000, "CommandLine": line}]))
    assert one == [Proc(WINDOWS, 4, 0, (), name="System")]
    assert many[0].argv == ("C:\\Program Files\\Py\\python.exe", "-s", "-m", "tools.hitl.zr10_emulator")


# ---- the census ------------------------------------------------------------------------------------------------
def test_a_running_chain_has_exactly_its_own_owner_for_each_role():
    result = census(chain_tables())
    assert result["ok"], result["problems"]
    assert {role: item["owners"] for role, item in result["roles"].items()} == {
        "autopilot": ["backing"], "bridge": ["bridge"], "emulator": ["emulator"], "publisher": ["publisher"]}
    emulator = result["roles"]["emulator"]["claims"]
    assert {(c["host"], c["pid"]) for c in emulator} == {(WSL, 400), (WINDOWS, 5000), (WINDOWS, 5001)}


def test_before_the_run_any_holder_is_refused_and_none_passes():
    assert census([Tables((), ()), Tables((), ())], running=False)["ok"]
    result = census(chain_tables(), running=False)
    assert not result["ok"] and len(result["problems"]) == 4


CONFLICTS = {
    "a second SITL listener": (WSL, (), (Sock(WSL, "tcp", 8168, "listen", 900),), "autopilot"),
    "a second serial0 owner": (WSL, (), (Sock(WSL, "tcp", 41000, "established", 900, 8160),), "autopilot"),
    "aas-hitl's flight launcher": (WINDOWS, (Proc(WINDOWS, 7000, 1, ("py", "-m", "aas_hitl", "run")),), (),
                                   "autopilot"),
    "a second HIL client": (WSL, (), (Sock(WSL, "tcp", 41001, "established", 901, 8168),), "bridge"),
    "aas-hitl's truth bridge": (WINDOWS, (Proc(WINDOWS, 7001, 1, ("py", "-m", "aas_hitl", "bridge")),), (), "bridge"),
    "another navpy bridge": (WSL, (Proc(WSL, 902, 1, ("python", "-m", "tools.tuning.hitl_bridge")),), (), "bridge"),
    "aas-hitl's ZR10 emulator": (WINDOWS, (Proc(WINDOWS, 7002, 1, ("py", "-m", "aas_hitl", "zr10", "emulate")),),
                                 (Sock(WINDOWS, "udp", 37260, "bound", 7002),), "emulator"),
    "a SIYI listener in WSL": (WSL, (), (Sock(WSL, "udp", 37260, "bound", 903),), "emulator"),
    "a SIYI socket of no known process": (WINDOWS, (), (Sock(WINDOWS, "udp", 37260, "bound", None),), "emulator"),
    "a second navpy emulator": (WINDOWS, (Proc(WINDOWS, 7003, 1, ("python.exe", *EMU_ARGS)),), (), "emulator"),
    "aas-hitl's FFmpeg publisher": (WINDOWS, (Proc(WINDOWS, 7004, 1, ("py", "-m", "aas_hitl", "video", "publish")),),
                                    (Sock(WINDOWS, "tcp", 18110, "listen", 7004),), "publisher"),
    "an RTSP listener on Windows": (WINDOWS, (), (Sock(WINDOWS, "tcp", 18554, "listen", 7005),), "publisher"),
}


@pytest.mark.parametrize("name", sorted(CONFLICTS))
def test_a_conflicting_listener_or_process_makes_a_second_owner(name):
    host, procs, socks, role = CONFLICTS[name]
    result = census(add(chain_tables(), host, procs, socks))
    assert not result["ok"] and sorted(result["roles"][role]["owners"]) == sorted(
        ["backing" if role == "autopilot" else role, "foreign"])
    expected = ["autopilot", "bridge"] if "launcher" in name else [role]      # `run` flies SITL and its bridge
    assert [problem.split(":")[0] for problem in result["problems"]] == expected
    assert not census(add(chain_tables(), host, procs, socks), running=False)["ok"]


def test_a_role_of_the_runner_on_another_roles_footprint_is_a_second_owner():
    tables = add(chain_tables(), WSL, (), (Sock(WSL, "udp", 37260, "bound", 700),))     # navpy bound the SIYI port
    result = census(tables)
    assert result["roles"]["emulator"]["owners"] == ["emulator", "navpy"] and not result["ok"]


COPIES = {
    "bridge": (WSL, Proc(WSL, 301, 300, ("python", "-m", "tools.tuning.hitl_bridge"), 300)),
    "publisher": (WSL, Proc(WSL, 502, 500, ("python3", "-m", "aas_hitl.video.gstreamer_encoder"), 500)),
    "emulator": (WINDOWS, Proc(WINDOWS, 5002, 5001, ("C:\\Python311\\python.exe", *EMU_ARGS))),
}


@pytest.mark.parametrize("role", sorted(COPIES))
def test_a_second_copy_inside_the_roles_own_tree_is_a_second_owner(role):
    host, proc = COPIES[role]
    result = census(add(chain_tables(), host, (proc,)))
    assert result["roles"][role]["owners"] == [role] and not result["ok"]
    assert result["problems"][0].endswith(f"2 or more copies on {host}")


def test_a_second_holder_of_a_role_socket_inside_its_own_tree_is_a_second_owner():
    """A copy whose command line Windows does not show (or a windowed Python) is still seen by its socket."""
    hidden = Proc(WINDOWS, 5003, 5001, ())
    result = census(add(chain_tables(), WINDOWS, (hidden,), (Sock(WINDOWS, "udp", 37260, "bound", 5003),)))
    assert result["roles"]["emulator"]["owners"] == ["emulator"] and not result["ok"]
    assert result["problems"][0].endswith("2 or more copies on windows emulator sockets")


def test_a_hidden_copy_holding_the_roles_other_socket_is_a_second_owner():
    """Codex round 3: the emulator holds 37260, a child with no command line holds 17113: two processes, one unit."""
    hidden = Proc(WINDOWS, 5003, 5001, ())
    tables = chain_tables()
    tables[1] = replace(tables[1], procs=tables[1].procs + (hidden,),
                        socks=tuple(replace(s, pid=5003) if s.local_port == 17113 else s for s in tables[1].socks))
    result = census(tables)
    assert not result["ok"] and result["problems"][0].endswith("2 or more copies on windows emulator sockets")


def test_the_publishers_two_servers_may_be_two_processes():
    """Live run 3: the native server (18110) and the RTSP server (18554) are separate publisher processes."""
    tables = chain_tables()
    wsl_socks = {s.local_port: s.pid for s in tables[0].socks if s.local_port in (18110, 18554)}
    assert len(set(wsl_socks.values())) == 2 and census(tables)["ok"]


def test_a_role_with_no_owner_fails():
    tables = chain_tables()
    tables[0] = replace(tables[0], socks=tuple(s for s in tables[0].socks if s.local_port not in (18110, 18554)),
                        procs=tuple(p for p in tables[0].procs if p.pid not in (500, 501)))
    result = census(tables)
    assert result["roles"]["publisher"]["owners"] == [] and result["problems"] == [
        "publisher: exactly 1 owner (publisher), found none"]


def test_unreadable_tables_fail_the_census():
    def broken():
        raise PermissionError("netstat failed")
    result = owners.census(Group(), broken, SELF, running=False)
    assert not result["ok"] and "netstat failed" in result["problems"][0]


def test_the_runner_check_raises_before_and_at_ready_and_only_records_at_the_end():
    run: dict = {}
    owners.check(run, Group(), owners.END, lambda: chain_tables())
    assert run == {"owners": {}}                     # the chain never came up: nothing to count
    with pytest.raises(RunnerError, match="owner census \\(preflight\\)"):
        owners.check(run, Group(), owners.PREFLIGHT, chain_tables)
    owners.check(run, Group(), owners.READY, chain_tables)
    owners.check(run, Group(), owners.END, lambda: add(chain_tables(), WSL, (), (Sock(WSL, "tcp", 8160, "listen", 9),)))
    passed, problems = owners.verdicts({**run, "sources": {"ok": True, "problems": []}})
    assert not passed and problems[0].startswith("owners preflight: ") and problems[-1].startswith("owners end: ")


# ---- the logs --------------------------------------------------------------------------------------------------
SESSION = "3e89dffbfdc644ec90e5c1ed2722286d"


def unreal_lines(session=SESSION, generations=(17110,), pose_ports=(17110,), slots=(SESSION,), gimbal=(SESSION,)):
    lines = [f"LogTemp: Display: NavPyRenderClock port={port} event=generation gen=1 render_us=1.0 next_slot=1"
             for port in generations]
    lines += [f"LogTemp: Display: NavPyPoseAdmit port={port} gen=1 admitted=1 board_us=1 seq=1" for port in pose_ports]
    lines += [f"LogTemp: Display: NavPyGimbalHistory sys_id=1 event=session session={g} seq=1 sender_time_s=1.0"
              for g in gimbal]
    lines += [f"LogTemp: Display: NavPyCaptureSlot outcome=sent sys_id=1 run=R gen=1 slot=1 board_us=1 frame=0 "
              f"gimbal_qpc=1.0 gimbal_session={s} gimbal_seq=1,2" for s in slots]
    lines.append("LogTemp: Display: NavPyCaptureSlot outcome=missed:late-tick sys_id=1 run=R gen=1 slot=2 count=1")
    return "\n".join(lines) + "\n"


def emulator_lines(sessions=1, other_system=0, regressions=0, summary=True):
    lines = [{"event": "start", "session_id": SESSION}, {"event": "step", "t": 1.0}]
    if summary:
        lines.append({"event": "summary", "valid": False, "vehicle_state": {
            "sessions": sessions, "other_system": other_system, "board_time_regressions": regressions, "gaps": 3}})
    return "".join(json.dumps(line, separators=(",", ":")) + "\n" for line in lines)


def sources(tmp_path, unreal=None, emulator=None):
    (tmp_path / "unreal.log").write_text(unreal if unreal is not None else unreal_lines(), encoding="utf-8")
    (tmp_path / "emulator.jsonl").write_text(emulator if emulator is not None else emulator_lines(), encoding="utf-8")
    return owners.sources(tmp_path / "unreal.log", tmp_path / "emulator.jsonl")


def test_one_pose_stream_one_gimbal_session_and_one_bridge_session_pass(tmp_path):
    result = sources(tmp_path)
    assert result["ok"], result["problems"]
    assert (result["gimbal_sessions"], result["bridge_sessions"]) == ([SESSION], 1)


@pytest.mark.parametrize("unreal, emulator, problem", [
    (unreal_lines(generations=(17110, 17110)), None, "pose streams"),                # a second sender session
    (unreal_lines(pose_ports=(17110, 17120)), None, "poses on ports"),
    (unreal_lines(gimbal=(SESSION, "f" * 32)), None, "gimbal sessions"),            # a second emulator
    (unreal_lines(gimbal=()), None, "gimbal sessions"),
    (unreal_lines(slots=(SESSION, "f" * 32)), None, "rendered from gimbal sessions"),
    (None, emulator_lines(sessions=2), "bridge session"),                           # a second bridge
    (None, emulator_lines(other_system=4), "another system"),
    (None, emulator_lines(regressions=1), "board time regression"),
    (None, emulator_lines(summary=False), "no emulator summary"),
    (None, '{"event":"start","session_id":"x"}\n{"event":"summary",', "unreadable"),
])
def test_a_second_source_in_the_logs_fails(tmp_path, unreal, emulator, problem):
    result = sources(tmp_path, unreal, emulator)
    assert not result["ok"] and any(problem in item for item in result["problems"]), result["problems"]


def test_a_missing_log_fails(tmp_path):
    result = owners.sources(tmp_path / "unreal.log", tmp_path / "emulator.jsonl")
    assert not result["ok"] and "unreadable" in result["problems"][0]
