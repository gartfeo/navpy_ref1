"""A wall cleanup reaps only its own launch's processes (todo 51).

The lockstep SITL's command line and environment come from their producer
(``lockstep_wsl``), and the wall SITL's environment from ``launch_command``,
so a change on either side that would let a wall cleanup reach a lockstep
SITL fails here.
"""

from __future__ import annotations

import itertools
import shlex
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from scripts import eval_fleet_span, lockstep_wsl, swarm_run_wsl, wall_reap_wsl
from gcs.backend import instance_ports as ip


def wall_environ(chat: int) -> list[str]:
    """The ``VAR=value`` prefix ``launch_command`` gives ``run_swarm.sh``, which
    its SITLs and router inherit, plus a login shell's own variables."""
    command = swarm_run_wsl.launch_command(
        1, ip.VEHICLES_PER_CHAT * chat, "14550,15550", 3, 2, home_coords="40,44,500,0")
    prefix = list(itertools.takewhile(lambda token: "=" in token, shlex.split(command)))
    return ["PATH=/usr/bin:/bin", "HOME=/home/gart", *prefix]


def wall_sitl_argv(sysid: int, *, joined: bool = False) -> list[str]:
    """``run_swarm.sh``'s SITL command line for global instance ``sysid``."""
    sysid_args = [f"--sysid={sysid}"] if joined else ["--sysid", str(sysid)]
    return ["/home/gart/ardupilot/build/sitl/bin/arduplane", "-S", "--model", "plane",
            "--speedup", "1", "--slave", "0", "--defaults", "/d/plane.parm",
            f"--serial0=udpclient:172.20.0.1:{5760 + 10 * (sysid - 1)}",
            "--serial1=udpclient:0.0.0.0:14600", "--serial2=udpclient:127.0.0.1:9",
            "--sim-address=127.0.0.1", f"-I{sysid - 1}", "--home", "40,44,500,0.0",
            *sysid_args]


def wall_router_argv(chat: int) -> list[str]:
    return ["mavlink-routerd", "-v", "-s", "255", "--tcp-port", "0",
            "--endpoint", f"172.20.0.1:{ip.mission_planner_port(chat)}",
            "--endpoint", f"172.20.0.1:{ip.monitor_port(chat)}", "127.0.0.1:14600"]


def lockstep_sitl(slot: int, sysid: int) -> tuple[list[str], list[str]]:
    """A lockstep SITL as ``lockstep_wsl_sitl`` starts it: ``env -i ... setarch -R``
    exec the binary, so the process holds the command after ``setarch -R``."""
    argv = shlex.split(lockstep_wsl.sitl_command(
        "/home/gart/navpy-eval-roots/lockstep-x", "40,44,500,0", 3 * slot + sysid - 1, sysid))
    environ = shlex.split(lockstep_wsl.sitl_environment(5, 30_000_000, 0))
    assert argv[:2] == ["setarch", "-R"] and environ[:2] == ["env", "-i"]
    return argv[2:], environ[2:]


def owned_by_chat(chat: int, argv: list[str], environ: list[str]) -> bool:
    return wall_reap_wsl.owned(argv, environ, ip.VEHICLES_PER_CHAT * chat,
                               ip.monitor_port(chat), set(ip.sysids_for_chat(chat)))


@pytest.mark.parametrize("slot", [0, 1, 40])
@pytest.mark.parametrize("sysid", [1, 2, 3])
def test_a_slot_0_wall_cleanup_never_matches_a_lockstep_sitl(slot, sysid):
    assert not owned_by_chat(0, *lockstep_sitl(slot, sysid))


@pytest.mark.parametrize("joined", [False, True])
@pytest.mark.parametrize("sysid", [1, 2, 3])
def test_a_wall_cleanup_matches_its_own_sitls_in_both_sysid_spellings(sysid, joined):
    assert owned_by_chat(0, wall_sitl_argv(sysid, joined=joined), wall_environ(0))


def test_a_wall_cleanup_matches_its_own_router_only():
    assert owned_by_chat(1, wall_router_argv(1), wall_environ(1))
    assert not owned_by_chat(0, wall_router_argv(1), wall_environ(1))
    assert not owned_by_chat(1, wall_router_argv(1), wall_environ(0))
    # Only a router: another process of the launch naming the port is not one.
    assert not owned_by_chat(1, ["socat", f"UDP:127.0.0.1:{ip.monitor_port(1)}"], wall_environ(1))


def test_another_chats_wall_sitl_is_not_matched_even_on_a_shared_sysid():
    # A fleet span on chat 0 launches sysids 1..6; chat 1's own SITL 4 is not its.
    span = set(range(1, 7))
    assert not wall_reap_wsl.owned(wall_sitl_argv(4), wall_environ(1), 0, 15550, span)
    assert wall_reap_wsl.owned(wall_sitl_argv(4), wall_environ(0), 0, 15550, span)
    # SWARM_OFFSET=30 is not SWARM_OFFSET=3.
    assert not owned_by_chat(1, wall_sitl_argv(4), wall_environ(10))


def test_owned_pids_reads_the_process_table(tmp_path):
    processes = {
        101: (wall_sitl_argv(1), wall_environ(0)),
        102: (wall_router_argv(0), wall_environ(0)),
        201: lockstep_sitl(40, 1),
        202: lockstep_sitl(0, 2),
        301: (wall_sitl_argv(4), wall_environ(1)),
        401: (["bash", "-lc", "python3 -c ... reap 0 15550 1 2 3"], ["PATH=/bin"]),
    }
    for pid, (argv, environ) in processes.items():
        directory = tmp_path / str(pid)
        directory.mkdir()
        (directory / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")
        (directory / "environ").write_bytes(b"\0".join(e.encode() for e in environ) + b"\0")
    (tmp_path / "999").mkdir()  # a process that exited mid-scan
    (tmp_path / "self").mkdir()
    assert wall_reap_wsl.owned_pids(str(tmp_path), 0, 15550, {1, 2, 3}) == [101, 102]


def test_launch_command_sets_the_offset_the_reaper_keys_on():
    assert "SWARM_OFFSET=120" in wall_environ(40)


def test_a_fleet_teardown_reaps_every_launched_sysid_through_the_same_helper():
    commands: list[str] = []

    def record(command: str) -> SimpleNamespace:
        commands.append(command)
        return SimpleNamespace(returncode=0, stderr=b"")

    eval_fleet_span.cleanup_sysids(2, [7, 8, 9, 10], wsl_run=record)
    assert shlex.split(commands[0])[3:] == ["reap", "6", str(ip.monitor_port(2)),
                                            "7", "8", "9", "10"]


def test_a_failed_reap_raises_so_the_slot_is_not_released():
    failed = SimpleNamespace(returncode=3, stderr=b"no pidfd support")
    with pytest.raises(RuntimeError, match="chat 0 failed .3.: no pidfd support"):
        swarm_run_wsl.cleanup(0, wsl_run=lambda command: failed)
    swarm_run_wsl.cleanup(0, wsl_run=lambda command: SimpleNamespace(returncode=0, stderr=b""))


def test_main_refuses_without_pidfd_support(monkeypatch):
    monkeypatch.delattr(wall_reap_wsl.os, "pidfd_open", raising=False)
    assert wall_reap_wsl.main(["reap", "0", "15550", "1"]) == 3


@pytest.mark.parametrize(("reports", "code"), [(["7 stopped", "8 killed"], 0),
                                              (["7 stopped", "8 survived-kill"], 1)])
def test_main_fails_when_a_process_survives_kill(monkeypatch, capsys, reports, code):
    monkeypatch.setattr(wall_reap_wsl.os, "pidfd_open", lambda pid: 0, raising=False)
    monkeypatch.setattr(wall_reap_wsl.signal, "pidfd_send_signal", lambda *a: None, raising=False)
    monkeypatch.setattr(wall_reap_wsl, "reap", lambda *a, **k: reports)
    assert wall_reap_wsl.main(["reap", "0", "15550", "1"]) == code
    assert capsys.readouterr().out.splitlines() == reports


def _wsl_ready() -> bool:
    if shutil.which("wsl") is None:
        return False
    try:
        return swarm_run_wsl.run_wsl("python3 -c 'import os; os.pidfd_open'").returncode == 0
    except OSError:
        return False


@pytest.mark.skipif(not _wsl_ready(), reason="needs WSL with python3 and pidfd")
def test_reap_in_wsl_kills_only_owned_processes_and_escalates_to_kill():
    """Real processes: offset 1 and sysid 250 are no launch's (offsets are 3*chat)."""
    sleeper = "python3 -c 'import signal, time; {ignore}time.sleep(60)' --sysid 250"
    stubborn = sleeper.format(ignore="signal.signal(signal.SIGTERM, signal.SIG_IGN); ")
    plain = sleeper.format(ignore="")
    spawn = "; ".join([
        f"SWARM_OFFSET=1 nohup {plain} >/dev/null 2>&1 & echo polite=$!",
        f"SWARM_OFFSET=1 nohup {stubborn} >/dev/null 2>&1 & echo stubborn=$!",
        f"env -i PATH=/usr/bin:/bin nohup {plain} >/dev/null 2>&1 & echo lockstep=$!",
        f"SWARM_OFFSET=10 nohup {plain} >/dev/null 2>&1 & echo other=$!",
        "sleep 0.5",
    ])
    pids = dict(line.split("=") for line in
                swarm_run_wsl.run_wsl(spawn).stdout.decode().split())
    try:
        source = swarm_run_wsl.REAP_HELPER.read_text(encoding="utf-8")
        result = swarm_run_wsl.run_wsl(f"python3 -c {shlex.quote(source)} reap 1 9 250")
        assert result.returncode == 0, result.stderr
        reports = sorted(result.stdout.decode().splitlines())
        assert reports == sorted([f"{pids['polite']} stopped", f"{pids['stubborn']} killed"])
        probe = "; ".join(f"kill -0 {pid} 2>/dev/null && echo {name}" for name, pid in pids.items())
        alive = swarm_run_wsl.run_wsl(probe).stdout.decode().split()
        assert alive == ["lockstep", "other"]
    finally:
        swarm_run_wsl.run_wsl("kill -KILL " + " ".join(pids.values()) + " 2>/dev/null")


def test_main_rejects_malformed_arguments(capsys):
    assert wall_reap_wsl.main(["reap", "0"]) == 2
    assert wall_reap_wsl.main(["reap", "x", "15550", "1"]) == 2
