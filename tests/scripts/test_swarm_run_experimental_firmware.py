"""Experimental firmware stays behind explicit, safely quoted launch paths."""

from pathlib import Path
import shlex
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import swarm_run_wsl as wsl  # noqa: E402
from swarm_run_cli import build_parser  # noqa: E402


def command(**kwargs):
    return wsl.launch_command(1, 120, "16590", 1, 0, **kwargs)


def test_default_launch_is_unchanged():
    assert "ARDUPILOT_DIR=" not in command()
    assert "BIN=" not in command()


def test_shell_metacharacters_stay_in_single_arguments():
    root = "/home/gart/fork ' $(false); echo bad"
    binary = root + "/probe"
    words = shlex.split(command(firmware_root=root, binary_path=binary))
    assert f"ARDUPILOT_DIR={root}" in words
    assert f"BIN={binary}" in words
    assert "echo" not in words


@pytest.mark.parametrize("options", [
    {"binary_path": "/tmp/a"}, {"firmware_root": "relative"},
    {"firmware_root": ""}, {"firmware_root": "/tmp/x\x00y"},
])
def test_invalid_paths_are_rejected(options):
    with pytest.raises(ValueError):
        command(**options)


def test_parser_and_default_binary():
    args = build_parser(10).parse_args(["--eval", "--sitl-root", "/tmp/probe"])
    assert args.sitl_root == "/tmp/probe"
    assert "BIN=/tmp/probe/build/sitl/bin/arduplane" in command(firmware_root=args.sitl_root)


def test_preflight_does_not_invoke_wsl_for_default():
    with patch.object(wsl.subprocess, "run") as run:
        wsl.ensure_experimental_paths(None, None)
    run.assert_not_called()


def test_invalid_preflight_exits_before_wsl():
    with patch.object(wsl.subprocess, "run") as run:
        with pytest.raises(SystemExit, match="requires an isolated firmware root"):
            wsl.ensure_experimental_paths(None, "/tmp/binary")
    run.assert_not_called()


def test_loopback_only_replaces_lan_endpoints():
    assert "PI_TARGETS_CSV" not in command()
    local = command(loopback_only=True)
    assert f"PI_TARGETS_CSV={wsl.LOOPBACK_SINK}" in local
    assert f"PI_ROUTER_EXTRA_CSV={wsl.LOOPBACK_SINK}" in local
    assert "192.168" not in local
    assert build_parser(10).parse_args(["--loopback-only"]).loopback_only


def test_instance_seed_copies_only_missing_instance_directories():
    root, seed = "/home/eval/roots/run-1", "/home/eval/roots/run-1/seed"
    text = wsl.launch_command(1, 120, "16590", 3, 0, firmware_root=root,
                              instance_seed=seed)
    prefix, _, launch = text.partition("; done; ")
    # The run's slot decides the instance numbers: SWARM_OFFSET + 1..n.
    assert prefix == (
        f"for d in {root}/121 {root}/122 {root}/123; do "
        f'[ -e "$d" ] || cp -a {seed} "$d" || exit 1'
    )
    # A seeded instance must never be overwritten by a template eeprom.
    assert "CLONE_FROM_TEMPLATE=0" in launch.split()
    assert "run_swarm.sh pi" in launch


def test_instance_seed_requires_an_isolated_root_and_absolute_path():
    with pytest.raises(ValueError, match="requires an isolated firmware root"):
        command(instance_seed="/tmp/seed")
    with pytest.raises(ValueError, match="absolute"):
        command(firmware_root="/tmp/root", instance_seed="seed")


def test_default_launch_clones_templates_as_before():
    assert "CLONE_FROM_TEMPLATE" not in command()
    assert not command().startswith("for ")


def test_instance_seed_reaches_the_launch_command():
    from types import SimpleNamespace
    from unittest.mock import MagicMock
    import swarm_run_runner as runner

    args = build_parser(10).parse_args([
        "--single-boot", "--sitl-root", "/r", "--sitl-instance-seed", "/r/seed",
    ])
    built = MagicMock(return_value="cmd")
    launch = runner.LaunchServices(
        start=lambda command: (SimpleNamespace(), []),
        verify=lambda *a: SimpleNamespace(),
        passed=lambda *a: True,
        describe=lambda *a: "",
        speed_fault=lambda *a: False,
        force_speed=lambda *a: None,
        kill=lambda *a: None,
    )
    runtime = SimpleNamespace(launch_command=built, emit=lambda *a, **k: None,
                              record_status=lambda *a, **k: None)
    with patch.object(runner, "_report_success"):
        runner._launch_with_retries(1, args, offset=3, win_ports="", expected=set(),
                                    launch=launch, runtime=runtime, max_attempts=1)
    assert built.call_args.kwargs["instance_seed"] == "/r/seed"
    assert built.call_args.kwargs["firmware_root"] == "/r"


def test_preflight_checks_the_instance_seed_exists():
    with patch.object(wsl.subprocess, "run") as run:
        run.return_value.returncode = 0
        wsl.ensure_experimental_paths("/r", "/r/bin/arduplane", "/r/seed")
    assert "test -d /r/seed" in run.call_args.args[0][-1]
