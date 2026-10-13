"""Hermetic SITL inputs gcs_launch accepts for evaluators and forwards to swarm_run."""

from __future__ import annotations

import argparse


#: gcs_launch flag -> help; each is forwarded to swarm_run under the same name,
#: except --sitl-defaults, which is swarm_run's --defaults.
_SWARM_PATH_FLAGS = {
    "--sitl-root": "isolated SITL firmware/state root (forwarded to swarm_run)",
    "--sitl-binary": "explicit SITL executable inside --sitl-root",
    "--sitl-defaults": "pre-boot SITL parameter file (swarm_run --defaults)",
    "--sitl-instance-seed": "directory each fresh SITL instance is copied from",
}
_SWARM_SWITCH_FLAGS = {
    "--single-boot": "SITL: no parameter healing or relaunch",
    "--loopback-only": "SITL: no LAN endpoints and no Mission Planner port",
}


def swarm_passthrough(args: argparse.Namespace) -> list[str]:
    forwarded: list[str] = []
    for flag in _SWARM_PATH_FLAGS:
        value = getattr(args, flag[2:].replace("-", "_"))
        if value is not None:
            forwarded += ["--defaults" if flag == "--sitl-defaults" else flag, value]
    for flag in _SWARM_SWITCH_FLAGS:
        if getattr(args, flag[2:].replace("-", "_")):
            forwarded.append(flag)
    return forwarded


def add_sitl_arguments(parser: argparse.ArgumentParser) -> None:
    for flag, help_text in _SWARM_PATH_FLAGS.items():
        parser.add_argument(flag, default=None, metavar="WSL_PATH", help=help_text)
    for flag, help_text in _SWARM_SWITCH_FLAGS.items():
        parser.add_argument(flag, action="store_true", help=help_text)


__all__ = ["add_sitl_arguments", "swarm_passthrough"]
