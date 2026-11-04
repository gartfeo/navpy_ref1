"""The app side of a lockstep run with the GCS app (``--app``): relay and frontend.

``deterministic_dev_sim.md`` section 5, "Launcher": the run's eval-band
lockstep slot also serves the app, on that slot's own ports (owner ruling 4
changed for app runs only): the relay (``lockstep_app_relay``) on the
backend port and the Vite dev server on the frontend port, proxying to the
relay. Both listen on 127.0.0.1 only, since a page that reaches them drives
the run. The frontend is a recorded child of the slot, so ``gcs_stop.py
--eval`` stops it with the rest; a normal end stops it here. Host side
only: nothing here reaches the run journal.
"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry_lockstep as lockstep
from scripts.eval_gcs_demo_constants import ROOT
from scripts.lockstep_app_relay import AppRelay

FRONTEND_DIR = ROOT / "src" / "gcs" / "frontend"
FRONTEND_LOG = "lockstep_frontend_host.log"
RELAY_HOST = "127.0.0.1"  # the page and the relay stay on this PC (todo 254)


@dataclass
class AppStack:
    relay: AppRelay
    frontend: subprocess.Popen | None
    slot: int
    identity: dict

    @property
    def url(self) -> str:
        return f"http://{RELAY_HOST}:{ip.frontend_port(self.slot)}"

    def stop(self) -> None:
        try:
            self.relay.stop()
        finally:
            self._stop_frontend()

    def _stop_frontend(self) -> None:
        if self.frontend is not None:
            stop_tree(self.frontend.pid)
            self.frontend.wait(30)
            lockstep.forget_child(self.slot, self.identity, self.frontend.pid)
            self.frontend = None


def stop_tree(pid: int) -> None:
    """Stop the frontend's whole tree (shell, npm, Vite), as ``gcs_stop.py`` does."""
    subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True)


def start_app_stack(run_dir: Path, slot: int, identity: dict, target: int, *,
                    frontend: bool = True) -> AppStack:
    """The relay on the slot's backend port, then (``frontend``) Vite on its frontend port.

    Windows only, as the lockstep fleet is (it drives WSL): stopping the
    frontend, here and in ``gcs_stop.py``, kills its whole tree with
    ``taskkill /T``, and a POSIX signal would reach only its shell."""
    if os.name != "nt":
        raise RuntimeError("--app needs a Windows host (taskkill /T stops the frontend's tree)")
    relay = AppRelay(target)
    relay.start(ip.backend_port(slot), RELAY_HOST)
    stack = AppStack(relay, None, slot, identity)
    if not frontend:
        return stack
    env = {**os.environ, "PORT": str(ip.frontend_port(slot)),
           "VITE_BACKEND_PORT": str(ip.backend_port(slot)), "VITE_BACKEND_HOST": RELAY_HOST,
           "VITE_GIT_BRANCH": "lockstep", "VITE_NAVPY_LOCKSTEP": "1"}
    try:
        with (run_dir / FRONTEND_LOG).open("ab") as out:
            process = subprocess.Popen(f"npm run dev -- --host {RELAY_HOST}", cwd=str(FRONTEND_DIR), env=env, shell=True,
                                       stdin=subprocess.DEVNULL, stdout=out,
                                       stderr=subprocess.STDOUT,
                                       creationflags=subprocess.CREATE_NO_WINDOW)
        stack.frontend = process
        if not lockstep.record_child(slot, identity, lockstep.local_child(process.pid, role="frontend")):
            raise RuntimeError("the lockstep slot is stopping: frontend not recorded")
    except BaseException:
        stack.stop()
        raise
    return stack


__all__ = ["AppStack", "FRONTEND_LOG", "start_app_stack", "stop_tree"]
