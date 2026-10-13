"""Acknowledged NavPy stop: request on stdin, ack on stdout, kill only on a miss."""

from __future__ import annotations

import logging
import subprocess
import sys
import threading

from gcs.backend.navpy_process_instance import NavpyInstance, read_navpy_output
from gcs.backend.navpy_process_stop import (
    STOP_ACKED,
    STOP_ALREADY_EXITED,
    STOP_TERMINATED,
    stop_navpy_instance,
)
from navpy.app.runtime_stop import NAVPY_STOP_REQUEST, NAVPY_STOPPED_MARKER


_LOG = logging.getLogger(__name__)

# A child that honours the protocol: wait for the request, ack, exit 0.
_ACKING_CHILD = (
    "import sys\n"
    "for line in sys.stdin:\n"
    f"    if line.strip() == {NAVPY_STOP_REQUEST!r}:\n"
    f"        print({NAVPY_STOPPED_MARKER!r}, flush=True)\n"
    "        break\n"
)
# A child whose cleanup hangs: it reads the request but never acks.
_HUNG_CHILD = "import sys, time\nsys.stdin.readline()\ntime.sleep(60)\n"


def _launch(code: str) -> NavpyInstance:
    proc = subprocess.Popen(
        [sys.executable, "-c", code],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    inst = NavpyInstance(sys_id=4, connection="udp:0.0.0.0:5760", process=proc)
    reader = threading.Thread(target=read_navpy_output, args=(inst,), daemon=True)
    inst._reader_thread = reader
    reader.start()
    return inst


def test_a_clean_stop_is_acked_without_a_kill():
    inst = _launch(_ACKING_CHILD)

    assert stop_navpy_instance(inst, _LOG, ack_timeout_s=30.0) == STOP_ACKED
    assert inst.process.returncode == 0
    assert inst.stop_acked is True
    assert NAVPY_STOPPED_MARKER in inst.output


def test_a_missed_ack_terminates_the_process():
    inst = _launch(_HUNG_CHILD)

    assert stop_navpy_instance(inst, _LOG, ack_timeout_s=1.0) == STOP_TERMINATED
    assert inst.process.poll() is not None
    assert inst.stop_acked is False


def test_an_exit_without_ack_is_not_a_clean_stop():
    inst = _launch("import sys\nsys.stdin.readline()\n")

    assert stop_navpy_instance(inst, _LOG, ack_timeout_s=30.0) == STOP_TERMINATED


def test_a_process_that_already_exited_is_reported_as_such():
    inst = _launch("pass\n")
    inst.process.wait(timeout=30.0)

    assert stop_navpy_instance(inst, _LOG) == STOP_ALREADY_EXITED


def test_a_stop_that_raises_reports_stop_failed_and_keeps_the_generation(monkeypatch):
    from unittest.mock import MagicMock

    from gcs.backend import navpy_process_manager as manager_mod
    from gcs.backend.navpy_process_stop import STOP_FAILED

    manager = manager_mod.NavpyProcessManager()
    inst = MagicMock()
    manager._instances[4] = inst
    monkeypatch.setattr(
        manager_mod, "stop_navpy_instance",
        lambda *_args: (_ for _ in ()).throw(OSError("access denied")),
    )

    assert manager.stop_all() == {4: STOP_FAILED}
    assert manager._instances[4] is inst


def test_a_retried_stop_resumes_after_the_request_was_already_sent():
    inst = _launch(_HUNG_CHILD)
    inst.process.stdin.write(f"{NAVPY_STOP_REQUEST}\n".encode("ascii"))
    inst.process.stdin.close()

    assert stop_navpy_instance(inst, _LOG, ack_timeout_s=1.0) == STOP_TERMINATED
    assert inst.process.poll() is not None
