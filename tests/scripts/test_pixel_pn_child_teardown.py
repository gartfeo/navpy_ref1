"""The child's teardown: every step attempted, the evidence last, then raise.

Review round 1 drove the child's real ``run()`` and found two defects in its
``finally`` (F1, F2). The first cleanup to raise stranded every step after it:
source, command worker, vehicle and logger all stayed open, and no determinism
artifact was written. And the summary was taken before the command worker
stopped, so the worker's last pass could record after it.

The helper is tested with fakes around a REAL trace, and then through the
child's real ``run()``: a correct helper that the child never calls, or calls
with the wrong arguments, is exactly the defect Landing 1 shipped.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from navpy.exception_groups import (
    BaseExceptionGroup,
    ExceptionGroup,
    _FallbackBaseExceptionGroup,
    _FallbackExceptionGroup,
)
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.sim import determinism_journal, determinism_trace
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_FENCED,
    OUTPUT_EMPTY,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_evidence import (
    KIND_ROW,
    MANIFEST_NAME,
)
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from scripts import direct_pixel_pn_child as child
from scripts.pixel_pn_admission_ledger import NO_ADMISSION_TAP
from scripts.pixel_pn_child_teardown import (
    MAVLINK_QUIESCENCE_UNVERIFIED,
    REPORT_INTERRUPTED,
    TEARDOWN_FAILED,
    WORKER_NEVER_STARTED,
    WORKER_NOT_CONFIRMED,
    WORKER_STOPPED,
    finish_child,
)
from scripts.pixel_pn_determinism_evidence import INTERRUPTED_DESCRIBING
from scripts.pixel_pn_determinism_summary import (
    REPR_FAILED,
    SUMMARY_NAME,
    UNKNOWN_EPOCH_ERROR,
)

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.
STEPS = [
    "flight_control_trace.write",
    "source.close",
    "navigation.stop",
    "cadence.close",
    "vehicle.close",
    "admission.cancel",
    "admission.evidence",
    "logger.close",
    "identity.finish",
]
# Through the child's real run() the evidence is read off the real
# subscription link, which records no step; the fake tap records its cancel.
RUN_STEPS = [step for step in STEPS if step != "admission.evidence"]
# What the harness's subscription reports: owed first..end, nothing lost,
# and subscribed before the source's window opened (D8.5).
EVIDENCE = {
    "live": True, "first": 3, "end": 9, "faults": 0, "faulted": False,
    "error": None, "window_rows": 0,
}
# And what the child's real run() reads off the fake tap's subscription,
# which is owed no ruling: none is made. It subscribed before start(),
# so the journal held no window row yet.
RUN_EVIDENCE = {
    "live": True, "first": 1, "end": 0, "faults": 0, "faulted": False,
    "error": None, "window_rows": 0,
}
# What the case's identity hands the evidence at teardown (D7b): the
# harness's fake, and the fake the child's real run() is handed.
IDENTITY = {
    "harness": {"sha256": "a" * 64, "files": 12},
    "case": {"name": "speed-1-run-1", "definition_sha256": "d" * 64},
    "given_error": None,
    "endpoints": {
        "start": {"sha256": "e" * 64, "files": 3},
        "teardown": {"sha256": "e" * 64, "files": 3},
    },
    "run": {"pid": 1},
}
# How the child's real run() starts its identity: with its own file.
STARTED_IDENTITY = ("start_identity", "direct_pixel_pn_child.py")


def _artifact(directory: Path) -> dict:
    return json.loads((directory / SUMMARY_NAME).read_text(encoding="utf-8"))


def _manifest(directory: Path) -> dict:
    return json.loads((directory / MANIFEST_NAME).read_text(encoding="utf-8"))


def _reported(*steps: tuple[str, BaseException]) -> list[dict[str, str]]:
    return [{"step": step, "error": repr(error)} for step, error in steps]


class _Unrepresentable:
    """An exception argument whose repr raises, so repr() of the exception
    that carries it raises too."""

    def __repr__(self) -> str:
        raise ValueError("this argument cannot be represented")


class _InterruptedRepr:
    """A value whose repr is interrupted once: a Ctrl+C lands once, while
    its text is made. Later reprs work, so a failing assertion can still
    show it, and a second description of it would not read REPR_FAILED."""

    def __init__(self, interrupt: BaseException) -> None:
        self._interrupt: BaseException | None = interrupt

    def __repr__(self) -> str:
        interrupt, self._interrupt = self._interrupt, None
        if interrupt is not None:
            raise interrupt
        return "_InterruptedRepr()"


class _Harness:
    """A fake for every resource the teardown touches, around a REAL trace.

    ``raises`` makes a step raise. ``during`` runs something inside a step,
    which places a late recording exactly where it can land. The source starts
    in the scored leg, epoch 1, and ``close()`` behaves as the real one does:
    it returns the ending epoch and moves the epoch on.
    """

    def __init__(
        self,
        *,
        raises: dict[str, BaseException] | None = None,
        during: dict | None = None,
        tracing: bool = True,
    ) -> None:
        self.trace = DeterminismTrace(PERIOD_US) if tracing else None
        self.epoch = 1
        self.order: list[str] = []
        self._raises = raises or {}
        self._during = during or {}

    def _step(self, name: str, result=None):
        def run():
            self.order.append(name)
            action = self._during.get(name)
            if action is not None:
                action(self)
            if name in self._raises:
                raise self._raises[name]
            return None if result is None else result()

        return run

    def _close_source(self) -> int:
        ending = self.epoch
        self.epoch += 1
        return ending

    def finish(self, directory: Path, **overrides: object) -> None:
        resources = {
            "flight_trace": SimpleNamespace(
                write=self._step("flight_control_trace.write")
            ),
            "source": SimpleNamespace(
                determinism_trace=self.trace,
                close=self._step("source.close", self._close_source),
            ),
            "navigation": SimpleNamespace(stop=self._step("navigation.stop")),
            "cadence": SimpleNamespace(close=self._step("cadence.close")),
            "vehicle": SimpleNamespace(close=self._step("vehicle.close")),
            "admission": SimpleNamespace(
                cancel=self._step("admission.cancel"),
                evidence=self._step(
                    "admission.evidence", lambda: dict(EVIDENCE)
                ),
            ),
            "logger": SimpleNamespace(close=self._step("logger.close")),
            "identity": SimpleNamespace(
                finish=self._step("identity.finish", lambda: dict(IDENTITY))
            ),
        }
        finish_child(directory, **{**resources, **overrides})


def test_every_step_runs_in_order_and_the_summary_comes_after_all_of_them(
    tmp_path,
):
    def late_callback(harness: _Harness) -> None:
        # A MAVLink callback still in flight: vehicle.close() does not prove
        # the reader stopped, and a pair captured before close() is fenced at
        # the ENDING epoch, so the row belongs to the leg that ended.
        harness.trace.record_association(
            epoch=1, outcome=ASSOCIATION_FENCED, source_s=1.0
        )

    harness = _Harness(during={"logger.close": late_callback})

    harness.finish(tmp_path)

    assert harness.order == STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"]["counts"]["association"] == {
        ASSOCIATION_FENCED: 1
    }
    assert artifact["finalization"] == {
        "worker": WORKER_STOPPED,
        "mavlink_quiescence": MAVLINK_QUIESCENCE_UNVERIFIED,
        "teardown_errors": [],
        "admission": EVIDENCE,
        "epoch": 1,
    }
    # Written last, from the same capture: the late row is in all three.
    manifest = _manifest(tmp_path)
    assert manifest["finalization"] == artifact["finalization"]
    assert manifest["identity"] == IDENTITY
    assert manifest["trace"]["kinds"][KIND_ROW] == 1
    assert artifact["summary"]["rows_recorded"] == 1


def test_the_epoch_is_the_one_close_returned_not_the_newest_row(tmp_path):
    """F1: the worker's last pass starts after close(), at the NEXT epoch."""

    def last_pass(harness: _Harness) -> None:
        harness.trace.command_log.note_iteration(1)
        harness.trace.record_output(
            epoch=harness.epoch, outcome=OUTPUT_EMPTY, taken_at_us=None
        )

    harness = _Harness(during={"navigation.stop": last_pass})
    harness.trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)

    harness.finish(tmp_path)

    artifact = _artifact(tmp_path)
    assert artifact["finalization"]["epoch"] == 1
    summary = artifact["summary"]
    assert summary["epoch"] == 1
    assert summary["counts"]["truth"] == {TRUTH_RECORDED: 1}
    assert summary["counts"]["output"] == {}
    # Taken after the worker stopped, so its last pass is in the ledger.
    assert summary["rows_recorded"] == 2
    assert summary["commands"]["iterations"] == 1


def test_a_close_that_raises_leaves_an_error_artifact_and_every_step_runs(
    tmp_path,
):
    error = RuntimeError("close failed")
    harness = _Harness(raises={"source.close": error})

    with pytest.raises(RuntimeError) as raised:
        harness.finish(tmp_path)

    assert raised.value is error
    assert harness.order == STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"] is None
    assert artifact["error"] == UNKNOWN_EPOCH_ERROR
    assert artifact["finalization"]["epoch"] is None
    assert artifact["finalization"]["teardown_errors"] == _reported(
        ("source.close", error)
    )


def test_several_failures_are_raised_together_and_the_worker_is_unconfirmed(
    tmp_path,
):
    stop_error = TimeoutError("command worker still alive")
    vehicle_error = RuntimeError("link would not close")
    harness = _Harness(
        raises={"navigation.stop": stop_error, "vehicle.close": vehicle_error}
    )

    with pytest.raises(ExceptionGroup) as raised:
        harness.finish(tmp_path)

    assert raised.value.message == TEARDOWN_FAILED
    assert list(raised.value.exceptions) == [stop_error, vehicle_error]
    assert harness.order == STEPS
    finalization = _artifact(tmp_path)["finalization"]
    assert finalization["worker"] == WORKER_NOT_CONFIRMED
    assert finalization["teardown_errors"] == _reported(
        ("navigation.stop", stop_error), ("vehicle.close", vehicle_error)
    )


def test_a_step_error_whose_repr_raises_still_leaves_the_artifact(tmp_path):
    """The report of what failed is made inside the summary's own step, so
    a repr that raised there took the whole artifact with it, finalization
    block and all. The step's error is reported in the fixed text instead,
    and raised as itself like any other."""
    error = RuntimeError(_Unrepresentable())
    harness = _Harness(raises={"vehicle.close": error})

    with pytest.raises(RuntimeError) as raised:
        harness.finish(tmp_path)

    assert raised.value is error
    assert harness.order == STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"]["epoch"] == 1
    assert artifact["finalization"]["teardown_errors"] == [
        {"step": "vehicle.close", "error": REPR_FAILED}
    ]


def test_a_step_error_whose_repr_is_interrupted_still_leaves_the_artifact(
    tmp_path,
):
    """Delivery step 5, review round 2. An interrupt landing while a step's
    error was described escaped the report, and with it the summary's
    step, before the summary was attempted. The entry reads the fixed
    text, the artifact is written, and the interrupt is raised after it
    with the rest.

    Caught as BaseException rather than with ``pytest.raises``, for the
    same reason as the grouped-interrupt test below.
    """
    describing = KeyboardInterrupt("ctrl+c while a step error is described")
    error = RuntimeError(_InterruptedRepr(describing))
    harness = _Harness(raises={"vehicle.close": error})

    try:
        harness.finish(tmp_path)
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert harness.order == STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"]["epoch"] == 1
    # And the evidence step's own failure after it, in both files, so D8.12
    # sees that step fail (delivery step 6's review).
    reported = [
        {"step": "vehicle.close", "error": REPR_FAILED},
        {"step": "determinism_evidence", "error": REPORT_INTERRUPTED},
    ]
    assert artifact["finalization"]["teardown_errors"] == reported
    assert _manifest(tmp_path)["finalization"]["teardown_errors"] == reported
    assert isinstance(raised, BaseExceptionGroup), repr(raised)
    assert not isinstance(raised, ExceptionGroup)
    assert raised.message == TEARDOWN_FAILED
    assert list(raised.exceptions) == [error, describing]


def test_the_files_tell_an_interrupted_description_from_a_failed_one(
    tmp_path,
):
    """Delivery step 6's review, the pair Codex compared: a step's error
    whose repr raises, and one whose repr is interrupted. The second is
    also the evidence step's own failure, raised with the rest, yet both
    runs reported the same one entry, so D8.12 could not see that step
    fail. Caught as BaseException: an interrupt escaping a test ends the
    whole session."""
    errors = {
        "raises": RuntimeError(_Unrepresentable()),
        "interrupted": RuntimeError(
            _InterruptedRepr(KeyboardInterrupt("ctrl+c while described"))
        ),
    }
    raised: dict[str, BaseException] = {}
    reported = {}
    for name, error in errors.items():
        directory = tmp_path / name
        directory.mkdir()
        try:
            _Harness(raises={"vehicle.close": error}).finish(directory)
        except BaseException as escaped:  # noqa: BLE001 - asserted on below
            raised[name] = escaped
        reported[name] = _manifest(directory)["finalization"][
            "teardown_errors"
        ]

    assert raised["raises"] is errors["raises"]
    assert isinstance(raised["interrupted"], BaseExceptionGroup)
    assert reported["raises"] == [
        {"step": "vehicle.close", "error": REPR_FAILED}
    ]
    assert reported["interrupted"] == [
        *reported["raises"],
        {"step": "determinism_evidence", "error": REPORT_INTERRUPTED},
    ]


def test_an_interrupt_is_grouped_as_a_base_exception_and_the_artifact_lands(
    tmp_path,
):
    """A Ctrl+C in one step strands nothing after it either.

    Caught as BaseException rather than with ``pytest.raises``: if a bare
    interrupt escaped, that would end the whole pytest session instead of
    failing this one test.
    """
    interrupt = KeyboardInterrupt()
    error = RuntimeError("logger would not close")
    harness = _Harness(raises={"cadence.close": interrupt, "logger.close": error})

    try:
        harness.finish(tmp_path)
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the teardown raised nothing")

    assert isinstance(raised, BaseExceptionGroup), repr(raised)
    assert not isinstance(raised, ExceptionGroup)
    assert list(raised.exceptions) == [interrupt, error]
    assert harness.order == STEPS
    assert (tmp_path / SUMMARY_NAME).exists()


def test_the_run_error_stays_attached_to_a_single_teardown_error(tmp_path):
    """The teardown runs in the child's ``finally``, so Python chains the
    run's own exception onto whatever it raises. Nothing is lost."""
    run_error = RuntimeError("the leg failed")
    write_error = OSError("csv locked")
    harness = _Harness(raises={"flight_control_trace.write": write_error})

    with pytest.raises(OSError) as raised:
        try:
            raise run_error
        finally:
            harness.finish(tmp_path)

    assert raised.value is write_error
    assert raised.value.__context__ is run_error


def test_a_navigation_never_built_is_reported_as_never_started(tmp_path):
    harness = _Harness()

    harness.finish(tmp_path, navigation=None)

    assert "navigation.stop" not in harness.order
    assert _artifact(tmp_path)["finalization"]["worker"] == WORKER_NEVER_STARTED


def test_nothing_built_past_the_logger_still_writes_and_closes_it(tmp_path):
    harness = _Harness()

    harness.finish(
        tmp_path,
        source=None,
        navigation=None,
        cadence=None,
        vehicle=None,
        admission=None,
    )

    assert harness.order == ["flight_control_trace.write", "logger.close"]
    assert list(tmp_path.iterdir()) == []


def test_tracing_off_writes_no_artifact_and_every_step_runs(tmp_path):
    """With tracing off there is no ledger and no identity, so the child
    subscribes nothing, starts no identity, and hands the teardown
    neither. The evidence step finds no trace and writes nothing."""
    harness = _Harness(tracing=False)

    harness.finish(tmp_path, admission=None, identity=None)

    assert harness.order == [
        step
        for step in STEPS
        if not step.startswith(("admission.", "identity."))
    ]
    assert list(tmp_path.iterdir()) == []


def test_a_cancel_that_raises_is_named_and_the_evidence_is_still_read(
    tmp_path,
):
    error = RuntimeError("the tap would not cancel")
    harness = _Harness(raises={"admission.cancel": error})

    with pytest.raises(RuntimeError) as raised:
        harness.finish(tmp_path)

    assert raised.value is error
    assert harness.order == STEPS
    finalization = _artifact(tmp_path)["finalization"]
    assert finalization["admission"] == EVIDENCE
    assert finalization["teardown_errors"] == _reported(
        ("admission.cancel", error)
    )


def test_an_evidence_read_that_raises_costs_only_the_evidence(tmp_path):
    error = RuntimeError("the evidence could not be read")
    harness = _Harness(raises={"admission.evidence": error})

    with pytest.raises(RuntimeError) as raised:
        harness.finish(tmp_path)

    assert raised.value is error
    assert harness.order == STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"] is not None
    assert artifact["finalization"]["admission"] is None
    assert artifact["finalization"]["teardown_errors"] == _reported(
        ("admission.evidence", error)
    )


def test_an_interrupt_in_the_evidence_read_still_lands_the_artifact(tmp_path):
    """Its own step, so a Ctrl+C there costs the evidence and nothing after
    it: the logger still closes and the artifact is still written.

    Caught as BaseException rather than with ``pytest.raises``, for the same
    reason as the grouped-interrupt test above.
    """
    interrupt = KeyboardInterrupt("ctrl+c reading the evidence")
    harness = _Harness(raises={"admission.evidence": interrupt})

    try:
        harness.finish(tmp_path)
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert raised is interrupt
    assert harness.order == STEPS
    finalization = _artifact(tmp_path)["finalization"]
    assert finalization["admission"] is None
    assert finalization["teardown_errors"] == _reported(
        ("admission.evidence", interrupt)
    )


def test_an_identity_that_cannot_finish_costs_only_the_identity(tmp_path):
    """D7b: its own step, after the logger's, so a failure there costs the
    identity and nothing else: the evidence is written, carrying none."""
    error = RuntimeError("the case file went away")
    harness = _Harness(raises={"identity.finish": error})

    with pytest.raises(RuntimeError) as raised:
        harness.finish(tmp_path)

    assert raised.value is error
    assert harness.order == STEPS
    manifest = _manifest(tmp_path)
    assert manifest["identity"] is None
    assert manifest["finalization"]["teardown_errors"] == _reported(
        ("identity.finish", error)
    )
    assert _artifact(tmp_path)["summary"]["epoch"] == 1


def test_an_interrupt_in_the_identity_still_lands_the_evidence(tmp_path):
    """A Ctrl+C while the tree is hashed at teardown costs the identity, and
    the evidence is still written before it is raised.

    Caught as BaseException rather than with ``pytest.raises``, for the same
    reason as the grouped-interrupt test above.
    """
    interrupt = KeyboardInterrupt("ctrl+c while the tree is hashed")
    harness = _Harness(raises={"identity.finish": interrupt})

    try:
        harness.finish(tmp_path)
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert raised is interrupt
    assert harness.order == STEPS
    manifest = _manifest(tmp_path)
    assert manifest["identity"] is None
    assert manifest["finalization"]["teardown_errors"] == _reported(
        ("identity.finish", interrupt)
    )


# --------------------------------------------------------------------------
# Through the child's real run(): only construction is replaced.
# --------------------------------------------------------------------------


class _FakeSource:
    """The real source's epoch contract around a REAL trace, and nothing else.

    activate() moves to the next epoch and records one truth row there;
    close() returns the ending epoch and then moves on, as the real
    ``DirectPoiPixelSource`` does; a dispatch records at the CURRENT epoch,
    so one that starts after close() lands in the next leg.
    """

    def __init__(self, run: "_ChildRun") -> None:
        self._run = run
        self.determinism_trace = DeterminismTrace(PERIOD_US)
        self.epoch = 0

    def start(self) -> None:
        self._run.wiring.append("source.start")
        if self._run.start_error is not None:
            raise self._run.start_error

    def activate(self) -> None:
        self.epoch += 1
        self.determinism_trace.record_truth(
            epoch=self.epoch, outcome=TRUTH_RECORDED
        )

    def dispatch_available(self) -> bool:
        self.determinism_trace.record_output(
            epoch=self.epoch, outcome=OUTPUT_EMPTY, taken_at_us=None
        )
        return False

    def close(self) -> int:
        self._run.calls.append("source.close")
        ending = self.epoch
        self.epoch += 1
        return ending


class _FakeNavigation:
    """Keeps what the child binds, and on stop() runs one last pass with it:
    the command worker's final iteration, started after close()."""

    def __init__(self, run: "_ChildRun") -> None:
        self._run = run
        self._dispatch = None
        self._observer = None
        self._started = False

    def bind_final_approach_source_dispatch(self, dispatch, observer) -> None:
        self._dispatch = dispatch
        self._observer = observer

    def start(self) -> None:
        self._started = True

    def init(self) -> None:
        pass

    def raise_if_failed(self) -> None:
        if self._run.loop_error is not None:
            raise self._run.loop_error

    def stop(self) -> None:
        self._run.calls.append("navigation.stop")
        if self._started:
            self._observer.note_iteration(1)
            self._dispatch()


class _FakeSubscription:
    """What the router's admission tap hands back: owed the rulings from
    ``first``, with ``end`` fixed by ``cancel``. No ruling is made here, so
    it is owed none: first 1, end 0."""

    def __init__(self, run: "_ChildRun") -> None:
        self._run = run
        self.first = 1
        self.end = None
        self.faults = 0
        self.faulted = False

    def cancel(self) -> None:
        self._run.calls.append("admission.cancel")
        self.end = 0


class _ChildRun:
    """The child's real ``run()``, with every resource it builds replaced.

    The order it builds, starts, activates and tears down in is the child's
    own, which is the point of driving it rather than the helper.
    """

    def __init__(
        self,
        monkeypatch: pytest.MonkeyPatch,
        directory: Path,
        *,
        start_error: BaseException | None = None,
        loop_error: BaseException | None = None,
        write_error: BaseException | None = None,
        admission: str = "live",
        admission_error: BaseException | None = None,
    ) -> None:
        self.directory = directory
        self._options_parser = child._parser()
        self.calls: list[str] = []
        # What the child wired before the leg, in order: its identity, the
        # ledger's subscription and the source's start.
        self.wiring: list = []
        self._monkeypatch = monkeypatch
        self.start_error = start_error
        self.loop_error = loop_error
        self.write_error = write_error
        self._admission = admission
        self._admission_error = admission_error
        self.subscription = _FakeSubscription(self)
        self.source = _FakeSource(self)
        self.navigation = _FakeNavigation(self)
        vehicle = SimpleNamespace(
            sim_speedup=1.0,
            is_armed=True,
            mission_items_next=3,
            get_mode=FlightMode.GUIDED,
            set_mode=lambda mode: True,
            close=lambda: self.calls.append("vehicle.close"),
            on_admission=self._on_admission,
        )
        built = {
            "initialize_logger": lambda *_: SimpleNamespace(
                close=lambda: self.calls.append("logger.close")
            ),
            "FlightControlTrace": lambda _path: SimpleNamespace(
                write=self._write, sample=lambda _vehicle: None
            ),
            "create_vehicle": lambda *_: vehicle,
            "SchedulerCadence": lambda *_: SimpleNamespace(
                close=lambda: self.calls.append("cadence.close")
            ),
            "NavigationArgs": lambda *_: SimpleNamespace(),
            "GeoRefCalc": lambda *_: SimpleNamespace(
                uas_seq="ZYX", degrees=True
            ),
            "MissionPlanner": lambda *_: SimpleNamespace(),
            "Navigation": lambda *_, **__: self.navigation,
            "DirectPoiPixelSource": lambda *_, **__: self.source,
            "start_identity": self._start_identity,
        }
        for name, factory in built.items():
            monkeypatch.setattr(child, name, factory)

    def _write(self) -> None:
        self.calls.append("flight_control_trace.write")
        if self.write_error is not None:
            raise self.write_error

    def _start_identity(
        self, options: argparse.Namespace, entry: Path
    ) -> object:
        """The child's identity (D7b), started as the real one is: noted
        with the file the child names, and finished as a teardown step."""
        self.wiring.append(("start_identity", Path(entry).name))
        return SimpleNamespace(finish=self._finish_identity)

    def _finish_identity(self) -> dict:
        self.calls.append("identity.finish")
        return dict(IDENTITY)

    def _on_admission(self, message_name: str, callback: object) -> object:
        """The router's tap as the vehicle exposes it: "live" hands back a
        subscription, "none" is a vehicle without one, "raises" fails."""
        self.wiring.append(("on_admission", message_name, callback))
        if self._admission == "raises":
            raise self._admission_error
        if self._admission == "none":
            return None
        return self.subscription

    def options(self) -> argparse.Namespace:
        # Exercise the real CLI-to-runtime contract, including renamed fields.
        return self._options_parser.parse_args([
            "--connection", "udp:127.0.0.1:14999",
            "--sysid", "1",
            "--poi-lat", "40.0",
            "--poi-lon", "44.0",
            "--poi-alt", "900.0",
            "--scoring-start-seq", "3",
            "--timeout", "5.0",
            "--result", str(self.directory / "result.json"),
            "--scoring-active", str(self.directory / "scoring_active.txt"),
        ])

    def run(self) -> None:
        child.run(self.options())

    def main(self) -> int:
        """The child's real entry point: ``run()``, then the result it
        publishes. Only the command line is replaced."""
        options = self.options()
        self._monkeypatch.setattr(
            child, "_parser", lambda: SimpleNamespace(parse_args=lambda: options)
        )
        return child.main()


def test_a_failing_csv_write_strands_nothing_in_the_real_run(
    monkeypatch, tmp_path
):
    """F2, through the child's real ``run()``.

    The source fails to start, and then the flight-control CSV cannot be
    written. Before the fix that write was the only cleanup that ran, and no
    determinism artifact was written.
    """
    start_error = RuntimeError("stream would not start")
    write_error = PermissionError("csv locked")
    run = _ChildRun(
        monkeypatch, tmp_path, start_error=start_error, write_error=write_error
    )

    with pytest.raises(PermissionError) as raised:
        run.run()

    assert raised.value is write_error
    assert raised.value.__context__ is start_error
    assert run.calls == RUN_STEPS
    artifact = _artifact(tmp_path)
    assert artifact["finalization"] == {
        "worker": WORKER_STOPPED,
        "mavlink_quiescence": MAVLINK_QUIESCENCE_UNVERIFIED,
        "teardown_errors": _reported(
            ("flight_control_trace.write", write_error)
        ),
        "admission": RUN_EVIDENCE,
        "epoch": 0,
    }
    assert artifact["summary"]["epoch"] == 0


def test_the_real_run_summarises_the_ended_leg_after_the_worker_stopped(
    monkeypatch, tmp_path
):
    """F1, through the child's real ``run()``.

    The command worker's last pass starts after close(): one iteration and one
    dispatch, recorded at the NEXT epoch. The summary is of the leg close()
    ended and is taken after that pass, so the pass is counted and the ended
    leg's own rows do not include it.
    """
    loop_error = RuntimeError("navigation failed mid-leg")
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)

    with pytest.raises(RuntimeError) as raised:
        run.run()

    assert raised.value is loop_error
    assert run.calls == RUN_STEPS
    artifact = _artifact(tmp_path)
    assert artifact["error"] is None
    assert artifact["finalization"]["epoch"] == 1
    assert artifact["finalization"]["teardown_errors"] == []
    summary = artifact["summary"]
    assert summary["epoch"] == 1
    assert summary["counts"]["truth"] == {TRUTH_RECORDED: 1}
    assert summary["counts"]["output"] == {}
    # Trace-wide: the activation's truth row and the last pass's dispatch.
    assert summary["rows_recorded"] == 2
    assert summary["commands"]["iterations"] == 1


def test_an_interrupted_summary_in_the_real_run_still_leaves_the_artifact(
    monkeypatch, tmp_path
):
    """Review round 3, through the child's real ``run()``.

    Every step before the summary keeps its interrupt and runs on, but the
    summary is the LAST step, and an interrupt inside it left no artifact at
    all. Now the artifact is written with the interrupt as its error, and the
    interrupt is raised on with the run's own error still attached.

    Caught as BaseException rather than with ``pytest.raises``, for the same
    reason as the grouped-interrupt test above.
    """
    loop_error = RuntimeError("navigation failed mid-leg")
    interrupt = KeyboardInterrupt("ctrl+c during the summary")
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)

    def interrupted_capture():
        raise interrupt

    monkeypatch.setattr(
        run.source.determinism_trace, "capture", interrupted_capture
    )

    try:
        run.run()
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert raised is interrupt
    assert raised.__context__ is loop_error
    assert run.calls == RUN_STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"] is None
    assert artifact["error"] == repr(interrupt)
    assert artifact["finalization"] == {
        "worker": WORKER_STOPPED,
        "mavlink_quiescence": MAVLINK_QUIESCENCE_UNVERIFIED,
        "teardown_errors": [],
        "admission": RUN_EVIDENCE,
        "epoch": 1,
    }


INTERRUPTS = {
    "keyboard_interrupt": lambda: KeyboardInterrupt("ctrl+c during the summary"),
    "system_exit": lambda: SystemExit(7),
}


@pytest.mark.parametrize("how", ["second_interrupt", "broken_stdout"])
@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_a_failed_write_after_an_interrupted_summary_loses_neither_in_the_real_run(
    monkeypatch, tmp_path, failing_summary_write, kind, how
):
    """Review round 4, through the child's real ``run()``.

    The write made for the interrupt failed as well, and that failure used to
    escape in the interrupt's place. The teardown raises it inside the child's
    ``finally``, where Python chains the run's error onto it, so the interrupt
    was nowhere in the chain. Codex reproduced it for a second Ctrl+C and for
    a broken stdout, after both a KeyboardInterrupt and a SystemExit.

    Caught as BaseException rather than with ``pytest.raises``, for the same
    reason as the grouped-interrupt test above.
    """
    loop_error = RuntimeError("navigation failed mid-leg")
    interrupt = INTERRUPTS[kind]()
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)

    def interrupted_capture():
        raise interrupt

    monkeypatch.setattr(
        run.source.determinism_trace, "capture", interrupted_capture
    )
    injected = failing_summary_write(how)

    try:
        run.run()
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert isinstance(raised, BaseExceptionGroup), repr(raised)
    assert not isinstance(raised, ExceptionGroup)
    assert list(raised.exceptions) == [interrupt, injected[-1]]
    assert raised.__context__ is loop_error
    assert run.calls == RUN_STEPS
    assert not (tmp_path / SUMMARY_NAME).exists()


@pytest.mark.parametrize("kind", ["exception", "system_exit"])
def test_a_summary_error_whose_repr_is_interrupted_in_the_real_run(
    monkeypatch, tmp_path, kind
):
    """Delivery step 5, review round 2, through the child's real ``run()``,
    as Codex reproduced it. The summary fails with an error whose repr is
    interrupted. No write was attempted, and for an interrupted summary
    the second interrupt escaped in place of the first, whose context the
    child's ``finally`` then replaced with the leg's error, so the first
    was lost. Now the artifact is written and every interrupt is raised,
    the summary's own first.

    Caught as BaseException rather than with ``pytest.raises``, for the
    same reason as the grouped-interrupt test above.
    """
    loop_error = RuntimeError("navigation failed mid-leg")
    describing = KeyboardInterrupt("ctrl+c while the error is described")
    failure = (RuntimeError if kind == "exception" else SystemExit)(
        _InterruptedRepr(describing)
    )
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)

    def failed_capture():
        raise failure

    monkeypatch.setattr(
        run.source.determinism_trace, "capture", failed_capture
    )

    try:
        run.run()
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert run.calls == RUN_STEPS
    artifact = _artifact(tmp_path)
    assert artifact["summary"] is None
    assert artifact["error"] == REPR_FAILED
    assert artifact["finalization"]["epoch"] == 1
    assert raised.__context__ is loop_error
    if kind == "exception":
        assert raised is describing
    else:
        assert isinstance(raised, BaseExceptionGroup), repr(raised)
        assert raised.message == INTERRUPTED_DESCRIBING
        assert list(raised.exceptions) == [failure, describing]


def test_a_step_error_whose_repr_is_interrupted_in_the_real_run(
    monkeypatch, tmp_path
):
    """The report's case, through the child's real ``run()``, as Codex
    reproduced it: the CSV write fails with an error whose repr is
    interrupted. The summary was never attempted; now the artifact is
    written, and both are raised with the leg's error attached.

    Caught as BaseException rather than with ``pytest.raises``, for the
    same reason as the grouped-interrupt test above.
    """
    loop_error = RuntimeError("navigation failed mid-leg")
    describing = KeyboardInterrupt("ctrl+c while a step error is described")
    write_error = RuntimeError(_InterruptedRepr(describing))
    run = _ChildRun(
        monkeypatch, tmp_path, loop_error=loop_error, write_error=write_error
    )

    try:
        run.run()
    except BaseException as escaped:  # noqa: BLE001 - asserted on below
        raised = escaped
    else:
        pytest.fail("the interrupt was swallowed")

    assert run.calls == RUN_STEPS
    assert _artifact(tmp_path)["finalization"]["teardown_errors"] == [
        {"step": "flight_control_trace.write", "error": REPR_FAILED},
        {"step": "determinism_evidence", "error": REPORT_INTERRUPTED},
    ]
    assert isinstance(raised, BaseExceptionGroup), repr(raised)
    assert not isinstance(raised, ExceptionGroup)
    assert raised.message == TEARDOWN_FAILED
    assert list(raised.exceptions) == [write_error, describing]
    assert raised.__context__ is loop_error


def test_an_interrupted_recording_is_failed_in_the_real_runs_summary(
    monkeypatch, tmp_path
):
    """Re-review of Landing 2 delivery step 3, through the child's real
    ``main()``.

    An interrupt that landed inside a recording, after the watermark moved
    and before the row was appended, left the journal changed and unflagged,
    and the summary the teardown wrote called the trace complete. The child
    failed on the interrupt all the same; the artifact must not call the
    trace complete either.
    """
    interrupt = KeyboardInterrupt("ctrl+c inside a recording")
    run = _ChildRun(monkeypatch, tmp_path)

    def cut_short(*_args):
        raise interrupt

    monkeypatch.setattr(determinism_journal, "association_row", cut_short)
    monkeypatch.setattr(
        run.navigation,
        "raise_if_failed",
        lambda: run.source.determinism_trace.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
        ),
    )

    assert run.main() == 1

    summary = _artifact(tmp_path)["summary"]
    assert (summary["failed"], summary["complete"]) == (True, False)
    assert run.source.determinism_trace.watermark_us() == 1_000_000
    result = json.loads(_published(tmp_path))
    assert result["passed"] is False
    assert result["error"] == f"KeyboardInterrupt: {interrupt}"


GROUP_CLASSES = {
    "native": (BaseExceptionGroup, ExceptionGroup),
    "fallback": (_FallbackBaseExceptionGroup, _FallbackExceptionGroup),
}


def _raise_groups_of(monkeypatch: pytest.MonkeyPatch, groups: str) -> None:
    """Make the child's teardown and evidence writer raise ``groups``' classes.

    "fallback" is what navpy.exception_groups selects on Python 3.10, and the
    traceback module renders it as one plain exception with its members
    dropped (review round 6). Patched where the CHILD looks the names up: it
    imports these modules as ``pixel_pn_child_teardown`` and
    ``pixel_pn_determinism_evidence``, not the ``scripts.`` copies imported
    here.
    """
    base, plain = GROUP_CLASSES[groups]
    teardown = sys.modules[child.finish_child.__module__]
    writer = sys.modules[teardown.write_determinism_evidence.__module__]
    monkeypatch.setattr(teardown, "BaseExceptionGroup", base)
    monkeypatch.setattr(teardown, "ExceptionGroup", plain)
    monkeypatch.setattr(writer, "BaseExceptionGroup", base)
    monkeypatch.setattr(writer, "ExceptionGroup", plain)


def _published(directory: Path) -> str:
    return (directory / "result.json").read_text(encoding="utf-8")


def _main_publishing(run: _ChildRun) -> int:
    """The child's ``main()``, failing the test cleanly if anything escapes.

    Named outside the handler, and by ``type.__repr__``, which reads nothing
    through a metaclass: what escapes can carry the very failure a formatter
    cannot render, or be a type whose name cannot be read, and pytest's own
    report of it then crashed the whole session (the malformed SyntaxError
    below did).
    """
    try:
        return run.main()
    except BaseException as error:  # noqa: BLE001 - reported below, by name
        escaped = type.__repr__(type(error))
    pytest.fail(f"main() raised {escaped} instead of publishing", pytrace=False)


@pytest.mark.parametrize("groups", sorted(GROUP_CLASSES))
@pytest.mark.parametrize("nested", [False, True], ids=["alone", "csv_failed_too"])
@pytest.mark.parametrize("how", ["second_interrupt", "broken_stdout"])
@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_the_published_result_keeps_every_failure_once_the_summary_is_lost(
    monkeypatch, tmp_path, failing_summary_write, kind, how, nested, groups
):
    """Review rounds 5 and 6, through the child's real ``main()``.

    Once the summary's own write has failed, the result the child publishes
    is the only record left of why the run failed, and it kept one line: the
    outer group's message and "(2 sub-exceptions)". The interrupt, the failed
    write and the run's own error were all gone. Every failure has to survive
    there, however deep its group or chain, and whichever class the groups
    are: round 6 found every member dropped again with the 3.10 fallback.
    """
    loop_error = RuntimeError("navigation failed mid-leg")
    csv_error = PermissionError("csv locked")
    interrupt = INTERRUPTS[kind]()
    run = _ChildRun(
        monkeypatch,
        tmp_path,
        loop_error=loop_error,
        write_error=csv_error if nested else None,
    )
    _raise_groups_of(monkeypatch, groups)

    def interrupted_capture():
        raise interrupt

    monkeypatch.setattr(
        run.source.determinism_trace, "capture", interrupted_capture
    )
    injected = failing_summary_write(how)

    status = _main_publishing(run)

    assert status == 1
    assert run.calls == RUN_STEPS
    assert not (tmp_path / SUMMARY_NAME).exists()
    published = _published(tmp_path)
    result = json.loads(published)
    assert result["passed"] is False
    # The child really raised this kind of group, so the check below is of it.
    assert result["error"].startswith(GROUP_CLASSES[groups][0].__name__ + ": ")
    failures = [loop_error, interrupt, *injected] + ([csv_error] if nested else [])
    for failure in failures:
        assert f"{type(failure).__name__}: {failure}" in published, failure


def test_a_formatter_that_raises_still_publishes_the_result(monkeypatch, tmp_path):
    """Review round 6: the rendering ran unprotected in ``main()``'s handler,
    so a formatter that raised cost the result it was describing -- no
    result.json and no result line, where the parent published both."""
    run = _ChildRun(
        monkeypatch, tmp_path, loop_error=RuntimeError("navigation failed mid-leg")
    )

    def formatter_down(*_: object, **__: object) -> list[str]:
        raise OSError("formatter down")

    # Restored before the assertions: pytest reports a failure through the
    # same module.
    with monkeypatch.context() as patch:
        for name in ("format_exception", "format_exception_only", "format_tb"):
            patch.setattr(f"traceback.{name}", formatter_down)
        status = _main_publishing(run)

    assert status == 1
    assert run.calls == RUN_STEPS
    result = json.loads(_published(tmp_path))
    assert result["passed"] is False
    assert result["error"] == "RuntimeError: navigation failed mid-leg"
    assert "RuntimeError" in result["traceback"]


def test_one_failure_the_formatter_cannot_render_costs_only_itself(
    monkeypatch, tmp_path, failing_summary_write
):
    """Review round 6, with no formatter replaced: the standard one raises for
    a SyntaxError whose ``text`` is not a string. That failure is the run's
    own; the summary's interrupt and failed write must still be published."""
    malformed = SyntaxError("bad extension syntax")
    malformed.text = 42
    interrupt = KeyboardInterrupt("ctrl+c during the summary")
    run = _ChildRun(monkeypatch, tmp_path, loop_error=malformed)

    def interrupted_capture():
        raise interrupt

    monkeypatch.setattr(
        run.source.determinism_trace, "capture", interrupted_capture
    )
    injected = failing_summary_write("second_interrupt")

    status = _main_publishing(run)

    assert status == 1
    assert run.calls == RUN_STEPS
    published = _published(tmp_path)
    assert json.loads(published)["passed"] is False
    assert "SyntaxError" in published
    for failure in (interrupt, *injected):
        assert f"{type(failure).__name__}: {failure}" in published, failure


def test_a_failure_whose_str_raises_still_publishes_the_result(monkeypatch, tmp_path):
    """The one-line error runs the exception's own ``__str__``, on the same
    path that must always publish."""

    class Unprintable(RuntimeError):
        def __str__(self) -> str:
            raise ValueError("no text")

    run = _ChildRun(monkeypatch, tmp_path, loop_error=Unprintable())

    status = _main_publishing(run)

    assert status == 1
    assert run.calls == RUN_STEPS
    result = json.loads(_published(tmp_path))
    assert result["passed"] is False
    assert result["error"].startswith("Unprintable: ")
    assert "Unprintable" in result["traceback"]


class _NameFails(type):
    """Classes whose ``__name__`` cannot be read (review round 7)."""

    def __getattribute__(cls, name: str) -> object:
        if name == "__name__":
            raise RuntimeError("exception type name unavailable")
        return super().__getattribute__(name)


class Nameless(RuntimeError, metaclass=_NameFails):
    pass


def test_a_failure_whose_type_has_no_readable_name_still_publishes_the_result(
    monkeypatch, tmp_path
):
    """Review round 7: the one-line error's fallback read the type's name
    again, unprotected, so a failure whose type's name could not be read cost
    the result -- no result.json and no result line."""
    run = _ChildRun(
        monkeypatch, tmp_path, loop_error=Nameless("navigation failed mid-leg")
    )
    # The report the child itself imported, as a top-level module.
    report = sys.modules[child.failure_payload.__module__]

    status = _main_publishing(run)

    assert status == 1
    assert run.calls == RUN_STEPS
    result = json.loads(_published(tmp_path))
    assert result["passed"] is False
    assert result["error"] == f"{report.NAME_UNREADABLE}: navigation failed mid-leg"
    assert "Nameless: navigation failed mid-leg" in result["traceback"]


def test_the_real_run_subscribes_the_ledger_before_the_source_starts(
    monkeypatch, tmp_path
):
    """D8.5, through the child's real ``run()``: the ATTITUDE ledger's
    ``append`` is subscribed to the router's rulings BEFORE ``source.start()``,
    so it is owed every ruling the source's own subscriptions can see; and
    the teardown cancels it and reads its evidence into the artifact."""
    loop_error = RuntimeError("navigation failed mid-leg")
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)

    with pytest.raises(RuntimeError) as raised:
        run.run()

    assert raised.value is loop_error
    ledger = run.source.determinism_trace.ledger
    assert run.wiring == [
        STARTED_IDENTITY,
        ("on_admission", "ATTITUDE", ledger.append),
        "source.start",
    ]
    assert run.calls == RUN_STEPS
    assert _artifact(tmp_path)["finalization"]["admission"] == RUN_EVIDENCE


@pytest.mark.parametrize("admission", ["none", "raises", "unrepresentable"])
def test_a_vehicle_that_cannot_subscribe_the_ledger_costs_only_the_evidence(
    monkeypatch, tmp_path, admission
):
    """Record-only: a vehicle without the tap, or whose tap raises, even with
    an error whose repr raises, still starts the source and flies the leg,
    and the artifact says why there is no evidence."""
    loop_error = RuntimeError("navigation failed mid-leg")
    if admission == "unrepresentable":
        tap_error: Exception = RuntimeError(_Unrepresentable())
        expected = REPR_FAILED
    else:
        tap_error = ValueError("no admission tap on this link")
        expected = NO_ADMISSION_TAP if admission == "none" else repr(tap_error)
    run = _ChildRun(
        monkeypatch,
        tmp_path,
        loop_error=loop_error,
        admission="none" if admission == "none" else "raises",
        admission_error=tap_error,
    )

    with pytest.raises(RuntimeError) as raised:
        run.run()

    assert raised.value is loop_error
    ledger = run.source.determinism_trace.ledger
    assert run.wiring == [
        STARTED_IDENTITY,
        ("on_admission", "ATTITUDE", ledger.append),
        "source.start",
    ]
    # No subscription to cancel.
    assert run.calls == [
        step for step in RUN_STEPS if step != "admission.cancel"
    ]
    finalization = _artifact(tmp_path)["finalization"]
    assert finalization["teardown_errors"] == []
    assert finalization["admission"] == {
        "live": False,
        "first": None,
        "end": None,
        "faults": None,
        "faulted": None,
        "error": expected,
        "window_rows": None,
    }


def test_the_real_run_starts_its_identity_first_and_the_evidence_carries_it(
    monkeypatch, tmp_path
):
    """D7b, through the child's real ``run()``: the identity is started
    before anything is built, so an interrupt there has nothing to close,
    and finished as a teardown step after the logger closed, into the
    evidence written last."""
    loop_error = RuntimeError("navigation failed mid-leg")
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)
    monkeypatch.setattr(
        child,
        "initialize_logger",
        lambda *_: run.wiring.append("logger") or SimpleNamespace(
            close=lambda: run.calls.append("logger.close")
        ),
    )

    with pytest.raises(RuntimeError) as raised:
        run.run()

    assert raised.value is loop_error
    assert run.wiring[:2] == [STARTED_IDENTITY, "logger"]
    assert run.calls[-2:] == ["logger.close", "identity.finish"]
    assert _manifest(tmp_path)["identity"] == IDENTITY


def test_a_run_that_cannot_be_described_still_flies_the_real_run(
    monkeypatch, tmp_path
):
    """Delivery step 6's review, through the child's startup boundary. The
    identity is run()'s first statement, so a description that raised
    there stopped the case before anything was built, where only an
    interrupt may pass. The REAL identity is started here, its random
    source failing: the leg flies to its own error, every step is
    attempted, and the evidence says why the run has no description."""
    # The module the child calls into, as the child imports it.
    identity_module = sys.modules[child.start_identity.__module__]
    loop_error = RuntimeError("navigation failed mid-leg")
    run = _ChildRun(monkeypatch, tmp_path, loop_error=loop_error)
    monkeypatch.setattr(
        child, "start_identity", identity_module.start_identity
    )
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    code = {"sha256": "e" * 64, "files": 3}
    monkeypatch.setattr(identity_module, "source_identity", lambda *_: code)
    failure = OSError("random source unavailable")

    def unavailable() -> object:
        raise failure

    monkeypatch.setattr(
        identity_module, "uuid", SimpleNamespace(uuid4=unavailable)
    )

    with pytest.raises(RuntimeError) as raised:
        run.run()

    assert raised.value is loop_error
    # The real identity's finish is a step too, and records no call here.
    assert run.calls == [
        step for step in RUN_STEPS if step != "identity.finish"
    ]
    identity = _manifest(tmp_path)["identity"]
    assert identity["run"] == {"error": repr(failure)}
    assert identity["endpoints"] == {"start": code, "teardown": code}
