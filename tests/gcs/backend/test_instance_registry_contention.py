"""Real multi-process races for the two registry locks.

Lock acquisition in `instance_registry_store` catches `PermissionError`
alongside `FileExistsError`, because on Windows a lock file another process has
just unlinked lingers in a delete-pending state and `O_CREAT|O_EXCL` against it
raises EACCES rather than EEXIST. Catching only `FileExistsError` killed the
loser of two simultaneous launches outright.

Measured on this machine, eight processes hammering one lock: ~1600 contended
opens each, of which ~50 (about 3%) raised `PermissionError` errno 13 and the
rest `FileExistsError` errno 17. The window is narrow but entirely ordinary.

Every other lock test drives the lock from a single process against a planted
file, so none of them can produce a delete-pending state at all -- that needs a
second process releasing the lock while the first is opening it. These tests
spawn real interpreters and race them.

Two independent properties:

* every worker exits 0 -- contention QUEUES, it never kills a launcher
* no two critical sections overlap in wall time -- exclusion actually holds

They want opposite things -- survival needs tight cycles to reach the
delete-pending window, exclusion needs a hold wide enough for spans to be
comparable -- so each gets its own race (`_SURVIVAL_RACE`, `_EXCLUSION_RACE`).

Exclusion has been silently blind three times, so it is now defended three
times over: by sharing `_EXCLUSION_RACE` with its control arm, by `_overlaps`
refusing degenerate input, and by `_collect` refusing an incomplete one:

1. Workers first buffered ordered enter/exit pairs and appended them after
   releasing, so the file came out grouped by writer; reading that file ORDER
   as execution order made the assertion hold by construction. Buffering after
   the loop was never the defect, and is still what happens -- the ordering
   evidence has to come from the timestamps.
2. The rewrite recorded real timestamps but ran the registry test at
   `hold_s=0`, where two adjacent `time.time()` calls return the SAME value, so
   every span was zero width and `_overlaps` could never fire. Its control arm
   used a 0.0005 s hold -- a workload the test it validated never ran.
   Measured: an unlocked 6x150 race detects overlap in 0/8 runs at `hold_s=0`
   and 8/8 at `hold_s=0.0005`.
3. Every worker appended to ONE shared file. A shared append is not atomic
   across processes on Windows -- the CRT seeks to end and then writes, as two
   separate operations -- so simultaneous flushes resolve end-of-file to the
   same offset and overwrite each other. The loss was SILENT: the survivors
   stayed perfectly well-formed and the file simply held fewer writers, so
   `_overlaps` was handed a fraction of the workload and reported on it as
   though it were the whole. Measured on the control arm, whose workers finish
   together and so collide hardest: spans lost in 5 of 6 runs, down to 40 of
   120 -- 2 of 6 workers -- with a torn line in the 6th, which is the only way
   this ever announced itself. The locked arms stagger their finishes behind
   the lock and lost nothing in 6 runs each, but that is a tendency, not
   immunity: they write outside their locks too. Each worker now writes its
   own file and `_collect` demands every one of them.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


_SRC = Path(__file__).resolve().parents[3] / "src"

# TWO workloads, because the two properties want opposite things and one
# race cannot serve both.
#
# Survival needs SPEED: the delete-pending window opens between one
# worker's unlink and another's open, so cycles must be tight. Measured --
# at hold_s=0.0005 this test PASSES against the reverted fix, i.e. it stops
# being a regression test at all. Sized up from 6x150 after measuring the
# detection rate; see the test docstring for what that rate actually is.
_SURVIVAL_RACE = {"workers": 8, "cycles": 400, "hold_s": 0.0}

# The clock these spans are compared on: `time.time()`, which here is
# GetSystemTimeAsFileTime -- advertised resolution 0.015625 s, observed tick
# median 0.001000 s (max 0.001596), adjustable, non-monotonic.
#
# It is kept because every number in this file was MEASURED against it, not
# because it is the only cross-process option. An earlier comment here claimed
# `perf_counter` was ruled out as per-process; that is false from Python 3.10,
# where it is system-wide on Windows (measured on 3.11.9: a child's reading
# falls between the parent's before and after readings). Switching clocks would
# invalidate this tolerance and both holds, which is the real reason not to.
#
# So an "overlap" smaller than a tick is not evidence of anything, and a
# span narrower than a tick cannot be placed at all. An earlier version
# held for 0.000500 s -- HALF a tick -- and spuriously failed against
# CORRECT code about 1 run in 10.
_CLOCK_TOLERANCE_S = 0.002

# Exclusion needs WIDTH, comfortably above that tolerance: at hold_s=0 two
# adjacent `time.time()` calls return the SAME value, so every span is zero
# width and `_overlaps` can never fire (measured on an unlocked 6x150 race:
# overlap detected in 0/8 runs at hold_s=0). The hold below is 5x the
# tolerance, so a real overlap is unambiguous and granularity cannot forge
# one. Cycles drop because exclusion needs resolvable sections, not many.
#
# Shared with the control arm by construction rather than by copied
# numbers: a control racing different parameters than the test it
# validates proves nothing about that test.
_EXCLUSION_RACE = {"workers": 6, "cycles": 20, "hold_s": 0.01}

# Written to a file and run by real interpreters: the delete-pending window
# exists only between separate processes.
_WORKER = '''
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.environ["CONTENTION_SRC"])
from gcs.backend import instance_registry_store as store

out = Path(os.environ["CONTENTION_OUT"])
tag = str(os.getpid())
hold_s = float(os.environ["CONTENTION_HOLD_S"])
spans = []


class NoLock:
    """The control arm: same shape, no exclusion."""

    def __enter__(self):
        return True

    def __exit__(self, *_):
        return False


def section():
    mode = os.environ["CONTENTION_LOCK"]
    if mode == "registry":
        return store.locked()
    if mode == "sitl":
        return store.sitl_launch_lock(timeout=60.0, stale_after=120.0)
    return NoLock()


# A shared wall deadline, so the workers collide instead of arriving in turn.
while time.time() < float(os.environ["CONTENTION_START"]):
    time.sleep(0.001)

for _ in range(int(os.environ["CONTENTION_CYCLES"])):
    with section() as acquired:
        if acquired is False:
            sys.exit(3)
        # Compared ACROSS processes, so this is the shared wall clock. Its
        # resolution is coarser than two adjacent calls -- back-to-back calls
        # return equal values here -- which is why the section needs a real
        # hold to produce a span with any width at all. (`perf_counter` would
        # compare across processes too; see `_CLOCK_TOLERANCE_S` for why
        # `time.time()` is nonetheless the one to keep.)
        entered = time.time()
        if hold_s:
            time.sleep(hold_s)
        spans.append((entered, time.time()))

# One write at the end, to this worker's OWN file. Two separate reasons:
#
# AT THE END, because writing inside the section would serialise the workers
# behind file I/O and widen every critical section, hiding the race -- and the
# ordering evidence lives in the timestamps, not in the write order.
#
# PRIVATE, because a shared append is not atomic across processes here: the CRT
# implements append as a seek to end followed by a separate write, under a lock
# that is per-process only, so workers flushing at the same instant resolve
# end-of-file to the same offset and overwrite one another. Measured, six
# workers x 20 lines, 20 rounds: appending to one shared file lost whole
# workers' output in 18 rounds and tore a line in 3. Neither O_APPEND nor a
# single os.write prevents that -- both take the same seek-then-write path, and
# both still lost output, in 18 of 20 rounds for one blob write and 20 of 20 for
# per-line writes. Private files lost nothing in 20 rounds. Synchronising the
# writers would work too; having no second writer is simply cheaper.
out.write_text(
    "".join("%.9f %.9f %s\\n" % (entered, left, tag) for entered, left in spans),
    encoding="utf-8",
)
'''


class RegistryLockContention(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self._root = Path(self._tmp.name)
        self._registry = self._root / "registry.json"
        self._worker = self._root / "worker.py"
        self._worker.write_text(_WORKER, encoding="utf-8")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _race(
        self,
        lock: str,
        *,
        workers: int,
        cycles: int = 1,
        hold_s: float = 0.0,
    ) -> list[tuple[float, float, str]]:
        """Run *workers* interpreters at the same lock; return their spans."""
        environment = dict(os.environ)
        environment.update(
            GCS_INSTANCE_REGISTRY=str(self._registry),
            CONTENTION_SRC=str(_SRC),
            CONTENTION_LOCK=lock,
            CONTENTION_CYCLES=str(cycles),
            CONTENTION_HOLD_S=str(hold_s),
            # Enough lead for every interpreter to finish importing, so they
            # all reach `os.open` within the same few milliseconds.
            CONTENTION_START=str(time.time() + 6.0),
        )
        # One output path per worker, so the spans never share a file. Named
        # here rather than globbed back afterwards: a worker that writes
        # nothing has to fail `_collect`, not quietly shrink the sample.
        #
        # A fresh directory per call, so those names cannot be satisfied by a
        # previous race's leftovers. No test races twice today; this is what
        # stops the first one that does from reading stale spans as new ones.
        where = Path(tempfile.mkdtemp(dir=self._root))
        outputs = [where / f"spans.{index}.txt" for index in range(workers)]
        running = [
            subprocess.Popen(
                [sys.executable, str(self._worker)],
                env={**environment, "CONTENTION_OUT": str(output)},
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            for output in outputs
        ]
        for process in running:
            _, error = process.communicate(timeout=300)
            self.assertEqual(
                process.returncode,
                0,
                # The regression: the loser of a race died here, on an
                # uncaught PermissionError, instead of waiting its turn.
                f"a contending worker did not survive: {error[-2000:]}",
            )
        return self._collect(outputs, cycles)

    def _collect(
        self,
        outputs: list[Path],
        cycles: int,
    ) -> list[tuple[float, float, str]]:
        """Every worker's spans, refusing anything short of all of them.

        The shared append this replaced did not usually announce itself. It
        overwrote whole workers and left the survivors well-formed, so the
        READABLE failure -- a torn line, which is what `split()` used to choke
        on -- was the rarer outcome, and the common one was a quiet pass over a
        third of the intended workload.

        So completeness is checked here, once, for all four tests: an absent
        file, a short batch, or a line that is not two finite timestamps and a
        tag fails loudly rather than shrinking what `_overlaps` is given to
        look at.

        Attribution is checked as tag CONSISTENCY -- one writer per file, and
        no writer in two files -- which is exactly what `_overlaps` leans on
        when it requires `later[2] != earlier[2]`. It is deliberately NOT
        checked as `tag == str(process.pid)`: this repo's own
        `.venv\\Scripts\\python.exe` is a launcher shim that spawns the real
        interpreter as a child, so `Popen.pid` is the shim's and never equals
        the worker's `os.getpid()` (measured: 17444 vs 4268, where the base
        interpreter matches). That check failed all 15 control runs it was
        tried on before being replaced.

        `len(spans) == workers * cycles` is not asserted on top: there is one
        file per worker and each is checked for exactly `cycles` records, so
        the total follows arithmetically from checks already made here.
        """
        spans: list[tuple[float, float, str]] = []
        writers: dict[str, str] = {}
        for output in outputs:
            self.assertTrue(
                output.is_file(),
                f"a worker left no {output.name} at all",
            )
            lines = output.read_text(encoding="utf-8").splitlines()
            self.assertEqual(
                len(lines),
                cycles,
                f"{output.name} holds {len(lines)} sections, not {cycles}",
            )
            here: list[tuple[float, float, str]] = []
            for line in lines:
                fields = line.split()
                self.assertEqual(
                    len(fields),
                    3,
                    f"{output.name} holds a malformed span: {line!r}",
                )
                entered, left, tag = fields
                try:
                    began, ended = float(entered), float(left)
                except ValueError:
                    self.fail(
                        f"{output.name} holds a non-numeric span: {line!r}"
                    )
                # ORDERED comparisons involving NaN are false, so any
                # overlap comparison touching a NaN span is missed, and a NaN
                # duration slips past the degenerate-input guard too
                # (`nan - nan <= tolerance` is false as well). Not a blanket
                # blindness -- other pairs still compare -- but quiet enough
                # to be worth refusing outright.
                self.assertTrue(
                    math.isfinite(began) and math.isfinite(ended),
                    f"{output.name} holds a non-finite span: {line!r}",
                )
                here.append((began, ended, tag))
            tags = {tag for _, _, tag in here}
            self.assertEqual(
                len(tags),
                1,
                f"{output.name} mixes {len(tags)} writers: {sorted(tags)}",
            )
            writer = tags.pop()
            self.assertNotIn(
                writer,
                writers,
                f"{output.name} and {writers.get(writer)} both hold writer "
                f"{writer}",
            )
            writers[writer] = output.name
            spans.extend(here)
        return spans

    def _overlaps(
        self, spans: list[tuple[float, float, str]]
    ) -> tuple[tuple, tuple] | None:
        """First pair of critical sections that overlapped, if any.

        Two failures refused outright. They are OPPOSITE failures, not two
        kinds of blindness.

        Blind: spans narrower than the clock can resolve. `later[0] <
        earlier[1]` is unsatisfiable when every span starts and ends inside one
        tick, so a caller would read a guaranteed pass as evidence of
        exclusion.

        False alarm: overlaps smaller than a tick. Granularity alone can make
        two genuinely serialised sections look interleaved, which failed
        CORRECT code about 1 run in 10. Only an overlap wider than
        `_CLOCK_TOLERANCE_S` counts.
        """
        if all(left - entered <= _CLOCK_TOLERANCE_S
               for entered, left, _ in spans):
            raise AssertionError(
                f"all {len(spans)} spans are within one clock tolerance "
                f"({_CLOCK_TOLERANCE_S}s): the overlap check cannot fire, so a "
                f"pass proves nothing. Hold the section longer than the clock."
            )
        ordered = sorted(spans)
        for earlier, later in zip(ordered, ordered[1:]):
            overlap_s = earlier[1] - later[0]
            if overlap_s > _CLOCK_TOLERANCE_S and later[2] != earlier[2]:
                return earlier, later
        return None

    def test_racing_launchers_all_survive_the_registry_lock(self) -> None:
        """Regression test for the delete-pending EACCES, but PROBABILISTIC.

        With the `except` narrowed back to `FileExistsError` alone, workers die
        with an uncaught `PermissionError` on the delete-pending open -- when
        the run reaches that window at all.

        MEASURED DETECTION RATE, stated rather than implied: against the
        reverted fix this failed 3/4 runs at 6x150 and 4/5 at 8x400. It is a
        real detector, not a guaranteed one. The window is scheduler-dependent
        and could not be forced: opening the lock with FILE_SHARE_DELETE and
        unlinking it frees the name immediately here, so the next O_CREAT|O_EXCL
        succeeds instead of raising EACCES. Racing for it is the only way in.

        The aggregate effect is not in doubt even though a single run is: eight
        processes hammering one lock took ~1600 contended opens each, of which
        ~50 (about 3%) raised `PermissionError` errno 13.

        No spurious failures have been seen against the FIXED code, which is the
        property that matters for keeping it in the suite.

        Deliberately asserts NOTHING about exclusion. Its spans are zero width
        by design -- any hold slow enough to measure is slow enough to stop
        reaching the delete-pending window -- so `_overlaps` would refuse them,
        correctly. Exclusion is the next test's job.
        """
        spans = self._race("registry", **_SURVIVAL_RACE)

        # `_collect` already demands exactly this, per worker. Kept as the
        # test's own statement of the workload it means to have run; the
        # property this test actually detects is the returncode check inside
        # `_race`, not this line.
        self.assertEqual(
            len(spans),
            _SURVIVAL_RACE["workers"] * _SURVIVAL_RACE["cycles"])

    def test_racing_launchers_never_share_the_registry_lock(self) -> None:
        """Mutual exclusion, from wall-clock spans wide enough to compare."""
        spans = self._race("registry", **_EXCLUSION_RACE)

        self.assertIsNone(self._overlaps(spans))

    def test_racing_launchers_all_survive_the_sitl_launch_lock(self) -> None:
        """Survival and exclusion under ordinary contention.

        This lock backs off 0.5 s per miss, so it cannot be cycled fast enough
        to reach the delete-pending window the way the registry lock can. It
        carries the identical `except (FileExistsError, PermissionError)`
        clause, but this test does NOT independently prove that clause -- it
        pins mutual exclusion and worker survival only.
        """
        spans = self._race("sitl", workers=3, hold_s=0.02)

        self.assertEqual(len(spans), 3)
        self.assertIsNone(self._overlaps(spans))

    def test_the_overlap_detector_catches_an_unlocked_race(self) -> None:
        """Control arm, so the exclusion assertion cannot go quietly blind.

        Runs `_EXCLUSION_RACE` -- the SAME workload as the exclusion test
        above, by construction rather than by copied numbers -- with no
        lock at all. An earlier control raced a 0.0005 s hold while the
        test it claimed to validate raced zero-width spans, so it proved
        nothing about that test.
        """
        spans = self._race("none", **_EXCLUSION_RACE)

        self.assertIsNotNone(
            self._overlaps(spans),
            "unlocked workers never overlapped: the detector proves nothing",
        )


class CollectRefusesIncompleteEvidence(unittest.TestCase):
    """`_collect` is the third defence, so it needs its own proof it fires.

    The corruption it replaced was SILENT -- whole workers were overwritten and
    the survivors stayed well-formed -- so a completeness check that quietly
    passed on partial evidence would restore the exact failure being fixed.
    Each case below plants one specific corruption and requires it to fail.

    Deterministic on purpose: no subprocesses, no clock, no lock. The races
    above cannot be made to corrupt on demand, which is how the harness went
    unchecked long enough to hand `_overlaps` a third of a workload without
    ever saying so.
    """

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self._root = Path(self._tmp.name)
        self._case = RegistryLockContention(
            "test_racing_launchers_never_share_the_registry_lock")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _plant(self, *bodies: "str | None") -> list[Path]:
        """One output path per body; `None` means that worker wrote no file."""
        outputs: list[Path] = []
        for index, body in enumerate(bodies):
            path = self._root / f"spans.{index}.txt"
            if body is not None:
                path.write_text(body, encoding="utf-8")
            outputs.append(path)
        return outputs

    @staticmethod
    def _record(tag: int, entered: float = 1.0) -> str:
        return "%.9f %.9f %d\n" % (entered, entered + 0.01, tag)

    def test_complete_evidence_parses(self) -> None:
        """The control arm for this class: without it, every refusal below
        could be passing for some unrelated reason."""
        outputs = self._plant(self._record(9000), self._record(9001))

        spans = self._case._collect(outputs, cycles=1)

        self.assertEqual(spans, [(1.0, 1.01, "9000"), (1.0, 1.01, "9001")])

    def test_a_worker_that_wrote_nothing_fails(self) -> None:
        """Why the paths are enumerated: a glob would just return fewer."""
        outputs = self._plant(self._record(9000), None)

        with self.assertRaisesRegex(AssertionError, "left no spans.1.txt"):
            self._case._collect(outputs, cycles=1)

    def test_a_short_batch_fails(self) -> None:
        """The old bug's usual shape: well-formed, and simply incomplete."""
        outputs = self._plant(
            self._record(9000) + self._record(9000), self._record(9001))

        with self.assertRaisesRegex(
                AssertionError, "spans.1.txt holds 1 sections, not 2"):
            self._case._collect(outputs, cycles=2)

    def test_a_torn_line_fails(self) -> None:
        """The old bug's rare, visible shape -- the `split()` ValueError that
        made this defect noticeable at all."""
        outputs = self._plant("1.000000000 9000\n")

        with self.assertRaisesRegex(AssertionError, "malformed span"):
            self._case._collect(outputs, cycles=1)

    def test_a_non_numeric_span_fails(self) -> None:
        """Three fields is not by itself enough to be a span."""
        outputs = self._plant("early late 9000\n")

        with self.assertRaisesRegex(AssertionError, "non-numeric span"):
            self._case._collect(outputs, cycles=1)

    def test_a_non_finite_span_fails(self) -> None:
        """`float()` accepts NaN. Ordered comparisons involving it are
        false, so an overlap comparison touching a NaN span is missed, and a
        NaN duration slips past the degenerate-input guard as well."""
        outputs = self._plant("nan nan 9000\n")

        with self.assertRaisesRegex(AssertionError, "non-finite span"):
            self._case._collect(outputs, cycles=1)

    def test_one_file_holding_two_writers_fails(self) -> None:
        """What a shared file produced. `_overlaps` could still tell these
        two tags apart; this guard enforces one consistent worker tag per
        assigned file."""
        outputs = self._plant(self._record(9000) + self._record(9001))

        with self.assertRaisesRegex(AssertionError, "mixes 2 writers"):
            self._case._collect(outputs, cycles=2)

    def test_one_writer_holding_two_files_fails(self) -> None:
        """The other half of attribution, and this one is BLINDNESS, not noise:
        `_overlaps` SKIPS any pair sharing a tag, so two workers collapsed into
        one identity would have their real overlaps silently suppressed."""
        outputs = self._plant(self._record(9000), self._record(9000))

        with self.assertRaisesRegex(AssertionError, "both hold writer 9000"):
            self._case._collect(outputs, cycles=1)


if __name__ == "__main__":
    unittest.main()
