"""The supervisor must stay the sole writer of its own stdout/stderr.

`wsl.exe` does not write through an inherited stdout handle's shared file
position: it latches the offset at spawn and writes at `latched + relayed`,
rewinding the shared pointer, so the Python parent's next write lands on top of
bytes already on disk. A 300-line-each write storm left only 22 of 300
supervisor lines intact and destroyed 27500 bytes. That silent damage already
caused one wrong root-cause diagnosis,
so these tests pin the primitives that keep a captured launcher log honest:

* only a REGULAR FILE is relayed — consoles and pipes have no file position to
  rewind and must keep the plain inherit path;
* a record is never torn, in either direction, however reads happen to be
  chunked;
* a broken destination NEVER stops the relay reading, because a pipe this
  process stops draining blocks the WSL child and freezes the whole swarm.
"""
import collections
import functools
import importlib.util
import io
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    with open(path, "rb") as fh:
        return fh.read().decode("utf-8", "replace")


def _lines(path: str) -> collections.Counter:
    """Line counts, CRLF normalized. A Counter rather than a set so a record
    that got duplicated fails just as loudly as one that went missing."""
    return collections.Counter(
        ln for ln in _read(path).replace("\r\n", "\n").split("\n") if ln)


def _load_swarm():
    spec = importlib.util.spec_from_file_location("swarm_run", _ROOT / "scripts" / "swarm_run.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


m = _load_swarm()


def _relay(fd, source, **kw):
    return m._Relay(fd=fd, name="stdout", source=source, **kw)


def _reader(payload: bytes, chunk: int = 8):
    """A source whose read1() hands back at most *chunk* bytes, so records are
    split across reads the way a real pipe splits them."""
    class _Chunked(io.BytesIO):
        def read1(self, _size=-1):
            return self.read(chunk)
    return _Chunked(payload)


class IsRegularFile(unittest.TestCase):
    def test_true_for_a_real_file(self):
        with tempfile.TemporaryFile() as fh:
            self.assertTrue(m._is_regular_file(fh.fileno()))

    def test_false_for_a_pipe(self):
        r, w = os.pipe()
        try:
            self.assertFalse(m._is_regular_file(r))
            self.assertFalse(m._is_regular_file(w))
        finally:
            os.close(r)
            os.close(w)

    def test_false_for_a_closed_fd(self):
        r, w = os.pipe()
        os.close(r)
        os.close(w)
        self.assertFalse(m._is_regular_file(r))


class WriteAll(unittest.TestCase):
    def test_completes_a_short_write(self):
        """os.write may write fewer bytes than asked; the loop is what makes a
        record atomic against the other writers on the descriptor."""
        seen = bytearray()
        real = os.write

        def one_byte_at_a_time(fd, data):
            real(fd, data[:1])
            seen.extend(data[:1])
            return 1

        with tempfile.TemporaryFile() as fh:
            with mock.patch("os.write", one_byte_at_a_time):
                m._write_all(fh.fileno(), b"abcdefghij")
            fh.seek(0)
            self.assertEqual(fh.read(), b"abcdefghij")
        self.assertEqual(bytes(seen), b"abcdefghij")

    def test_a_zero_length_write_raises_instead_of_spinning(self):
        with tempfile.TemporaryFile() as fh:
            with mock.patch("os.write", lambda fd, data: 0):
                with self.assertRaises(OSError):
                    m._write_all(fh.fileno(), b"abc")

    def test_an_interrupted_write_is_retried_not_treated_as_broken(self):
        """EINTR is a signal, not a dead destination."""
        real = os.write
        state = {"calls": 0}

        def interrupt_once(fd, data):
            state["calls"] += 1
            if state["calls"] == 1:
                raise InterruptedError("signal")
            return real(fd, data)

        with tempfile.TemporaryFile() as fh:
            with mock.patch("os.write", interrupt_once):
                m._write_all(fh.fileno(), b"payload")
            fh.seek(0)
            self.assertEqual(fh.read(), b"payload")


class RelayKeepsDraining(unittest.TestCase):
    """A destination failure must never turn into backpressure on the child."""

    def test_broken_destination_still_reads_to_eof(self):
        payload = b"".join(b"line-%03d\n" % i for i in range(50))
        r, w = os.pipe()
        os.close(w)  # any write to r's peer fails; writing to a read fd raises
        try:
            relay = _relay(r, _reader(payload))
            m._relay_stream(relay)
        finally:
            os.close(r)
        self.assertFalse(relay.writes_enabled)
        self.assertIsNotNone(relay.error)
        self.assertEqual(relay.discarded, len(payload))
        self.assertEqual(relay.source.read(), b"")  # drained to EOF

    def test_latches_only_the_first_error(self):
        r, _w = os.pipe()
        os.close(_w)
        try:
            relay = _relay(r, _reader(b"a\nb\nc\n"))
            m._relay_stream(relay)
        finally:
            os.close(r)
        first = relay.error
        m._relay_write(relay, b"more\n")
        self.assertIs(relay.error, first)

    def test_partial_write_counts_only_the_unwritten_suffix(self):
        """Accounting must not overstate the loss: bytes that reached the file
        before the failure are not discarded bytes."""
        real = os.write
        state = {"calls": 0}

        def fail_after_three(fd, data):
            state["calls"] += 1
            if state["calls"] == 1:
                return real(fd, data[:3])
            raise OSError("destination gone")

        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _reader(b""))
            with mock.patch("os.write", fail_after_three):
                m._relay_write(relay, b"0123456789\n")
            fh.seek(0)
            self.assertEqual(fh.read(), b"012")
        self.assertEqual(relay.discarded, len(b"0123456789\n") - 3)

    def test_disabled_relay_writes_nothing_and_counts_everything(self):
        payload = b"x\ny\nz\n"
        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _reader(payload), writes_enabled=False)
            m._relay_stream(relay)
            fh.seek(0)
            self.assertEqual(fh.read(), b"")
        self.assertEqual(relay.discarded, len(payload))
        self.assertIsNone(relay.error)

    def test_retire_silences_a_relay(self):
        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _reader(b"a\n"))
            m._retire(relay)
            m._relay_write(relay, b"b\n")
            fh.seek(0)
            self.assertEqual(fh.read(), b"")
        self.assertTrue(relay.retiring)


class SourceFailures(unittest.TestCase):
    """A broken source truncates the capture. That is worth saying; an expected
    close during retirement is not."""

    def _reader_failing_after_one_read(self, first: bytes):
        class _Fails(io.BytesIO):
            def read1(self, _size=-1):
                if self.tell() == 0:
                    return self.read(len(first))
                raise OSError("pipe went away")
        return _Fails(first)

    def test_a_mid_stream_source_failure_is_recorded_and_flushes_pending(self):
        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), self._reader_failing_after_one_read(b"half"))
            m._relay_stream(relay)
            fh.seek(0)
            self.assertEqual(fh.read(), b"half")  # flushed exactly once
        self.assertIsInstance(relay.source_error, OSError)

    def test_a_retired_relay_does_not_report_its_own_close_as_a_fault(self):
        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), self._reader_failing_after_one_read(b"half"))
            m._retire(relay)
            m._relay_stream(relay)
        self.assertIsNone(relay.source_error)

    def test_a_source_that_fails_on_the_first_read_writes_nothing(self):
        class _DeadOnArrival(io.BytesIO):
            def read1(self, _size=-1):
                raise OSError("pipe went away")

        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _DeadOnArrival(b"unreachable\n"))
            m._relay_stream(relay)
            fh.seek(0)
            self.assertEqual(fh.read(), b"")
        self.assertIsInstance(relay.source_error, OSError)


class RecordsStayWhole(unittest.TestCase):
    def test_a_trailing_line_without_a_newline_is_still_written(self):
        with tempfile.TemporaryFile() as fh:
            m._relay_stream(_relay(fh.fileno(), _reader(b"whole\npartial")))
            fh.seek(0)
            self.assertEqual(fh.read(), b"whole\npartial")

    def test_a_newline_free_run_is_flushed_before_eof_not_only_at_it(self):
        """A child that never emits a newline must not be buffered forever.

        Asserting the final byte count is NOT enough — the EOF flush satisfies
        that on its own. What matters is that bytes reach the file WHILE the
        stream is still open, so the size is sampled at EOF, before the trailing
        flush can run.
        """
        payload = b"z" * (m._MAX_PENDING_RECORD * 2)
        size_at_eof = []

        with tempfile.TemporaryFile() as fh:
            class _SampleAtEof(io.BytesIO):
                def read1(self, _size=-1):
                    chunk = self.read(m._RELAY_CHUNK)
                    if not chunk:
                        size_at_eof.append(os.fstat(fh.fileno()).st_size)
                    return chunk

            m._relay_stream(_relay(fh.fileno(), _SampleAtEof(payload)))
            fh.seek(0)
            self.assertEqual(len(fh.read()), len(payload))

        self.assertTrue(size_at_eof, "the source never reached EOF")
        self.assertGreaterEqual(
            size_at_eof[0], m._MAX_PENDING_RECORD,
            "a newline-free run was held in memory until EOF instead of being "
            "flushed at the safety valve")

    def test_a_supervisor_line_cannot_split_a_half_read_child_line(self):
        """The contract, forced deterministically rather than left to timing.

        The source hands back half a child record, and the supervisor prints
        before the rest arrives — exactly the window a chunk-boundary write
        would tear. Relaying raw chunks instead of whole records fails here.
        """
        emitted = []

        class _SplitMidRecord(io.BytesIO):
            """Emits a supervisor line between the two halves of one record."""
            def read1(self, _size=-1):
                if not emitted and self.tell() > 0:
                    emitted.append(True)
                    m._emit("PARENT-0000")
                return self.read(len(b"CHILD-0000-firsthalf"))

        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "split.log"
            with out.open("wb", buffering=0) as raw:
                text = io.TextIOWrapper(io.FileIO(raw.fileno(), "w", closefd=False))
                real_stdout, sys.stdout = sys.stdout, text
                try:
                    m._relay_stream(_relay(
                        raw.fileno(),
                        _SplitMidRecord(b"CHILD-0000-firsthalf" b"CHILD-0000-secondhalf\n")))
                    text.flush()
                finally:
                    sys.stdout = real_stdout
            data = out.read_bytes().replace(b"\r\n", b"\n")
        self.assertTrue(emitted, "the supervisor never printed mid-record")
        self.assertIn(b"CHILD-0000-firsthalfCHILD-0000-secondhalf\n", data,
                      f"child record was torn: {data!r}")
        self.assertIn(b"PARENT-0000\n", data)

    def test_no_supervisor_line_lands_inside_a_child_line(self):
        """The real interleaving contract, with the relay on its own thread and
        reads deliberately cut mid-record."""
        n_child, n_parent = 200, 200
        child = b"".join(b"CHILD-%04d-%s\n" % (i, b"c" * 60) for i in range(n_child))
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "out.log"
            with out.open("wb", buffering=0) as raw:
                relay = _relay(raw.fileno(), _reader(child, chunk=7))
                t = threading.Thread(target=m._relay_stream, args=(relay,))
                t.start()
                text = io.TextIOWrapper(io.FileIO(raw.fileno(), "w", closefd=False),
                                        write_through=False)
                real_stdout, sys.stdout = sys.stdout, text
                try:
                    for i in range(n_parent):
                        m._emit(f"PARENT-{i:04d}-{'p' * 60}")
                    text.flush()
                finally:
                    sys.stdout = real_stdout
                t.join(timeout=30)
                self.assertFalse(t.is_alive())
            data = out.read_bytes()
        lines = data.replace(b"\r\n", b"\n").split(b"\n")
        if lines and lines[-1] == b"":
            lines.pop()
        self.assertEqual(len(lines), n_child + n_parent, "a record was torn or lost")
        for i in range(n_child):
            self.assertIn(b"CHILD-%04d-%s" % (i, b"c" * 60), lines)
        for i in range(n_parent):
            self.assertIn(("PARENT-%04d-%s" % (i, "p" * 60)).encode(), lines)


class EmitPreservesTodaysBytes(unittest.TestCase):
    def test_emit_goes_through_the_text_layer(self):
        """_emit must keep writing through print() rather than hand-encoding.

        Whatever the platform's text layer does to line endings, the supervisor
        keeps doing it — hand-encoding would silently change the line endings of
        every captured log, which is not a change to smuggle into a fix whose
        point is that logs can be trusted. (This asserts the text stream is
        used; it does not itself prove Windows CRLF translation.)
        """
        buf = io.StringIO()
        real_stdout, sys.stdout = sys.stdout, buf
        try:
            m._emit("hello")
        finally:
            sys.stdout = real_stdout
        self.assertEqual(buf.getvalue(), "hello\n")

    def test_emit_routes_errors_to_stderr(self):
        buf = io.StringIO()
        real_stderr, sys.stderr = sys.stderr, buf
        try:
            m._emit("boom", err=True)
        finally:
            sys.stderr = real_stderr
        self.assertEqual(buf.getvalue(), "boom\n")


class _FakePopen:
    def __init__(self, stdout=None, stderr=None):
        self.stdout, self.stderr = stdout, stderr
        self.killed = False

    def kill(self):
        self.killed = True


class StartLauncher(unittest.TestCase):
    """Which streams get piped is decided per stream, from THIS process's fds.

    The eval harness gives swarm_run a separate `swarm.out.log` and
    `swarm.err.log`; folding stderr into stdout would break that, so the two
    decisions stay independent.
    """

    def _start(self, regular_fds):
        captured = {}

        def fake_popen(argv, stdout=None, stderr=None):
            captured["argv"] = argv
            captured["stdout"], captured["stderr"] = stdout, stderr
            return _FakePopen(
                stdout=_reader(b"") if stdout is m.subprocess.PIPE else None,
                stderr=_reader(b"") if stderr is m.subprocess.PIPE else None)

        with mock.patch.object(m, "_is_regular_file", lambda fd: fd in regular_fds), \
                mock.patch.object(m.subprocess, "Popen", fake_popen):
            proc, relays = m._start_launcher("run_swarm.sh")
        for relay in relays:
            relay.thread.join(timeout=10)
        return captured, proc, relays

    def test_neither_stream_regular_inherits_exactly_as_before(self):
        captured, _proc, relays = self._start(set())
        self.assertIsNone(captured["stdout"])
        self.assertIsNone(captured["stderr"])
        self.assertEqual(relays, [])

    def test_only_stdout_regular_pipes_only_stdout(self):
        captured, _proc, relays = self._start({1})
        self.assertIs(captured["stdout"], m.subprocess.PIPE)
        self.assertIsNone(captured["stderr"])
        self.assertEqual([r.name for r in relays], ["stdout"])
        self.assertEqual([r.fd for r in relays], [1])

    def test_only_stderr_regular_pipes_only_stderr(self):
        captured, _proc, relays = self._start({2})
        self.assertIsNone(captured["stdout"])
        self.assertIs(captured["stderr"], m.subprocess.PIPE)
        self.assertEqual([r.name for r in relays], ["stderr"])
        self.assertEqual([r.fd for r in relays], [2])

    def test_both_regular_pipes_both_as_separate_relays(self):
        captured, _proc, relays = self._start({1, 2})
        self.assertIs(captured["stdout"], m.subprocess.PIPE)
        self.assertIs(captured["stderr"], m.subprocess.PIPE)
        self.assertEqual([r.fd for r in relays], [1, 2])

    def test_the_command_still_goes_through_the_shared_wsl_argv(self):
        # --exec included: without it wsl.exe hands the command line to the
        # distro's default shell, which re-parses it before bash sees it.
        captured, _proc, _relays = self._start({1})
        self.assertEqual(captured["argv"], m._wsl_argv("run_swarm.sh"))
        self.assertEqual(captured["argv"],
                         ["wsl", "--exec", "bash", "-lc", "run_swarm.sh"])

    def test_a_child_whose_relay_cannot_start_is_killed(self):
        """A piped child with nothing draining it blocks on a full pipe and
        wedges the launch, so it must not be left running."""
        proc = _FakePopen(stdout=_reader(b""))
        with mock.patch.object(m, "_is_regular_file", lambda fd: fd == 1), \
                mock.patch.object(m.subprocess, "Popen", lambda *a, **k: proc), \
                mock.patch.object(m.threading, "Thread",
                                  side_effect=RuntimeError("can't start thread")):
            with self.assertRaises(RuntimeError):
                m._start_launcher("run_swarm.sh")
        self.assertTrue(proc.killed)

    def test_a_partial_relay_start_retires_the_relay_that_did_start(self):
        """stdout's relay starts, stderr's blows up. The started one must not
        keep writing for an attempt that never happened, and the child must be
        killed AND reaped."""
        proc = mock.MagicMock()
        proc.stdout, proc.stderr = _reader(b""), _reader(b"")
        made = []

        def one_good_thread(*_a, **kw):
            if made:
                raise RuntimeError("can't start thread")
            thread = mock.MagicMock()
            made.append(thread)
            return thread

        with mock.patch.object(m, "_is_regular_file", lambda fd: True), \
                mock.patch.object(m.subprocess, "Popen", lambda *a, **k: proc), \
                mock.patch.object(m.threading, "Thread", side_effect=one_good_thread), \
                mock.patch.object(m, "_retire") as retire:
            with self.assertRaises(RuntimeError):
                m._start_launcher("run_swarm.sh")

        self.assertEqual(retire.call_count, 1, "the started relay was not silenced")
        self.assertEqual(retire.call_args.args[0].name, "stdout")
        proc.kill.assert_called_once()
        proc.wait.assert_called_once()


class RetireRelays(unittest.TestCase):
    def test_a_stuck_relay_is_silenced_so_the_next_attempt_is_safe(self):
        """The restart-safety contract: attempt N's relay may still be blocked
        in read1 when attempt N+1 starts, and must not be able to write."""
        released = threading.Event()

        class _Blocks(io.BytesIO):
            def read1(self, _size=-1):
                released.wait(timeout=30)
                return self.read(len(b"late\n"))

        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _Blocks(b"late\n"))
            relay.thread = threading.Thread(target=m._relay_stream, args=(relay,),
                                            daemon=True)
            relay.thread.start()
            with mock.patch.object(m, "RELAY_DRAIN_TIMEOUT", 0.05):
                m._retire_relays(7, [relay], reaped=True)
            self.assertTrue(relay.retiring)
            self.assertFalse(relay.writes_enabled)

            released.set()          # the stuck relay wakes up AFTER retirement
            relay.thread.join(timeout=30)
            self.assertFalse(relay.thread.is_alive())
            fh.seek(0)
            self.assertEqual(fh.read(), b"", "a retired relay wrote to the fd")
        self.assertEqual(relay.discarded, len(b"late\n"))

    def test_the_drain_budget_is_shared_across_relays_not_per_relay(self):
        """It is spent holding the machine-wide launch lock, so two stuck
        streams must not cost twice the wait.

        Asserted on the join timeouts with a fake clock, not on the wall clock:
        an elapsed-time assertion is flaky on a loaded box and would need real
        threads sleeping through the test.
        """
        joins = []

        class _StuckThread:
            def join(self, timeout=None):
                joins.append(timeout)

            def is_alive(self):
                return True

        relays = []
        for name in ("stdout", "stderr"):
            r = m._Relay(fd=1, name=name, source=_reader(b""))
            r.thread = _StuckThread()
            relays.append(r)

        clock = iter([0.0, 0.0, 0.3, 0.3, 0.3])
        with mock.patch.object(m, "RELAY_DRAIN_TIMEOUT", 0.3), \
                mock.patch.object(m.time, "monotonic", lambda: next(clock)), \
                mock.patch.object(m, "_warn_about_relay"):
            m._retire_relays(7, relays, reaped=True)

        self.assertEqual(len(joins), 2)
        self.assertAlmostEqual(joins[0], 0.3)
        self.assertEqual(joins[1], 0.0,
                         "the second relay was granted its own fresh budget")
        self.assertTrue(all(r.retiring for r in relays))

    def test_a_clean_relay_is_not_retired_and_is_reported_as_not_timed_out(self):
        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _reader(b"all\ngood\n"))
            relay.thread = threading.Thread(target=m._relay_stream, args=(relay,),
                                            daemon=True)
            relay.thread.start()
            with mock.patch.object(m, "_warn_about_relay") as warn:
                m._retire_relays(7, [relay], reaped=True)
            fh.seek(0)
            self.assertEqual(fh.read(), b"all\ngood\n")
        self.assertFalse(relay.retiring)
        warn.assert_called_once_with(7, relay, False, reaped=True)


class DrainRescuesTheTail(unittest.TestCase):
    """What the reaped drain is FOR, made deterministic.

    A real-WSL version of this is vacuous: the relay runs concurrently, so a
    child that writes and exits is usually fully relayed before ``wait()``
    returns, leaving no tail to rescue. Here the relay is held at the final
    record on purpose, so skipping the drain provably loses it.
    """

    def test_a_record_still_in_flight_at_wait_is_written_by_the_drain(self):
        held = threading.Event()
        blocked = threading.Event()
        release = threading.Event()
        real_write = m._relay_write

        def hold_the_last_record(relay, data):
            if b"TAIL" in data and not held.is_set():
                held.set()
                blocked.set()
                release.wait(timeout=30)
            real_write(relay, data)

        with tempfile.TemporaryFile() as fh:
            relay = _relay(fh.fileno(), _reader(b"EARLY\nTAIL\n"))
            relay.thread = threading.Thread(target=m._relay_stream, args=(relay,),
                                            daemon=True)
            try:
                with mock.patch.object(m, "_relay_write", hold_the_last_record):
                    relay.thread.start()
                    self.assertTrue(blocked.wait(timeout=30),
                                    "the relay never reached the final record")
                    fh.seek(0)
                    self.assertNotIn(b"TAIL", fh.read(),
                                     "the tail was already written; nothing to rescue")
                    release.set()
                    m._retire_relays(7, [relay], reaped=True)
            finally:
                release.set()   # never strand the thread, however this exits
                relay.thread.join(timeout=30)

            fh.seek(0)
            self.assertIn(b"TAIL", fh.read(),
                          "the drain did not wait for the in-flight record")
        self.assertFalse(relay.retiring, "a drainable relay must not be retired")


class KillLaunchReaps(unittest.TestCase):
    """`kill` only REQUESTS termination. Without a following wait, the drain
    cannot know whether the pipe's only writer is really gone."""

    def _kill(self, terminate_ok: bool, kill_wait_ok: bool):
        proc = mock.MagicMock()
        waits = []

        def wait(timeout=None):
            waits.append(timeout)
            ok = terminate_ok if len(waits) == 1 else kill_wait_ok
            if not ok:
                raise subprocess.TimeoutExpired("wsl", timeout)

        proc.wait.side_effect = wait
        with mock.patch.object(m, "cleanup"),                 mock.patch.object(m, "_retire_relays") as drain:
            m._kill_launch(proc, 7, [])
        return proc, drain, waits

    def test_a_clean_terminate_reports_the_child_reaped(self):
        _proc, drain, _waits = self._kill(terminate_ok=True, kill_wait_ok=True)
        self.assertTrue(drain.call_args.kwargs["reaped"])

    def test_a_terminate_that_raises_still_reaches_the_kill_and_wait(self):
        """`terminate()` can fail outright, not just time out in its wait."""
        proc = mock.MagicMock()
        proc.terminate.side_effect = OSError("access denied")
        with mock.patch.object(m, "cleanup"),                 mock.patch.object(m, "_retire_relays") as drain:
            m._kill_launch(proc, 7, [])
        proc.kill.assert_called_once()
        proc.wait.assert_called_once()
        self.assertTrue(drain.call_args.kwargs["reaped"])

    def test_a_killed_child_is_waited_for_before_the_drain_trusts_it(self):
        proc, drain, waits = self._kill(terminate_ok=False, kill_wait_ok=True)
        proc.kill.assert_called_once()
        self.assertEqual(len(waits), 2, "kill() was not followed by a wait()")
        self.assertTrue(drain.call_args.kwargs["reaped"])

    def test_a_child_that_survives_the_kill_is_not_reported_reaped(self):
        _proc, drain, _waits = self._kill(terminate_ok=False, kill_wait_ok=False)
        self.assertFalse(drain.call_args.kwargs["reaped"],
                         "an unreaped child must not get the tail-waiting path")


class KillLaunchOrder(unittest.TestCase):
    def test_relays_are_retired_only_after_the_child_is_dead(self):
        """`cleanup` is a SIGTERM, a two-second grace and a SIGKILL. The dying
        SITL and router still have things to say in it, and the relays must
        still be draining so the pipes cannot backpressure a child we are
        already trying to shut down."""
        order = []
        proc = mock.MagicMock()
        proc.terminate.side_effect = lambda: order.append("terminate")
        proc.wait.side_effect = lambda timeout=None: order.append("wait")
        with mock.patch.object(m, "cleanup", side_effect=lambda c: order.append("cleanup")), \
                mock.patch.object(m, "_retire_relays",
                                  side_effect=lambda c, r, **kw: order.append("retire")):
            m._kill_launch(proc, 7, [])
        self.assertEqual(order, ["cleanup", "terminate", "wait", "retire"])


class RelayWarnings(unittest.TestCase):
    """Diagnostics must be loud when something is wrong and silent otherwise —
    and must never be able to decide a launch."""

    def _relay_with(self, **kw):
        return m._Relay(fd=1, name="stdout", source=_reader(b""), **kw)

    def test_a_healthy_relay_says_nothing(self):
        self.assertIsNone(m._relay_warning(self._relay_with(), timed_out=False, reaped=True))

    def test_a_broken_destination_is_named_with_the_dropped_byte_count(self):
        msg = m._relay_warning(
            self._relay_with(error=OSError("gone"), discarded=1234), timed_out=False, reaped=True)
        self.assertIn("destination write failed", msg)
        self.assertIn("discarded 1234 bytes", msg)

    def test_a_broken_source_says_the_capture_may_be_truncated(self):
        msg = m._relay_warning(
            self._relay_with(source_error=OSError("pipe")), timed_out=False, reaped=True)
        self.assertIn("truncated", msg)

    def test_a_timeout_says_the_relay_was_silenced(self):
        msg = m._relay_warning(self._relay_with(), timed_out=True, reaped=False)
        self.assertIn("did not reach EOF", msg)
        self.assertIn("silenced", msg)

    def test_every_timeout_is_marked_a_capture_problem(self):
        """The marker is the grep handle for "this log may be short", so it has
        to track that and not which cause was the surprising one.

        The unreaped path is the one that routinely fires, and a silenced relay
        discards a still-running child's continuing output — so marking only the
        reaped path would put the handle on the case that has never been
        observed and leave the common one quiet.
        """
        for reaped in (True, False):
            with self.subTest(reaped=reaped):
                msg = m._relay_warning(self._relay_with(), timed_out=True,
                                       reaped=reaped)
                self.assertIn("LOG CAPTURE INCOMPLETE", msg)
                # Hedged on purpose: a retained writer can hold the pipe open
                # without writing, so an unfinished drain is not proof of loss.
                self.assertIn("may be missing its tail", msg)

    def test_the_timeout_cause_is_still_distinguishable(self):
        """Marking both must not cost the distinction — which one happened
        decides whether an assumption broke or a child simply outlived us."""
        reaped = m._relay_warning(self._relay_with(), timed_out=True, reaped=True)
        unreaped = m._relay_warning(self._relay_with(), timed_out=True, reaped=False)
        self.assertIn("was reaped", reaped)
        self.assertNotIn("could not be reaped", reaped)
        self.assertIn("could not be reaped", unreaped)

    def test_a_healthy_relay_is_never_marked(self):
        """The marker has to stay rare enough to mean something."""
        for reaped in (True, False):
            with self.subTest(reaped=reaped):
                self.assertIsNone(m._relay_warning(self._relay_with(),
                                                   timed_out=False, reaped=reaped))

    def test_every_problem_is_reported_in_one_line(self):
        msg = m._relay_warning(
            self._relay_with(error=OSError("gone"), source_error=OSError("pipe"),
                             discarded=9), timed_out=True, reaped=False)
        self.assertNotIn("\n", msg)
        for expected in ("destination write failed", "truncated",
                         "discarded 9 bytes", "did not reach EOF"):
            self.assertIn(expected, msg)

    def test_a_stdout_problem_is_reported_on_stderr(self):
        """The relay's own destination is exactly what may be broken."""
        with mock.patch.object(m, "_emit") as emit:
            m._warn_about_relay(3, self._relay_with(error=OSError("gone")), False, reaped=True)
        self.assertTrue(emit.call_args.kwargs["err"])

    def test_a_stderr_problem_is_reported_on_stdout(self):
        relay = m._Relay(fd=2, name="stderr", source=_reader(b""), error=OSError("gone"))
        with mock.patch.object(m, "_emit") as emit:
            m._warn_about_relay(3, relay, False, reaped=True)
        self.assertFalse(emit.call_args.kwargs["err"])

    def test_falls_back_to_the_other_stream_then_gives_up_quietly(self):
        relay = self._relay_with(error=OSError("gone"))
        with mock.patch.object(m, "_emit", side_effect=OSError("both broken")) as emit:
            m._warn_about_relay(3, relay, False, reaped=True)  # must not raise
        self.assertEqual(emit.call_count, 2)

    def test_a_non_oserror_from_print_cannot_abort_a_launch(self):
        """This runs inside _kill_launch. `print` to a closed text stream raises
        ValueError, not OSError — letting that escape would fail a launch
        because a WARNING could not be printed."""
        relay = self._relay_with(error=OSError("gone"))
        with mock.patch.object(m, "_emit",
                               side_effect=ValueError("I/O operation on closed file")):
            m._warn_about_relay(3, relay, False, reaped=True)  # must not raise

    def test_a_discard_count_is_reported_as_running_not_final(self):
        """A silenced relay can still be draining while this line is written."""
        msg = m._relay_warning(self._relay_with(discarded=5), timed_out=True, reaped=False)
        self.assertIn("discarded 5 bytes so far", msg)


#: How long to let the whole storm run before calling it wedged.
_STORM_TIMEOUT = 120.0
#: Records per stream per side. Large enough that the old shape lost most of the
#: supervisor's output (18 of 300 survived when this was measured by hand).
_STORM_RECORDS = 300

#: Waits to be released, THEN writes. That ordering is the defect: by the time
#: the child writes, the supervisor has already advanced the shared file
#: position, and an inherited handle makes wsl.exe write at the offset it
#: latched at spawn — on top of what the supervisor put there.
_WSL_CHILD = """\
n="$1"; ready="$2"; go="$3"
: > "$ready"
while [ ! -f "$go" ]; do sleep 0.05; done
i=0
while [ "$i" -lt "$n" ]; do
  printf 'CHILD-OUT-%04d-UDP-connection-172.23.112.1-sniffing-all-messages\\n' "$i"
  printf 'CHILD-ERR-%04d-UDP-connection-172.23.112.1-sniffing-all-messages\\n' "$i" >&2
  i=$((i+1))
done
"""

#: Runs in its OWN process so fd 1 and fd 2 are ordinary real files, exactly as
#: they are under `python scripts/swarm_run.py > out.log 2> err.log`. Repointing
#: the test runner's own fd 1 with dup2 would fight pytest's capture, which
#: replaces fd 1 and `sys.stdout` independently — the relay writes raw to the fd
#: while _emit goes through the text wrapper, so the two halves could land in
#: different places and the test would prove nothing.
_DRIVER = '''\
import importlib.util
import os
import sys
import time

swarm_path, cmd, n, ready, go = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4], sys.argv[5]

spec = importlib.util.spec_from_file_location("swarm_run", swarm_path)
m = importlib.util.module_from_spec(spec)
sys.modules["swarm_run"] = m
spec.loader.exec_module(m)

proc, relays = m._start_launcher(cmd)
try:
    deadline = time.monotonic() + 60
    while not os.path.exists(ready):
        if time.monotonic() > deadline:
            raise SystemExit("the WSL child never signalled ready")
        time.sleep(0.02)
    for i in range(n):
        m._emit("PARENT-OUT-%04d-SITL-chat-0-all-3-instances-at-SIM_SPEEDUP=1" % i)
        m._emit("PARENT-ERR-%04d-SITL-chat-0-all-3-instances-at-SIM_SPEEDUP=1" % i, err=True)
    open(go, "w").close()
    proc.wait(timeout=90)
finally:
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except Exception:
            proc.kill()
            try:
                proc.wait(timeout=10)
            except Exception:
                pass
    m._retire_relays(0, relays, reaped=True)
'''


def _wsl_path(win_dir: str) -> str:
    """Ask WSL where this directory is, rather than assuming ``/mnt/<drive>``:
    a distro can configure a different automount root."""
    # Forward slashes: wsl.exe eats backslashes out of its arguments.
    out = subprocess.run(["wsl", "wslpath", "-u", os.path.abspath(win_dir).replace(os.sep, "/")],
                         capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        raise RuntimeError(f"wslpath failed: {out.stderr.strip()}")
    return out.stdout.strip()


@functools.lru_cache(maxsize=1)
def _wsl_available() -> bool:
    try:
        return subprocess.run(["wsl", "bash", "-lc", "true"],
                              capture_output=True, timeout=10).returncode == 0
    except Exception:
        return False


@unittest.skipUnless(sys.platform == "win32", "the wsl.exe clobber is Windows-only")
class RealWslWriteStorm(unittest.TestCase):
    """The regression that would have caught the original defect.

    A real `wsl bash -lc` child and a real supervisor both producing records
    into real capture files. Measured by hand before the fix, at these exact
    volumes: 18 of 300 supervisor records survived and the file was 24594 bytes
    instead of 48000. After it: everything present, both directions.
    """

    @classmethod
    def setUpClass(cls):
        # Probed lazily, not in a class decorator: a decorator runs the probe at
        # import time even for a run that deselects these tests.
        if not _wsl_available():
            raise unittest.SkipTest("wsl.exe is not available on this machine")

    def _run_storm(self):
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        paths = {name: os.path.join(tmp, name)
                 for name in ("driver.py", "child.sh", "ready", "go",
                              "storm.out.log", "storm.err.log")}
        # newline="\n": bash will not run a script with CRLF line endings.
        with open(paths["child.sh"], "w", newline="\n") as fh:
            fh.write(_WSL_CHILD)
        with open(paths["driver.py"], "w") as fh:
            fh.write(_DRIVER)

        wsl_tmp = _wsl_path(tmp)
        cmd = (f'bash "{wsl_tmp}/child.sh" {_STORM_RECORDS} '
               f'"{wsl_tmp}/ready" "{wsl_tmp}/go"')
        argv = [sys.executable, paths["driver.py"],
                str(_ROOT / "scripts" / "swarm_run.py"), cmd,
                str(_STORM_RECORDS), paths["ready"], paths["go"]]

        with open(paths["storm.out.log"], "wb") as out, \
                open(paths["storm.err.log"], "wb") as err:
            proc = subprocess.Popen(argv, cwd=str(_ROOT), stdout=out, stderr=err)
            try:
                proc.wait(timeout=_STORM_TIMEOUT)
            except subprocess.TimeoutExpired:
                # This exact process tree only. An image-wide taskkill would
                # take out unrelated python, and `wsl pkill` would kill other
                # sessions' SITL.
                subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                               capture_output=True)
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    # Say so plainly; letting this one escape would replace the
                    # real failure with a confusing second timeout.
                    self.fail(f"the write-storm driver (pid {proc.pid}) survived "
                              f"taskkill /T /F")
                self.fail("the write-storm driver wedged")
        self.assertEqual(proc.returncode, 0,
                         f"driver failed:\n{_read(paths['storm.err.log'])[-2000:]}")
        return _lines(paths["storm.out.log"]), _lines(paths["storm.err.log"])

    def test_no_record_is_lost_or_torn_when_the_capture_is_a_regular_file(self):
        out, err = self._run_storm()
        # Counted, not compared to a byte total: an unrelated WSL profile line
        # must not fail the test, but a missing or merged record must.
        for stream, lines, parent, child in (
                ("stdout", out, "PARENT-OUT", "CHILD-OUT"),
                ("stderr", err, "PARENT-ERR", "CHILD-ERR")):
            for i in range(_STORM_RECORDS):
                for prefix, tail in ((parent, "SITL-chat-0-all-3-instances-at-SIM_SPEEDUP=1"),
                                     (child, "UDP-connection-172.23.112.1-sniffing-all-messages")):
                    record = f"{prefix}-{i:04d}-{tail}"
                    self.assertEqual(lines[record], 1,
                                     f"{stream}: {record!r} appears {lines[record]} times")

    def test_stdout_and_stderr_land_in_their_own_files(self):
        """The eval harness reads a separate swarm.out.log and swarm.err.log.
        A future `stderr=subprocess.STDOUT` would fold them together."""
        out, err = self._run_storm()
        for capture, lines, wrong in (("stdout", out, "-ERR-"), ("stderr", err, "-OUT-")):
            leaked = [ln for ln in lines if wrong in ln]
            # Count plus one example: a bare assertFalse on the list prints all
            # 300 leaked records and buries the point.
            self.assertEqual(
                len(leaked), 0,
                f"{len(leaked)} {wrong.strip('-')} records leaked into the "
                f"{capture} capture, e.g. {leaked[0] if leaked else ''!r}")


if __name__ == "__main__":
    unittest.main()
