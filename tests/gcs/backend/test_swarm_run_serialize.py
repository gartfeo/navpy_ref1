"""swarm_run must PROVE a launched swarm before declaring it up.

Two independent silent-pass bugs are locked out here:

* a stuck instance that streams nothing must not pass as "up" (the old serial0
  TCP-accept check let it), and
* a swarm that comes up at the wrong SIM_SPEEDUP must not pass at all — a
  ``--speedup 1`` launch that actually ran at 10 contaminated a whole
  speed-sensitive A/B before this gate existed.

The verify port failing to bind used to mean "assume up", i.e. a launch that
verified nothing reported success. That path must now fail the attempt.

A third silent-pass path is locked out here too: SIM_SPEEDUP proves achieved
CONFIGURATION, not process PROVENANCE, so a survivor of an earlier swarm that
happens to run at the requested speed passed every check above. The boot-age gate
rejects it.
"""
import contextlib
import importlib.util
import os
import pathlib
import sys
import threading
import time
import unittest
from unittest.mock import DEFAULT, MagicMock, patch

from gcs.backend import instance_registry as _real_reg

_ROOT = pathlib.Path(__file__).resolve().parents[3]

#: Short enough to keep the tests fast, long enough that a single request round
#: is always sent before the phase gives up.
PARAM_T = 0.4
#: Same idea for the boot-freshness window. It is driven by the fake monotonic
#: clock below, not by real time, so this is a budget in fake seconds.
BOOT_T = 0.4
#: Where the fake clock starts, and therefore the attempt's launch instant.
LAUNCHED_AT = 1000.0

#: A distinctive code from the WSL child, so "the child's exit code is what
#: swarm_run exits with" is legible rather than a mock comparing equal to itself.
CHILD_EXIT_CODE = 23


def _load_swarm():
    spec = importlib.util.spec_from_file_location("swarm_run", _ROOT / "scripts" / "swarm_run.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _hb(sysid):
    msg = MagicMock()
    msg.get_type.return_value = "HEARTBEAT"
    msg.get_srcSystem.return_value = sysid
    msg.get_srcComponent.return_value = 1
    return msg


def _param(sysid, value, name=b"SIM_SPEEDUP\x00\x00\x00\x00\x00"):
    msg = MagicMock()
    msg.get_type.return_value = "PARAM_VALUE"
    msg.get_srcSystem.return_value = sysid
    msg.get_srcComponent.return_value = 1
    msg.param_id = name
    msg.param_value = value
    return msg


def _boot(sysid, boot_ms, mtype="SYSTEM_TIME", comp=1):
    """A message carrying a boot clock, as the freshness phase sees it.

    Note these are MagicMocks, so an unset attribute is a child Mock rather than
    missing — ``time_boot_ms`` is always set here on purpose, and the source's
    numeric guard is what keeps a Mock from ever counting as a boot time.
    """
    msg = MagicMock()
    msg.get_type.return_value = mtype
    msg.get_srcSystem.return_value = sysid
    msg.get_srcComponent.return_value = comp
    msg.time_boot_ms = boot_ms
    return msg


class _Clock:
    """Deterministic stand-in for time.monotonic: every read advances by *step*.

    Real sleeps would make these tests slow and flaky, and the freshness bound is
    a function of the clock, so the clock has to be controlled rather than waited
    on. Reads advance so every loop terminates at its deadline.

    It governs ALL THREE verification phases. It used to govern only freshness
    while the heartbeat and speed phases ran on the real wall clock, so their
    timeouts were real seconds: under load a phase-2 deadline expired while this
    clock had barely moved, the speed gate returned early, and a freshness test
    failed reporting a speed-phase reason instead of the one it asserted.
    """

    def __init__(self, step=0.01, start=LAUNCHED_AT):
        self.t = start
        self.step = step

    def __call__(self):
        self.t += self.step
        return self.t


class _FakeConn:
    """Mimics the pymavlink connection closely enough to exercise both phases.

    ``recv_match(type=...)`` DISCARDS non-matching messages, like the real one —
    so a test can't accidentally rely on a filtered read leaving a message behind.
    Parameter replies are produced by *responder* in answer to an actual
    ``param_request_read_send``, which is also how the real vehicles behave and
    means a missing request shows up as a timeout.
    """

    def __init__(self, msgs, responder=None, boot_responder=None):
        self.queue = list(msgs)
        self.responder = responder or (lambda sysid, attempt: None)
        self.boot_responder = boot_responder or (lambda sysid, attempt: None)
        self.requests = []
        self.boot_requests = []
        self.closed = False
        self.mav = MagicMock()
        self.mav.param_request_read_send.side_effect = self._on_request
        self.mav.command_long_send.side_effect = self._on_boot_request

    def _queue(self, reply):
        if isinstance(reply, _Pair):
            self.queue.extend(reply.msgs)
        elif reply is not None:
            self.queue.append(reply)

    def _on_request(self, sysid, comp, name, index):
        self.requests.append((sysid, comp, name, index))
        self._queue(self.responder(sysid, len(
            [r for r in self.requests if r[0] == sysid])))

    def _on_boot_request(self, sysid, comp, command, confirmation, *params):
        self.boot_requests.append((sysid, comp, command, params[0]))
        self._queue(self.boot_responder(sysid, len(
            [r for r in self.boot_requests if r[0] == sysid])))

    def recv_match(self, type=None, blocking=False, timeout=None):
        while self.queue:
            msg = self.queue.pop(0)
            if type is None or msg.get_type() == type:
                return msg
        return None

    def close(self):
        self.closed = True


class _Pair:
    """Marker: one request produces two messages (see _FakeConn._on_request)."""

    def __init__(self, *msgs):
        self.msgs = msgs


def _responder(values, name=b"SIM_SPEEDUP\x00\x00\x00\x00\x00",
               extra=None, answer_on=1):
    """Answer SIM_SPEEDUP with *values* (sysid -> value); silent for unlisted.

    *extra* rides along ahead of the first real answer. Replies are produced in
    answer to a request, i.e. AFTER the drain — the only place a source-filter
    test means anything, since whatever is queued before the drain is discarded
    by the drain rather than by the filter.

    *answer_on* is the request number a vehicle finally answers, so a test can
    require the re-send to actually happen.
    """
    injected = []

    def reply(sysid, attempt):
        msgs = []
        if extra is not None and not injected:
            injected.append(True)
            msgs.append(extra)
        if sysid in values and attempt >= answer_on:
            msgs.append(_param(sysid, values[sysid], name=name))
        if not msgs:
            return None
        return msgs[0] if len(msgs) == 1 else _Pair(*msgs)
    return reply


def _boot_responder(values, mtype="SYSTEM_TIME", comp=1, extra=None, answer_on=1):
    """Answer the directed SYSTEM_TIME request with *values* (sysid -> boot_ms).

    Mirrors _responder: silent for unlisted sysids, *extra* rides along ahead of
    the first real answer, and *answer_on* is the request number a vehicle
    finally answers so a test can require the re-send to happen.
    """
    injected = []

    def reply(sysid, attempt):
        msgs = []
        if extra is not None and not injected:
            injected.append(True)
            msgs.append(extra)
        if sysid in values and attempt >= answer_on:
            msgs.append(_boot(sysid, values[sysid], mtype=mtype, comp=comp))
        if not msgs:
            return None
        return msgs[0] if len(msgs) == 1 else _Pair(*msgs)
    return reply


#: Boot clocks a fresh instance could plausibly report, for a swarm launched at
#: LAUNCHED_AT on the fake clock. Well inside the bound at either speed.
#:
#: NOTE these are CONSTANT, i.e. they model a vehicle whose sim clock is STOPPED.
#: That was invisible while the only speed check read the SIM_SPEEDUP parameter,
#: and it is exactly the blind spot that let a swarm report ``speedup=1`` while
#: running at 10x. Now that the gate measures the clock, a test that needs a
#: HEALTHY swarm must use _healthy() below; FRESH remains only for tests about
#: freshness and staleness, which decide before the rate is ever consulted.
FRESH = {1: 100.0, 2: 100.0, 3: 100.0}

#: Least sample span the rate gate accepts, in FAKE seconds. The production value
#: (CLOCK_MIN_SPAN_S = 1.0 real seconds) is far longer than a test's fake window,
#: so it is injected, exactly as the other three budgets already are.
MIN_SPAN_T = 0.05


class _StreamedBoot:
    """A boot-clock message whose ``time_boot_ms`` is evaluated WHEN IT IS READ.

    A real vehicle stamps its boot clock at the instant it sends. This fake
    connection cannot: it builds a reply when the request goes out and the
    verifier consumes it some number of iterations later, so a value fixed at
    build time is dated to the wrong instant. That is not a cosmetic difference —
    with the queue re-filled every iteration and drained one message at a time,
    fixed stamps fall behind by the queue depth and the measured rate comes out
    divided by the number of instances.

    A plain MagicMock cannot express this, because the point is that the attribute
    is a property rather than a stored value.
    """

    def __init__(self, sysid, clock, speedup, base=100.0,
                 mtype="SYSTEM_TIME", comp=1):
        self._sysid = sysid
        self._clock = clock
        self._speedup = speedup
        self._base = base
        self._mtype = mtype
        self._comp = comp

    def get_type(self):
        return self._mtype

    def get_srcSystem(self):
        return self._sysid

    def get_srcComponent(self):
        return self._comp

    @property
    def time_boot_ms(self):
        return self._base + (self._clock.t - LAUNCHED_AT) * 1000.0 * self._speedup


def _healthy(speedup=1, base=100.0, sysids=(1, 2, 3), clock=None):
    """A ``(clock, boot_responder)`` pair modelling instances that really run at
    *speedup*.

    The boot clock advances at *speedup* times the fake wall clock — which is what
    a real instance does, and what the measured-rate gate checks. The two are built
    together because the message reads the same clock the verifier times its
    samples with; a message wired to a different clock would report a rate
    unrelated to the elapsed time the verifier measures.
    """
    clock = clock or _Clock()

    def reply(sysid, attempt):
        if sysid not in sysids:
            return None
        return _StreamedBoot(sysid, clock, speedup, base=base)
    return clock, reply


#: Captured before anything patches it, so the guard below can still answer
#: honestly on threads it is not policing.
_REAL_TIME = time.time


def _no_wall_clock():
    """Build a time.time() stand-in that makes a wall-clock read in verification fatal.

    Not just an assertion aid — it removes a nasty failure mode. A deadline built
    from the real epoch is unreachable by the fake monotonic clock, so a stray
    wall-clock read does not fail a test, it HANGS one, and which test hangs
    depends on file order. Raising turns that into an immediate, named failure in
    whichever test touches it first.

    Scoped to the CALLING thread, and that is not a detail. swarm_run does
    ``import time``, so ``patch.object(m.time, "time", ...)`` replaces time.time
    for the WHOLE PROCESS — any background thread belonging to any other test that
    happens to be running (a logger, a relay, a poller) would hit this guard and
    die, surfacing as an unhandled-thread-exception warning blamed on whichever
    test was unlucky. A cross-test flake is exactly what this file exists to
    prevent, so the assertion applies to the thread under test and every other
    thread gets the real clock.
    """
    owner = threading.current_thread()

    def guard():
        if threading.current_thread() is owner:
            raise AssertionError("_verify_swarm read the wall clock")
        return _REAL_TIME()
    return guard


def _verify(m, conn, chat=0, instances=3, speedup=1, timeout=5,
            param_timeout=PARAM_T, boot_timeout=BOOT_T, launched_at=LAUNCHED_AT,
            clock=None, min_span=MIN_SPAN_T, boot_interval=0.0):
    """Run a verification against *conn* on a deterministic clock.

    All three budgets — *timeout*, *param_timeout*, *boot_timeout* — are in FAKE
    seconds and are spent by clock READS, so they have to be sized against the
    clock's step. A coarse-step clock can blow through an early phase's budget
    before it reads a single message, which returns at the speed gate and leaves
    a later phase looking like it never ran.

    *boot_interval* defaults to 0, i.e. re-request the boot clock every iteration.
    That is the faithful stand-in for a real vehicle, which streams boot-clock
    messages continuously rather than only when asked, and it is what gives the
    rate gate more than one sample to work with: at the production interval of
    BOOT_REQUEST_INTERVAL seconds a fake window only fits ONE round, so every
    sys_id would have a single sample, zero span, and no measurable rate. Pass
    ``None`` to leave the real constant alone.
    """
    interval = (patch.object(m, "BOOT_REQUEST_INTERVAL", boot_interval)
                if boot_interval is not None else contextlib.nullcontext())
    with patch("pymavlink.mavutil.mavlink_connection", return_value=conn), \
         patch.object(m.time, "time", _no_wall_clock()), \
         interval, \
         patch.object(m.time, "monotonic", clock or _Clock()):
        return m._verify_swarm(chat, instances, speedup, launched_at,
                               timeout=timeout, param_timeout=param_timeout,
                               boot_timeout=boot_timeout, min_span=min_span)


class TestHeartbeatVerification(unittest.TestCase):
    def test_all_streaming_and_correct_speedup_passes(self):
        m = _load_swarm()
        # chat 0 -> sysids 1,2,3; 255 is the router heartbeat and must be ignored.
        clock, boots = _healthy(speedup=1)
        conn = _FakeConn([_hb(1), _hb(255), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}), boots)
        r = _verify(m, conn, clock=clock)
        self.assertEqual(set(r.heartbeats), {1, 2, 3})
        self.assertEqual(r.speedups, {1: 1.0, 2: 1.0, 3: 1.0})
        self.assertIsNone(r.error)
        self.assertEqual(set(r.boot_evidence), {1, 2, 3})
        self.assertEqual(r.stale, {})
        self.assertEqual(set(r.rates), {1, 2, 3})
        for sysid, rate in r.rates.items():
            self.assertAlmostEqual(rate, 1.0, delta=0.25, msg=f"sys_id {sysid}")
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_missing_instance_fails_and_skips_param_phase(self):
        m = _load_swarm()
        # sysid 2 never streams. Don't even ask for the speedup — the swarm is
        # already known bad, and asking would just burn the param timeout.
        conn = _FakeConn([_hb(1), _hb(3)], _responder({1: 1.0, 3: 1.0}))
        r = _verify(m, conn, timeout=0.3)
        self.assertEqual(set(r.heartbeats), {1, 3})
        self.assertEqual(r.speedups, {})
        self.assertEqual(conn.requests, [])
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertIn("never streamed", m._describe_failure(r, {1, 2, 3}, 1))

    def test_ignores_foreign_sysids(self):
        m = _load_swarm()
        # Another chat's vehicle (sysid 7) must not count toward chat 0 — neither
        # its heartbeat nor its parameter reply. The foreign reply is injected
        # AFTER the drain (via the responder), so the source filter is what has
        # to reject it — queueing it up front would only prove the drain works.
        conn = _FakeConn([_hb(1), _hb(7), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}, extra=_param(7, 10.0)))
        r = _verify(m, conn)
        self.assertEqual(set(r.heartbeats), {1, 2, 3})
        self.assertNotIn(7, r.speedups)
        self.assertEqual(r.speedups, {1: 1.0, 2: 1.0, 3: 1.0})

    def test_bind_failure_fails_the_attempt(self):
        m = _load_swarm()
        # Used to "assume up" and return the full expected set: a launch that
        # verified NOTHING reported success.
        with patch("pymavlink.mavutil.mavlink_connection", side_effect=OSError("busy")):
            r = m._verify_swarm(0, 3, 1, LAUNCHED_AT, timeout=5,
                                param_timeout=PARAM_T, boot_timeout=BOOT_T)
        self.assertEqual(set(r.heartbeats), set())
        self.assertEqual(r.speedups, {})
        self.assertIsNotNone(r.error)
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertIn("verify port", m._describe_failure(r, {1, 2, 3}, 1))


class TestSpeedupReadback(unittest.TestCase):
    """The gate that catches a swarm silently running at the wrong sim speed."""

    def test_requests_go_to_every_expected_sysid(self):
        m = _load_swarm()
        from pymavlink import mavutil
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)], _responder({1: 1.0, 2: 1.0, 3: 1.0}))
        _verify(m, conn)
        self.assertEqual({r[0] for r in conn.requests}, {1, 2, 3})
        # One ask each: a vehicle that already answered is not asked again.
        self.assertEqual(len(conn.requests), 3)
        for sysid, comp, name, index in conn.requests:
            self.assertEqual(comp, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
            self.assertEqual(name, b"SIM_SPEEDUP")
            self.assertEqual(index, -1)

    def test_mismatched_speedup_fails(self):
        m = _load_swarm()
        # The exact contamination: asked for 1, swarm came up at 10.
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)], _responder({1: 10.0, 2: 10.0, 3: 10.0}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(r.speedups, {1: 10.0, 2: 10.0, 3: 10.0})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        msg = m._describe_failure(r, {1, 2, 3}, 1)
        self.assertIn("SIM_SPEEDUP mismatch", msg)
        self.assertIn("sys_id 1=10", msg)

    def test_one_vehicle_off_speed_fails_the_whole_swarm(self):
        m = _load_swarm()
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)], _responder({1: 1.0, 2: 10.0, 3: 1.0}))
        r = _verify(m, conn, speedup=1)
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        msg = m._describe_failure(r, {1, 2, 3}, 1)
        # The mismatch must be reported as a mismatch, not conflated with silence.
        self.assertIn("mismatch", msg)
        self.assertNotIn("never answered", msg)
        self.assertIn("sys_id 2=10", msg)

    def test_silent_vehicle_is_not_reported_as_mismatch(self):
        m = _load_swarm()
        # sysid 3 heartbeats but never answers the read-back.
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)], _responder({1: 1.0, 2: 1.0}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.speedups), {1, 2})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        msg = m._describe_failure(r, {1, 2, 3}, 1)
        self.assertIn("never answered SIM_SPEEDUP", msg)
        self.assertIn("[3]", msg)

    def test_other_parameter_from_the_right_vehicle_does_not_count(self):
        m = _load_swarm()
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}, name=b"SIM_RATE_HZ"))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(r.speedups, {})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_router_sysid_reply_does_not_count(self):
        m = _load_swarm()
        # The router (sysid 255) answering must not stand in for the vehicle that
        # stayed silent. Injected after the drain so the filter is what rejects it.
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0}, extra=_param(255, 1.0)))
        r = _verify(m, conn, speedup=1)
        self.assertNotIn(255, r.speedups)
        self.assertEqual(set(r.speedups), {1, 2})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_late_reply_does_not_overwrite_the_first(self):
        m = _load_swarm()
        # First-reply-wins. Last-wins would be fail-OPEN: neuter the drain and a
        # stale 10.0 is silently replaced by the fresh 1.0 that lands after it.
        # This is the committed form of that mutation check.
        with patch.object(m, "_drain"):
            conn = _FakeConn([_hb(1), _hb(2), _hb(3), _param(1, 10.0)],
                             _responder({1: 1.0, 2: 1.0, 3: 1.0}))
            r = _verify(m, conn, speedup=1)
        self.assertEqual(r.speedups[1], 10.0)
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_unanswered_vehicle_is_asked_again(self):
        m = _load_swarm()
        # UDP: a request or its reply can simply be dropped. A vehicle that only
        # answers the second ask must still verify.
        clock, boots = _healthy(speedup=1)
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}, answer_on=2), boots)
        with patch.object(m, "PARAM_RESEND_INTERVAL", 0.0):
            r = _verify(m, conn, speedup=1, clock=clock)
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))
        # Every vehicle really was asked more than once — drop the re-send and
        # this swarm never verifies.
        for sysid in (1, 2, 3):
            self.assertGreaterEqual(len([r for r in conn.requests if r[0] == sysid]), 2)

    def test_nul_padded_param_name_is_accepted(self):
        m = _load_swarm()
        # The wire field is NUL-padded to 16 chars; a naive == would reject it.
        clock, boots = _healthy(speedup=1)
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0},
                                    name=b"SIM_SPEEDUP\x00\x00\x00\x00\x00"), boots)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_stale_reply_queued_before_the_request_is_drained(self):
        m = _load_swarm()
        # A datagram from the PREVIOUS attempt's swarm, still in the socket buffer
        # when this attempt's heartbeats complete. It must not be mistaken for
        # this attempt's answer.
        clock, boots = _healthy(speedup=1)
        conn = _FakeConn([_hb(1), _hb(2), _hb(3), _param(1, 10.0)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}), boots)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertEqual(r.speedups[1], 1.0)
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_socket_error_fails_the_attempt_instead_of_raising(self):
        m = _load_swarm()
        # Windows raises ConnectionResetError on a UDP socket after an ICMP
        # port-unreachable. It must fail the attempt, not escape as a traceback
        # that skips the caller's teardown.
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)])
        conn.mav.param_request_read_send.side_effect = ConnectionResetError("icmp")
        r = _verify(m, conn, speedup=1)
        self.assertIsNotNone(r.error)
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertTrue(conn.closed)

    def test_connection_is_closed_on_every_path(self):
        m = _load_swarm()
        for name, conn in (
            ("pass", _FakeConn([_hb(1), _hb(2), _hb(3)], _responder({1: 1.0, 2: 1.0, 3: 1.0}),
                               _boot_responder(FRESH))),
            ("mismatch", _FakeConn([_hb(1), _hb(2), _hb(3)], _responder({1: 9.0, 2: 9.0, 3: 9.0}))),
            ("param timeout", _FakeConn([_hb(1), _hb(2), _hb(3)])),
            ("no heartbeat", _FakeConn([])),
            # The freshness phase is the last thing holding the socket, so it is
            # the newest way to leak it.
            ("boot evidence timeout", _FakeConn([_hb(1), _hb(2), _hb(3)],
                                                _responder({1: 1.0, 2: 1.0, 3: 1.0}))),
            ("stale", _FakeConn([_hb(1), _hb(2), _hb(3)],
                                _responder({1: 1.0, 2: 1.0, 3: 1.0}),
                                _boot_responder({1: 9e6, 2: 9e6, 3: 9e6}))),
        ):
            with self.subTest(name):
                _verify(m, conn, timeout=0.3)
                # The next attempt binds this same port — a leaked socket would
                # make the swarm unable to recover.
                self.assertTrue(conn.closed)


class TestBootFreshness(unittest.TestCase):
    """The gate that catches a swarm cleanup() failed to kill.

    Every vehicle here reports the REQUESTED speed, so the SIM_SPEEDUP gate above
    passes in all of these — that is the whole point. Only the boot clock can
    tell a fresh instance from a survivor.
    """

    def _fresh_conn(self, boot_responder=None, speeds=None):
        return _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder(speeds or {1: 1.0, 2: 1.0, 3: 1.0}),
                         boot_responder if boot_responder is not None
                         else _boot_responder(FRESH))

    def test_requests_system_time_from_every_expected_sysid(self):
        m = _load_swarm()
        from pymavlink import mavutil
        conn = self._fresh_conn()
        _verify(m, conn)
        self.assertEqual({r[0] for r in conn.boot_requests}, {1, 2, 3})
        for sysid, comp, command, msgid in conn.boot_requests:
            self.assertEqual(comp, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
            self.assertEqual(command, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)
            self.assertEqual(msgid, m.SYSTEM_TIME_MSG_ID)

    def test_survivor_of_an_earlier_swarm_fails(self):
        m = _load_swarm()
        # The whole point: right sys_ids, right speed, but a boot clock far older
        # than this attempt. Before this gate the launch reported success.
        #
        # The survivor's clock RUNS, at the requested rate — that is what a real
        # leftover process looks like, and it is what makes this test prove the
        # boot-age gate specifically. A frozen boot clock would be caught by the
        # rate gate first and this would pass for the wrong reason.
        clock, boots = _healthy(speedup=1, base=4.0e6)
        conn = self._fresh_conn(boots)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertEqual(r.speedups, {1: 1.0, 2: 1.0, 3: 1.0})
        self.assertEqual(set(r.stale), {1, 2, 3})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        msg = m._describe_failure(r, {1, 2, 3}, 1)
        self.assertIn("not booted by this launch", msg)
        self.assertIn("sys_id 1 time_boot_ms=4000", msg)

    def test_one_stale_vehicle_fails_the_whole_swarm(self):
        m = _load_swarm()
        # Partial replacement is the dangerous case: two fresh, one survivor.
        conn = self._fresh_conn(_boot_responder({1: 100.0, 2: 4.0e6, 3: 100.0}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.stale), {2})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertIn("sys_id 2", m._describe_failure(r, {1, 2, 3}, 1))

    def test_no_boot_evidence_fails_instead_of_passing(self):
        m = _load_swarm()
        # Heartbeat and SIM_SPEEDUP both pass; nobody reports a boot clock. Must
        # fail closed — "we could not look" is not "we looked and it was fine".
        conn = self._fresh_conn(_boot_responder({}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.boot_evidence), set())
        self.assertEqual(r.stale, {})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertIn("never reported SYSTEM_TIME",
                      m._describe_failure(r, {1, 2, 3}, 1))

    def test_silent_vehicle_is_not_reported_as_stale(self):
        m = _load_swarm()
        # sys_id 3 answers the speed read-back but never its boot clock. That is
        # "we could not look at 3", not "3 is a survivor".
        conn = self._fresh_conn(_boot_responder({1: 100.0, 2: 100.0}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.boot_evidence), {1, 2})
        msg = m._describe_failure(r, {1, 2, 3}, 1)
        self.assertIn("never reported SYSTEM_TIME", msg)
        self.assertIn("[3]", msg)
        self.assertNotIn("not booted by this launch", msg)

    def test_bound_scales_with_the_requested_speedup(self):
        m = _load_swarm()
        # A sim clock 20s in is impossible 0.1s after a 1x launch, and entirely
        # normal after a 10x one. An unscaled threshold gets one of these wrong.
        for speedup, stale in ((1, True), (10, False)):
            with self.subTest(speedup=speedup):
                # The clock has to RUN at the speed being asked for, or the
                # not-stale arm fails on the rate gate instead of proving the
                # bound scaled. base=20_000 is the uptime under test.
                clock, boots = _healthy(speedup=speedup, base=20_000.0)
                conn = self._fresh_conn(boots,
                                        speeds={1: speedup, 2: speedup, 3: speedup})
                r = _verify(m, conn, speedup=speedup, clock=clock)
                self.assertEqual(bool(r.stale), stale)
                self.assertEqual(m._verification_passed(r, {1, 2, 3}, speedup),
                                 not stale)

    def test_a_fresh_sample_does_not_excuse_a_later_stale_one(self):
        m = _load_swarm()
        # Worst-sample-wins. The old and new process can BOTH stream under one
        # sys_id; first-wins (or last-wins) would let whichever arrived at the
        # convenient moment decide. Here sys_id 1 answers fresh, then its twin
        # answers stale on the next round.
        def boot_reply(sysid, attempt):
            if sysid != 1:
                return _boot(sysid, 100.0)
            return _boot(1, 100.0) if attempt == 1 else _boot(1, 4.0e6)

        conn = self._fresh_conn(boot_reply)
        with patch.object(m, "BOOT_REQUEST_INTERVAL", 0.0):
            r = _verify(m, conn, speedup=1)
        self.assertIn(1, r.boot_evidence)      # it did report a boot clock...
        self.assertEqual(set(r.stale), {1})    # ...and it also betrayed a twin
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_later_larger_boot_sample_does_not_excuse_earlier_stale_one(self):
        m = _load_swarm()
        # A 0.75x clock remains inside the measured-rate tolerance. Its boot
        # age is initially impossible for this launch, then falls inside the
        # growing requested-speed bound while its boot counter keeps rising.
        clock = _Clock()
        readings = {}

        class ObservedBoot(_StreamedBoot):
            @property
            def time_boot_ms(self):
                value = super().time_boot_ms
                readings.setdefault(self._sysid, []).append((clock.t, value))
                return value

        def boots(sysid, _attempt):
            return ObservedBoot(sysid, clock, 0.75, base=2340.0)

        conn = self._fresh_conn(boots)
        r = _verify(m, conn, speedup=1, clock=clock)

        self.assertEqual(r.boot_evidence, {1, 2, 3})
        self.assertEqual(r.rates, {1: 0.75, 2: 0.75, 3: 0.75})
        self.assertTrue(m._rate_matches(r.rates[1], 1))
        # The final value is larger yet plausible at its own receipt time;
        # otherwise the old last-sample rule would also reject this launch.
        last_wall, last_boot = readings[1][-1]
        self.assertLessEqual(
            last_boot,
            m._boot_limit_ms(1, LAUNCHED_AT, last_wall + clock.step),
        )
        self.assertEqual(set(r.stale), {1, 2, 3})
        # The reported value is the largest OFFENDING sample, not the final
        # (larger but plausible) boot reading.
        self.assertAlmostEqual(r.stale[1], 2550.0)
        self.assertLess(r.stale[1], last_boot)
        self.assertIn(
            "not booted by this launch", m._describe_failure(r, {1, 2, 3}, 1)
        )
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_slow_in_tolerance_new_swarm_still_passes(self):
        m = _load_swarm()
        clock, boots = _healthy(speedup=0.75, base=100.0)
        r = _verify(m, self._fresh_conn(boots), speedup=1, clock=clock)

        self.assertEqual(r.stale, {})
        self.assertEqual(r.rates, {1: 0.75, 2: 0.75, 3: 0.75})
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_worst_sample_is_the_one_reported(self):
        m = _load_swarm()
        # Keeping the max is what tells an operator HOW old the survivor is.
        def boot_reply(sysid, attempt):
            if sysid != 1:
                return _boot(sysid, 100.0)
            return _boot(1, 4.0e6) if attempt == 1 else _boot(1, 9.0e6)

        conn = self._fresh_conn(boot_reply)
        with patch.object(m, "BOOT_REQUEST_INTERVAL", 0.0):
            r = _verify(m, conn, speedup=1)
        self.assertEqual(r.stale[1], 9.0e6)

    def test_every_sysid_is_asked_again_even_after_answering(self):
        m = _load_swarm()
        # Asking only the sys_ids still missing would stop probing one the moment
        # a fresh sample arrived — exactly when its stale twin is still unheard.
        conn = self._fresh_conn()
        with patch.object(m, "BOOT_REQUEST_INTERVAL", 0.0):
            _verify(m, conn, speedup=1)
        for sysid in (1, 2, 3):
            self.assertGreaterEqual(
                len([r for r in conn.boot_requests if r[0] == sysid]), 2)

    def test_foreign_sysid_boot_time_is_ignored(self):
        m = _load_swarm()
        # Another chat's vehicle (7) and the router (255) must neither satisfy
        # our completeness nor condemn us as stale.
        conn = self._fresh_conn(
            _boot_responder({1: 100.0, 2: 100.0}, extra=_boot(7, 4.0e6)))
        r = _verify(m, conn, speedup=1)
        self.assertNotIn(7, r.boot_evidence)
        self.assertNotIn(7, r.stale)
        self.assertEqual(set(r.boot_evidence), {1, 2})

    def test_non_autopilot_component_does_not_count(self):
        m = _load_swarm()
        # A gimbal under the SAME sys_id keeps its own boot clock, so it can
        # neither prove the vehicle fresh nor prove it stale.
        conn = self._fresh_conn(_boot_responder(FRESH, comp=154))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.boot_evidence), set())
        self.assertEqual(r.stale, {})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_attitude_can_condemn_but_not_absolve(self):
        m = _load_swarm()
        # ATTITUDE carries the autopilot's own boot clock, so a stale one is
        # proof of a survivor — but only SYSTEM_TIME satisfies completeness.
        conn = self._fresh_conn(
            _boot_responder({1: 4.0e6, 2: 4.0e6, 3: 4.0e6}, mtype="ATTITUDE"))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.stale), {1, 2, 3})
        self.assertEqual(set(r.boot_evidence), set())

    def test_non_numeric_boot_field_is_not_evidence(self):
        m = _load_swarm()
        # A dialect or malformed packet that yields a non-numeric time_boot_ms
        # must not satisfy the gate: too-small is the direction that PASSES, so
        # junk here would be fail-open.
        conn = self._fresh_conn(_boot_responder({1: object(), 2: -1.0,
                                                 3: float("nan")}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.boot_evidence), set())
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_boolean_boot_field_is_not_evidence(self):
        m = _load_swarm()
        # Its own case because bool IS an int in Python: without the explicit
        # rejection, True sails through the numeric and range guards and becomes
        # a boot clock of 1ms — the freshest possible reading, i.e. fail-open.
        conn = self._fresh_conn(_boot_responder({1: True, 2: True, 3: True}))
        r = _verify(m, conn, speedup=1)
        self.assertEqual(set(r.boot_evidence), set())
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_stale_sample_left_in_the_buffer_is_drained(self):
        m = _load_swarm()
        # A boot sample from the PREVIOUS attempt's swarm, still queued when the
        # speed phase ends, must not condemn this attempt. It has to be injected
        # so that it OUTLIVES the speed phase — anything queued earlier is eaten
        # by the phases above, which would make this test prove nothing.
        def param_reply(sysid, attempt):
            reply = _param(sysid, 1.0)
            if sysid != max({1, 2, 3}):
                return reply
            # Rides out behind the last answer, so the speed loop stops before
            # reading it and it is still there when freshness starts.
            return _Pair(reply, _boot(1, 4.0e6))

        clock, boots = _healthy(speedup=1)
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)], param_reply, boots)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertEqual(r.stale, {})
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_bound_grows_with_the_time_since_launch(self):
        m = _load_swarm()
        # 90s of uptime is impossible right after a launch and unremarkable 100s
        # after one. Measuring the bound from "now" instead of from the launch
        # instant collapses it to the grace and condemns a healthy slow boot.
        boots = {1: 90_000.0, 2: 90_000.0, 3: 90_000.0}
        for launched_at, stale in ((LAUNCHED_AT, True),
                                   (LAUNCHED_AT - 100.0, False)):
            with self.subTest(launched_at=launched_at):
                conn = self._fresh_conn(_boot_responder(boots))
                r = _verify(m, conn, speedup=1, launched_at=launched_at)
                self.assertEqual(bool(r.stale), stale)

    def test_resend_happens_on_the_configured_interval(self):
        m = _load_swarm()
        # Uses the REAL BOOT_REQUEST_INTERVAL, not a patched one: the other
        # re-send test patches it to 0, so it cannot notice an interval so large
        # that no re-send ever fits inside the window. Deliberately NO assertion
        # on the constant's value — that would catch such a mutation before the
        # re-send path ran, and prove nothing about the path.
        conn = self._fresh_conn()
        # Coarse clock so the window really spans several request intervals. Every
        # budget is scaled to that step: they are all spent by clock reads now, so
        # a step-sized PARAM_T would expire phase 2 before it read one reply and
        # this phase would never run at all — which reads as "no re-sends" rather
        # than as the setup error it is.
        # boot_interval=None: this is the ONE test that must see the real
        # constant, so it opts out of the helper's default of re-requesting every
        # iteration — patching the interval here would defeat the whole point.
        _verify(m, conn, speedup=1, clock=_Clock(step=1.0),
                timeout=50.0, param_timeout=20.0, boot_interval=None,
                boot_timeout=10 * m.BOOT_REQUEST_INTERVAL)
        for sysid in (1, 2, 3):
            self.assertGreaterEqual(
                len([r for r in conn.boot_requests if r[0] == sysid]), 2)

    def test_socket_error_during_freshness_fails_the_attempt(self):
        m = _load_swarm()
        conn = self._fresh_conn()
        conn.mav.command_long_send.side_effect = ConnectionResetError("icmp")
        r = _verify(m, conn, speedup=1)
        self.assertIsNotNone(r.error)
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertTrue(conn.closed)

    def test_speedup_mismatch_skips_the_freshness_window(self):
        m = _load_swarm()
        # A swarm already known bad must not pay the boot window, and must keep
        # reporting the mismatch rather than a freshness reason.
        conn = self._fresh_conn(speeds={1: 10.0, 2: 10.0, 3: 10.0})
        r = _verify(m, conn, speedup=1)
        self.assertEqual(conn.boot_requests, [])
        self.assertIn("SIM_SPEEDUP mismatch", m._describe_failure(r, {1, 2, 3}, 1))


class TestMeasuredClockRate(unittest.TestCase):
    """The gate that catches what a parameter read cannot see.

    SIM_SPEEDUP is CONFIGURATION. A vehicle can report ``SIM_SPEEDUP=1`` and run
    its clock at 10x, and one did: a run logged ``chat 0: ... speedup=1 verified``
    and then flew a dive at an implied 261 m/s. Everything here is about the swarm
    answering the parameter question correctly and still being wrong.
    """

    def _conn(self, clock_speedup, reported=1.0):
        clock, boots = _healthy(speedup=clock_speedup)
        return clock, _FakeConn(
            [_hb(1), _hb(2), _hb(3)],
            _responder({1: reported, 2: reported, 3: reported}), boots)

    def test_right_parameter_but_wrong_clock_fails(self):
        m = _load_swarm()
        # THE bug. Every vehicle reports exactly the speed that was asked for, so
        # the parameter gate is satisfied — and the clock runs 10x too fast.
        clock, conn = self._conn(clock_speedup=10, reported=1.0)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertEqual(r.speedups, {1: 1.0, 2: 1.0, 3: 1.0})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        msg = m._describe_failure(r, {1, 2, 3}, 1)
        self.assertIn("MEASURED clock rate", msg)
        # The number itself has to be in the message: "wrong speed" sends an
        # operator looking at config, "10.0x" tells them the clock is the problem.
        self.assertIn("10.0", msg)

    def test_right_clock_passes(self):
        m = _load_swarm()
        clock, conn = self._conn(clock_speedup=1, reported=1.0)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_a_stopped_clock_fails(self):
        m = _load_swarm()
        # A constant time_boot_ms — the swarm is alive and correctly configured but
        # its sim clock is not advancing. Rate 0 is not "close to 1".
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}),
                         _boot_responder(FRESH))
        r = _verify(m, conn, speedup=1)
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))

    def test_an_unmeasurable_rate_fails_closed(self):
        m = _load_swarm()
        # Samples that do not span min_span produce NO rate. That must fail: "we
        # could not measure the clock" is not "the clock is fine". Fail-open here
        # would rebuild the silent pass this gate exists to remove.
        clock, conn = self._conn(clock_speedup=1, reported=1.0)
        r = _verify(m, conn, speedup=1, clock=clock, min_span=1e9)
        self.assertEqual(r.rates, {})
        self.assertFalse(m._verification_passed(r, {1, 2, 3}, 1))
        self.assertIn("no usable clock-rate measurement",
                      m._describe_failure(r, {1, 2, 3}, 1))

    def test_tolerance_absorbs_real_world_jitter(self):
        m = _load_swarm()
        # The measured healthy arms were 1.006x at 1x and 10.043x at 10x. A tight
        # comparison — the one the PARAMETER check uses — would reject both.
        for measured, requested in ((1.006, 1), (10.043, 10), (0.98, 1)):
            with self.subTest(measured=measured):
                self.assertTrue(m._rate_matches(measured, requested))

    def test_tolerance_still_rejects_an_order_of_magnitude(self):
        m = _load_swarm()
        for measured, requested in ((10.0, 1), (1.0, 10), (0.0, 1), (5.0, 10)):
            with self.subTest(measured=measured):
                self.assertFalse(m._rate_matches(measured, requested))

    def test_a_survivor_at_the_correct_speed_is_still_reported_as_stale(self):
        m = _load_swarm()
        # Ordering guard, the other direction. This is a REAL survivor: huge
        # uptime, but its clock ticks at exactly the requested rate, so the rate
        # gate is satisfied and the message must still name the survivor rather
        # than fall through to "unknown verification failure".
        clock, boots = _healthy(speedup=1, base=9e6)
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}), boots)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertTrue(r.stale)
        self.assertIn("not booted by this launch", m._describe_failure(r, {1, 2, 3}, 1))

    def test_an_over_speed_swarm_is_not_misreported_as_a_survivor(self):
        m = _load_swarm()
        # A clock running 10x too fast ALSO blows the boot-age bound, because it
        # outruns the line that bound draws — so this swarm is flagged stale even
        # though nothing survived. Reporting that first would send an operator
        # hunting a leftover process that does not exist, the same class of wrong
        # diagnosis this launcher has already cost once. The rate has to win.
        clock, conn = self._conn(clock_speedup=10, reported=1.0)
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertTrue(r.stale, "expected the over-speed clock to trip the bound")
        self.assertIn("MEASURED clock rate", m._describe_failure(r, {1, 2, 3}, 1))
        # ...and it must still be healable, or the commonest real fault — asking
        # for 1x and getting 10x — would never get its corrective write.
        self.assertTrue(m._speed_is_the_fault(r, {1, 2, 3}, 1))


class _HealConn:
    """Fake connection for the healing PARAM_SET.

    Separate from _FakeConn because the shape differs: there is no request index to
    model, and the behaviour under test is which echo the code is willing to treat
    as proof the write landed.
    """

    def __init__(self, echoes, answer_on=1, telemetry=True):
        #: sysid -> the value it echoes back. Absent means it stays silent.
        self.echoes = echoes
        self.answer_on = answer_on
        #: Seeded so the socket can learn a peer address, like a real one does
        #: from the first inbound datagram. Set telemetry=False to model a port
        #: nothing is streaming to.
        self.queue = [_hb(121)] if telemetry else []
        self.received = False
        self.sets = []
        self.closed = False
        self.mav = MagicMock()
        self.mav.param_set_send.side_effect = self._on_set

    def _on_set(self, sysid, comp, name, value, ptype):
        # A udpin socket has nowhere to send until a datagram has arrived; a write
        # before that is silently dropped, not an error. Modelled so a regression
        # that sends first shows up as an unacknowledged write rather than passing.
        if not self.received:
            return
        self.sets.append((sysid, comp, name, value, ptype))
        nth = len([s for s in self.sets if s[0] == sysid])
        if sysid in self.echoes and nth >= self.answer_on:
            self.queue.append(_param(sysid, self.echoes[sysid]))

    def recv_match(self, type=None, blocking=False, timeout=None):
        while self.queue:
            msg = self.queue.pop(0)
            self.received = True
            if type is None or msg.get_type() == type:
                return msg
        return None

    def close(self):
        self.closed = True


def _heal(m, conn, chat=0, expected=(1, 2, 3), speedup=1, timeout=0.4, clock=None):
    with patch("pymavlink.mavutil.mavlink_connection", return_value=conn), \
         patch.object(m.time, "monotonic", clock or _Clock()):
        return m._force_speedup(chat, set(expected), speedup, timeout=timeout)


class TestForceSpeedup(unittest.TestCase):
    """Writing SIM_SPEEDUP into the live swarm — the only lever that beats eeprom.

    ``--speedup N`` reaches ArduPilot as ``set_default_by_name``, and a DEFAULT is
    a no-op for a parameter already saved in eeprom. Since every SITL runs in a
    persistent instance directory, a saved SIM_SPEEDUP silently discarded every
    later ``--speedup`` request — which is why ``--speedup 1`` looked intermittent
    rather than simply broken. A PARAM_SET to a RUNNING vehicle goes through
    ``set_and_save`` after the eeprom load, so it sticks.
    """

    def test_writes_the_requested_value_to_every_instance(self):
        m = _load_swarm()
        from pymavlink import mavutil
        conn = _HealConn({1: 0.5, 2: 0.5, 3: 0.5})
        result = _heal(m, conn, speedup=0.5)
        self.assertTrue(result.all_echoes_matched, result.detail)
        self.assertTrue(result.write_attempted)
        self.assertEqual({s[0] for s in conn.sets}, {1, 2, 3})
        for sysid, comp, name, value, ptype in conn.sets:
            self.assertEqual(comp, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
            self.assertEqual(name, b"SIM_SPEEDUP")
            self.assertEqual(value, 0.5)
            # SIM_SPEEDUP is an AP_Float; an int type would be rejected or
            # truncated, and 1 vs 1.0 is exactly the case that must not truncate.
            self.assertEqual(ptype, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def test_only_an_acknowledged_echo_counts(self):
        m = _load_swarm()
        # sysid 2 echoes a DIFFERENT value: it has not taken the write. Counting
        # the send as success would rebuild the silent pass this all exists to
        # kill — the relaunch would then be just as blind as before.
        conn = _HealConn({1: 1.0, 2: 10.0, 3: 1.0})
        result = _heal(m, conn, speedup=1)
        self.assertFalse(result.all_echoes_matched)
        self.assertIn("2", result.detail)

    def test_a_silent_instance_fails_the_heal(self):
        m = _load_swarm()
        conn = _HealConn({1: 1.0, 3: 1.0})
        result = _heal(m, conn, speedup=1)
        self.assertFalse(result.all_echoes_matched)
        self.assertIn("did not acknowledge", result.detail)
        self.assertTrue(result.write_attempted)

    def test_resends_to_an_instance_that_has_not_answered(self):
        m = _load_swarm()
        # UDP: the write or its echo can be dropped outright.
        conn = _HealConn({1: 1.0, 2: 1.0, 3: 1.0}, answer_on=2)
        with patch.object(m, "HEAL_RESEND_INTERVAL", 0.0):
            result = _heal(m, conn, speedup=1)
        self.assertTrue(result.all_echoes_matched, result.detail)
        self.assertTrue(result.write_attempted)
        for sysid in (1, 2, 3):
            self.assertGreaterEqual(len([s for s in conn.sets if s[0] == sysid]), 2)

    def test_waits_for_telemetry_before_writing(self):
        m = _load_swarm()
        # A udpin socket learns where to send from the first inbound datagram, so
        # the write has to be preceded by a receive. The verification phases never
        # meet this because their heartbeat phase receives first; this function
        # binds its own socket.
        conn = _HealConn({1: 1.0, 2: 1.0, 3: 1.0}, telemetry=False)
        result = _heal(m, conn, speedup=1)
        self.assertFalse(result.all_echoes_matched)
        self.assertIn("no peer address", result.detail)
        self.assertEqual(conn.sets, [])
        self.assertFalse(result.write_attempted)

    def test_bind_failure_returns_a_verdict_instead_of_raising(self):
        m = _load_swarm()
        # Best-effort by design: failing to heal is no worse than the blind
        # relaunch that came before it, so it must not take the launch down.
        with patch("pymavlink.mavutil.mavlink_connection", side_effect=OSError("busy")):
            result = m._force_speedup(0, {1, 2, 3}, 1)
        self.assertFalse(result.all_echoes_matched)
        self.assertIn("verify port", result.detail)
        self.assertFalse(result.write_attempted)

    def test_socket_error_returns_a_verdict_instead_of_raising(self):
        m = _load_swarm()
        conn = _HealConn({})
        conn.mav.param_set_send.side_effect = ConnectionResetError("icmp")
        result = _heal(m, conn, speedup=1)
        self.assertFalse(result.all_echoes_matched)
        self.assertIn("heal socket error", result.detail)
        self.assertTrue(result.write_attempted)
        self.assertTrue(conn.closed)

    def test_socket_is_closed_on_every_path(self):
        m = _load_swarm()
        for name, echoes in (("ok", {1: 1.0, 2: 1.0, 3: 1.0}),
                             ("partial", {1: 1.0}),
                             ("silent", {})):
            with self.subTest(name):
                conn = _HealConn(echoes)
                _heal(m, conn, speedup=1)
                # The next attempt binds this same port.
                self.assertTrue(conn.closed)

    def test_peer_arriving_after_deadline_does_not_imply_a_send(self):
        m = _load_swarm()
        conn = _HealConn({})
        with patch.object(m.speed_control, "await_peer", return_value=True), \
             patch("pymavlink.mavutil.mavlink_connection", return_value=conn), \
             patch.object(m.time, "monotonic", side_effect=[100.0, 102.0]):
            result = m._force_speedup(0, {1}, 1, timeout=1)
        self.assertFalse(result.write_attempted)
        self.assertFalse(result.all_echoes_matched)
        self.assertIn("did not acknowledge", result.detail)
        self.assertEqual(conn.sets, [])
        self.assertTrue(conn.closed)


class TestSpeedIsTheFault(unittest.TestCase):
    """The guard on the heal: only write when speed is genuinely the problem."""

    def _result(self, m, **kw):
        base = dict(speedups={1: 10.0, 2: 10.0, 3: 10.0},
                    boot_evidence=frozenset({1, 2, 3}),
                    rates={1: 10.0, 2: 10.0, 3: 10.0})
        base.update(kw)
        return m.VerificationResult(frozenset({1, 2, 3}), **base)

    def test_true_when_the_reported_speed_is_wrong(self):
        m = _load_swarm()
        self.assertTrue(m._speed_is_the_fault(self._result(m), {1, 2, 3}, 1))

    def test_true_when_only_the_measured_rate_is_wrong(self):
        m = _load_swarm()
        # Correctly configured, wrong clock — the case a parameter read misses.
        r = self._result(m, speedups={1: 1.0, 2: 1.0, 3: 1.0})
        self.assertTrue(m._speed_is_the_fault(r, {1, 2, 3}, 1))

    def test_false_when_the_speed_is_right(self):
        m = _load_swarm()
        r = self._result(m, speedups={1: 1.0, 2: 1.0, 3: 1.0},
                         rates={1: 1.0, 2: 1.0, 3: 1.0})
        self.assertFalse(m._speed_is_the_fault(r, {1, 2, 3}, 1))

    def test_true_when_the_parameter_is_wrong_and_freshness_never_ran(self):
        m = _load_swarm()
        # THE live regression. _verify_swarm returns the moment the reported
        # SIM_SPEEDUP disagrees, deliberately not paying for the freshness window,
        # so boot_evidence and rates are both EMPTY exactly when the parameter is
        # wrong. Requiring boot evidence here made the write dead code: a real
        # `--speedup 1` launch reported `sys_id 121=10` on all three attempts and
        # never once tried to correct it.
        r = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        self.assertEqual(set(r.boot_evidence), set())
        self.assertEqual(r.rates, {})
        self.assertTrue(m._speed_is_the_fault(r, {1, 2, 3}, 1))

    def test_false_when_an_instance_never_streamed(self):
        m = _load_swarm()
        # The reading may not even belong to the swarm we launched, so writing
        # into it would teach us nothing and could hit someone else's vehicle.
        r = m.VerificationResult(frozenset({1, 2}), {1: 10.0, 2: 10.0},
                                 boot_evidence=frozenset({1, 2}),
                                 rates={1: 10.0, 2: 10.0})
        self.assertFalse(m._speed_is_the_fault(r, {1, 2, 3}, 1))

    def test_staleness_does_not_veto_the_write(self):
        m = _load_swarm()
        # A swarm running faster than requested trips the freshness bound as a side
        # effect, so vetoing on staleness would refuse to heal exactly the fault
        # this exists for. Safe to be liberal: the relaunch re-proves provenance
        # from scratch, so a write aimed at a real survivor costs one PARAM_SET
        # sent to a process that is about to be killed.
        r = self._result(m, stale={1: 9e6})
        self.assertTrue(m._speed_is_the_fault(r, {1, 2, 3}, 1))

    def test_false_when_verification_could_not_run(self):
        m = _load_swarm()
        r = m.VerificationResult(frozenset(), {}, error="could not bind verify port")
        self.assertFalse(m._speed_is_the_fault(r, {1, 2, 3}, 1))


def _good(m):
    """A result that passes every gate, for driving main()'s loop.

    ``rates`` is not decoration: without a measured rate per sys_id the result
    fails verification, because "could not measure the clock" is a failure and not
    a pass. Omitting it here would make every launch-loop test below silently
    exercise the 3-attempts-then-abort path instead of the success path.
    """
    return m.VerificationResult(frozenset({1, 2, 3}), {1: 1.0, 2: 1.0, 3: 1.0},
                                boot_evidence=frozenset({1, 2, 3}),
                                rates={1: 1.0, 2: 1.0, 3: 1.0})


class TestLaunchLoopFailsLoudly(unittest.TestCase):
    """The verifier's verdict has to actually decide the launch."""

    def _run_main(self, m, results, reg_error=None, retract_error=None,
                   status_outcome="recorded", relays=None, real_drain=False,
                   begin_error=None, heal=None,
                   launch_token=None, speedup=1, single_boot=False):
        if heal is None:
            heal = m.speed_control.SpeedCorrectionResult(True, True, "acknowledged")
        corrections = iter(heal) if isinstance(heal, list) else None
        argv = ["swarm_run.py", "--chat", "0", "--speedup", str(speedup)]
        if single_boot:
            argv.append("--single-boot")
        if launch_token is not None:
            argv.extend(("--launch-token", launch_token))
        # cleanup and reg hang off ONE parent so their relative order is
        # observable — see test_terminal_failure_retracts_after_cleanup.
        parent = MagicMock()
        order = []
        real_retire = m._retire_relays
        with patch.object(sys, "argv", argv), \
             patch.object(m, "ensure_fork_supports_companion_udp"), \
             patch.object(m, "atexit", parent.atexit), \
             patch.object(m, "cleanup", parent.cleanup), \
             patch.object(m, "_kill_launch") as kill, \
             patch.object(m, "_retire_relays") as drain, \
             patch.object(m, "reg", parent.reg) as reg, \
             patch.object(m, "_start_launcher") as start, \
             patch.object(m.time, "sleep"), \
             patch.object(m, "_verify_swarm", side_effect=results) as verify, \
             patch.object(m, "_force_speedup") as force, \
             patch("builtins.print") as out:
            # Heal and kill share one log, because "the write reached the vehicles
            # while they were still alive" is an ORDERING claim: after the kill the
            # PARAM_SET would land nowhere and the relaunch would be as blind as
            # the one this replaced.
            steps = []
            force.side_effect = lambda *a, **k: (
                steps.append("heal"), next(corrections) if corrections else heal
            )[1]
            kill.side_effect = lambda *a, **k: steps.append("kill")
            # Patch the launcher rather than `subprocess`: _start_launcher reads
            # THIS process's fd 1/2 to decide whether to pipe, and under pytest
            # capture those can be real files — which would start real relay
            # threads reading a mock. These tests are about the launch loop.
            proc = MagicMock()
            proc.wait.return_value = CHILD_EXIT_CODE
            # A sentinel relay list, so a test can prove the attempt's OWN
            # relays reach _kill_launch rather than an empty placeholder.
            relays = [MagicMock(name="relay")] if relays is None else relays
            start.return_value = (proc, relays)
            # `or DEFAULT` keeps the mock's own return_value as the exit code.
            proc.wait.side_effect = lambda *a, **k: order.append("wait") or DEFAULT
            if real_drain:   # let the REAL drain run, to prove it is not a verdict
                drain.side_effect = lambda *a, **k: (order.append("drain"),
                                                     real_retire(*a, **k))[1]
            else:
                drain.side_effect = lambda *a, **k: order.append("drain")
            reg.sitl_launch_lock.return_value.__enter__.return_value = True
            # A bare MagicMock would never equal "recorded", so every run would
            # look like a refused write.
            reg.record_sitl_status.return_value = status_outcome
            if begin_error is not None:
                reg.begin_sitl_launch.side_effect = begin_error
            if reg_error is not None:
                reg.record_sitl_status.side_effect = reg_error
            if retract_error is not None:
                reg.retract_sitl.side_effect = retract_error
            code = None
            try:
                m.main()
            except SystemExit as exc:
                code = exc.code
            self._reg = reg
            self._parent = parent
            self._proc = proc
            self._relays = relays
            self._drain = drain
            self._terminal_order = order
            self._force = force
            self._steps = steps
            self._start = start
            self._printed_calls = out.call_args_list
        printed = " ".join(str(c.args[0]) for c in out.call_args_list if c.args)
        return code, kill, verify, proc, printed

    def _status_calls(self):
        return self._reg.record_sitl_status.call_args_list

    def test_no_send_bind_failure_retries_through_the_real_boundary(self):
        m = _load_swarm()
        real_force = m._force_speedup
        with patch("pymavlink.mavutil.mavlink_connection", side_effect=OSError("busy")) as bind:
            # _run_main patches the facade; evaluate its real result for each correction.
            class Corrections(list):
                def __iter__(self):
                    for _ in range(2):
                        yield real_force(0, {1, 2, 3}, 1)
            self._run_main(m, [self._mis_speeded(m)] * 3, heal=Corrections([None]))
        self.assertEqual(bind.call_count, 2)
        self.assertEqual(self._force.call_count, 2)
        self.assertEqual(self._steps, ["heal", "kill", "heal", "kill", "kill"])
        self.assertTrue(all("CLONE_FROM_TEMPLATE=0" not in c.args[0]
                            for c in self._start.call_args_list))

    def test_retry_preservation_matrix(self):
        m = _load_swarm()
        result = m.speed_control.SpeedCorrectionResult
        no_send = result(False, False, "no peer")
        sent = result(True, False, "partial echo")
        acknowledged = result(True, True, "matching")
        bad = self._mis_speeded(m)
        non_speed = m.VerificationResult(frozenset(), {})
        cases = [
            ([bad]*3, [sent], [False, True, True], 1),
            ([bad]*3, [acknowledged], [False, True, True], 1),
            ([bad]*3, [no_send, no_send], [False, False, False], 2),
            ([bad]*3, [no_send, sent], [False, False, True], 2),
            ([non_speed, bad, bad], [sent], [False, False, True], 1),
            ([non_speed, non_speed, bad], [], [False, False, False], 0),
            ([non_speed]*3, [], [False, False, False], 0),
            ([bad, non_speed, bad], [sent], [False, True, True], 1),
            ([bad, _good(m)], [no_send], [False, False], 1),
        ]
        for verdicts, corrections, flags, count in cases:
            with self.subTest(flags=flags, count=count, corrections=corrections):
                self._run_main(m, verdicts, heal=corrections)
                self.assertEqual(self._force.call_count, count)
                self.assertEqual(["CLONE_FROM_TEMPLATE=0" in c.args[0]
                                  for c in self._start.call_args_list], flags)

    def test_correction_logs_describe_observations_on_correct_stream(self):
        m = _load_swarm()
        result = m.speed_control.SpeedCorrectionResult
        for correction, phrase, is_error in [
            (result(False, False, "bind"), "could NOT send SIM_SPEEDUP; next launch uses default instance setup", True),
            (result(True, False, "partial"), "PARAM_SET attempted; not all expected echoes matched; preserving instance state for next verification", True),
            (result(True, True, "echoes"), "matching echoes observed; preserving instance state for next verification", False),
        ]:
            with self.subTest(correction=correction):
                self._run_main(m, [self._mis_speeded(m), _good(m)], heal=correction)
                calls = [c for c in self._printed_calls if phrase in str(c.args[0])]
                self.assertEqual(len(calls), 1)
                self.assertEqual(calls[0].kwargs.get("file") is sys.stderr, is_error)
        self._run_main(m, [m.VerificationResult(frozenset(), {})] * 2 + [self._mis_speeded(m)])
        calls = [c for c in self._printed_calls if "no further correction attempted: no relaunch remains" in str(c.args[0])]
        self.assertEqual(len(calls), 1)
        self.assertIs(calls[0].kwargs.get("file"), sys.stderr)
        self._force.assert_not_called()

    def test_single_boot_never_heals_or_relaunches_and_unwinds_lock(self):
        m = _load_swarm()
        code, kill, verify, proc, printed = self._run_main(
            m, [self._mis_speeded(m)], single_boot=True)
        self.assertEqual(code, 1)
        self.assertEqual(self._start.call_count, 1)
        self.assertEqual(verify.call_count, 1)
        self._force.assert_not_called()
        self._reg.sitl_launch_lock.return_value.__exit__.assert_called_once()
        self.assertIn("stopping swarm", printed)
        self.assertNotIn("restarting swarm", printed)

    def _order(self, name):
        """Indices of *name* in the interleaved cleanup/registry call log."""
        return [i for i, c in enumerate(self._parent.mock_calls) if c[0] == name]

    def _mis_speeded(self, m):
        """Alive, fresh, correctly answering — and running at the wrong speed.

        The only fault a heal is allowed to act on.
        """
        return m.VerificationResult(frozenset({1, 2, 3}),
                                    {1: 10.0, 2: 10.0, 3: 10.0},
                                    boot_evidence=frozenset({1, 2, 3}),
                                    rates={1: 10.0, 2: 10.0, 3: 10.0})

    def test_a_verified_launch_publishes_the_MEASURED_rate_not_just_the_request(self):
        """Another session reading this slot must be able to see achieved speed.

        The requested speedup alone is a setting, and a swarm once reported
        ``SIM_SPEEDUP=1`` while flying at 10x. Recording only the request would
        leave every later reader with the same blind spot the measured gate was
        added to close.
        """
        m = _load_swarm()
        measured = m.VerificationResult(frozenset({1, 2, 3}),
                                        {1: 1.0, 2: 1.0, 3: 1.0},
                                        boot_evidence=frozenset({1, 2, 3}),
                                        rates={1: 1.06, 2: 1.05, 3: 1.07})
        code, kill, verify, proc, printed = self._run_main(m, [measured])
        self.assertEqual(code, CHILD_EXIT_CODE)
        call = self._status_calls()[-1]
        self.assertTrue(call.kwargs["verified"])
        self.assertEqual(call.kwargs["speedup"], 1)
        self.assertEqual(call.kwargs["measured_rates"],
                         {1: 1.06, 2: 1.05, 3: 1.07})
        # And the operator sees it too, because a line quoting only the request
        # is not evidence of anything.
        self.assertIn("measured clock 1=1.06x", printed)

    def test_a_failed_launch_publishes_no_measured_rate(self):
        # There is nothing to publish, and a rate carried over from a failed
        # attempt would describe a swarm that was rejected.
        m = _load_swarm()
        code, kill, verify, proc, printed = self._run_main(
            m, [self._mis_speeded(m)] * m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(code, 1)
        call = self._status_calls()[-1]
        self.assertFalse(call.kwargs["verified"])
        self.assertIsNone(call.kwargs.get("measured_rates"))

    def test_wrong_speed_is_written_to_the_live_swarm_before_the_kill(self):
        m = _load_swarm()
        # The fix for the flake. `--speedup N` only sets a DEFAULT, which a saved
        # eeprom value beats, so a blind relaunch boots from the very state that
        # just failed and all three attempts reproduce it. Writing to the LIVE
        # swarm (set_and_save) corrects the eeprom the relaunch will read.
        code, kill, verify, proc, printed = self._run_main(
            m, [self._mis_speeded(m), _good(m)])
        self.assertEqual(code, CHILD_EXIT_CODE)
        self.assertEqual(self._force.call_count, 1)
        # Order, not just occurrence: after the kill there is nothing to write to.
        self.assertEqual(self._steps[:2], ["heal", "kill"])
        # And it wrote the value that was REQUESTED, to the expected sys_ids.
        chat, expected, speedup = self._force.call_args.args[:3]
        self.assertEqual(speedup, 1)
        self.assertEqual(set(expected), {1, 2, 3})
        self.assertIn("matching echoes observed; preserving instance state for next verification", printed)

    def test_the_relaunch_after_a_heal_preserves_the_corrected_eeprom(self):
        m = _load_swarm()
        # Without CLONE_FROM_TEMPLATE=0 the write is pointless for every cloned
        # instance — which is most chats, the eval band included: run_swarm.sh
        # re-copies eeprom.bin from templates 1-3 on each launch and would undo it.
        code, kill, verify, proc, printed = self._run_main(
            m, [self._mis_speeded(m), _good(m)])
        cmds = [c.args[0] for c in self._start.call_args_list]
        self.assertEqual(len(cmds), 2)
        # First attempt syncs from the template as usual...
        self.assertNotIn("CLONE_FROM_TEMPLATE=0", cmds[0])
        # ...the corrective relaunch must not.
        self.assertIn("CLONE_FROM_TEMPLATE=0", cmds[1])
        # The requested speed still travels on every attempt.
        for cmd in cmds:
            self.assertIn("SPEEDUP=1 ", cmd)

    def test_fractional_speed_survives_parse_heal_relaunch_and_status(self):
        m = _load_swarm()
        corrected = m.VerificationResult(
            frozenset({1, 2, 3}),
            {1: 0.5, 2: 0.5, 3: 0.5},
            boot_evidence=frozenset({1, 2, 3}),
            rates={1: 0.5, 2: 0.5, 3: 0.5},
        )

        code, _kill, _verify, _proc, _printed = self._run_main(
            m,
            [self._mis_speeded(m), corrected],
            speedup=0.5,
        )

        self.assertEqual(code, CHILD_EXIT_CODE)
        self.assertEqual(self._force.call_args.args[2], 0.5)
        commands = [call.args[0] for call in self._start.call_args_list]
        self.assertEqual(len(commands), 2)
        # sim_vehicle.py accepts only an integer boot speed. The exact 0.5 is
        # carried by the REAL32 heal above and retained by the state-preserving
        # relaunch, while both process launches use the supported 1x floor.
        self.assertIn("SPEEDUP=1 ", commands[0])
        self.assertIn("SPEEDUP=1 ", commands[1])
        self.assertIn("CLONE_FROM_TEMPLATE=0", commands[1])
        status = self._status_calls()[-1]
        self.assertEqual(status.kwargs["speedup"], 0.5)
        self.assertEqual(
            status.kwargs["measured_rates"],
            {1: 0.5, 2: 0.5, 3: 0.5},
        )

    def test_a_relaunch_with_no_heal_still_syncs_from_the_template(self):
        m = _load_swarm()
        # Anti-drift is the default and must stay the default: only a heal earns
        # the right to keep per-instance state.
        short = m.VerificationResult(frozenset({1, 2}), {1: 1.0, 2: 1.0},
                                     boot_evidence=frozenset({1, 2}),
                                     rates={1: 1.0, 2: 1.0})
        code, kill, verify, proc, printed = self._run_main(m, [short, _good(m)])
        for cmd in (c.args[0] for c in self._start.call_args_list):
            self.assertNotIn("CLONE_FROM_TEMPLATE=0", cmd)

    def test_the_heal_happens_once_across_attempts(self):
        m = _load_swarm()
        # A second write cannot succeed where the first was acknowledged and still
        # did not take, so retrying it would only add noise to every failed launch.
        code, kill, verify, proc, printed = self._run_main(
            m, [self._mis_speeded(m)] * m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(code, 1)
        self.assertEqual(verify.call_count, m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(self._force.call_count, 1)

    def test_a_failed_heal_does_not_take_the_launch_down(self):
        m = _load_swarm()
        # Best-effort: a launch that cannot heal is no worse off than before the
        # heal existed, so it must still spend its remaining attempts.
        code, kill, verify, proc, printed = self._run_main(
            m, [self._mis_speeded(m), _good(m)], heal=m.speed_control.SpeedCorrectionResult(True, False, "no ack"))
        self.assertEqual(code, CHILD_EXIT_CODE)
        self.assertIn("PARAM_SET attempted; not all expected echoes matched; preserving instance state for next verification", printed)

    def test_no_heal_when_an_instance_never_streamed(self):
        m = _load_swarm()
        # Nothing proves the speed reading belongs to the swarm we launched, so a
        # write could land in someone else's vehicle and teach us nothing.
        short = m.VerificationResult(frozenset({1, 2}), {1: 1.0, 2: 1.0},
                                     boot_evidence=frozenset({1, 2}),
                                     rates={1: 1.0, 2: 1.0})
        code, kill, verify, proc, printed = self._run_main(m, [short, _good(m)])
        self.assertEqual(self._force.call_count, 0)

    def test_no_heal_for_a_survivor_running_at_the_right_speed(self):
        m = _load_swarm()
        # Stale but correctly speeded: nothing about the speed is wrong, so there
        # is nothing for a write to fix and the relaunch is the whole remedy.
        survivor = m.VerificationResult(
            frozenset({1, 2, 3}), {1: 1.0, 2: 1.0, 3: 1.0},
            boot_evidence=frozenset({1, 2, 3}), stale={1: 4.0e6},
            rates={1: 1.0, 2: 1.0, 3: 1.0})
        code, kill, verify, proc, printed = self._run_main(m, [survivor, _good(m)])
        self.assertEqual(self._force.call_count, 0)

    def test_a_good_launch_never_heals(self):
        m = _load_swarm()
        code, kill, verify, proc, printed = self._run_main(m, [_good(m)])
        self.assertEqual(code, CHILD_EXIT_CODE)
        self.assertEqual(self._force.call_count, 0)

    def test_the_success_line_quotes_the_measured_rate(self):
        m = _load_swarm()
        # A log line quoting only the REQUESTED speed is not evidence: the
        # parameter-only gate once printed "speedup=1 verified" for a swarm that
        # flew at 10x, and that line is what made the bad run look trustworthy.
        code, kill, verify, proc, printed = self._run_main(m, [_good(m)])
        self.assertIn("measured clock", printed)
        self.assertIn("1.00x", printed)

    def test_speedup_mismatch_retries_then_exits_nonzero(self):
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        code, kill, verify, proc, printed = self._run_main(m, [bad] * m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(code, 1)
        self.assertEqual(verify.call_count, m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(kill.call_count, m.MAX_LAUNCH_ATTEMPTS)
        # Never claim success, and name the real reason.
        self.assertNotIn("streaming telemetry", printed)
        self.assertIn("SIM_SPEEDUP mismatch", printed)
        # This console is about to close and take the message with it, so the
        # verdict has to reach the registry too — once, as a final verdict, not
        # once per retry.
        self.assertEqual(len(self._status_calls()), 1)
        kw = self._status_calls()[0].kwargs
        self.assertFalse(kw["verified"])
        self.assertIn("SIM_SPEEDUP mismatch", kw["error"])

    def test_recovers_when_a_relaunch_comes_up_at_the_right_speed(self):
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        good = _good(m)
        code, kill, verify, proc, printed = self._run_main(m, [bad, good])
        self.assertEqual(verify.call_count, 2)
        self.assertEqual(kill.call_count, 1)
        # The failed attempt's own relays must be handed to _kill_launch, or a
        # zombie relay from attempt N could still write over attempt N+1.
        self.assertIs(kill.call_args.args[2], self._relays)
        self.assertIn("streaming telemetry at SIM_SPEEDUP=1", printed)
        # main() ends by waiting on the launcher process, not by exiting nonzero.
        self.assertEqual(code, CHILD_EXIT_CODE)
        # Recorded once the verifier passed — not on the failed first attempt.
        self.assertEqual(len(self._status_calls()), 1)
        self.assertEqual(self._status_calls()[0].kwargs["verified"], True)
        self.assertEqual(self._status_calls()[0].kwargs["speedup"], 1)

    def test_the_terminal_path_drains_after_wait_and_keeps_the_child_exit_code(self):
        """A verified swarm runs for hours; when it finally ends, the pipe tail
        still has to reach the log — and the drain must not touch the verdict."""
        m = _load_swarm()
        good = _good(m)
        code, _kill, _verify, proc, _printed = self._run_main(m, [good])
        self.assertEqual(self._terminal_order, ["wait", "drain"])
        self.assertEqual(code, CHILD_EXIT_CODE)
        self._drain.assert_called_once_with(0, self._relays, reaped=True)

    def test_the_terminal_drain_runs_even_when_nothing_was_relayed(self):
        """Guards against a future `if relays:` quietly dropping the drain — the
        console/pipe case has no relays and must still take the same path."""
        m = _load_swarm()
        good = _good(m)
        self._run_main(m, [good], relays=[])
        self._drain.assert_called_once_with(0, [], reaped=True)

    def test_a_relay_that_will_not_drain_cannot_change_the_exit_code(self):
        """End to end, through the REAL drain: a swarm that heartbeated and
        reported the right SIM_SPEEDUP stays a passed launch even when its log
        relay is wedged. Verification is authoritative about the swarm; the
        relay is not — the same rule that keeps log text out of the gate.
        """
        m = _load_swarm()
        good = _good(m)

        class _NeverFinishes:
            def join(self, timeout=None):
                pass

            def is_alive(self):
                return True

        relay = m._Relay(fd=1, name="stdout", source=MagicMock(),
                         thread=_NeverFinishes())
        with patch.object(m, "RELAY_DRAIN_TIMEOUT", 0.01):
            code, _kill, _verify, proc, printed = self._run_main(
                m, [good], relays=[relay], real_drain=True)

        self.assertEqual(code, CHILD_EXIT_CODE)
        self.assertTrue(relay.retiring, "the wedged relay was not silenced")
        # Loud, but only as a diagnostic.
        self.assertIn("did not reach EOF", printed)
        self.assertIn("streaming telemetry at SIM_SPEEDUP=1", printed)
        self.assertEqual(self._status_calls()[0].kwargs["verified"], True)

    def test_registry_write_failure_does_not_turn_a_good_launch_bad(self):
        m = _load_swarm()
        good = _good(m)
        # The verdict is already decided; a registry that can't be written must
        # not change it in either direction.
        code, kill, verify, proc, printed = self._run_main(
            m, [good], reg_error=OSError("locked"))
        self.assertEqual(code, CHILD_EXIT_CODE)
        self.assertIn("streaming telemetry at SIM_SPEEDUP=1", printed)
        self.assertIn("could not record launch status", printed)

    def test_registry_write_failure_does_not_rescue_a_bad_launch(self):
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        code, kill, verify, proc, printed = self._run_main(
            m, [bad] * m.MAX_LAUNCH_ATTEMPTS, reg_error=OSError("locked"))
        self.assertEqual(code, 1)

    def test_terminal_failure_retracts_after_cleanup(self):
        # Leaving the entry as-is let a fast failure keep reading as a LIVE
        # swarm: _is_alive() answers True for the whole startup grace before it
        # ever looks at a pid.
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        code, _, _, _, printed = self._run_main(m, [bad] * m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(code, 1)
        retract = self._reg.retract_sitl
        self.assertEqual(retract.call_count, 1)
        self.assertEqual(retract.call_args.args, (0,))
        # Our own pid: retract_sitl refuses to touch another supervisor's state.
        self.assertEqual(retract.call_args.kwargs["sitl_pid"], os.getpid())
        # Verdict -> teardown -> drop the exit hook -> retract, in that order.
        # Retracting while a teardown can still fire would advertise the slot as
        # SITL-free while something is still killing by sys_id, so a concurrent
        # claim could lose its brand-new swarm to it. The last cleanup is the
        # terminal one (the first runs at startup), and the atexit hook would
        # otherwise run one MORE cleanup after sys.exit — after the retraction.
        after_cleanup = max(self._order("cleanup"))
        self.assertLess(self._order("reg.record_sitl_status")[0], after_cleanup)
        self.assertLess(after_cleanup, self._order("atexit.unregister")[0])
        self.assertLess(self._order("atexit.unregister")[0],
                        self._order("reg.retract_sitl")[0])
        # The hook that was registered, not some other callable.
        self.assertIs(self._parent.atexit.unregister.call_args.args[0],
                      self._parent.atexit.register.call_args.args[0])
        self.assertIn("registry retract=", printed)

    def test_status_is_written_as_this_supervisor(self):
        # The registry refuses a verdict it can't tie to the supervisor the entry
        # records, so the pid has to travel with the write.
        m = _load_swarm()
        good = _good(m)  # carries the boot evidence the freshness gate requires
        self._run_main(m, [good])
        self.assertEqual(self._status_calls()[0].kwargs["sitl_pid"], os.getpid())

    def test_a_refused_status_write_is_not_silent(self):
        # "not-ours" is not an error — it means the entry no longer records us,
        # so --list will not carry our reason. The operator has to be told, and
        # told something different from "the registry write failed".
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        code, _, _, _, printed = self._run_main(
            m, [bad] * m.MAX_LAUNCH_ATTEMPTS, status_outcome="not-ours")
        self.assertEqual(code, 1)
        self.assertIn("launch status not recorded (not-ours)", printed)

    def test_a_recorded_status_write_says_nothing(self):
        m = _load_swarm()
        good = _good(m)  # carries the boot evidence the freshness gate requires
        _, _, _, _, printed = self._run_main(m, [good], status_outcome="recorded")
        self.assertNotIn("launch status not recorded", printed)

    def test_successful_launch_keeps_its_teardown_hook(self):
        # The hook is what tears the swarm down on exit; only the terminal path,
        # which has already cleaned up explicitly, may drop it.
        m = _load_swarm()
        good = _good(m)  # carries the boot evidence the freshness gate now requires
        self._run_main(m, [good])
        self._parent.atexit.unregister.assert_not_called()

    def test_successful_launch_never_retracts(self):
        m = _load_swarm()
        good = _good(m)  # carries the boot evidence the freshness gate now requires
        self._run_main(m, [good])
        self._reg.retract_sitl.assert_not_called()

    def test_retracted_swarm_is_not_retracted_once_per_attempt(self):
        # Each failed attempt kills its own swarm, but the claim is only
        # withdrawn once the whole launch is given up on.
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        good = _good(m)  # carries the boot evidence the freshness gate now requires
        self._run_main(m, [bad, good])
        self._reg.retract_sitl.assert_not_called()

    def test_retract_failure_does_not_change_the_verdict(self):
        # Same rule as the status write: the launch already failed, and an
        # unwritable registry must not rescue it. It must still say so.
        m = _load_swarm()
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        code, _, _, _, printed = self._run_main(
            m, [bad] * m.MAX_LAUNCH_ATTEMPTS, retract_error=OSError("locked"))
        self.assertEqual(code, 1)
        self.assertIn("could not retract registry SITL state", printed)
    def test_stale_swarm_retries_then_exits_nonzero(self):
        m = _load_swarm()
        # Right sys_ids, right speed, wrong processes. Before the freshness gate
        # this was reported as a successful launch.
        # rates present and correct: a real survivor runs at the requested speed,
        # so the launch must fail on PROVENANCE rather than on the clock.
        survivor = m.VerificationResult(
            frozenset({1, 2, 3}), {1: 1.0, 2: 1.0, 3: 1.0},
            boot_evidence=frozenset({1, 2, 3}), stale={1: 4.0e6},
            rates={1: 1.0, 2: 1.0, 3: 1.0})
        code, kill, verify, proc, printed = self._run_main(
            m, [survivor] * m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(code, 1)
        self.assertEqual(kill.call_count, m.MAX_LAUNCH_ATTEMPTS)
        self.assertNotIn("streaming telemetry", printed)
        self.assertIn("not booted by this launch", printed)
        self.assertIn("not booted by this launch",
                      self._status_calls()[0].kwargs["error"])

    def test_launch_instant_is_stamped_once_per_attempt(self):
        m = _load_swarm()
        # Each attempt gets its OWN reference instant. Hoisting the stamp out of
        # the retry loop would measure attempt 2 against attempt 1's launch, and
        # the bound would then grow by however long the failed attempt took —
        # which is exactly enough to let that attempt's survivors look fresh.
        # The fake clock advances per read, so a shared stamp shows up as equal
        # values here; real monotonic resolution is too coarse to see it.
        bad = m.VerificationResult(frozenset({1, 2, 3}), {1: 10.0, 2: 10.0, 3: 10.0})
        clock = _Clock(step=1.0)
        with patch.object(m.time, "monotonic", clock):
            code, kill, verify, proc, printed = self._run_main(m, [bad, _good(m)])
        stamps = [c.args[3] for c in verify.call_args_list]
        self.assertEqual(stamps, [LAUNCHED_AT + 1.0, LAUNCHED_AT + 2.0])

    def test_bind_failure_does_not_pass_the_launch(self):
        m = _load_swarm()
        blind = m.VerificationResult(frozenset(), {}, error="could not bind verify port 16000 (busy)")
        code, kill, verify, proc, printed = self._run_main(m, [blind] * m.MAX_LAUNCH_ATTEMPTS)
        self.assertEqual(code, 1)
        self.assertNotIn("streaming telemetry", printed)
        self.assertIn("could not bind verify port", printed)


class TestCleanupForceKills(unittest.TestCase):
    """A wedged SITL ignores SIGTERM, so cleanup must escalate to SIGKILL — else it
    keeps holding its serial0 port and the relaunched instance can't bind."""

    def test_escalates_sigterm_then_sigkill_for_each_sysid(self):
        m = _load_swarm()
        with patch.object(m, "_wsl") as wsl:
            m.cleanup(1)  # chat 1 -> sysids 4,5,6
        cmd = wsl.call_args[0][0]
        for s in (4, 5, 6):
            # A polite terminate for every instance...
            self.assertIn(f"pkill -f -- '[-]-sysid[[:space:]=]+{s}([[:space:]]|$)'", cmd)
            # ...and a forced kill for every instance.
            self.assertIn(
                f"pkill -KILL -f -- '[-]-sysid[[:space:]=]+{s}([[:space:]]|$)'",
                cmd,
            )
        # SIGTERM is issued before SIGKILL (grace period between).
        self.assertLess(
            cmd.index("pkill -f -- '[-]-sysid[[:space:]=]+4([[:space:]]|$)'"),
            cmd.index("pkill -KILL -f -- '[-]-sysid[[:space:]=]+4([[:space:]]|$)'"),
        )

    def test_cleanup_patterns_do_not_match_invoking_shell(self):
        m = _load_swarm()
        with patch.object(m, "_wsl") as wsl:
            m.cleanup(0)
        cmd = wsl.call_args[0][0]
        # `pkill -f` sees the cleanup shell's own command line. The regex text
        # must not contain the literal process marker it is trying to kill, or
        # the shell can terminate before the SIGKILL pass runs.
        self.assertNotIn("'--sysid 1", cmd)
        # Both spellings ArduPilot accepts are killed; the bracket escape still
        # keeps the pattern text from naming the process it hunts.
        self.assertNotIn("'--sysid=1", cmd)
        self.assertIn("'[-]-sysid[[:space:]=]+1([[:space:]]|$)'", cmd)
        self.assertNotIn("'mavlink-routerd", cmd)
        self.assertIn("'[m]avlink-routerd.*:15550([[:space:]]|$)'", cmd)

    def test_force_kills_only_this_chats_router(self):
        m = _load_swarm()
        with patch.object(m, "_wsl") as wsl:
            m.cleanup(1)
        cmd = wsl.call_args[0][0]
        # chat 1's monitor port is 15551; another chat's router must be untouched.
        self.assertIn("[m]avlink-routerd.*:15551([[:space:]]|$)", cmd)
        self.assertNotIn(":15550([[:space:]]|$)", cmd)
        self.assertNotIn(":15552([[:space:]]|$)", cmd)


if __name__ == "__main__":
    unittest.main()


class TestOneClockDomain(unittest.TestCase):
    """Verification measures elapsed time on ONE clock, and it is not the wall clock."""

    def test_verification_never_reads_the_wall_clock(self):
        # The regression lock for the flake this fixed. A wall-clock read in any
        # phase makes that phase's timeout REAL seconds while the tests' clock is
        # faked, so a loaded machine can expire a deadline the test never moved —
        # and the failure surfaces as the wrong reason, somewhere else entirely.
        # Making time.time() fatal here means a reintroduced read fails loudly
        # and deterministically, in the test that owns the invariant.
        m = _load_swarm()
        clock, boots = _healthy(speedup=1)
        conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                         _responder({1: 1.0, 2: 1.0, 3: 1.0}), boots)

        # _verify installs the guard for EVERY test; this one states the
        # invariant explicitly so it is greppable and cannot be dropped silently.
        r = _verify(m, conn, speedup=1, clock=clock)
        self.assertTrue(m._verification_passed(r, {1, 2, 3}, 1))

    def test_the_wall_clock_guard_still_fires_on_the_thread_under_test(self):
        # The guard is only worth having if it actually trips. It is scoped to one
        # thread so unrelated tests' background threads survive it — that scoping
        # must not have quietly turned it into a no-op where it matters.
        guard = _no_wall_clock()
        with self.assertRaises(AssertionError):
            guard()
        # ...and on any other thread it answers honestly instead of exploding.
        seen = {}
        thread = threading.Thread(target=lambda: seen.setdefault("t", guard()))
        thread.start()
        thread.join()
        self.assertIsInstance(seen["t"], float)

    def test_a_slow_speed_phase_does_not_steal_the_freshness_verdict(self):
        # The flake itself, made deterministic: phase 2 running out is reported
        # as a phase-2 problem, and must not be reachable just because the host
        # was busy. With one clock the phases advance together, so the freshness
        # reason survives however long the machine actually took.
        m = _load_swarm()
        for name, param_timeout, expected in (
            ("phase 2 completes", PARAM_T, "never reported SYSTEM_TIME"),
            ("phase 2 starved", 0.0, "never answered SIM_SPEEDUP"),
        ):
            with self.subTest(name):
                conn = _FakeConn([_hb(1), _hb(2), _hb(3)],
                                 _responder({1: 1.0, 2: 1.0, 3: 1.0}),
                                 _boot_responder({}))
                # Through _verify, not _verify_swarm directly: the helper is what
                # installs the wall-clock guard, and a direct call would reopen
                # the hang this test exists to keep closed.
                r = _verify(m, conn, speedup=1, param_timeout=param_timeout)
                self.assertIn(expected, m._describe_failure(r, {1, 2, 3}, 1))


class TestOneSupervisorPerChat(unittest.TestCase):
    """Two swarm_run supervisors on one chat is a fight, not a relaunch."""

    # Borrow the launch-loop harness rather than SUBCLASSING it: inheriting the
    # class would silently re-run all of its tests under this name too.
    _run_main = TestLaunchLoopFailsLoudly._run_main
    _order = TestLaunchLoopFailsLoudly._order

    def _busy(self, m):
        return _real_reg.SitlSupervisorActive(
            {"chat_index": 0, "sitl_pid": 4242})

    def test_a_refused_launch_destroys_nothing(self):
        # THE invariant. cleanup() kills this chat's SITL and router, so a guard
        # that ran after it would refuse only once the incumbent's swarm was
        # already dead — worse than no guard. Nothing destructive may happen on
        # the refusal path, and no exit hook may be left armed to do it later.
        m = _load_swarm()
        code, kill, verify, proc, printed = self._run_main(
            m, [_good(m)], begin_error=self._busy(m))
        self.assertNotEqual(code, 0)
        self._parent.cleanup.assert_not_called()
        self._parent.atexit.register.assert_not_called()
        self._reg.sitl_launch_lock.assert_not_called()
        verify.assert_not_called()

    def test_a_refused_launch_names_the_incumbent_and_the_way_out(self):
        m = _load_swarm()
        code, _, _, _, _ = self._run_main(
            m, [_good(m)], begin_error=self._busy(m))
        # sys.exit(str) puts the message in the exit code, not on stdout.
        self.assertIn("already has a live SITL supervisor", str(code))
        self.assertIn("pid=4242", str(code))
        self.assertIn("gcs_stop.py --chat 0", str(code))

    def test_ownership_is_taken_before_cleanup(self):
        # Ordering, positively: claim the chat, THEN tear it down. The reverse
        # leaves a window where cleanup is killing a chat nothing owns yet.
        m = _load_swarm()
        self._run_main(m, [_good(m)])
        self.assertLess(self._order("reg.begin_sitl_launch")[0],
                        self._order("cleanup")[0])
        self.assertEqual(
            self._reg.begin_sitl_launch.call_args.kwargs["supervisor_pid"],
            os.getpid())

    def test_launch_token_is_recorded_with_the_real_supervisor(self):
        m = _load_swarm()
        self._run_main(m, [_good(m)], launch_token="case-token")
        self.assertEqual(
            self._reg.begin_sitl_launch.call_args.kwargs["launch_token"],
            "case-token",
        )

    def test_a_registry_fault_still_does_not_stop_a_launch(self):
        # Only a live incumbent refuses. An unwritable registry must not — that
        # would let a logging-grade fault decide whether a swarm comes up.
        m = _load_swarm()
        code, kill, verify, proc, printed = self._run_main(
            m, [_good(m)], begin_error=OSError("locked"))
        self.assertEqual(code, proc.wait.return_value)
        self.assertIn("could not record launch ownership", printed)
        self._parent.cleanup.assert_called()
