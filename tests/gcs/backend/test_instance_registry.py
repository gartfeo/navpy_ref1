"""Tests for the cross-clone GCS instance registry (sync + per-chat ownership)."""
import json
import os
import tempfile
import time
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import MagicMock, patch

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry as reg


class _RegistryTestBase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._path = Path(self._tmp.name) / "instances.json"
        self._env = patch.dict(os.environ, {"GCS_INSTANCE_REGISTRY": str(self._path)})
        self._env.start()
        # Treat every slot's ports as free so claims don't depend on real sockets.
        self._free = patch.object(reg, "slot_ports_free", lambda n: True)
        self._free.start()

    def tearDown(self):
        self._free.stop()
        self._env.stop()
        self._tmp.cleanup()

    def _write(self, instances: dict):
        self._path.write_text(json.dumps({"instances": instances}), encoding="utf-8")


class TestClaim(_RegistryTestBase):
    def test_claim_takes_lowest_free_and_advances(self):
        a = reg.claim(label="x")
        b = reg.claim(label="y")
        self.assertEqual(a["chat_index"], 0)
        self.assertEqual(b["chat_index"], 1)
        self.assertEqual(a["backend"], 8000)
        self.assertEqual(b["monitor"], 15551)

    def test_claim_persists_to_registry_file(self):
        reg.claim()
        data = json.loads(self._path.read_text(encoding="utf-8"))
        self.assertIn("0", data["instances"])
        self.assertEqual(data["instances"]["0"]["sysids"], [1, 2, 3])

    def test_claim_prefer_specific(self):
        e = reg.claim(prefer=3)
        self.assertEqual(e["chat_index"], 3)

    def test_claim_prefer_taken_raises(self):
        reg.claim(prefer=0)
        with self.assertRaises(RuntimeError):
            reg.claim(prefer=0)

    def test_claim_skips_port_busy_slot(self):
        with patch.object(reg, "slot_ports_free", lambda n: n != 0):
            e = reg.claim()
        self.assertEqual(e["chat_index"], 1)


class TestLiveness(_RegistryTestBase):
    def _entry(self, n, started_at, backend=None, *, sitl=False):
        return {
            "chat_index": n, "frontend": 3000 + n, "backend": backend or (8000 + n),
            "monitor": 15550 + n, "mission_planner": 14550 + n,
            "sysids": [3 * n + 1, 3 * n + 2, 3 * n + 3],
            "backend_pid": None, "frontend_pid": None, "sitl": sitl,
            "started_at": started_at,
        }

    def test_dead_entry_pruned_when_backend_port_free(self):
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        self._write({"0": self._entry(0, old)})
        with patch.object(reg, "_port_bound", return_value=False):
            self.assertEqual(reg.live(), [])

    def test_live_entry_kept_when_backend_port_bound(self):
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        self._write({"0": self._entry(0, old)})
        with patch.object(reg, "_port_bound", return_value=True):
            self.assertEqual(len(reg.live()), 1)

    def test_fresh_reservation_survives_grace_even_if_port_unbound(self):
        fresh = datetime.now(timezone.utc).isoformat()
        self._write({"0": self._entry(0, fresh)})
        with patch.object(reg, "_port_bound", return_value=False):
            self.assertEqual(len(reg.live()), 1)

    def test_eval_entry_stays_live_from_recorded_swarm_pid(self):
        # Primary sitl liveness: the swarm_run supervisor pid. Companion
        # sockets are only bound per-episode in eval runs, so the pid — not a
        # port — must carry the entry between episodes.
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        entry = self._entry(ip.EVAL_CHAT_MIN, old, sitl=True)
        entry["sitl_pid"] = os.getpid()
        entry["sitl_pid_start"] = reg._pid_start_time(os.getpid())
        self._write({str(ip.EVAL_CHAT_MIN): entry})

        with patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False):
            self.assertEqual(len(reg.live()), 1)

    def test_eval_entry_stays_live_from_companion_udp_holder(self):
        # Secondary sitl liveness: a companion process holding its UDP port.
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        entry = self._entry(ip.EVAL_CHAT_MIN, old, sitl=True)
        self._write({str(ip.EVAL_CHAT_MIN): entry})
        live_port = ip.companion_port(entry["sysids"][0])

        with patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held",
                          side_effect=lambda port: port == live_port):
            self.assertEqual(len(reg.live()), 1)

    def test_old_eval_entry_stays_live_from_legacy_tcp_companion_port(self):
        # Entries recorded by pre-UDP branches have no sitl_pid and their
        # SITL still serves serial0 over TCP — the old probe must keep working
        # while both branch generations share this machine.
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        entry = self._entry(ip.EVAL_CHAT_MIN, old, sitl=True)
        self._write({str(ip.EVAL_CHAT_MIN): entry})
        live_port = ip.companion_port(entry["sysids"][0])

        with patch.object(reg, "_port_bound", side_effect=lambda port: port == live_port), \
             patch.object(reg, "udp_port_held", return_value=False):
            self.assertEqual(len(reg.live()), 1)

    def test_old_eval_entry_prunes_when_backend_and_sitl_are_both_dead(self):
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        entry = self._entry(ip.EVAL_CHAT_MIN, old, sitl=True)
        entry["sitl_pid"] = 999999999  # long-dead pid
        self._write({str(ip.EVAL_CHAT_MIN): entry})

        with patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False):
            self.assertEqual(reg.live(), [])

    def test_claim_reuses_slot_freed_by_dead_instance(self):
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        self._write({"0": self._entry(0, old)})
        with patch.object(reg, "_port_bound", return_value=False):
            e = reg.claim()
        self.assertEqual(e["chat_index"], 0)  # dead slot reclaimed

    def test_old_entry_with_live_backend_process_survives_prune(self):
        # A backend that exists but never bound (or lost) its port is still a
        # live stack: keep the entry so its pids stay stoppable and a relaunch
        # is refused instead of spawning a duplicate.
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        entry = self._entry(0, old)
        entry["backend_pid"] = 777
        self._write({"0": entry})
        with patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 777):
            self.assertEqual(len(reg.live()), 1)


class TestLaunchGuard(_RegistryTestBase):
    """claim(launcher_pid=...) doubles as a launch guard: while this directory's
    previous launch is still active (launcher alive, backend process alive but
    not yet bound, or backend port bound) a second launch must be refused —
    duplicate stacks double-bind the monitor UDP port and duel over serial0."""

    def _claim(self, **kw):
        kw.setdefault("label", "gcs")
        kw.setdefault("owner", "dir:w")
        kw.setdefault("launcher_pid", 4242)
        return reg.claim(**kw)

    def test_new_entry_records_launcher_pid(self):
        e = self._claim()
        self.assertEqual(e["launcher_pid"], 4242)
        self.assertEqual(reg.get(e["chat_index"])["launcher_pid"], 4242)

    def test_second_launch_refused_while_first_launcher_alive(self):
        self._claim()
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 4242):
            with self.assertRaises(reg.SlotBusyError):
                self._claim(launcher_pid=5555)

    def test_second_launch_refused_while_backend_spawned_but_unbound(self):
        # The exact race window that produced duplicate stacks: launcher A has
        # exited, backend A exists as a process but has not bound its port yet.
        e = self._claim()
        reg.record_pids(e["chat_index"], backend_pid=777)
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 777), \
             patch.object(reg, "_port_bound", return_value=False):
            with self.assertRaises(reg.SlotBusyError):
                self._claim(launcher_pid=5555)

    def test_second_launch_refused_while_backend_port_bound(self):
        self._claim()
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=True):
            with self.assertRaises(reg.SlotBusyError):
                self._claim(launcher_pid=5555)

    def test_slot_busy_error_carries_the_existing_entry(self):
        e = self._claim()
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 4242):
            with self.assertRaises(reg.SlotBusyError) as ctx:
                self._claim(launcher_pid=5555)
        self.assertEqual(ctx.exception.entry["chat_index"], e["chat_index"])

    def test_stale_slot_taken_over_in_place(self):
        # Prior stack fully dead: same chat index is reused (no slot leak) with
        # fresh launcher_pid and cleared process pids.
        e = self._claim()
        reg.record_pids(e["chat_index"], backend_pid=777, frontend_pid=888)
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=False):
            e2 = self._claim(branch="b2", launcher_pid=5555)
        self.assertEqual(e2["chat_index"], e["chat_index"])
        self.assertEqual(e2["launcher_pid"], 5555)
        self.assertIsNone(e2["backend_pid"])
        self.assertIsNone(e2["frontend_pid"])
        self.assertEqual(e2["branch"], "b2")

    def test_takeover_clears_the_previous_launchs_sitl_verdict(self):
        # A stale verdict carried into a fresh launch is exactly the silent
        # staleness this evidence exists to expose.
        e = self._claim()
        reg.record_pids(e["chat_index"], sitl=True, sitl_pid=999)
        reg.record_sitl_status(e["chat_index"], speedup=10, verified=False,
                               sitl_pid=999, error="SIM_SPEEDUP mismatch")
        # The verdict really landed, so the assertions below are about the
        # takeover clearing it and not about it never having been written.
        self.assertFalse(reg.get(e["chat_index"])["sitl_verified"])
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False):
            e2 = self._claim(launcher_pid=5555)
        self.assertIsNone(e2["sitl_speedup"])
        self.assertIsNone(e2["sitl_verified"])
        self.assertIsNone(e2["sitl_error"])

    def test_takeover_clears_a_dead_swarms_sitl_flag(self):
        e = self._claim()
        reg.record_pids(e["chat_index"], sitl=True, sitl_pid=999)
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False):
            e2 = self._claim(launcher_pid=5555)
        self.assertFalse(e2["sitl"])
        self.assertIsNone(e2["sitl_pid"])

    def test_takeover_clears_a_dead_swarms_launch_token(self):
        e = self._claim()
        reg.begin_sitl_launch(
            e["chat_index"], supervisor_pid=999, launch_token="old-token"
        )
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False):
            e2 = self._claim(launcher_pid=5555)
        self.assertIsNone(e2["sitl_launch_token"])

    def test_takeover_keeps_a_swarm_that_outlived_its_backend(self):
        # gcs_stop needs the supervisor pid to kill it; clearing the flag here
        # would orphan a running swarm.
        e = self._claim()
        reg.record_pids(e["chat_index"], sitl=True, sitl_pid=999)
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 999), \
             patch.object(reg, "_port_bound", return_value=False):
            e2 = self._claim(launcher_pid=5555)
        self.assertTrue(e2["sitl"])
        self.assertEqual(e2["sitl_pid"], 999)

    def test_takeover_keeps_a_live_swarms_launch_token(self):
        e = self._claim()
        reg.begin_sitl_launch(
            e["chat_index"], supervisor_pid=999, launch_token="live-token"
        )
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 999), \
             patch.object(reg, "_port_bound", return_value=False):
            e2 = self._claim(launcher_pid=5555)
        self.assertEqual(e2["sitl_launch_token"], "live-token")

    def test_takeover_keeps_a_swarm_known_only_by_a_held_companion_port(self):
        # Eval runs bind companion UDP sockets per-episode, so this fallback is
        # sometimes the only surviving liveness signal.
        e = self._claim()
        reg.record_pids(e["chat_index"], sitl=True, sitl_pid=999)
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=True):
            e2 = self._claim(launcher_pid=5555)
        self.assertTrue(e2["sitl"])
        self.assertEqual(e2["sitl_pid"], 999)

    def test_takeover_keeps_a_swarm_known_only_by_a_legacy_serial0_listener(self):
        e = self._claim()
        reg.record_pids(e["chat_index"], sitl=True, sitl_pid=999)
        companion = {ip.companion_port(s) for s in e["sysids"]}
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False), \
             patch.object(reg, "_port_bound", side_effect=lambda p: p in companion):
            e2 = self._claim(launcher_pid=5555)
        self.assertTrue(e2["sitl"])
        self.assertEqual(e2["sitl_pid"], 999)

    def test_claim_without_launcher_pid_keeps_plain_reuse(self):
        # swarm_run-style callers (no launcher_pid) reuse the entry untouched,
        # busy or not — the guard is strictly opt-in.
        self._claim()
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 4242):
            e = reg.claim(label="gcs", owner="dir:w")
        self.assertEqual(e["chat_index"], 0)
        self.assertEqual(e["launcher_pid"], 4242)  # untouched


class TestPidAlive(unittest.TestCase):
    def test_own_pid_is_alive(self):
        self.assertTrue(reg._pid_alive(os.getpid()))

    def test_exited_process_is_dead(self):
        import subprocess as sp
        import sys as _sys
        proc = sp.Popen([_sys.executable, "-c", "pass"])
        proc.wait(timeout=30)
        self.assertFalse(reg._pid_alive(proc.pid))

    def test_bogus_values_are_dead(self):
        for bogus in (None, 0, -5, "", "x"):
            self.assertFalse(reg._pid_alive(bogus), repr(bogus))


class TestPidIdentity(_RegistryTestBase):
    """pid + creation stamp: a recycled pid must not hold a slot busy."""

    def test_pid_matches_degrades_to_liveness_without_stamp(self):
        self.assertTrue(reg.pid_matches(os.getpid(), None))
        self.assertFalse(reg.pid_matches(0, None))

    def test_pid_matches_rejects_a_different_creation_stamp(self):
        with patch.object(reg, "_pid_alive", return_value=True), \
             patch.object(reg, "_pid_start_time", return_value=222):
            self.assertTrue(reg.pid_matches(777, 222))
            self.assertFalse(reg.pid_matches(777, 111))

    def test_own_pid_matches_its_real_stamp(self):
        stamp = reg._pid_start_time(os.getpid())
        # On Windows a real stamp round-trips; elsewhere stamp is None and
        # pid_matches degrades to liveness — both must pass.
        self.assertTrue(reg.pid_matches(os.getpid(), stamp))

    def test_recycled_backend_pid_frees_the_slot(self):
        # Record backend pid 777 with stamp 100; later the SAME number belongs
        # to a different process (stamp 111): the slot is NOT busy -> takeover.
        e = reg.claim(label="gcs", owner="dir:w", launcher_pid=4242)
        with patch.object(reg, "_pid_start_time", return_value=100):
            reg.record_pids(e["chat_index"], backend_pid=777)
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 777), \
             patch.object(reg, "_pid_start_time", return_value=111), \
             patch.object(reg, "_port_bound", return_value=False):
            e2 = reg.claim(label="gcs", owner="dir:w", launcher_pid=5555)
        self.assertEqual(e2["chat_index"], e["chat_index"])
        self.assertEqual(e2["launcher_pid"], 5555)

    def test_same_backend_process_keeps_the_slot_busy(self):
        e = reg.claim(label="gcs", owner="dir:w", launcher_pid=4242)
        with patch.object(reg, "_pid_start_time", return_value=100):
            reg.record_pids(e["chat_index"], backend_pid=777)
        with patch.object(reg, "_pid_alive", side_effect=lambda pid: pid == 777), \
             patch.object(reg, "_pid_start_time", return_value=100), \
             patch.object(reg, "_port_bound", return_value=False):
            with self.assertRaises(reg.SlotBusyError):
                reg.claim(label="gcs", owner="dir:w", launcher_pid=5555)


class TestRegistryLockStale(unittest.TestCase):
    """A stale lock is broken only when its holder is dead; live holders keep it."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._path = Path(self._tmp.name) / "instances.json"
        self._env = patch.dict(os.environ, {"GCS_INSTANCE_REGISTRY": str(self._path)})
        self._env.start()

    def tearDown(self):
        self._env.stop()
        self._tmp.cleanup()

    def _plant_stale_lock(self, content: str):
        lock = reg._lock_path()
        lock.parent.mkdir(parents=True, exist_ok=True)
        lock.write_text(content)
        old = time.time() - 10_000
        os.utime(lock, (old, old))
        return lock

    def test_dead_holders_stale_lock_is_broken(self):
        self._plant_stale_lock("999999999")  # nonexistent pid
        with reg._locked():
            pass  # acquired despite the leftover lock

    def test_live_holders_stale_lock_is_not_broken(self):
        lock = self._plant_stale_lock(str(os.getpid()))  # "holder" is alive
        with patch.object(reg, "_LOCK_TIMEOUT_S", 0.4):
            with self.assertRaises(TimeoutError):
                with reg._locked():
                    pass
        self.assertTrue(lock.exists())  # the live holder's lock survived
        lock.unlink()

    def test_release_never_unlinks_a_foreign_lock(self):
        # If OUR lock was (wrongly) replaced mid-section, releasing must not
        # delete the replacement — that would cascade broken mutual exclusion.
        lock = self._plant_stale_lock("999999999")
        reg._unlink_if_owner(lock)
        self.assertTrue(lock.exists())  # not ours -> untouched
        lock.unlink()

    def test_empty_lock_file_is_breakable(self):
        # Crash between open and pid write leaves an empty lock; a live holder
        # always has its pid written, so empty = safe to break once stale.
        self._plant_stale_lock("")
        with reg._locked():
            pass


class TestRegistryLockContention(unittest.TestCase):
    """Contention must make a caller WAIT, never fail.

    On Windows a file that another process has just unlinked lingers in a
    delete-pending state, and ``os.open(O_CREAT | O_EXCL)`` against it raises
    PermissionError (EACCES) rather than FileExistsError. Only the latter was
    retried, so ordinary lock contention -- two launches in the same instant --
    killed one of them outright instead of making it queue.

    Measured, not theorised: two concurrent eval swarm launches, one died at
    +5.4s with "PermissionError: [Errno 13] Permission denied:
    'C:\\Users\\GARTF\\.gcs\\instances.lock'". That is the whole reason parallel
    runs had to be staggered.
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._path = Path(self._tmp.name) / "instances.json"
        self._env = patch.dict(os.environ, {"GCS_INSTANCE_REGISTRY": str(self._path)})
        self._env.start()

    def tearDown(self):
        self._env.stop()
        self._tmp.cleanup()

    def _flaky_open(self, failures: int | None, error: OSError):
        """Real os.open, except attempts on the lock raise.

        ``failures=None`` never clears. A finite count is not a substitute: the
        retry loop spins in microseconds, so even 10_000 failures are consumed
        inside a 0.3s timeout and the call then succeeds.
        """
        real_open = os.open
        seen = {"n": 0}

        def opener(path, flags, *args, **kwargs):
            if str(path) == str(reg._lock_path()) and (
                failures is None or seen["n"] < failures
            ):
                seen["n"] += 1
                raise error
            return real_open(path, flags, *args, **kwargs)

        return opener, seen

    def test_delete_pending_permission_error_is_retried(self):
        opener, seen = self._flaky_open(2, PermissionError(13, "Permission denied"))
        with patch.object(os, "open", opener):
            with reg._locked():
                pass  # acquired after waiting out the contention
        self.assertEqual(seen["n"], 2)

    def test_permission_error_that_never_clears_times_out(self):
        # A retry loop must still give up, and say what it was actually fighting:
        # a genuinely unwritable directory would otherwise spin then report a
        # bare timeout with no cause.
        opener, _ = self._flaky_open(None, PermissionError(13, "Permission denied"))
        with patch.object(reg, "_LOCK_TIMEOUT_S", 0.3):
            with patch.object(os, "open", opener):
                with self.assertRaises(TimeoutError) as caught:
                    with reg._locked():
                        pass
        self.assertIn("Permission denied", str(caught.exception))

    def test_launch_lock_retries_delete_pending_permission_error(self):
        # Same hole, same fix, in the machine-wide SITL launch serializer.
        real_open = os.open
        seen = {"n": 0}

        def opener(path, flags, *args, **kwargs):
            if str(path) == str(reg._sitl_launch_lock_path()) and seen["n"] < 2:
                seen["n"] += 1
                raise PermissionError(13, "Permission denied")
            return real_open(path, flags, *args, **kwargs)

        with patch.object(os, "open", opener):
            with reg.sitl_launch_lock(timeout=5.0) as acquired:
                self.assertTrue(acquired)
        self.assertEqual(seen["n"], 2)


class TestRetractSitl(_RegistryTestBase):
    """A terminally failed swarm has to stop advertising itself as a live one.

    swarm_run kills its WSL processes and exits, but nothing used to touch the
    entry — and ``_is_alive``'s startup grace answers True before it looks at any
    pid, so a fast failure left an entry reading LIVE with ``sitl: True``.
    """

    SUPERVISOR = 4242
    SUPERVISOR_START = 132_000_000_000_000_000  # creation FILETIME, any nonzero

    def _entry(self, *, launch_token=None, **overrides):
        """A slot whose SITL was recorded by supervisor pid ``SUPERVISOR``.

        The identity stamp is forced non-None: ``_pid_start_time`` returns None
        for a pid that doesn't exist, which would make "the stamp is cleared"
        assertions pass without the production code clearing anything.
        """
        e = reg.claim(label="sitl", clone="c", owner="dir:w")
        with patch.object(reg, "_pid_start_time",
                          lambda p: self.SUPERVISOR_START if p == self.SUPERVISOR else None):
            reg.record_pids(0, sitl=True, sitl_pid=self.SUPERVISOR, **overrides)
            if launch_token is not None:
                reg.begin_sitl_launch(
                    0,
                    supervisor_pid=self.SUPERVISOR,
                    launch_token=launch_token,
                )
        self.assertEqual(reg.get(0)["sitl_pid_start"], self.SUPERVISOR_START)
        return e

    def _retract(self, pid=SUPERVISOR, *, alive=(), bound=()):
        """Retract with exactly *alive* pids and *bound* ports considered live."""
        with patch.object(reg, "_pid_alive", lambda p: p in alive), \
             patch.object(reg, "_port_bound", lambda p: p in bound), \
             patch.object(reg, "udp_port_held", return_value=False):
            return reg.retract_sitl(0, sitl_pid=pid)

    def test_sole_owner_entry_is_released(self):
        self._entry()
        self.assertEqual(self._retract(), "released")
        self.assertIsNone(reg.get(0))

    def test_released_even_inside_the_startup_grace(self):
        # The bug itself: the entry was just claimed, so _is_alive() returns True
        # unconditionally for STARTUP_GRACE_S and a reader still sees a live slot.
        e = self._entry()
        age = (datetime.now(timezone.utc)
               - datetime.fromisoformat(e["started_at"])).total_seconds()
        self.assertLess(age, reg.STARTUP_GRACE_S)  # the grace is really in play
        self._retract()
        with patch.object(reg, "_pid_alive", return_value=False), \
             patch.object(reg, "_port_bound", return_value=False), \
             patch.object(reg, "udp_port_held", return_value=False):
            self.assertEqual(reg.live(), [])

    def test_kept_while_the_backend_process_is_alive(self):
        # Spawned by gcs_launch: the same entry owns a live GCS stack, so only
        # the SITL fields may go.
        self._entry(backend_pid=111)
        self.assertEqual(self._retract(alive=(111,)), "cleared")
        e = reg.get(0)
        self.assertEqual((e["sitl"], e["sitl_pid"], e["sitl_pid_start"]),
                         (False, None, None))

    def test_kept_entry_clears_the_launch_token(self):
        self._entry(backend_pid=111, launch_token="case-token")
        self.assertEqual(self._retract(alive=(111,)), "cleared")
        self.assertIsNone(reg.get(0)["sitl_launch_token"])

    def test_kept_while_the_backend_port_is_bound(self):
        e = self._entry()
        self.assertEqual(self._retract(bound=(e["backend"],)), "cleared")
        self.assertIsNotNone(reg.get(0))

    def test_kept_while_only_the_frontend_process_is_alive(self):
        # _launch_active ignores Vite on purpose; releasing here would orphan it
        # onto this slot's port with nothing left to stop it by.
        self._entry(frontend_pid=222)
        self.assertEqual(self._retract(alive=(222,)), "cleared")
        self.assertIsNotNone(reg.get(0))

    def test_kept_while_only_the_frontend_port_is_bound(self):
        # A Vite that outlived the pid we recorded for it is still live.
        e = self._entry()
        self.assertEqual(self._retract(bound=(e["frontend"],)), "cleared")
        self.assertIsNotNone(reg.get(0))

    def test_a_newer_supervisors_state_is_left_alone(self):
        # An older swarm_run failing late must not clear the swarm that replaced
        # it. Same guard covers our own record_pids having silently failed.
        self._entry()
        before = dict(reg.get(0))
        with patch.object(reg, "_save") as save:
            self.assertEqual(self._retract(pid=9999), "not-ours")
        save.assert_not_called()  # "no write at all" is the contract
        self.assertEqual(reg.get(0), before)

    def test_a_recorded_failure_verdict_survives_the_clear(self):
        # The verdict is why the entry is kept at all under gcs_launch: that
        # console is gone, so --list is the operator's only copy.
        self._entry(backend_pid=111)
        reg.record_sitl_status(0, speedup=1, verified=False,
                               sitl_pid=self.SUPERVISOR, error="boom")
        self.assertEqual(self._retract(alive=(111,)), "cleared")
        e = reg.get(0)
        self.assertEqual((e["sitl_speedup"], e["sitl_verified"], e["sitl_error"]),
                         (1, False, "boom"))

    def test_missing_slot_is_a_noop(self):
        with patch.object(reg, "_save") as save:
            self.assertEqual(reg.retract_sitl(7, sitl_pid=self.SUPERVISOR), "missing")
        save.assert_not_called()
        self.assertIsNone(reg.get(7))


class TestVerdictIsScopedToItsSupervisor(_RegistryTestBase):
    """A verdict belongs to ONE launch, and the entry must be able to say which.

    Two ways it used to lie. ``swarm_run._resolve_chat`` REUSES an entry through
    ``find_for_owner``, which — unlike ``claim``'s takeover — does not clear the
    verdict, so a previous run's ``sitl_verified: True`` survived into the next
    launch. And ``record_sitl_status`` wrote unconditionally, so a launcher that
    failed late could publish its verdict onto the entry of the launch that had
    already replaced it. Recording a supervisor pid now starts a generation, and
    only that supervisor may write its result.
    """

    OLD, NEW = 4242, 5555

    def _claimed_with(self, pid):
        reg.claim(label="sitl", clone="c", owner="dir:w")
        reg.record_pids(0, sitl=True, sitl_pid=pid)
        return reg.get(0)

    def test_the_measured_rate_is_stored_beside_the_requested_speedup(self):
        """Requested and MEASURED are separate facts and get separate fields.

        A vehicle can report ``SIM_SPEEDUP=1`` while its clock runs at 10x, so
        another session reading this slot cannot learn the achieved speed from
        the request. Storing only one of the two would leave it guessing.
        """
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=self.OLD,
                               measured_rates={121: 10.06, 122: 9.94})
        e = reg.get(0)
        self.assertEqual(e["sitl_speedup"], 10)
        # JSON has no integer keys, so the STORED shape is the string-keyed one
        # a reader gets back — not the dict that was passed in.
        self.assertEqual(e["sitl_measured_rates"], {"121": 10.06, "122": 9.94})

    def test_measured_rates_survive_a_json_round_trip_unchanged(self):
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=self.OLD,
                               measured_rates={121: 10.06})
        first = reg.get(0)["sitl_measured_rates"]
        # get() re-reads the file, so a second read proves the on-disk shape is
        # stable rather than merely the in-memory one.
        self.assertEqual(first, reg.get(0)["sitl_measured_rates"])

    def test_no_measurement_stores_none_rather_than_an_empty_reading(self):
        # A failed launch has no rate. An empty dict would read as "measured,
        # and it was nothing", which is not what happened.
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=False, sitl_pid=self.OLD,
                               error="boom")
        self.assertIsNone(reg.get(0)["sitl_measured_rates"])
        reg.record_sitl_status(0, speedup=10, verified=False, sitl_pid=self.OLD,
                               error="boom", measured_rates={})
        self.assertIsNone(reg.get(0)["sitl_measured_rates"])

    def test_an_unusable_rate_is_dropped_rather_than_stored_as_a_measurement(self):
        self._claimed_with(self.OLD)
        reg.record_sitl_status(
            0, speedup=10, verified=True, sitl_pid=self.OLD,
            measured_rates={121: float("nan"), 122: None, 123: 10.0})
        self.assertEqual(reg.get(0)["sitl_measured_rates"], {"123": 10.0})

    def test_a_new_supervisor_resets_the_measured_rate_too(self):
        # Same generation rule as the rest of the verdict: a rate that described
        # the previous launch must not be readable as this one's.
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=self.OLD,
                               measured_rates={121: 10.0})
        reg.record_pids(0, sitl=True, sitl_pid=self.NEW)
        self.assertIsNone(reg.get(0)["sitl_measured_rates"])

    def test_begin_sitl_launch_also_clears_the_previous_measured_rate(self):
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=self.OLD,
                               measured_rates={121: 10.0})
        reg.begin_sitl_launch(0, supervisor_pid=self.NEW)
        self.assertIsNone(reg.get(0)["sitl_measured_rates"])

    def test_a_new_supervisor_resets_the_previous_launchs_verdict(self):
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=self.OLD)
        # Reuse, not takeover: exactly the path that kept the stale verdict.
        reg.record_pids(0, sitl=True, sitl_pid=self.NEW)
        e = reg.get(0)
        self.assertEqual((e["sitl_speedup"], e["sitl_verified"], e["sitl_error"]),
                         (None, None, None))

    def test_repeating_the_same_supervisor_still_resets(self):
        # The contract is "recording a supervisor starts a generation", not
        # "recording a DIFFERENT one" — a reader can't tell the calls apart.
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=self.OLD)
        reg.record_pids(0, sitl=True, sitl_pid=self.OLD)
        self.assertIsNone(reg.get(0)["sitl_verified"])

    def test_the_flag_alone_does_not_reset_the_verdict(self):
        # sitl=True with no pid proves no new generation, so it must not erase
        # durable evidence.
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=10, verified=False, sitl_pid=self.OLD,
                               error="boom")
        reg.record_pids(0, sitl=True)
        e = reg.get(0)
        self.assertEqual((e["sitl_verified"], e["sitl_error"]), (False, "boom"))

    def test_a_gcs_stacks_pids_do_not_reset_the_verdict(self):
        # gcs_launch records backend/frontend pids on the SAME entry while its
        # swarm_run child is verifying.
        self._claimed_with(self.OLD)
        reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=self.OLD)
        reg.record_pids(0, backend_pid=111)
        reg.record_pids(0, frontend_pid=222)
        e = reg.get(0)
        self.assertEqual((e["sitl_speedup"], e["sitl_verified"]), (1, True))

    def test_the_recorded_supervisor_may_write(self):
        self._claimed_with(self.OLD)
        self.assertEqual(
            reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=self.OLD),
            "recorded")
        self.assertTrue(reg.get(0)["sitl_verified"])

    def test_a_foreign_supervisor_may_not_write(self):
        self._claimed_with(self.NEW)
        with patch.object(reg, "_save") as save:
            self.assertEqual(
                reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=self.OLD),
                "not-ours")
        save.assert_not_called()

    def test_an_entry_with_no_supervisor_may_not_be_written(self):
        # record_pids is best-effort in swarm_run, so it can silently never land.
        # Then nothing proves the verdict is about this entry, and the durable
        # record is lost rather than guessed at.
        reg.claim(label="sitl", clone="c", owner="dir:w")
        with patch.object(reg, "_save") as save:
            self.assertEqual(
                reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=self.OLD),
                "not-ours")
        save.assert_not_called()

    def test_missing_slot_writes_nothing(self):
        with patch.object(reg, "_save") as save:
            self.assertEqual(
                reg.record_sitl_status(7, speedup=1, verified=True, sitl_pid=self.OLD),
                "missing")
        save.assert_not_called()

    def test_a_superseded_launcher_cannot_overwrite_the_new_verdict(self):
        # End to end: OLD is replaced by NEW, NEW records its result, and OLD
        # then fails and tries to publish. NEW's verdict has to survive.
        self._claimed_with(self.OLD)
        reg.record_pids(0, sitl=True, sitl_pid=self.NEW)
        reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=self.NEW)
        self.assertEqual(
            reg.record_sitl_status(0, speedup=10, verified=False, sitl_pid=self.OLD,
                                   error="SIM_SPEEDUP mismatch"),
            "not-ours")
        e = reg.get(0)
        self.assertEqual((e["sitl_speedup"], e["sitl_verified"], e["sitl_error"]),
                         (1, True, None))


class TestMutations(_RegistryTestBase):
    def test_record_pids(self):
        reg.claim()
        reg.record_pids(0, backend_pid=111, frontend_pid=222, sitl=True,
                        sitl_pid=333)
        e = reg.get(0)
        self.assertEqual(
            (e["backend_pid"], e["frontend_pid"], e["sitl"], e["sitl_pid"]),
            (111, 222, True, 333),
        )

    def test_record_pids_noop_for_unknown_slot(self):
        reg.record_pids(7, backend_pid=999)  # must not raise / create
        self.assertIsNone(reg.get(7))

    def test_new_entry_starts_with_no_sitl_verdict(self):
        e = reg.claim()
        self.assertIsNone(e["sitl_verified"])
        self.assertIsNone(e["sitl_speedup"])
        self.assertIsNone(e["sitl_error"])

    def test_record_sitl_status_success(self):
        reg.claim()
        reg.record_pids(0, backend_pid=111, sitl=True, sitl_pid=333)
        reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=333)
        e = reg.get(0)
        self.assertEqual((e["sitl_speedup"], e["sitl_verified"], e["sitl_error"]),
                         (1, True, None))
        # Process state is a separate concern and must survive untouched.
        self.assertEqual((e["backend_pid"], e["sitl_pid"]), (111, 333))

    def test_successful_mutation_uses_the_facade_persistence_seam(self):
        reg.claim()
        reg.record_pids(0, sitl=True, sitl_pid=333)
        with patch.object(reg, "_save", wraps=reg._save) as save:
            outcome = reg.record_sitl_status(
                0, speedup=1, verified=True, sitl_pid=333
            )
        self.assertEqual(outcome, "recorded")
        save.assert_called_once()

    def test_record_sitl_status_failure_keeps_the_reason(self):
        reg.claim()
        reg.record_pids(0, sitl=True, sitl_pid=333)
        reg.record_sitl_status(0, speedup=1, verified=False, sitl_pid=333,
                               error="SIM_SPEEDUP mismatch (requested 1): sys_id 1=10")
        e = reg.get(0)
        self.assertFalse(e["sitl_verified"])
        self.assertIn("sys_id 1=10", e["sitl_error"])

    def test_record_sitl_status_clears_a_previous_error_on_success(self):
        reg.claim()
        reg.record_pids(0, sitl=True, sitl_pid=333)
        reg.record_sitl_status(0, speedup=1, verified=False, sitl_pid=333, error="boom")
        reg.record_sitl_status(0, speedup=1, verified=True, sitl_pid=333)
        self.assertIsNone(reg.get(0)["sitl_error"])

    def test_record_sitl_status_noop_for_unknown_slot(self):
        # must not raise / create
        reg.record_sitl_status(7, speedup=1, verified=True, sitl_pid=333)
        self.assertIsNone(reg.get(7))

    def test_release(self):
        reg.claim()
        reg.release(0)
        self.assertIsNone(reg.get(0))

    def test_live_sorted_by_index(self):
        reg.claim(prefer=2)
        reg.claim(prefer=0)
        self.assertEqual([e["chat_index"] for e in reg.live()], [0, 2])


class TestOwnerResolution(_RegistryTestBase):
    """A stack resolves by its working directory — no --chat passing needed."""

    def test_owner_for_is_directory_based(self):
        import os.path as osp
        expected = f"dir:{osp.normcase(osp.abspath(r'C:/repos/navpy'))}"
        self.assertEqual(reg.owner_for(r"C:/repos/navpy"), expected)

    def test_owner_for_ignores_session_id(self):
        # Directory-anchored: the session id must NOT change the owner.
        with patch.dict(os.environ, {"CLAUDE_CODE_SESSION_ID": "sess-abc"}):
            with_sid = reg.owner_for(r"C:/repos/navpy")
        env = {k: v for k, v in os.environ.items() if k != "CLAUDE_CODE_SESSION_ID"}
        with patch.dict(os.environ, env, clear=True):
            without_sid = reg.owner_for(r"C:/repos/navpy")
        self.assertEqual(with_sid, without_sid)
        self.assertTrue(with_sid.startswith("dir:"))

    def test_owner_for_stable_across_path_case_and_separators(self):
        # Same directory written differently -> same owner (normalized).
        a = reg.owner_for(r"C:\repos\navpy")
        b = reg.owner_for("C:/repos/navpy")
        self.assertEqual(a, b)

    def test_claim_reuses_same_owner_chat(self):
        a = reg.claim(owner="session:s1")
        b = reg.claim(owner="session:s1")   # same session -> same chat, no new slot
        self.assertEqual(a["chat_index"], b["chat_index"])
        self.assertEqual(len(reg.live()), 1)

    def test_different_owners_get_different_slots(self):
        a = reg.claim(owner="session:s1")
        b = reg.claim(owner="session:s2")
        self.assertNotEqual(a["chat_index"], b["chat_index"])

    def test_find_for_owner(self):
        reg.claim(owner="session:s1")
        self.assertEqual(reg.find_for_owner("session:s1")["chat_index"], 0)
        self.assertIsNone(reg.find_for_owner("session:nope"))

    def test_prefer_overrides_owner_reuse(self):
        reg.claim(owner="session:s1")            # chat 0
        e = reg.claim(owner="session:s1", prefer=5)  # explicit override
        self.assertEqual(e["chat_index"], 5)


class TestChatBands(_RegistryTestBase):
    """Interactive vs eval slot bands stay disjoint."""

    def test_hi_bound_caps_allocation(self):
        # Interactive-style cap: only 0..2 may be allocated.
        got = [reg.claim(hi=2)["chat_index"] for _ in range(3)]
        self.assertEqual(got, [0, 1, 2])
        with self.assertRaises(RuntimeError):
            reg.claim(hi=2)  # band full

    def test_lo_bound_starts_eval_band(self):
        e = reg.claim(lo=ip.EVAL_CHAT_MIN)
        self.assertEqual(e["chat_index"], ip.EVAL_CHAT_MIN)

    def test_interactive_and_eval_bands_are_disjoint(self):
        gui = reg.claim(owner="dir:a", hi=ip.interactive_chat_hi())
        ev = reg.claim(owner="dir:b", lo=ip.EVAL_CHAT_MIN)
        self.assertLess(gui["chat_index"], ip.EVAL_CHAT_MIN)
        self.assertGreaterEqual(ev["chat_index"], ip.EVAL_CHAT_MIN)

    def test_prefer_ignores_band(self):
        # Explicit --chat override can land anywhere, even across the split.
        e = reg.claim(prefer=ip.EVAL_CHAT_MIN + 5, hi=ip.interactive_chat_hi())
        self.assertEqual(e["chat_index"], ip.EVAL_CHAT_MIN + 5)


class TestBandScopedOwnerLookup(_RegistryTestBase):
    """One directory can own an interactive AND an eval slot at once. Lookups and
    claims must stay scoped to the band they mean, so an eval sweep's teardown
    never resolves (and stops) a concurrent interactive session's slot in the
    same directory — the bug behind ``gcs_stop --eval``."""

    def _coexisting(self, owner="dir:w"):
        """Claim an interactive slot and an eval slot for the SAME owner."""
        gui = reg.claim(label="gcs", owner=owner, hi=ip.interactive_chat_hi())
        ev = reg.claim(label="sitl-eval", owner=owner, lo=ip.EVAL_CHAT_MIN)
        return gui, ev

    def test_interactive_and_eval_coexist_as_two_independent_slots(self):
        gui, ev = self._coexisting()
        # The eval claim did NOT reuse the interactive slot (band-scoped reuse).
        self.assertLess(gui["chat_index"], ip.EVAL_CHAT_MIN)
        self.assertGreaterEqual(ev["chat_index"], ip.EVAL_CHAT_MIN)
        self.assertEqual(len(reg.live()), 2)

    def test_find_for_owner_gcs_label_returns_interactive_slot(self):
        gui, _ = self._coexisting()
        self.assertEqual(
            reg.find_for_owner("dir:w", label="gcs")["chat_index"], gui["chat_index"])

    def test_find_for_owner_eval_label_returns_eval_slot(self):
        _, ev = self._coexisting()
        self.assertEqual(
            reg.find_for_owner("dir:w", label="sitl-eval")["chat_index"], ev["chat_index"])

    def test_find_for_owner_sitl_label_shares_interactive_band(self):
        gui, _ = self._coexisting()
        # Standalone interactive SITL ("sitl") lives in the interactive band.
        self.assertEqual(
            reg.find_for_owner("dir:w", label="sitl")["chat_index"], gui["chat_index"])

    def test_find_for_owner_without_label_is_band_agnostic(self):
        gui, _ = self._coexisting()
        # Legacy callers (no label) still get the lowest-index owned slot.
        self.assertEqual(reg.find_for_owner("dir:w")["chat_index"], gui["chat_index"])

    def test_eval_lookup_is_none_when_only_interactive_exists(self):
        reg.claim(label="gcs", owner="dir:w", hi=ip.interactive_chat_hi())
        self.assertIsNone(reg.find_for_owner("dir:w", label="sitl-eval"))

    def test_interactive_lookup_is_none_when_only_eval_exists(self):
        reg.claim(label="sitl-eval", owner="dir:w", lo=ip.EVAL_CHAT_MIN)
        self.assertIsNone(reg.find_for_owner("dir:w", label="gcs"))

    def test_interactive_claim_does_not_reuse_an_existing_eval_slot(self):
        ev = reg.claim(label="sitl-eval", owner="dir:w", lo=ip.EVAL_CHAT_MIN)
        gui = reg.claim(label="gcs", owner="dir:w", hi=ip.interactive_chat_hi())
        self.assertNotEqual(gui["chat_index"], ev["chat_index"])
        self.assertLess(gui["chat_index"], ip.EVAL_CHAT_MIN)

    def test_eval_claim_does_not_reuse_an_existing_interactive_slot(self):
        gui = reg.claim(label="gcs", owner="dir:w", hi=ip.interactive_chat_hi())
        ev = reg.claim(label="sitl-eval", owner="dir:w", lo=ip.EVAL_CHAT_MIN)
        self.assertNotEqual(ev["chat_index"], gui["chat_index"])
        self.assertGreaterEqual(ev["chat_index"], ip.EVAL_CHAT_MIN)


class TestSitlLaunchLock(_RegistryTestBase):
    """Machine-wide lock serializing SITL launches (one init at a time)."""

    def test_exclusive_while_held_then_released(self):
        with reg.sitl_launch_lock() as got:
            self.assertTrue(got)
            self.assertTrue(reg._sitl_launch_lock_path().exists())
            # a second acquirer can't get it while held -> yields False (no hang)
            with reg.sitl_launch_lock(timeout=0.3) as got2:
                self.assertFalse(got2)
            # ... and the failed acquirer must NOT have deleted the held lock
            self.assertTrue(reg._sitl_launch_lock_path().exists())
        self.assertFalse(reg._sitl_launch_lock_path().exists())  # released on exit

    def test_steals_stale_lock(self):
        p = reg._sitl_launch_lock_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("999 crashed")
        old = time.time() - 10_000
        os.utime(p, (old, old))
        with reg.sitl_launch_lock(stale_after=1.0) as got:
            self.assertTrue(got)  # stole the crashed holder's lock
        self.assertFalse(p.exists())


class TestSlotPortsFree(unittest.TestCase):
    def test_companion_ports_are_probed_with_exclusive_bind(self):
        chat = ip.EVAL_CHAT_MIN
        probed: list[int] = []

        def record(port):
            probed.append(port)
            return False

        with patch.object(reg, "_port_bindable", return_value=True), \
             patch.object(reg, "udp_port_held", side_effect=record):
            self.assertTrue(reg.slot_ports_free(chat))
        self.assertEqual(probed, ip.companion_ports_for_chat(chat))

    def test_companion_udp_holder_blocks_slot_reuse(self):
        # pymavlink binds companion ports with SO_REUSEADDR, so a plain probe
        # bind SUCCEEDS while the port is held (verified live) — the
        # exclusive-bind probe is what must catch the holder.
        chat = ip.EVAL_CHAT_MIN
        held = ip.companion_ports_for_chat(chat)[1]

        with patch.object(reg, "_port_bindable", return_value=True), \
             patch.object(reg, "udp_port_held",
                          side_effect=lambda port: port == held):
            self.assertFalse(reg.slot_ports_free(chat))

    def test_slot_is_free_when_all_gcs_and_companion_ports_bind(self):
        with patch.object(reg, "_port_bindable", return_value=True), \
             patch.object(reg, "udp_port_held", return_value=False):
            self.assertTrue(reg.slot_ports_free(ip.EVAL_CHAT_MIN))


class TestUdpPortHeld(unittest.TestCase):
    def test_detects_reuseaddr_holder_and_release(self):
        # The real primitive against a real socket: a SO_REUSEADDR bind (what
        # pymavlink does) must read as held; releasing it must read as free.
        import socket as socket_mod
        port = 5993  # scratch port in no session's band arithmetic use
        holder = socket_mod.socket(socket_mod.AF_INET, socket_mod.SOCK_DGRAM)
        holder.setsockopt(socket_mod.SOL_SOCKET, socket_mod.SO_REUSEADDR, 1)
        try:
            holder.bind(("0.0.0.0", port))
            self.assertTrue(reg.udp_port_held(port))
        finally:
            holder.close()
        self.assertFalse(reg.udp_port_held(port))


class TestReapPort(unittest.TestCase):
    """Reap orphans (e.g. a Vite dev server) squatting a slot's port."""

    NETSTAT = (
        "Active Connections\n\n"
        "  Proto  Local Address     Foreign Address   State        PID\n"
        "  TCP    0.0.0.0:3001      0.0.0.0:0         LISTENING    30512\n"
        "  TCP    [::]:3001         [::]:0            LISTENING    30512\n"
        "  TCP    0.0.0.0:13001     0.0.0.0:0         LISTENING    9999\n"
        "  TCP    127.0.0.1:8001    0.0.0.0:0         LISTENING    42024\n"
        "  UDP    0.0.0.0:15551     *:*                            5555\n"
        "  TCP    0.0.0.0:3001      1.2.3.4:5555      ESTABLISHED  11111\n"
    )

    def test_listeners_on_port_only_listening_tcp_exact_port(self):
        with patch.object(reg.os, "name", "nt"), \
             patch.object(reg.subprocess, "run", return_value=MagicMock(stdout=self.NETSTAT)):
            pids = reg.listeners_on_port(3001)
        # 30512 (both v4+v6); NOT 13001 (different port), the UDP row, or ESTABLISHED.
        self.assertEqual(pids, [30512])

    def test_listeners_on_port_udp_matches_bound_udp_socket(self):
        with patch.object(reg.os, "name", "nt"), \
             patch.object(reg.subprocess, "run", return_value=MagicMock(stdout=self.NETSTAT)):
            self.assertEqual(reg.listeners_on_port(15551, kind="udp"), [5555])
            # UDP lookup must not match TCP rows on the same port number...
            self.assertEqual(reg.listeners_on_port(3001, kind="udp"), [])
            # ...and TCP lookup must not match the UDP row.
            self.assertEqual(reg.listeners_on_port(15551), [])

    def test_listeners_on_port_empty_off_windows(self):
        with patch.object(reg.os, "name", "posix"):
            self.assertEqual(reg.listeners_on_port(3001), [])

    def test_reap_port_tree_kills_each_listener(self):
        with patch.object(reg, "listeners_on_port", return_value=[123, 456]), \
             patch.object(reg.os, "name", "nt"), \
             patch.object(reg.subprocess, "run") as mrun:
            reaped = reg.reap_port(3001)
        self.assertEqual(reaped, [123, 456])
        cmds = [c.args[0] for c in mrun.call_args_list]
        self.assertIn(["taskkill", "/PID", "123", "/T", "/F"], cmds)
        self.assertIn(["taskkill", "/PID", "456", "/T", "/F"], cmds)

    def test_reap_port_exclude_spares_registered_pids(self):
        # A registered live session's process holding the port (e.g. a cross-slot
        # debugging connect) must never be collateral of another slot's cleanup.
        with patch.object(reg, "listeners_on_port", return_value=[123, 456]), \
             patch.object(reg.os, "name", "nt"), \
             patch.object(reg.subprocess, "run") as mrun:
            reaped = reg.reap_port(3001, exclude={123})
        self.assertEqual(reaped, [456])
        cmds = [c.args[0] for c in mrun.call_args_list]
        self.assertNotIn(["taskkill", "/PID", "123", "/T", "/F"], cmds)
        self.assertIn(["taskkill", "/PID", "456", "/T", "/F"], cmds)


class TestRegisteredPids(_RegistryTestBase):
    def test_collects_pids_of_live_entries(self):
        e = reg.claim(owner="dir:a")
        reg.record_pids(e["chat_index"], backend_pid=11, frontend_pid=22)
        self.assertEqual(reg.registered_pids(), {11, 22})

    def test_empty_registry_yields_no_pids(self):
        self.assertEqual(reg.registered_pids(), set())


if __name__ == "__main__":
    unittest.main()


class TestBeginSitlLaunch(_RegistryTestBase):
    """One SITL supervisor per chat, decided under the lock that records it."""

    def test_begin_takes_ownership_and_resets_the_previous_verdict(self):
        reg.claim()
        reg.record_pids(0, sitl=True, sitl_pid=333)
        reg.record_sitl_status(0, speedup=10, verified=True, sitl_pid=333)
        self.assertTrue(reg.begin_sitl_launch(0, supervisor_pid=444))
        e = reg.get(0)
        self.assertEqual(e["sitl_pid"], 444)
        self.assertTrue(e["sitl"])
        # A new supervisor is a new generation; the old verdict described the
        # launch it replaced, and anything gating on one must not read it.
        self.assertIsNone(e["sitl_verified"])
        self.assertIsNone(e["sitl_speedup"])
        self.assertIsNone(e["sitl_error"])

    def test_begin_records_launch_token_without_replacing_supervisor_pid(self):
        reg.claim()
        self.assertTrue(
            reg.begin_sitl_launch(
                0,
                supervisor_pid=444,
                launch_token="case-token",
            )
        )
        entry = reg.get(0)
        self.assertEqual(entry["sitl_pid"], 444)
        self.assertEqual(entry["sitl_launch_token"], "case-token")

    def test_legacy_sitl_pid_record_starts_a_tokenless_generation(self):
        reg.claim()
        reg.begin_sitl_launch(0, supervisor_pid=333, launch_token="case-token")
        reg.record_pids(0, sitl=True, sitl_pid=444)
        self.assertIsNone(reg.get(0)["sitl_launch_token"])

    def test_gcs_pid_records_preserve_the_sitl_launch_token(self):
        reg.claim()
        reg.begin_sitl_launch(0, supervisor_pid=333, launch_token="case-token")
        reg.record_pids(0, backend_pid=111)
        reg.record_pids(0, frontend_pid=222)
        self.assertEqual(reg.get(0)["sitl_launch_token"], "case-token")

    def test_begin_refuses_a_live_supervisor(self):
        reg.claim()
        reg.record_pids(0, sitl=True, sitl_pid=333)
        with patch.object(reg, "pid_matches", return_value=True):
            with self.assertRaises(reg.SitlSupervisorActive) as caught:
                reg.begin_sitl_launch(0, supervisor_pid=444)
        self.assertIn("live SITL supervisor", str(caught.exception))
        self.assertEqual(caught.exception.entry["sitl_pid"], 333)
        # ...and the incumbent's ownership is left exactly as it was.
        self.assertEqual(reg.get(0)["sitl_pid"], 333)

    def test_begin_refuses_even_our_own_live_pid(self):
        # Not idempotent on purpose: a second begin would reset a verdict this
        # same launch had already published.
        reg.claim()
        reg.record_pids(0, sitl=True, sitl_pid=333)
        with patch.object(reg, "pid_matches", return_value=True):
            with self.assertRaises(reg.SitlSupervisorActive):
                reg.begin_sitl_launch(0, supervisor_pid=333)

    def test_begin_ignores_a_supervisorless_survivor(self):
        # _sitl_alive would also count a held companion port, which proves some
        # SITL process is up but NOT that a competing supervisor is. Blocking on
        # that would refuse a legitimate relaunch over a legacy swarm — which is
        # precisely what cleanup(chat) exists to clear.
        reg.claim()
        reg.record_pids(0, sitl=True)  # sitl flag, no supervisor pid
        with patch.object(reg, "_port_bound", return_value=True), \
             patch.object(reg, "udp_port_held", return_value=True):
            self.assertTrue(reg.begin_sitl_launch(0, supervisor_pid=444))
        self.assertEqual(reg.get(0)["sitl_pid"], 444)

    def test_begin_ignores_a_live_pid_on_an_entry_with_sitl_cleared(self):
        # retract_sitl clears the flag but leaves the pid fields alone on a kept
        # entry; that is "no swarm here", not "a supervisor owns this".
        reg.claim()
        reg.record_pids(0, sitl=False, sitl_pid=333)
        with patch.object(reg, "pid_matches", return_value=True):
            self.assertTrue(reg.begin_sitl_launch(0, supervisor_pid=444))
        self.assertEqual(reg.get(0)["sitl_pid"], 444)

    def test_begin_reports_a_missing_entry_rather_than_raising(self):
        # `swarm_run --chat N` against an unregistered stack resolves a chat with
        # no entry at all. That has always been allowed to run; refusing it would
        # break the flow, so it is reported as "nothing recorded", not a conflict.
        self.assertFalse(reg.begin_sitl_launch(7, supervisor_pid=444))
        self.assertIsNone(reg.get(7))
