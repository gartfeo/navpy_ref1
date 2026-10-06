import unittest
from dataclasses import FrozenInstanceError
from unittest.mock import Mock

import numpy as np

from navpy.modules.vision.focus_monitor import FocusMonitor, laplacian_focus
from navpy.modules.vision.focus_policy import FocusPolicy


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


class TestLaplacianFocus(unittest.TestCase):
    def test_sharp_scores_higher_than_blurred(self):
        import cv2
        rng = np.random.default_rng(0)
        sharp = rng.integers(0, 255, (200, 200, 3)).astype(np.uint8)
        blurred = cv2.GaussianBlur(sharp, (21, 21), 8)
        self.assertGreater(laplacian_focus(sharp), laplacian_focus(blurred) * 3)

    def test_none_frame_is_zero(self):
        self.assertEqual(laplacian_focus(None), 0.0)

    def _textured(self, h=100, w=100):
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[::2, ::2] = 255
        frame[1::2, 1::2] = 255
        return frame

    def test_bbox_crop_isolates_roi(self):
        # texture only in the top-left quarter: a bbox over the flat region
        # scores ~0; over the textured region scores >> 0.
        frame = np.full((100, 100, 3), 128, dtype=np.uint8)
        frame[:50, :50] = self._textured(50, 50)
        hot = laplacian_focus(frame, (25, 25, 40, 40))
        cold = laplacian_focus(frame, (75, 75, 40, 40))
        self.assertGreater(hot, cold)
        self.assertAlmostEqual(cold, 0.0, places=3)

    def test_bbox_clamps_to_frame_bounds(self):
        focus = laplacian_focus(self._textured(), (95, 95, 40, 40))
        self.assertGreaterEqual(focus, 0.0)

    def test_bbox_fully_outside_falls_back_to_whole_frame(self):
        frame = self._textured()
        self.assertAlmostEqual(laplacian_focus(frame, (-100, -100, 40, 40)),
                               laplacian_focus(frame), places=3)


class TestFocusMonitor(unittest.TestCase):
    def setUp(self):
        self.clock = _Clock()
        self.af_calls = []
        # focus is driven by a script keyed off how many AFs have fired, so the
        # tests are deterministic and independent of any real image.
        self.frame = np.zeros((4, 4, 3), np.uint8)

    def _monitor(self, focus_fn, **kw):
        params = dict(settle_delay=0.7, af_settle=0.8, improve_ratio=1.10, max_attempts=4)
        params.update(kw)
        return FocusMonitor(
            lambda: self.af_calls.append(self.clock.t),
            focus_metric=lambda frame, bbox=None: focus_fn(),
            clock=self.clock, **params)

    def test_no_af_without_a_zoom_change(self):
        m = self._monitor(lambda: 500.0)
        for _ in range(20):
            m.tick(self.frame, current_zoom=2.0, now=self.clock.advance(0.1))
        self.assertEqual(self.af_calls, [])

    def test_zoom_change_triggers_af_after_settle(self):
        m = self._monitor(lambda: 500.0)
        m.tick(self.frame, 2.0, now=self.clock.t)           # establish baseline zoom
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))  # zoom change -> settle
        self.assertEqual(self.af_calls, [])                  # not yet (settling)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.7))  # past settle -> AF
        self.assertEqual(len(self.af_calls), 1)

    def test_af_retries_through_glass_until_sharp(self):
        # AF improves focus only on the 3rd try (the live SIYI-through-glass
        # pattern 121 -> 134 -> 540): the monitor must keep AFing until sharp.
        seq = iter([120.0,            # f_before (settle)
                    121.0,            # after AF1 (barely better -> not >10%, but...)
                    134.0,            # after AF2
                    540.0,            # after AF3 (locked)
                    545.0])           # after AF4 (no real gain -> stop)
        vals = [120.0, 134.0, 540.0, 545.0, 545.0]
        state = {"i": 0}

        def focus():
            return vals[min(state["i"], len(vals) - 1)]

        m = self._monitor(focus, improve_ratio=1.05)
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))     # zoom change
        m.tick(self.frame, 4.0, now=self.clock.advance(0.7))     # settle -> measure 120, AF1
        state["i"] = 1
        for _ in range(6):                                        # drive verify/retry
            fired = m.tick(self.frame, 4.0, now=self.clock.advance(0.8))
            if fired:
                state["i"] += 1
        # 134>120*1.05 -> AF; 540>134*1.05 -> AF; 545 !>540*1.05 -> stop
        self.assertGreaterEqual(len(self.af_calls), 3)
        self.assertLessEqual(len(self.af_calls), 4)

    def test_stops_when_first_af_already_sharp(self):
        # AF1 makes it sharp; AF2 yields no further gain -> stop (<=2 AFs).
        vals = [100.0, 600.0, 600.0, 600.0]
        state = {"i": 0}
        m = self._monitor(lambda: vals[min(state["i"], len(vals) - 1)])
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))
        m.tick(self.frame, 4.0, now=self.clock.advance(0.7))     # measure 100, AF1
        state["i"] = 1
        for _ in range(5):
            fired = m.tick(self.frame, 4.0, now=self.clock.advance(0.8))
            if fired:
                state["i"] += 1
        self.assertLessEqual(len(self.af_calls), 2)
        self.assertGreaterEqual(len(self.af_calls), 1)

    def test_caps_attempts_when_af_never_helps(self):
        # AF never improves focus (hardware can't lock): bounded by max_attempts.
        m = self._monitor(lambda: 100.0, max_attempts=4)
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))
        m.tick(self.frame, 4.0, now=self.clock.advance(0.7))
        for _ in range(10):
            m.tick(self.frame, 4.0, now=self.clock.advance(0.8))
        self.assertLessEqual(len(self.af_calls), 4)

    def test_steady_state_drift_recovers_focus(self):
        # No zoom change, but focus collapses (e.g. AF drifted): the guard must
        # eventually AF — AND only after SUSTAINED blur, not on the first dip.
        focus = {"v": 600.0}
        m = self._monitor(lambda: focus["v"], drift_ratio=0.45, drift_seconds=1.5)
        m.tick(self.frame, 2.0, now=self.clock.t)
        for _ in range(5):
            m.tick(self.frame, 2.0, now=self.clock.advance(0.3))   # build the peak at 600
        focus["v"] = 100.0                                          # sharp -> blurry
        # within the first drift_seconds of low ticks: NO AF yet (debounce)
        early = False
        for _ in range(4):                                         # 4*0.3 = 1.2s < 1.5s
            early = m.tick(self.frame, 2.0, now=self.clock.advance(0.3)) or early
        self.assertFalse(early, "AF fired before sustained-blur debounce elapsed")
        # past drift_seconds of sustained blur: AF fires
        fired = False
        for _ in range(6):
            fired = m.tick(self.frame, 2.0, now=self.clock.advance(0.3)) or fired
        self.assertTrue(fired)

    def test_transient_blur_does_not_trigger_drift_af(self):
        # A single-tick dip (e.g. a passing occluder/motion blur) that recovers
        # must NOT trigger autofocus — the time-debounce holds.
        focus = {"v": 600.0}
        m = self._monitor(lambda: focus["v"], drift_ratio=0.45, drift_seconds=1.5)
        m.tick(self.frame, 2.0, now=self.clock.t)
        for _ in range(5):
            m.tick(self.frame, 2.0, now=self.clock.advance(0.3))   # peak 600
        fired = False
        focus["v"] = 100.0
        fired = m.tick(self.frame, 2.0, now=self.clock.advance(0.3)) or fired  # one low tick
        focus["v"] = 600.0                                          # recovered
        for _ in range(6):
            fired = m.tick(self.frame, 2.0, now=self.clock.advance(0.3)) or fired
        self.assertFalse(fired, "transient dip fired AF — debounce not load-bearing")

    def test_zoom_jitter_below_epsilon_does_not_af(self):
        # SIYI hardware zoom readback jitters at ~0.1 resolution on a steady
        # zoom; deltas under the epsilon must NOT trigger an AF recovery.
        m = self._monitor(lambda: 500.0, zoom_epsilon=0.15)
        m.tick(self.frame, 4.70, now=self.clock.t)            # baseline
        for z in (4.74, 4.68, 4.72, 4.66, 4.78):              # jitter < 0.15
            for _ in range(3):
                m.tick(self.frame, z, now=self.clock.advance(0.3))
        self.assertEqual(self.af_calls, [])

    def test_focus_metric_exception_is_swallowed(self):
        # The metric runs on the operator main loop; a transient error must
        # degrade to "no AF this tick", never propagate.
        def boom(frame, bbox=None):
            raise RuntimeError("cv2 blew up")
        m = FocusMonitor(lambda: self.af_calls.append(0), focus_metric=boom,
                         clock=self.clock, settle_delay=0.7)
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))   # zoom change
        # the settle tick calls the (raising) metric — must not propagate
        fired = m.tick(self.frame, 4.0, now=self.clock.advance(0.7))
        self.assertFalse(fired)

    def test_none_zoom_does_not_crash_or_af(self):
        m = self._monitor(lambda: 500.0)
        for _ in range(5):
            m.tick(self.frame, current_zoom=None, now=self.clock.advance(0.2))
        self.assertEqual(self.af_calls, [])

    def test_nonfinite_zoom_does_not_look_like_a_zoom_change(self):
        m = self._monitor(lambda: 500.0)
        m.tick(self.frame, current_zoom=2.0, now=self.clock.t)

        for _ in range(5):
            m.tick(
                self.frame,
                current_zoom=float("nan"),
                now=self.clock.advance(0.2),
            )

        self.assertEqual(m.state, "idle")
        self.assertEqual(self.af_calls, [])

    def test_autofocus_failure_leaves_settling_cycle_retryable(self):
        events = []
        af_attempt = {"count": 0}

        def autofocus():
            events.append("autofocus")
            af_attempt["count"] += 1
            if af_attempt["count"] == 1:
                raise OSError("camera unavailable")

        def focus(_frame, _bbox=None):
            events.append("focus")
            return 500.0

        m = FocusMonitor(
            autofocus,
            focus_metric=focus,
            clock=self.clock,
            settle_delay=0.7,
        )
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))
        events.clear()

        self.assertFalse(
            m.tick(self.frame, 4.0, now=self.clock.advance(0.7))
        )
        self.assertEqual(m.state, "settling")
        self.assertTrue(m.tick(self.frame, 4.0, now=self.clock.t))
        self.assertEqual(m.state, "verify")
        self.assertEqual(
            events,
            ["focus", "autofocus", "focus", "autofocus"],
        )

    def test_explicit_now_does_not_read_injected_clock(self):
        clock = Mock(side_effect=AssertionError("clock should not be read"))
        m = FocusMonitor(
            lambda: None,
            focus_metric=lambda *_args: 500.0,
            clock=clock,
        )

        self.assertFalse(m.tick(self.frame, 2.0, now=1.25))

        clock.assert_not_called()

    def test_implicit_now_reads_clock_even_when_frame_is_missing(self):
        clock = Mock(return_value=5.0)
        m = FocusMonitor(lambda: None, clock=clock)

        self.assertFalse(m.tick(None, 2.0))

        clock.assert_called_once_with()

    def test_reset_clears_drift_peak_and_low_debounce(self):
        focus = {"v": 600.0}
        m = self._monitor(
            lambda: focus["v"],
            drift_ratio=0.45,
            drift_seconds=1.5,
        )
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 2.0, now=self.clock.advance(0.3))
        focus["v"] = 100.0
        m.tick(self.frame, 2.0, now=self.clock.advance(0.3))

        m.reset()
        for _ in range(8):
            m.tick(self.frame, 2.0, now=self.clock.advance(0.3))

        self.assertEqual(m.state, "idle")
        self.assertEqual(self.af_calls, [])

    def test_reset_clears_state(self):
        m = self._monitor(lambda: 500.0)
        m.tick(self.frame, 2.0, now=self.clock.t)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.1))
        m.reset()
        self.assertEqual(m.state, "idle")
        # after reset, the next zoom is a fresh baseline (no immediate AF)
        m.tick(self.frame, 4.0, now=self.clock.advance(0.7))
        self.assertEqual(self.af_calls, [])


class TestFocusPolicy(unittest.TestCase):
    def test_policy_normalizes_once_and_is_immutable(self):
        policy = FocusPolicy.from_values(
            settle_delay=1,
            af_settle=2,
            improve_ratio=3,
            max_attempts=0,
            zoom_epsilon=4,
            drift_ratio=5,
            drift_seconds=6,
            drift_window=7,
        )

        self.assertEqual(policy.max_attempts, 1)
        self.assertIsInstance(policy.settle_delay, float)
        with self.assertRaises(FrozenInstanceError):
            policy.settle_delay = 8.0


if __name__ == "__main__":
    unittest.main()
