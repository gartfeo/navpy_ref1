"""PrearmReasons: ArduPilot's current PreArm reasons on the autopilot clock."""
import unittest

from gcs.backend.prearm_reasons import PREARM_REASON_STALE_MS, PrearmReasons


class TestPrearmReasons(unittest.TestCase):
    def test_reason_ages_out_after_stale_window(self):
        r = PrearmReasons()
        r.observe_clock(1_000_000)
        r.observe("Accels inconsistent")
        r.observe_clock(1_000_000 + PREARM_REASON_STALE_MS)
        self.assertEqual(r.current(), {"accels inconsistent"})
        r.observe_clock(1_000_000 + PREARM_REASON_STALE_MS + 1)
        self.assertEqual(r.current(), frozenset())

    def test_mode_only_requires_a_current_mode_reason(self):
        r = PrearmReasons()
        self.assertFalse(r.mode_not_armable_only())
        r.observe_clock(10_000)
        r.observe("RTL mode not armable")
        self.assertTrue(r.mode_not_armable_only())
        r.observe("Accels inconsistent")
        self.assertFalse(r.mode_not_armable_only())

    def test_reason_before_first_clock_is_stamped_at_first_clock(self):
        r = PrearmReasons()
        r.observe("Accels inconsistent")
        r.observe_clock(500_000)
        self.assertIn("accels inconsistent", r.current())
        r.observe_clock(500_000 + PREARM_REASON_STALE_MS + 1)
        self.assertNotIn("accels inconsistent", r.current())

    def test_reordered_older_sample_does_not_rewind_clock(self):
        r = PrearmReasons()
        r.observe_clock(200_000)
        r.observe("Accels inconsistent")
        r.observe_clock(199_000)  # late packet
        r.observe_clock(200_000 + PREARM_REASON_STALE_MS + 1)
        self.assertEqual(r.current(), frozenset())

    def test_reboot_forgets_reasons(self):
        """A clock reset is a reboot: old reasons are forgotten, so with
        nothing known the exemption is off (the gate stays closed)."""
        r = PrearmReasons()
        r.observe_clock(400_000)
        r.observe("RTL mode not armable")
        r.observe_clock(1_000)
        self.assertEqual(r.current(), frozenset())
        self.assertFalse(r.mode_not_armable_only())


if __name__ == "__main__":
    unittest.main()
