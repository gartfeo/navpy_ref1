"""Tests for gcs.backend.param_readonly.is_readonly.

The read-only set is grounded in ArduPilot's firmware `@ReadOnly: True` tags
(statistics, item counts, hardware device IDs, firmware-calibrated values).
The critical property is that the patterns are specific enough NOT to lock the
many user-settable ``*_ID`` params.
"""
import unittest

from gcs.backend.param_readonly import is_readonly


class TestIsReadonly(unittest.TestCase):
    def test_exact_readonly(self):
        for n in ("MIS_TOTAL", "FENCE_TOTAL", "RALLY_TOTAL",
                  "SYS_NUM_RESETS", "VTX_FREQ"):
            self.assertTrue(is_readonly(n), n)

    def test_stat_prefix(self):
        for n in ("STAT_BOOTCNT", "STAT_FLTTIME", "STAT_RUNTIME", "STAT_RESET"):
            self.assertTrue(is_readonly(n), n)

    def test_sensor_device_ids(self):
        # COMPASS_DEV_ID / *_DEVID families incl. numbered variants.
        for n in ("COMPASS_DEV_ID", "COMPASS_DEV_ID2", "COMPASS_DEV_ID8",
                  "BARO_DEVID", "BARO1_DEVID", "BARO3_DEVID",
                  "ARSPD_DEVID", "ARSPD2_DEVID", "SIM_MAG1_DEVID"):
            self.assertTrue(is_readonly(n), n)

    def test_imu_accel_gyro_device_ids(self):
        for n in ("INS_ACC_ID", "INS_ACC2_ID", "INS_ACC3_ID",
                  "INS_GYR_ID", "INS_GYR2_ID", "INS_GYR3_ID",
                  "INS4_ACC_ID", "INS5_GYR_ID"):
            self.assertTrue(is_readonly(n), n)

    def test_gps_can_node_ids(self):
        for n in ("GPS_CAN_NODEID1", "GPS_CAN_NODEID2",
                  "GPS1_CAN_NODEID", "GPS2_CAN_NODEID"):
            self.assertTrue(is_readonly(n), n)

    def test_baro_ground_pressure(self):
        # Primary instance is the bare name (no digit), like BARO_DEVID.
        for n in ("BARO_GND_PRESS", "BARO2_GND_PRESS", "BARO3_GND_PRESS"):
            self.assertTrue(is_readonly(n), n)
        # Must not over-match nearby baro params.
        for n in ("BARO_GND_TEMP", "BARO_ALT_OFFSET", "XBARO_GND_PRESS"):
            self.assertFalse(is_readonly(n), n)

    def test_temp_calibration_learned_values(self):
        for n in ("TCAL_TEMP_MIN", "TCAL_TEMP_MAX", "TCAL_BARO_EXP"):
            self.assertTrue(is_readonly(n), n)

    def test_case_insensitive(self):
        self.assertTrue(is_readonly("mis_total"))
        self.assertTrue(is_readonly("ins_acc_id"))

    def test_writable_params_not_readonly(self):
        # Regular tunables, and the important *_ID near-misses that must stay
        # editable (a blanket _ID rule would wrongly lock these):
        for n in ("ARMING_CHECK", "TKOFF_THR_DELAY", "SYSID_THISMAV",
                  "RC1_OPTION", "AHRS_EKF_TYPE", "MIS_OPTIONS",
                  "COMPASS_PRIO1_ID", "COMPASS_PRIO2_ID", "COMPASS_PRIO3_ID",
                  "FRSKY_DNLINK_ID", "FRSKY_DNLINK1_ID", "FRSKY_UPLINK_ID",
                  "ADSB_ICAO_ID",
                  "GPS1_CAN_OVRIDE", "GPS2_CAN_OVRIDE"):
            self.assertFalse(is_readonly(n), n)

    def test_empty_or_none(self):
        self.assertFalse(is_readonly(""))
        self.assertFalse(is_readonly(None))


if __name__ == "__main__":
    unittest.main()
