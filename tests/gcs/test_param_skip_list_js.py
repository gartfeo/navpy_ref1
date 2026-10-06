"""Tests for src/gcs/frontend/src/utils/paramSkipList.js.

Locks the ArduPilot identity/calibration SKIP vs bus/port KEEP classification
used by the Compare-parameters feature:
    - identity/calibration params (compass/accel/gyro offsets, device IDs, level
      trim, board stats) classify as UAV-specific (skip by default),
    - bus/port/protocol/geometry config params classify as keep,
    - the two lists never overlap (the regression guard against an over-broad
      SKIP regex silently protecting a wiring param),
    - instance-digit variants and the dangerous named collisions are correct.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "paramSkipList.js",
))

_JS_SRC = (
    open(_UTIL, encoding="utf-8").read()
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _classify(names):
    """Return {name: {skip, keep, uav}} classifying every name in one node call."""
    script = (
        f"const names={json.dumps(names)};"
        "console.log(JSON.stringify(names.map(n=>({name:n,"
        "skip:matchesIdentitySkip(n),"
        "keep:matchesBusPortKeep(n),"
        "uav:isUavSpecificParam(n)}))));"
    )
    return {r["name"]: r for r in json.loads(_run_js(script))}


# Per-board identity / calibration → must be skipped by default.
SKIP_NAMES = [
    "COMPASS_OFS_X", "COMPASS2_OFS_X", "COMPASS_OFS2_X",
    "COMPASS_DIA_X", "COMPASS_ODI_Z", "COMPASS_MOT_Y",
    "COMPASS_SCALE", "COMPASS2_SCALE",
    "COMPASS_DEV_ID", "COMPASS_DEV_ID2", "COMPASS_PRIO1_ID",
    "INS_ACCOFFS_X", "INS_ACC2OFFS_Y", "INS_ACCSCAL_X",
    "INS_GYROFFS_Z", "INS_GYR3OFFS_X",
    "INS_ACC_CALTEMP", "INS_GYR2_CALTEMP",
    "INS_ACC_ID", "INS_GYR2_ID",
    "INS_TCAL1_ENABLE", "INS_TCAL1_ACC1_X",
    "BARO_DEVID", "BARO2_DEVID", "BARO_GND_PRESS", "BARO2_GND_PRESS",
    "BARO_GND_TEMP", "BARO_ALT_OFFSET",
    "AHRS_TRIM_X", "AHRS_TRIM_Y", "AHRS_TRIM_Z",
    "GPS_CAN_NODEID1", "GPS_CAN_NODEID2", "CAN_D1_UC_NODE", "CAN_D2_UC_NODE",
    "STAT_BOOTCNT", "STAT_FLTTIME", "STAT_RUNTIME", "STAT_RESET",
    "SYSID_THISMAV", "FORMAT_VERSION", "BRD_SERIAL_NUM",
    # Airspeed calibration / identity (ArduPlane) — Codex gate-02 finding 1.
    "ARSPD_OFFSET", "ARSPD2_OFFSET", "ARSPD_RATIO", "ARSPD2_RATIO",
    "ARSPD_DEVID", "ARSPD2_DEVID",
    # Battery monitor calibration / identity — Codex gate-02 finding 2.
    "BATT_VOLT_MULT", "BATT2_VOLT_MULT", "BATT_AMP_PERVLT",
    "BATT_AMP_OFFSET", "BATT_SERIAL_NUM",
    # Higher instance digits — Codex gate-02 coverage note.
    "COMPASS_DEV_ID8", "COMPASS_PRIO3_ID", "BARO1_DEVID", "BARO3_DEVID",
    "INS_ACC3SCAL_Z", "INS_TCAL3_GYR3_Z",
]

# Wiring / port / protocol / geometry → must be kept (pushed).
KEEP_NAMES = [
    "SERIAL2_PROTOCOL", "SERIAL1_BAUD", "SERIAL3_OPTIONS", "SERIAL_PASS1",
    "CAN_P1_DRIVER", "CAN_P1_BITRATE", "CAN_D1_PROTOCOL",
    "GPS_TYPE", "GPS_TYPE2", "GPS1_TYPE", "GPS2_TYPE",
    "GPS_GNSS_MODE", "GPS_RATE_MS", "GPS_AUTO_SWITCH", "GPS_PRIMARY",
    "GPS1_CAN_OVRIDE",
    "BARO_EXT_BUS", "BARO_PRIMARY",
    "COMPASS_EXTERNAL", "COMPASS2_EXTERNAL",
    "COMPASS_ORIENT", "COMPASS2_ORIENT",
    "COMPASS_USE", "COMPASS_USE2", "COMPASS_DEC", "COMPASS_ENABLE",
    "INS_POS1_X", "INS_POS2_Y", "GPS_POS1_X",
    "AHRS_ORIENTATION", "AHRS_EKF_TYPE",
    "ARSPD_TYPE", "ARSPD_BUS", "ARSPD2_TYPE",
]

# Ordinary tune/config params → neither skipped nor on the keep guard list.
NEUTRAL_NAMES = [
    "ATC_RAT_RLL_P", "TECS_SPDWEIGHT", "NAVL1_PERIOD", "PTCH2SRV_P",
    "BATT_CAPACITY", "BATT_MONITOR", "BATT_LOW_VOLT", "ARMING_CHECK",
    "SYSID_MYGCS", "INS_USE", "INS_GYRO_FILTER",
]

# Near-miss names that must NOT be skipped (anchoring guards) — Codex gate-02.
NEAR_MISS_NAMES = [
    "GPS_CAN_NODEIDX",  # NODEID needs a trailing digit
    "AHRS_TRIM_XY",     # axis suffix is a single [XYZ]
    "COMPASS_DEV_IDX",  # DEV_ID needs optional digit, not a letter
    "STAT_OPTIONS",     # STAT_ is enumerated, not a bare prefix
]


class TestClassification(unittest.TestCase):
    def test_skip_names(self):
        c = _classify(SKIP_NAMES)
        for name in SKIP_NAMES:
            with self.subTest(name=name):
                self.assertTrue(c[name]["skip"], f"{name} should match a SKIP pattern")
                self.assertFalse(c[name]["keep"], f"{name} should not match a KEEP pattern")
                self.assertTrue(c[name]["uav"], f"{name} should be UAV-specific (skip)")

    def test_keep_names(self):
        c = _classify(KEEP_NAMES)
        for name in KEEP_NAMES:
            with self.subTest(name=name):
                self.assertTrue(c[name]["keep"], f"{name} should match a KEEP pattern")
                self.assertFalse(c[name]["uav"], f"{name} should be kept, not skipped")

    def test_neutral_names(self):
        c = _classify(NEUTRAL_NAMES)
        for name in NEUTRAL_NAMES:
            with self.subTest(name=name):
                self.assertFalse(c[name]["skip"], f"{name} should not match SKIP")
                self.assertFalse(c[name]["uav"], f"{name} should not be skipped")

    def test_near_miss_names_not_skipped(self):
        c = _classify(NEAR_MISS_NAMES)
        for name in NEAR_MISS_NAMES:
            with self.subTest(name=name):
                self.assertFalse(c[name]["skip"], f"{name} should not match SKIP")
                self.assertFalse(c[name]["uav"], f"{name} should not be skipped")

    def test_no_keep_sample_matches_skip(self):
        # Regression guard: the dangerous over-match cases (E2/E4/E7 in the
        # ArduPilot analysis). If a future SKIP edit broadens to catch a KEEP
        # param, this fails.
        c = _classify(KEEP_NAMES)
        offenders = [n for n in KEEP_NAMES if c[n]["skip"]]
        self.assertEqual(offenders, [], f"KEEP names matched a SKIP pattern: {offenders}")


class TestInstanceDigits(unittest.TestCase):
    def test_compass_offset_instances(self):
        names = ["COMPASS_OFS_X", "COMPASS2_OFS_X", "COMPASS_OFS2_X"]
        c = _classify(names)
        for name in names:
            self.assertTrue(c[name]["uav"], name)

    def test_ins_device_id_instances(self):
        names = ["INS_ACC_ID", "INS_ACC2_ID", "INS_GYR_ID", "INS_GYR3_ID"]
        c = _classify(names)
        for name in names:
            self.assertTrue(c[name]["uav"], name)


class TestNamedCollisions(unittest.TestCase):
    """The cases anchored regexes must split correctly."""

    def test_compass_odi_skip_vs_orient_keep(self):
        c = _classify(["COMPASS_ODI_X", "COMPASS_ORIENT", "COMPASS2_ORIENT"])
        self.assertTrue(c["COMPASS_ODI_X"]["uav"])
        self.assertFalse(c["COMPASS_ORIENT"]["uav"])
        self.assertFalse(c["COMPASS2_ORIENT"]["uav"])

    def test_ahrs_trim_skip_vs_other_ahrs_keep(self):
        c = _classify(["AHRS_TRIM_X", "AHRS_ORIENTATION", "AHRS_EKF_TYPE"])
        self.assertTrue(c["AHRS_TRIM_X"]["uav"])
        self.assertFalse(c["AHRS_ORIENTATION"]["uav"])
        self.assertFalse(c["AHRS_EKF_TYPE"]["uav"])

    def test_gps_nodeid_skip_vs_can_ovride_keep(self):
        c = _classify(["GPS_CAN_NODEID1", "GPS1_CAN_OVRIDE"])
        self.assertTrue(c["GPS_CAN_NODEID1"]["uav"])
        self.assertFalse(c["GPS1_CAN_OVRIDE"]["uav"])

    def test_sysid_thismav_skip_vs_mygcs_keep(self):
        c = _classify(["SYSID_THISMAV", "SYSID_MYGCS"])
        self.assertTrue(c["SYSID_THISMAV"]["uav"])
        self.assertFalse(c["SYSID_MYGCS"]["uav"])

    def test_compass_dev_id_skip_vs_dec_keep(self):
        c = _classify(["COMPASS_DEV_ID", "COMPASS_DEC"])
        self.assertTrue(c["COMPASS_DEV_ID"]["uav"])
        self.assertFalse(c["COMPASS_DEC"]["uav"])


class TestCaseInsensitive(unittest.TestCase):
    def test_lowercase_input(self):
        c = _classify(["compass_ofs_x", "serial2_protocol"])
        self.assertTrue(c["compass_ofs_x"]["uav"])
        self.assertFalse(c["serial2_protocol"]["uav"])


if __name__ == "__main__":
    unittest.main()
