"""Detector camera and video stream rows of preflightReadiness.js (runs under Node).

docs/design/gcs_preflight_camera_row.md: every camera state, the advertise
grace (``camera_wait_over``), the row order, and that the rows are caution at
worst and never reach the launch gate.
"""
import json
import unittest
from pathlib import Path

from tests.gcs.test_preflight_readiness_js import _GOOD_LINKED, _by_key, _run_js
from tests.gcs.test_preflight_remediation_js import _rem, _top

CAM1, CAM2 = 100, 101
URL = "http://192.168.144.10:8889/uav1/whep"


def _calls(fn, cases):
    """Results of ``fn(case)`` for every case, in one Node run."""
    script = (f"console.log(JSON.stringify({json.dumps(cases)}.map((c) => {fn}(c))));")
    return json.loads(_run_js(script))


def _camera(component=CAM1, **fields):
    camera = {"component": component, "lost": False, "vendor": "navpy", "status": "active",
              "discovery": "done", "has_video": False, "rejected": []}
    camera.update(fields)
    return camera


def _vehicle(cameras=(), **fields):
    vehicle = dict(_GOOD_LINKED, cameras=list(cameras), video_streams=[],
                   camera_wait_over=False, video_stream_expected=False,
                   video_stream_duplicates=[])
    vehicle.update(fields)
    return vehicle


def _rows(fn, vehicles):
    return [(r["status"], (r["detail"] or {}).get("state")) for r in _calls(fn, vehicles)]


class TestDetectorCameraRow(unittest.TestCase):
    FN = "computeDetectorCameraCheck"

    def test_dash_while_the_companion_is_not_ok(self):
        out = _calls(self.FN, [
            _vehicle([_camera()], companion_status=status) for status in ("checking", "down", None)
        ])
        self.assertEqual(out, [{"key": "detectorCamera", "status": "na", "detail": None}] * 3)

    def test_no_camera_checks_until_the_grace_is_over(self):
        self.assertEqual(_rows(self.FN, [_vehicle(), _vehicle(camera_wait_over=True)]), [
            ("na", "checking"), ("warn", "notAdvertised"),
        ])

    def test_each_camera_state(self):
        cases = [
            (_camera(lost=True), ("warn", "lost")),
            (_camera(vendor=None, status="active", discovery="running"), ("na", "checking")),
            (_camera(vendor=None, discovery="failed"), ("na", "notReported")),
            (_camera(vendor="other"), ("na", "notReported")),
            (_camera(vendor="navpy"), ("go", "ready")),
            (_camera(vendor="navpy_sim"), ("go", "simulated")),
            (_camera(vendor="navpy", status="critical"), ("warn", "noFrames")),
            (_camera(vendor="navpy_sim", status="critical"), ("warn", "noFrames")),
            (_camera(vendor="navpy", status="other"), ("na", "notReported")),
            # Lost wins over everything a lost record still holds.
            (_camera(lost=True, vendor="other", status="other"), ("warn", "lost")),
        ]
        vehicles = [_vehicle([camera], camera_wait_over=True) for camera, _ in cases]
        self.assertEqual(_rows(self.FN, vehicles), [expected for _, expected in cases])

    def test_the_worst_camera_wins_with_its_count(self):
        ready, frames = _camera(), _camera(CAM2, status="critical")
        cases = [
            ([ready, frames], ("warn", "noFrames", 1, 2)),
            ([frames, dict(frames, component=CAM1)], ("warn", "noFrames", 2, 2)),
            ([ready, dict(ready, component=CAM2)], ("go", "ready", 2, 2)),
            ([ready, _camera(CAM2, vendor=None, discovery="running")], ("na", "checking", 1, 2)),
            ([_camera(vendor="other"), _camera(CAM2, vendor=None, discovery="running")],
             ("na", "checking", 1, 2)),
            ([_camera(vendor="other"), ready], ("na", "notReported", 1, 2)),
            ([_camera(lost=True), frames], ("warn", "lost", 1, 2)),  # same rank: the first
        ]
        out = _calls(self.FN, [_vehicle(cameras) for cameras, _ in cases])
        self.assertEqual(
            [(r["status"], r["detail"]["state"], r["detail"]["count"], r["detail"]["total"])
             for r in out],
            [expected for _, expected in cases],
        )

    def test_one_camera_has_no_count(self):
        (out,) = _calls(self.FN, [_vehicle([_camera()])])
        self.assertEqual(out["detail"], {"state": "ready"})


class TestVideoStreamRow(unittest.TestCase):
    FN = "computeVideoStreamCheck"

    def test_dash_while_the_companion_is_not_ok(self):
        out = _calls(self.FN, [
            _vehicle([_camera()], video_streams=[{"whep_url": URL}], companion_status=status)
            for status in ("checking", "down")
        ])
        self.assertEqual(out, [{"key": "videoStream", "status": "na", "detail": None}] * 2)

    def test_each_state(self):
        stream = [{"key": f"{CAM1}:1", "whep_url": URL}]
        rejected = [{"stream_id": 1, "reason": "userinfo"}]
        cases = [
            (_vehicle([_camera(has_video=True)], video_streams=stream), ("go", "streams")),
            (_vehicle([_camera(has_video=True)], video_streams=stream,
                      video_stream_duplicates=[{"key": f"{CAM1}:1", "sys_ids": [5]}]),
             ("warn", "duplicate")),
            (_vehicle([_camera(has_video=True, rejected=rejected)]), ("warn", "uriRejected")),
            (_vehicle([_camera(vendor=None, discovery="failed")]), ("warn", "discoveryFailed")),
            (_vehicle([_camera(lost=True, has_video=True)]), ("warn", "lost")),
            (_vehicle([_camera(has_video=True, discovery="running")]), ("na", "checking")),
            (_vehicle([_camera()]), ("na", "noneConfigured")),
            (_vehicle([_camera(), _camera(CAM2, lost=True)]), ("na", "noneConfigured")),
            (_vehicle(video_stream_expected=True), ("na", "checking")),
            (_vehicle(video_stream_expected=True, camera_wait_over=True),
             ("warn", "notAdvertised")),
            (_vehicle([_camera()], video_stream_expected=True, camera_wait_over=True),
             ("warn", "notAdvertised")),
            (_vehicle(), ("na", None)),
            (_vehicle(camera_wait_over=True), ("na", None)),  # none expected: the detector row says why
            # Design rule 9 counts every camera seen, lost ones too.
            (_vehicle([_camera(lost=True, has_video=False)]), ("na", "noneConfigured")),
            (_vehicle([_camera(lost=True, has_video=None)]), ("na", None)),
        ]
        self.assertEqual(_rows(self.FN, [v for v, _ in cases]), [e for _, e in cases])

    def test_details(self):
        stream = {"key": f"{CAM1}:1", "whep_url": URL}
        out = _calls(self.FN, [
            _vehicle([_camera(has_video=True)], video_streams=[stream, dict(stream, key="100:2")]),
            _vehicle([_camera(has_video=True)], video_streams=[stream], video_stream_duplicates=[
                {"key": "100:1", "sys_ids": [9, 5]}, {"key": "100:2", "sys_ids": [5, 7]}]),
            _vehicle([_camera(has_video=True, rejected=[{"stream_id": 2, "reason": "port"},
                                                        {"stream_id": 3, "reason": "path"}])]),
        ])
        self.assertEqual([r["detail"] for r in out], [
            {"state": "streams", "count": 2},
            {"state": "duplicate", "sysIds": [5, 7, 9]},
            {"state": "uriRejected", "reason": "port"},
        ])

    def test_first_that_applies(self):
        stream = [{"key": f"{CAM1}:1", "whep_url": URL}]
        duplicate = [{"key": f"{CAM1}:1", "sys_ids": [5]}]
        rejected = [{"stream_id": 2, "reason": "port"}]
        cases = [
            (_vehicle([_camera(has_video=True, rejected=rejected), _camera(CAM2, discovery="failed")],
                      video_streams=stream, video_stream_duplicates=duplicate), "duplicate"),
            (_vehicle([_camera(has_video=True, rejected=rejected), _camera(CAM2, discovery="failed")],
                      video_streams=stream), "uriRejected"),
            (_vehicle([_camera(discovery="failed"), _camera(CAM2, lost=True, has_video=True)]),
             "discoveryFailed"),
            (_vehicle([_camera(discovery="running"), _camera(CAM2, lost=True, has_video=True)]),
             "lost"),
            (_vehicle([_camera(has_video=True), _camera(CAM2, discovery="running")],
                      video_streams=stream), "checking"),
            (_vehicle([_camera(has_video=True)], video_streams=stream,
                      video_stream_expected=True, camera_wait_over=True), "streams"),
            # A lost camera's rejected stream is not this generation's problem.
            (_vehicle([_camera(lost=True, has_video=False, rejected=rejected)]),
             "noneConfigured"),
        ]
        self.assertEqual([state for _, state in _rows(self.FN, [v for v, _ in cases])],
                         [e for _, e in cases])


class TestCautionOnly(unittest.TestCase):
    """The camera rows raise a UAV to caution at worst and never touch LAUNCH."""

    CAMERA_STATES = [
        dict(cameras=[], camera_wait_over=True),
        dict(cameras=[_camera(status="critical")]),
        dict(cameras=[_camera(lost=True, has_video=True)]),
        dict(cameras=[_camera(vendor=None, discovery="failed")]),
        dict(cameras=[_camera(has_video=True, rejected=[{"stream_id": 1, "reason": "port"}])]),
        dict(cameras=[_camera(has_video=True)], video_streams=[{"key": "100:1", "whep_url": URL}],
             video_stream_duplicates=[{"key": "100:1", "sys_ids": [5]}]),
        dict(video_stream_expected=True, camera_wait_over=True),
    ]

    def _overall_and_launch(self, vehicles):
        script = (
            f"console.log(JSON.stringify({json.dumps(vehicles)}.map((v) => "
            "[computeVehicleReadiness(v).overall, computeLaunchReadiness(v).ready, "
            "computeFleetStatus([v])])));"
        )
        return json.loads(_run_js(script))

    def test_each_caution_reads_caution_and_launch_is_unchanged(self):
        out = self._overall_and_launch([_vehicle(**s) for s in self.CAMERA_STATES])
        self.assertEqual(out, [["warn", True, "warn"]] * len(self.CAMERA_STATES))

    def test_a_launch_blocker_stays_nogo_and_blocked_whatever_the_cameras(self):
        blocked = [_vehicle(**dict(s, gps_fix=0)) for s in self.CAMERA_STATES]
        self.assertEqual(self._overall_and_launch(blocked),
                         [["nogo", False, "nogo"]] * len(self.CAMERA_STATES))

    def test_launch_readiness_ignores_every_camera_field(self):
        plain = dict(_GOOD_LINKED)
        variants = [plain] + [dict(plain, **s) for s in self.CAMERA_STATES]
        script = (
            f"console.log(JSON.stringify({json.dumps(variants)}.map((v) => "
            "computeLaunchReadiness(v))));"
        )
        out = json.loads(_run_js(script))
        self.assertTrue(all(r == out[0] for r in out))

    def test_ready_cameras_keep_go(self):
        v = _vehicle([_camera(has_video=True)], video_streams=[{"key": "100:1", "whep_url": URL}])
        script = f"console.log(JSON.stringify(computeVehicleReadiness({json.dumps(v)})));"
        out = json.loads(_run_js(script))
        self.assertEqual(out["overall"], "go")
        rows = _by_key(out)
        self.assertEqual((rows["detectorCamera"]["status"], rows["videoStream"]["status"]),
                         ("go", "go"))


class TestCameraHints(unittest.TestCase):
    def test_each_caution_state_has_its_hint(self):
        cases = {
            ("detectorCamera", "noFrames"): "preflight.fix.cameraNoFrames",
            ("detectorCamera", "lost"): "preflight.fix.cameraLost",
            ("detectorCamera", "notAdvertised"): "preflight.fix.cameraNotAdvertised",
            ("videoStream", "duplicate"): "preflight.fix.streamDuplicate",
            ("videoStream", "uriRejected"): "preflight.fix.streamUriRejected",
            ("videoStream", "discoveryFailed"): "preflight.fix.streamDiscoveryFailed",
            ("videoStream", "lost"): "preflight.fix.streamLost",
            ("videoStream", "notAdvertised"): "preflight.fix.streamNotAdvertised",
        }
        for (key, state), hint in cases.items():
            check = {"key": key, "status": "warn", "detail": {"state": state}}
            self.assertEqual(_rem(check), {"hintKey": hint, "action": None}, (key, state))

    def test_grey_and_go_have_no_hint(self):
        self.assertIsNone(_rem({"key": "detectorCamera", "status": "na",
                                "detail": {"state": "checking"}}))
        self.assertIsNone(_rem({"key": "videoStream", "status": "go",
                                "detail": {"state": "streams"}}))

    def test_camera_cautions_follow_the_launch_gate_rows(self):
        camera = {"key": "detectorCamera", "status": "warn", "detail": {"state": "noFrames"}}
        stream = {"key": "videoStream", "status": "warn", "detail": {"state": "lost"}}
        battery = {"key": "battery", "status": "warn", "detail": {}}
        ekf = {"key": "ekf", "status": "warn", "detail": {}}
        mission = {"key": "mission", "status": "nogo", "detail": {}}
        self.assertEqual(_top([ekf, stream, camera, battery])["check"]["key"], "battery")
        self.assertEqual(_top([ekf, stream, camera])["check"]["key"], "detectorCamera")
        self.assertEqual(_top([ekf, stream])["check"]["key"], "videoStream")
        self.assertEqual(_top([camera, mission])["check"]["key"], "mission")



LOCALES = Path(__file__).resolve().parents[2] / "src" / "gcs" / "frontend" / "src" / "locales"
CAMERA_STATES = ("ready", "simulated", "noFrames", "lost", "notAdvertised", "checking",
                 "notReported")
STREAM_STATES = ("checking", "noneConfigured", "discoveryFailed", "uriRejected", "duplicate",
                 "lost", "notAdvertised")
HINTS = ("cameraNoFrames", "cameraLost", "cameraNotAdvertised", "streamDuplicate",
         "streamUriRejected", "streamDiscoveryFailed", "streamLost", "streamNotAdvertised")


class TestCameraRowLocales(unittest.TestCase):
    def test_every_camera_row_text_is_translated(self):
        keys = ["detectorCamera", "videoStream", "ccConnected", "ccTooltip", "cameraCount",
                "streamCount_one", "streamCount_other"]
        keys += [f"cameraState.{s}" for s in CAMERA_STATES]
        keys += [f"streamState.{s}" for s in STREAM_STATES]
        keys += [f"fix.{h}" for h in HINTS]
        for name in ("en.json", "hy.json"):
            preflight = json.loads((LOCALES / name).read_text(encoding="utf-8"))["preflight"]
            for key in keys:
                node = preflight
                for part in key.split("."):
                    node = node.get(part) if isinstance(node, dict) else None
                self.assertIsInstance(node, str, (name, key))
                self.assertTrue(node.strip(), (name, key))
            self.assertNotIn("ccActive", preflight, name)  # now ccConnected

    def test_the_badge_tooltip_says_what_connected_means(self):
        en = json.loads((LOCALES / "en.json").read_text(encoding="utf-8"))
        self.assertEqual(en["preflight"]["ccConnected"], "Connected")
        self.assertEqual(en["preflight"]["ccTooltip"],
                         "NavPy heartbeat in the last 5 s; not mission or approach state")
        self.assertEqual(en["vehicle"]["ccActive"],
                         "Companion connected: NavPy heartbeat in the last 5 s; "
                         "not mission or approach state")


if __name__ == "__main__":
    unittest.main()
