"""Tests for vision profile optimized_altitude persistence."""
import json
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

from gcs.backend.routes.vision_profiles import router, _build_catalog

# Minimal profile data for testing
SAMPLE_PROFILES_DATA = {
    "default_profile": "profile_a",
    "profiles": {
        "profile_a": {
            "detector": {
                "reference_height_m": 2.0,
                "imgsz": 640,
                "min_pitch": -60,
                "max_pitch": 20,
                "min_altitude": 30,
            },
            "devices": [
                {
                    "name": "cam1",
                    "camera": {
                        "image_width": 1920,
                        "image_height": 1080,
                        "intrinsics": {
                            "zooms": {"1": {"fx": 2000, "fy": 2000}},
                        },
                    },
                    "gimbal": {"camera_pitch": -15},
                },
            ],
        },
        "profile_b": {
            "detector": {
                "reference_height_m": 2.0,
                "imgsz": 640,
                "min_pitch": -60,
                "max_pitch": 20,
                "min_altitude": 30,
                "optimized_altitude": 250,
            },
            "devices": [
                {
                    "name": "cam2",
                    "camera": {
                        "image_width": 1280,
                        "image_height": 720,
                        "intrinsics": {
                            "zooms": {"1": {"fx": 1500, "fy": 1500}},
                        },
                    },
                    "gimbal": {"camera_pitch": -20},
                },
            ],
        },
    },
}


def _make_app():
    from fastapi import FastAPI
    app = FastAPI()
    app.include_router(router)
    return app


class TestOptimizedAltitude(unittest.TestCase):
    """Test optimized_altitude field in vision profile catalog and update."""

    def setUp(self):
        self.app = _make_app()
        self.client = TestClient(self.app)
        # Deep-copy so each test starts fresh
        self._data = json.loads(json.dumps(SAMPLE_PROFILES_DATA))

    def _patch_load(self):
        """Patch load_profiles to use in-memory data with a fake Path."""
        test = self

        class FakePath:
            def read_text(self_path):
                return json.dumps(test._data)

            def write_text(self_path, text):
                test._data = json.loads(text)

        fake_path = FakePath()

        def _load():
            d = test._data
            return d["profiles"], d["default_profile"], fake_path

        return patch(
            "gcs.backend.routes.vision_profiles.load_profiles",
            side_effect=_load,
        )

    def test_catalog_returns_null_when_no_optimized_altitude(self):
        with self._patch_load():
            resp = self.client.get("/api/vision-profiles")
        self.assertEqual(resp.status_code, 200)
        cat = resp.json()
        self.assertIsNone(cat["profiles"]["profile_a"]["optimized_altitude"])

    def test_catalog_returns_stored_optimized_altitude(self):
        with self._patch_load():
            resp = self.client.get("/api/vision-profiles")
        self.assertEqual(resp.status_code, 200)
        cat = resp.json()
        self.assertEqual(cat["profiles"]["profile_b"]["optimized_altitude"], 250)

    def test_catalog_exposes_detect_and_confirm_ranges_per_zoom(self):
        with self._patch_load():
            cat = _build_catalog()
        zoom = cat["profiles"]["profile_a"]["devices"][0]["zooms"]["1"]
        self.assertIn("detect_range_m", zoom)
        self.assertIn("confirm_range_m", zoom)
        self.assertGreater(zoom["detect_range_m"], zoom["confirm_range_m"])
        # Ranges derive from the diagonal class sizes (the single source);
        # compute from the live values so this stays order-independent.
        from navpy.modules.vision import vision_profiles as _vp
        self.assertAlmostEqual(
            zoom["detect_range_m"], 2000 * _vp.get_class_detect_size(0) / 8, places=2)
        self.assertAlmostEqual(
            zoom["confirm_range_m"], 2000 * _vp._MIN_CLASS_SIZE / 20, places=2)
        # The catalog exposes the per-class dims (single source the frontend reads).
        self.assertIn("detector_class_dimensions", cat)
        tc0 = cat["detector_class_dimensions"]["0"]
        self.assertEqual((tc0["width_m"], tc0["height_m"]), _vp.get_detector_class_dimensions(0))
        self.assertAlmostEqual(tc0["size_m"], _vp.get_class_detect_size(0), places=4)

    def test_put_optimized_altitude_persists(self):
        with self._patch_load():
            resp = self.client.put(
                "/api/vision-profiles/profile_a",
                json={"optimized_altitude": 300},
            )
        self.assertEqual(resp.status_code, 200)
        # Verify it was written to the data
        det = self._data["profiles"]["profile_a"]["detector"]
        self.assertEqual(det["optimized_altitude"], 300)

    def test_put_optimized_altitude_appears_in_catalog(self):
        with self._patch_load():
            resp = self.client.put(
                "/api/vision-profiles/profile_a",
                json={"optimized_altitude": 320},
            )
        self.assertEqual(resp.status_code, 200)
        cat = resp.json()
        self.assertEqual(cat["profiles"]["profile_a"]["optimized_altitude"], 320)

    def test_put_nonexistent_profile_returns_404(self):
        with self._patch_load():
            resp = self.client.put(
                "/api/vision-profiles/nonexistent",
                json={"optimized_altitude": 100},
            )
        self.assertEqual(resp.status_code, 404)

    # --- single-source-of-truth consolidation: pitch + presets persist here ---

    def test_put_dock_presets_persists_to_detector(self):
        """Operator scan presets are owned by the profile JSON, persisted via
        the profile endpoint into detector.dock_presets (the GCS no longer
        keeps a duplicate copy in gcs_settings.json)."""
        with self._patch_load():
            resp = self.client.put(
                "/api/vision-profiles/profile_a",
                json={"dock_presets": {
                    "small": {"altitude_m": 130, "min_pixel_size": 44, "label": "Small"},
                }},
            )
        self.assertEqual(resp.status_code, 200)
        presets = self._data["profiles"]["profile_a"]["detector"]["dock_presets"]
        self.assertEqual(presets["small"]["altitude_m"], 130)
        self.assertEqual(presets["small"]["min_pixel_size"], 44)
        self.assertEqual(
            resp.json()["profiles"]["profile_a"]["dock_presets"]["small"]["altitude_m"],
            130,
        )

    def test_put_dock_presets_merges_per_class(self):
        """Partial preset updates merge — untouched fields are preserved."""
        self._data["profiles"]["profile_a"]["detector"]["dock_presets"] = {
            "medium": {"altitude_m": 200, "min_pixel_size": 45, "label": "Medium"},
        }
        with self._patch_load():
            self.client.put(
                "/api/vision-profiles/profile_a",
                json={"dock_presets": {"medium": {"altitude_m": 175}}},
            )
        medium = self._data["profiles"]["profile_a"]["detector"]["dock_presets"]["medium"]
        self.assertEqual(medium["altitude_m"], 175)
        self.assertEqual(medium["min_pixel_size"], 45)  # untouched field preserved

    def test_old_preset_request_key_is_rejected_before_writing(self):
        before = json.dumps(self._data, sort_keys=True)
        with self._patch_load(), patch("gcs.backend.routes.vision_profiles._restart_navpy_if_active_profile_changed") as restart:
            response = self.client.put("/api/vision-profiles/profile_a", json={"target_presets": {}})
        self.assertEqual(response.status_code, 422)
        self.assertEqual(json.dumps(self._data, sort_keys=True), before)
        restart.assert_not_called()

    def test_put_device_pitch_persists_to_camera_pitch(self):
        """The gimbal-pitch fix: the frontend persists pitch via the device
        endpoint into gimbal.camera_pitch (the value NavPy reads for the mount).
        Previously the gimbal optimize path never wrote it, so it stayed stale."""
        with self._patch_load():
            resp = self.client.put(
                "/api/vision-profiles/profile_a/devices/cam1",
                json={"zoom": "1", "pitch_deg": -30},
            )
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(
            self._data["profiles"]["profile_a"]["devices"][0]["gimbal"]["camera_pitch"],
            -30,
        )


if __name__ == "__main__":
    unittest.main()
