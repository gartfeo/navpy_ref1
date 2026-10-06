"""Source-level checks that the map entity hooks gate on viewerReady.

Regression guard for the fresh-load bug where the mission-boundary outline
(and zone fills / launch-zone outline) failed to render until a manual
"Download Plan" click. Root cause: these hooks' effects ran before the async
Cesium viewer finished init, bailed on `!viewer`, and never re-ran because
their dependency arrays lacked a viewer-ready signal (unlike every sibling
entity hook).

The primary fix (PR #98/#99) covered the high-exposure hooks
(polygon/zone/launch-zone + Ц/НП labels); see TestViewerReadyPolygonHooks.
TestViewerReadyRemainingHooks closes the class fully by asserting the same
gating on the three lower-exposure holdouts (partition handle, sim track-wp
markers, available-task markers) so no entity hook is left on the old
ungated pattern.
"""
import os
import re
import unittest


def _read_map_file(*parts):
    path = os.path.normpath(os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "gcs", "frontend", "src", "components", "map",
        *parts,
    ))
    with open(path, encoding="utf-8") as f:
        return f.read()


def _compact(src):
    return re.sub(r"\s+", " ", src)


_MAP_SRC = _read_map_file("CesiumMap.jsx")
_POLYGON_SRC = _read_map_file("hooks", "usePolygonLayer.js")
_ZONE_SRC = _read_map_file("hooks", "useZoneLayer.js")
_LAUNCH_ZONE_SRC = _read_map_file("hooks", "useLaunchZoneLayer.js")
_TARGET_SRC = _read_map_file("hooks", "useDockMarkers.js")
_DETECTION_SRC = _read_map_file("hooks", "useDetectionMarkers.js")
_PARTITION_SRC = _read_map_file("hooks", "usePartitionHandle.js")
_TRACK_WP_SRC = _read_map_file("hooks", "useTrackWpMarkers.js")
_AVAILABLE_TASK_SRC = _read_map_file("hooks", "useAvailableTaskMarkers.js")


class TestViewerReadyPolygonHooks(unittest.TestCase):
    def test_polygon_layer_accepts_and_depends_on_viewer_ready(self):
        compact = _compact(_POLYGON_SRC)
        # Signature includes viewerReady as the trailing param
        self.assertIn(
            "usePolygonLayer(cesiumRef, viewerRef, entitiesRef, polygonRef, polygon, phase, viewerReady)",
            compact,
        )
        # Effect dependency array includes viewerReady so it re-runs once ready.
        # Membership check (not exact-array) so it survives future added deps —
        # see TestViewerReadyRemainingHooks / the module docstring.
        self.assertRegex(compact, r"\[polygon,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_zone_layer_accepts_and_depends_on_viewer_ready(self):
        compact = _compact(_ZONE_SRC)
        self.assertIn(
            "useZoneLayer(cesiumRef, viewerRef, entitiesRef, zonePosRef, zoneCount, showZones, searchPattern, viewerReady)",
            compact,
        )
        self.assertRegex(compact, r"\[zoneCount,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_launch_zone_layer_accepts_and_depends_on_viewer_ready(self):
        compact = _compact(_LAUNCH_ZONE_SRC)
        self.assertIn(
            "useLaunchZoneLayer(cesiumRef, viewerRef, entitiesRef, lzPosRef, minLzPosRef, "
            "lzLen, minLzLen, showLaunchZone, viewerReady)",
            compact,
        )
        self.assertRegex(compact, r"\[lzLen,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_target_markers_accepts_and_depends_on_viewer_ready(self):
        # Ц1/Ц2 sim-target labels — same fresh-load race as the polygon outline.
        compact = _compact(_TARGET_SRC)
        self.assertIn(
            "useDockMarkers(cesiumRef, viewerRef, entitiesRef, simDocks, viewerReady)",
            compact,
        )
        # Membership check (not exact-array): PR #99 appended i18n.language to
        # these deps, silently breaking the old exact-substring assertion.
        self.assertRegex(compact, r"\[simDocks,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_detection_markers_accepts_and_depends_on_viewer_ready(self):
        # НП observation-post labels — same fresh-load race as the polygon outline.
        compact = _compact(_DETECTION_SRC)
        self.assertIn(
            "useDetectionMarkers(cesiumRef, viewerRef, entitiesRef, detectionPoints, viewerReady)",
            compact,
        )
        # Membership check (not exact-array): PR #99 appended i18n.language to
        # these deps, silently breaking the old exact-substring assertion.
        self.assertRegex(compact, r"\[detectionPoints,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_cesium_map_passes_viewer_ready_to_all_hooks(self):
        compact = _compact(_MAP_SRC)
        self.assertIn(
            "usePolygonLayer(cesiumRef, viewerRef, entitiesRef, polygonRef, polygon, phase, viewerReady);",
            compact,
        )
        self.assertIn(
            "useZoneLayer(cesiumRef, viewerRef, entitiesRef, zonePosRef, zoneCount, showZones, searchPattern, viewerReady);",
            compact,
        )
        self.assertIn(
            "useLaunchZoneLayer(cesiumRef, viewerRef, entitiesRef, lzPosRef, minLzPosRef, "
            "lzLen, minLzLen, showLaunchZone, viewerReady);",
            compact,
        )
        self.assertIn(
            "useDockMarkers(cesiumRef, viewerRef, entitiesRef, stableTargets, viewerReady);",
            compact,
        )
        self.assertIn(
            "useDetectionMarkers(cesiumRef, viewerRef, entitiesRef, detectionPoints, viewerReady);",
            compact,
        )


class TestViewerReadyRemainingHooks(unittest.TestCase):
    """Lower-exposure holdouts fixed to fully close the fresh-load render race.

    These three entity hooks bailed on `!viewer` without a viewer-ready signal
    in their deps, same structural gap as the polygon hooks. Real fresh-load
    exposure is lower (partition handle only shows in PLANNING once the viewer
    is up; track-wp markers are sim-mode only; available-task markers are
    runtime-event-driven), but leaving them on the old pattern keeps a latent
    race. These assertions pin them to the sibling pattern.
    """

    def test_partition_handle_accepts_and_depends_on_viewer_ready(self):
        # PLANNING-only partition direction line / rotation handle.
        compact = _compact(_PARTITION_SRC)
        self.assertIn(
            "polygon, partitionAngleDeg, effectiveSets, searchPattern, phase, viewerReady",
            compact,
        )
        # Membership check (not exact-array) so it survives future added deps —
        # exact-substring deps assertions are what silently broke when PR #99
        # added i18n.language to the target/detection hooks (see module docstring).
        self.assertRegex(compact, r"\[polygon, effectiveSets,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_track_wp_markers_accepts_and_depends_on_viewer_ready(self):
        # Sim-mode track waypoint markers — can race on a fresh load in sim mode.
        compact = _compact(_TRACK_WP_SRC)
        self.assertIn(
            "plan, simMode, phase, simDockWps, viewerReady,",
            compact,
        )
        self.assertRegex(compact, r"\[active,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_available_task_markers_accepts_and_depends_on_viewer_ready(self):
        # Available-task markers — runtime-event-driven, low fresh-load exposure.
        compact = _compact(_AVAILABLE_TASK_SRC)
        self.assertIn(
            "useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks, viewerReady)",
            compact,
        )
        self.assertRegex(compact, r"\[availableTasks,[^\]]*\bviewerReady\b[^\]]*\]")

    def test_cesium_map_passes_viewer_ready_to_remaining_hooks(self):
        compact = _compact(_MAP_SRC)
        self.assertIn(
            "usePartitionHandle( cesiumRef, viewerRef, entitiesRef, polygonRef, "
            "partitionAngleDegRef, polygon, partitionAngleDeg, effectiveSets, "
            "searchPattern, phase, viewerReady, );",
            compact,
        )
        self.assertIn(
            "useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks || {}, viewerReady);",
            compact,
        )
        self.assertIn(
            "useTrackWpMarkers(cesiumRef, viewerRef, entitiesRef, trackPosRef, "
            "plan, simMode, phase, simDockWps, viewerReady);",
            compact,
        )


if __name__ == "__main__":
    unittest.main()
