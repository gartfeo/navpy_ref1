"""Source-level checks for terrain-correct Cesium marker placement.

Covers the five marker hooks under
``src/gcs/frontend/src/components/map/hooks``: ``useAssignmentMarkers``,
``useAvailableTaskMarkers``, ``useDockMarkers``, ``useDetectionMarkers`` and
``useFallbackLocationLayer``.

Of these five, the four live-marker hooks place *self-clamping* Cesium entities
(``heightReference`` ``CLAMP_TO_GROUND`` / ``RELATIVE_TO_GROUND``), so they do
NOT gate on ``terrainReady``: a self-clamping billboard/model needs no
pre-sampled terrain height. They instead gate on ``viewerReady`` (the fresh-load
render race, owned by ``test_viewer_ready_polygon_hooks_js.py``), including
``useAvailableTaskMarkers``: its data is event-driven and largely unexposed to
the race, but PR #105's "close the class fully" pass gated it on ``viewerReady``
anyway for consistency, so its shipped 4th param is ``viewerReady`` — not the
``terrainReady`` the stale test expected. Only ``useFallbackLocationLayer`` gates on
``terrainReady`` here, because its assignment polylines consume terrain-sampled
zone-track heights.

An earlier partially-landed refactor tried to gate all five hooks on
``terrainReady``; that was superseded by the ``viewerReady`` fix.
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


_CLAMP = "heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,"

_MAP_SRC = _read_map_file("CesiumMap.jsx")
_ASSIGNMENT_SRC = _read_map_file("hooks", "useAssignmentMarkers.js")
_AVAILABLE_SRC = _read_map_file("hooks", "useAvailableTaskMarkers.js")
_POI_SRC = _read_map_file("hooks", "useDockMarkers.js")
_DETECTION_SRC = _read_map_file("hooks", "useDetectionMarkers.js")
_DOCK_SRC = _read_map_file("hooks", "useFallbackLocationLayer.js")


class TestTerrainReadyMarkerHooks(unittest.TestCase):
    def test_assignment_markers_self_clamp(self):
        compact = _compact(_ASSIGNMENT_SRC)
        self.assertIn(
            "useAssignmentMarkers(cesiumRef, viewerRef, assignments, storeRef, viewerReady)",
            compact,
        )
        # Both the crosshair billboard and its label self-clamp to terrain, so
        # no terrainReady gate is needed.
        self.assertGreaterEqual(_ASSIGNMENT_SRC.count(_CLAMP), 2)
        self.assertIn("delete existing[tidStr];", _ASSIGNMENT_SRC)
        # Gated on viewerReady (fresh-load race), not terrainReady.
        self.assertIn("[assignments, viewerReady, sysIdKey]", compact)
        self.assertNotIn("if (!terrainReady)", _ASSIGNMENT_SRC)

    def test_available_task_markers_self_clamp(self):
        compact = _compact(_AVAILABLE_SRC)
        self.assertIn(
            "useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks, viewerReady)",
            compact,
        )
        # Icon billboard + label both clamp to terrain.
        self.assertGreaterEqual(_AVAILABLE_SRC.count(_CLAMP), 2)
        # Gated on viewerReady (PR #105's "close the class fully" consistency
        # pass), not terrainReady.
        self.assertIn("[availableTasks, viewerReady]", compact)
        self.assertNotIn("if (!terrainReady)", _AVAILABLE_SRC)

    def test_poi_markers_self_clamp(self):
        # Signature / viewerReady deps are owned by
        # test_viewer_ready_polygon_hooks_js.py; here we only pin the
        # terrain-correct placement and the absence of a terrainReady gate.
        self.assertIn(
            "if (!simDocks || simDocks.length === 0) { ents.simDocks = []; return; }",
            _compact(_POI_SRC),
        )
        # Detection class 0 model + label both clamp to terrain.
        self.assertGreaterEqual(_POI_SRC.count(_CLAMP), 2)
        self.assertNotIn("if (!terrainReady)", _POI_SRC)

    def test_detection_markers_clamp_relative_to_ground(self):
        self.assertIn(
            "if (!detectionPoints || detectionPoints.length === 0) { ents.detectionStart = []; return; }",
            _compact(_DETECTION_SRC),
        )
        # Observation posts sit at plan altitude — relative-to-ground, not clamped.
        self.assertIn(
            "heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,",
            _DETECTION_SRC,
        )
        self.assertNotIn("if (!terrainReady)", _DETECTION_SRC)

    def test_dock_layer_gates_on_terrain_ready(self):
        compact = _compact(_DOCK_SRC)
        self.assertIn(
            "useFallbackLocationLayer(cesiumRef, viewerRef, fallbackLocations, fallbackLocationAssignments, plan, viewerReady, terrainReady, trackPosRef, terrainBaseRef, showTracks)",
            compact,
        )
        # DOCK consumes terrain-sampled zone-track heights for its assignment
        # polylines, so it — and only it, among these five hooks — waits for terrainReady.
        self.assertIn("if (!terrainReady) { entitiesRef.current = []; return; }", compact)
        self.assertIn("[fallbackLocations, fallbackLocationAssignments, plan, showTracks, terrainReady]", compact)

    def test_cesium_map_call_sites(self):
        compact = _compact(_MAP_SRC)
        # Self-clamping live markers are called without terrainReady.
        self.assertIn(
            "useAssignmentMarkers(cesiumRef, viewerRef, assignments || {}, storeRef, viewerReady);",
            compact,
        )
        self.assertIn(
            "useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks || {}, viewerReady);",
            compact,
        )
        self.assertIn(
            "useDockMarkers(cesiumRef, viewerRef, entitiesRef, stablePois, viewerReady);",
            compact,
        )
        self.assertIn(
            "useDetectionMarkers(cesiumRef, viewerRef, entitiesRef, detectionPoints, viewerReady);",
            compact,
        )
        # DOCK layer consumes terrain-sampled heights — the only one called with terrainReady.
        self.assertIn(
            "useFallbackLocationLayer(cesiumRef, viewerRef, fallbackLocations, fallbackLocationAssignments, plan, viewerReady, terrainReady, trackPosRef, terrainBaseRef, showTracks);",
            compact,
        )


if __name__ == "__main__":
    unittest.main()
