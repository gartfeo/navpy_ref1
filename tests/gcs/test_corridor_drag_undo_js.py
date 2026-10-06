"""Tests for drag undo — handleDragStart records pre-drag positions."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_HANDLERS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks", "handlers",
))


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_FILE = os.path.join(_HANDLERS_DIR, "handleDragStart.js")
_DRAG_START_JS = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)

# Provide stubs for pickCartographic / pickEntity that handleDragStart imports
_MOCK_PREAMBLE = """\
let _mockHit = null;
let _mockLatLon = null;
function pickEntity(C, v, pos) { return _mockHit; }
function pickCartographic(C, v, pos) { return _mockLatLon; }
"""


def _run_js(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    full = _MOCK_PREAMBLE + _DRAG_START_JS + "\n" + script
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestCorridorDragRecord(unittest.TestCase):
    """handleDragStart should call onCorridorDragRecord with pre-drag position."""

    def test_corridor_point_drag_records_old_position(self):
        """Dragging a corridor point records its pre-drag position for undo."""
        result = _run_js("""
        _mockHit = { type: 'setCorridorPoint', setIdx: 0, corridorIndex: 1 };
        _mockLatLon = { lat: 32.0, lon: 35.0 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          onSetCorridorPointDrag: () => {},
          onCorridorDragRecord: (si, ci, pos) => recorded.push({ si, ci, pos }),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[
            { lat: 31.0, lon: 34.0 },
            { lat: 31.5, lon: 34.5 },
            { lat: 32.0, lon: 35.0 },
          ]] },
          setLaunchPointsRef: { current: [{ lat: 30.0, lon: 33.0 }] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        const result = handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({
          recorded,
          dragType: result?.dragState?.type,
          corridorIndex: result?.dragState?.corridorIndex,
        }));
        """)
        self.assertEqual(len(result["recorded"]), 1)
        rec = result["recorded"][0]
        self.assertEqual(rec["si"], 0)
        self.assertEqual(rec["ci"], 1)
        self.assertAlmostEqual(rec["pos"]["lat"], 31.5)
        self.assertAlmostEqual(rec["pos"]["lon"], 34.5)
        self.assertEqual(result["dragType"], "setCorridorPoint")

    def test_launch_point_drag_records_old_position(self):
        """Dragging a launch point (corridorIndex -1) records it for undo."""
        result = _run_js("""
        _mockHit = { type: 'setCorridorPoint', setIdx: 0, corridorIndex: -1 };
        _mockLatLon = { lat: 32.0, lon: 35.0 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          onSetCorridorPointDrag: () => {},
          onCorridorDragRecord: (si, ci, pos) => recorded.push({ si, ci, pos }),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[]] },
          setLaunchPointsRef: { current: [{ lat: 30.0, lon: 33.0 }] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        const result = handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({
          recorded,
          dragType: result?.dragState?.type,
          corridorIndex: result?.dragState?.corridorIndex,
        }));
        """)
        self.assertEqual(len(result["recorded"]), 1)
        rec = result["recorded"][0]
        self.assertEqual(rec["si"], 0)
        self.assertEqual(rec["ci"], -1)
        self.assertAlmostEqual(rec["pos"]["lat"], 30.0)
        self.assertAlmostEqual(rec["pos"]["lon"], 33.0)

    def test_no_corridor_record_for_vertex_drag(self):
        """Vertex drags should not call onCorridorDragRecord."""
        result = _run_js("""
        _mockHit = { type: 'vertex', index: 2 };
        _mockLatLon = { lat: 32.0, lon: 35.0 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          isDrawing: false,
          polygon: [{ lat: 31, lon: 34 }, { lat: 31, lon: 35 }, { lat: 32, lon: 35 }],
          onVertexDrag: () => {},
          onCorridorDragRecord: (si, ci, pos) => recorded.push({ si, ci, pos }),
          onPolygonDragRecord: () => {},
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[]] },
          setLaunchPointsRef: { current: [null] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({ recorded }));
        """)
        self.assertEqual(len(result["recorded"]), 0)

    def test_multi_set_corridor_drag_records_correct_set(self):
        """Dragging a corridor point in set 1 records the correct set index."""
        result = _run_js("""
        _mockHit = { type: 'setCorridorPoint', setIdx: 1, corridorIndex: 0 };
        _mockLatLon = { lat: 32.0, lon: 35.0 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          onSetCorridorPointDrag: () => {},
          onCorridorDragRecord: (si, ci, pos) => recorded.push({ si, ci, pos }),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [
            [{ lat: 31.0, lon: 34.0 }],
            [{ lat: 40.0, lon: 44.0 }, { lat: 41.0, lon: 45.0 }],
          ] },
          setLaunchPointsRef: { current: [{ lat: 30.0, lon: 33.0 }, { lat: 39.0, lon: 43.0 }] },
          activeSetIndex: 1,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({ recorded }));
        """)
        self.assertEqual(len(result["recorded"]), 1)
        rec = result["recorded"][0]
        self.assertEqual(rec["si"], 1)
        self.assertEqual(rec["ci"], 0)
        self.assertAlmostEqual(rec["pos"]["lat"], 40.0)
        self.assertAlmostEqual(rec["pos"]["lon"], 44.0)


class TestPolygonDragRecord(unittest.TestCase):
    """handleDragStart should call onPolygonDragRecord for polygon-related drags."""

    def test_vertex_drag_records_polygon(self):
        """Dragging a vertex records the full polygon snapshot for undo."""
        result = _run_js("""
        const poly = [{ lat: 31, lon: 34 }, { lat: 31, lon: 35 }, { lat: 32, lon: 35 }];
        _mockHit = { type: 'vertex', index: 1 };
        _mockLatLon = { lat: 32.0, lon: 35.0 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          isDrawing: false,
          polygon: poly,
          onVertexDrag: () => {},
          onPolygonDragRecord: (old) => recorded.push(old),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[]] },
          setLaunchPointsRef: { current: [null] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({ recorded }));
        """)
        self.assertEqual(len(result["recorded"]), 1)
        snap = result["recorded"][0]
        self.assertEqual(len(snap), 3)
        self.assertAlmostEqual(snap[1]["lat"], 31)
        self.assertAlmostEqual(snap[1]["lon"], 35)

    def test_polygon_move_records_polygon(self):
        """Dragging the whole polygon records the full polygon snapshot for undo."""
        result = _run_js("""
        const poly = [{ lat: 31, lon: 34 }, { lat: 31, lon: 35 }, { lat: 32, lon: 35 }];
        _mockHit = { type: 'polygon' };
        _mockLatLon = { lat: 31.5, lon: 34.5 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          isDrawing: false,
          polygon: poly,
          onPolygonMove: () => {},
          onPolygonDragRecord: (old) => recorded.push(old),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[]] },
          setLaunchPointsRef: { current: [null] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({ recorded }));
        """)
        self.assertEqual(len(result["recorded"]), 1)
        snap = result["recorded"][0]
        self.assertEqual(len(snap), 3)

    def test_midpoint_drag_records_polygon(self):
        """Dragging from a midpoint records the pre-insert polygon snapshot."""
        result = _run_js("""
        const poly = [{ lat: 31, lon: 34 }, { lat: 31, lon: 35 }, { lat: 32, lon: 35 }];
        _mockHit = { type: 'midpoint', index: 0 };
        _mockLatLon = { lat: 31.0, lon: 34.5 };

        const recorded = [];
        const cbRef = { current: {
          editable: true,
          isDrawing: false,
          polygon: poly,
          onMidpointInsert: () => {},
          onPolygonDragRecord: (old) => recorded.push(old),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[]] },
          setLaunchPointsRef: { current: [null] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({ recorded }));
        """)
        self.assertEqual(len(result["recorded"]), 1)
        # Snapshot should be the original 3-vertex polygon (before midpoint insert)
        snap = result["recorded"][0]
        self.assertEqual(len(snap), 3)

    def test_no_polygon_record_for_corridor_drag(self):
        """Corridor point drags should not call onPolygonDragRecord."""
        result = _run_js("""
        _mockHit = { type: 'setCorridorPoint', setIdx: 0, corridorIndex: 0 };
        _mockLatLon = { lat: 32.0, lon: 35.0 };

        const polyRecorded = [];
        const cbRef = { current: {
          editable: true,
          onSetCorridorPointDrag: () => {},
          onCorridorDragRecord: () => {},
          onPolygonDragRecord: (old) => polyRecorded.push(old),
          onDragStart: () => {},
        }};
        const refs = {
          setCorridorPointsRef: { current: [[{ lat: 31.0, lon: 34.0 }]] },
          setLaunchPointsRef: { current: [{ lat: 30.0, lon: 33.0 }] },
          activeSetIndex: 0,
          lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
        };

        handleDragStart({}, {
          scene: { screenSpaceCameraController: { enableRotate: true } }
        }, { position: {} }, cbRef, refs);
        console.log(JSON.stringify({ polyRecorded }));
        """)
        self.assertEqual(len(result["polyRecorded"]), 0)


if __name__ == "__main__":
    unittest.main()
