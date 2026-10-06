"""Tests for lerpAnim.js — interpolation helpers for smooth entity animation."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "utils",
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


_JS_FILE = os.path.join(_UTILS_DIR, "lerpAnim.js")
_LERP_JS = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)

# Minimal Cesium stubs for Node.js
_CESIUM_STUB = """\
function _Cartesian3(x, y, z) { this.x = x || 0; this.y = y || 0; this.z = z || 0; }
_Cartesian3.lerp = function(a, b, t, result) {
  result.x = a.x + (b.x - a.x) * t;
  result.y = a.y + (b.y - a.y) * t;
  result.z = a.z + (b.z - a.z) * t;
  return result;
};
function _Quaternion(x, y, z, w) { this.x = x || 0; this.y = y || 0; this.z = z || 0; this.w = w || 0; }
_Quaternion.slerp = function(a, b, t, result) {
  result.x = a.x + (b.x - a.x) * t;
  result.y = a.y + (b.y - a.y) * t;
  result.z = a.z + (b.z - a.z) * t;
  result.w = a.w + (b.w - a.w) * t;
  return result;
};
const Cesium = { Cartesian3: _Cartesian3, Quaternion: _Quaternion };
"""


def _run_js(script):
    code = _CESIUM_STUB + "\n" + _LERP_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestLerpFraction(unittest.TestCase):
    def test_at_start(self):
        out = _run_js("console.log(lerpFraction(Date.now()));")
        self.assertAlmostEqual(float(out), 0.0, places=1)

    def test_after_duration(self):
        out = _run_js("console.log(lerpFraction(Date.now() - 200));")
        self.assertAlmostEqual(float(out), 1.0, places=1)

    def test_clamped_past_duration(self):
        out = _run_js("console.log(lerpFraction(Date.now() - 1000));")
        self.assertEqual(float(out), 1.0)

    def test_midpoint(self):
        out = _run_js("console.log(lerpFraction(Date.now() - 100));")
        val = float(out)
        self.assertGreater(val, 0.3)
        self.assertLess(val, 0.7)


class TestAdvanceAnim(unittest.TestCase):
    def test_first_sample_from_equals_to(self):
        out = _run_js("""
            const pos = { x: 1, y: 2, z: 3 };
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            const a = advanceAnim(undefined, pos, ori, Cesium);
            console.log(JSON.stringify({
                fromEqTo: a.fromPos === a.toPos,
                oriEqTo: a.fromOri === a.toOri,
            }));
        """)
        data = json.loads(out)
        self.assertTrue(data["fromEqTo"])
        self.assertTrue(data["oriEqTo"])

    def test_first_sample_default_duration(self):
        out = _run_js("""
            const a = advanceAnim(undefined, {x:0,y:0,z:0}, {x:0,y:0,z:0,w:1}, Cesium);
            console.log(JSON.stringify({ d: a.duration, ema: a.emaInterval }));
        """)
        data = json.loads(out)
        self.assertEqual(data["d"], 600)
        self.assertEqual(data["ema"], 200)

    def test_adaptive_duration_at_5hz(self):
        """At 200ms intervals, EMA settles near 200 → duration near 600."""
        out = _run_js("""
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            let a = advanceAnim(undefined, {x:0,y:0,z:0}, ori, Cesium);
            // Simulate several 200ms intervals
            for (let i = 0; i < 10; i++) {
                a.startTime = Date.now() - 200;
                a = advanceAnim(a, {x:i,y:0,z:0}, ori, Cesium);
            }
            console.log(JSON.stringify({ d: a.duration, ema: a.emaInterval }));
        """)
        data = json.loads(out)
        self.assertAlmostEqual(data["ema"], 200, delta=5)
        self.assertAlmostEqual(data["d"], 600, delta=15)

    def test_noise_resistance(self):
        """A single jittery packet shouldn't drastically change duration."""
        out = _run_js("""
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            let a = advanceAnim(undefined, {x:0,y:0,z:0}, ori, Cesium);
            // 5 normal 200ms intervals
            for (let i = 0; i < 5; i++) {
                a.startTime = Date.now() - 200;
                a = advanceAnim(a, {x:i,y:0,z:0}, ori, Cesium);
            }
            const beforeSpike = a.duration;
            // One noisy 800ms interval
            a.startTime = Date.now() - 800;
            a = advanceAnim(a, {x:99,y:0,z:0}, ori, Cesium);
            const afterSpike = a.duration;
            // Raw jump would be 800*3=2400 vs ~600; EMA limits actual change
            const rawJump = 800 * 3 - beforeSpike;
            const actualJump = afterSpike - beforeSpike;
            console.log(JSON.stringify({
                before: beforeSpike, after: afterSpike,
                rawJump: rawJump, actualJump: actualJump,
            }));
        """)
        data = json.loads(out)
        # EMA absorbs most of the spike — actual jump is < 50% of raw
        self.assertLess(data["actualJump"], data["rawJump"] * 0.5)

    def test_duration_clamped_upper(self):
        """Duration clamped to 2000 after sustained long intervals."""
        out = _run_js("""
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            let a = advanceAnim(undefined, {x:0,y:0,z:0}, ori, Cesium);
            // Repeated 5000ms intervals to push EMA well above clamp
            for (let i = 0; i < 20; i++) {
                a.startTime = Date.now() - 5000;
                a = advanceAnim(a, {x:i,y:0,z:0}, ori, Cesium);
            }
            console.log(a.duration);
        """)
        self.assertEqual(int(out), 2000)

    def test_duration_clamped_lower(self):
        """Duration clamped to 100 after sustained very short intervals."""
        out = _run_js("""
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            let a = advanceAnim(undefined, {x:0,y:0,z:0}, ori, Cesium);
            // Repeated 10ms intervals to push EMA well below clamp
            for (let i = 0; i < 20; i++) {
                a.startTime = Date.now() - 10;
                a = advanceAnim(a, {x:i,y:0,z:0}, ori, Cesium);
            }
            console.log(a.duration);
        """)
        self.assertEqual(int(out), 100)

    def test_subsequent_sample_captures_current(self):
        """After full duration, fromPos should equal previous toPos."""
        out = _run_js("""
            const p1 = { x: 0, y: 0, z: 0 };
            const p2 = { x: 10, y: 0, z: 0 };
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            let a = advanceAnim(undefined, p1, ori, Cesium);
            a.startTime = Date.now() - a.duration - 100;
            const b = advanceAnim(a, p2, ori, Cesium);
            console.log(JSON.stringify({
                fromX: b.fromPos.x,
                toX: b.toPos.x,
            }));
        """)
        data = json.loads(out)
        self.assertAlmostEqual(data["fromX"], 0.0, places=1)
        self.assertAlmostEqual(data["toX"], 10.0, places=1)

    def test_mid_animation_captures_interpolated(self):
        """If interrupted mid-animation, fromPos is the interpolated position."""
        out = _run_js("""
            const p1 = { x: 0, y: 0, z: 0 };
            const p2 = { x: 100, y: 0, z: 0 };
            const p3 = { x: 200, y: 0, z: 0 };
            const ori = { x: 0, y: 0, z: 0, w: 1 };
            let a = advanceAnim(undefined, p1, ori, Cesium);
            a = advanceAnim(a, p2, ori, Cesium);
            a.startTime = Date.now() - a.duration / 2;
            const b = advanceAnim(a, p3, ori, Cesium);
            console.log(JSON.stringify({ fromX: b.fromPos.x }));
        """)
        data = json.loads(out)
        self.assertGreater(data["fromX"], 30)
        self.assertLess(data["fromX"], 70)

    def test_lerp_position_uses_anim_duration(self):
        """lerpPosition uses anim.duration, not a fixed constant."""
        out = _run_js("""
            const a = {
                fromPos: { x: 0, y: 0, z: 0 },
                toPos: { x: 100, y: 0, z: 0 },
                startTime: Date.now() - 300,
                duration: 600,
            };
            const scratch = new Cesium.Cartesian3();
            lerpPosition(a, Cesium, scratch);
            console.log(JSON.stringify({ x: scratch.x }));
        """)
        data = json.loads(out)
        self.assertGreater(data["x"], 30)
        self.assertLess(data["x"], 70)

    def test_slerp_orientation_uses_anim_duration(self):
        """slerpOrientation uses anim.duration, not a fixed constant."""
        out = _run_js("""
            const a = {
                fromOri: { x: 0, y: 0, z: 0, w: 0 },
                toOri: { x: 0, y: 0, z: 0, w: 1 },
                startTime: Date.now() - 300,
                duration: 600,
            };
            const scratch = new Cesium.Quaternion();
            slerpOrientation(a, Cesium, scratch);
            console.log(JSON.stringify({ w: scratch.w }));
        """)
        data = json.loads(out)
        self.assertGreater(data["w"], 0.3)
        self.assertLess(data["w"], 0.7)


if __name__ == "__main__":
    unittest.main()
