"""Tests for per-waypoint altitude resolution in usePlanSync.js.

Verifies the nullish coalescing pattern (p.alt ?? zoneAlt) used in
usePlanSync.js correctly handles per-point alt, zone fallback, and alt=0.
"""
import unittest
from tests.gcs.js_runner import run_node
import json


def _run_js(script):
    result = run_node(script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return json.loads(result.stdout.strip())


# The altitude resolution expression from usePlanSync.js (used in both
# initial sync and terrain sampling callback):
#   p.alt ?? zoneAlt
_RESOLVE_ALT = """
function resolveAltitudes(track, zoneAlt) {
  return track.map(p => p.alt ?? zoneAlt);
}
"""


class TestPerPointAltitude(unittest.TestCase):
    """Verify per-waypoint altitude resolution matches usePlanSync.js logic."""

    def test_per_point_alt_used_when_present(self):
        result = _run_js(_RESOLVE_ALT + """
            const track = [
              { lat: 32.0, lon: 34.0, alt: 200 },
              { lat: 32.1, lon: 34.1, alt: 120 },
            ];
            console.log(JSON.stringify(resolveAltitudes(track, 150)));
        """)
        self.assertEqual(result, [200, 120])

    def test_zone_fallback_when_alt_absent(self):
        result = _run_js(_RESOLVE_ALT + """
            const track = [
              { lat: 32.0, lon: 34.0 },
              { lat: 32.1, lon: 34.1 },
            ];
            console.log(JSON.stringify(resolveAltitudes(track, 150)));
        """)
        self.assertEqual(result, [150, 150])

    def test_alt_zero_preserved(self):
        """alt: 0 must not be treated as falsy — nullish coalescing required."""
        result = _run_js(_RESOLVE_ALT + """
            const track = [
              { lat: 32.0, lon: 34.0, alt: 0 },
            ];
            console.log(JSON.stringify(resolveAltitudes(track, 150)));
        """)
        self.assertEqual(result, [0])

    def test_mixed_alt_present_and_absent(self):
        """Corridor approach (with alt) + generated scan (no alt) in same track."""
        result = _run_js(_RESOLVE_ALT + """
            const track = [
              { lat: 32.0, lon: 34.0, alt: 200 },
              { lat: 32.1, lon: 34.1, alt: 200 },
              { lat: 32.2, lon: 34.2 },
              { lat: 32.3, lon: 34.3 },
            ];
            console.log(JSON.stringify(resolveAltitudes(track, 120)));
        """)
        self.assertEqual(result, [200, 200, 120, 120])

    def test_alt_null_falls_back(self):
        """Explicit null alt should fall back to zone altitude."""
        result = _run_js(_RESOLVE_ALT + """
            const track = [
              { lat: 32.0, lon: 34.0, alt: null },
            ];
            console.log(JSON.stringify(resolveAltitudes(track, 100)));
        """)
        self.assertEqual(result, [100])

    def test_alt_undefined_falls_back(self):
        """Explicit undefined alt should fall back to zone altitude."""
        result = _run_js(_RESOLVE_ALT + """
            const track = [
              { lat: 32.0, lon: 34.0, alt: undefined },
            ];
            console.log(JSON.stringify(resolveAltitudes(track, 100)));
        """)
        self.assertEqual(result, [100])


if __name__ == "__main__":
    unittest.main()
