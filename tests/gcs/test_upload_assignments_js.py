"""Tests for uploadAssignments.js — resolveUploadAssignments upload-payload logic.

Mirrors the Node-subprocess pattern used by the other *_js.py GCS tests:
the JS util is read, ES-module syntax stripped, and the program is fed to Node
via stdin through the shared `run_node` harness (avoids the Windows
command-line length limit / WinError 206). resolveUploadAssignments depends on
autoAssignFallbackLocations, so both source files are concatenated.
"""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
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


def _load(name):
    with open(os.path.join(_UTILS_DIR, name), encoding="utf-8") as f:
        return _strip_es_modules(f.read())


# autoAssignFallbackLocations first (uploadAssignments imports it), then uploadAssignments.
_JS = _load("fallbackLocationAssignment.js") + "\n" + _load("uploadAssignments.js")


def _run_js(script):
    full = _JS + "\n" + script
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


# Two zones ending at lat 40 / 41; Docks A@40, B@41. autoAssignFallbackLocations picks the
# optimal unique assignment [0, 1]; [1, 0] is the non-optimal/flipped variant.
_TWO_ZONES = "[{track:[{lat:40,lon:44}]},{track:[{lat:41,lon:44}]}]"
_TWO_DOCKS = "[{name:'A',lat:40,lon:44},{name:'B',lat:41,lon:44}]"


class TestResolveUploadAssignments(unittest.TestCase):

    def test_no_zones(self):
        r = _run_js("""
        const r = resolveUploadAssignments([], [{lat:40, lon:44}], [], false, true);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [])
        self.assertEqual(r["missingLabels"], [])

    def test_auto_regenerated_ignores_empty_current(self):
        """Regenerated auto plan rederives from the plan even with current []."""
        r = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}, {lat:40.05, lon:44}]},
          {track: [{lat:40, lon:44}, {lat:40.5, lon:44}]},
        ];
        const fallbackLocations = [{name:'Near', lat:40.06, lon:44}, {name:'Far', lat:40.49, lon:44}];
        const r = resolveUploadAssignments(zones, fallbackLocations, [], false, true);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [0, 1])
        self.assertEqual(r["missingLabels"], [])

    def test_auto_preserves_current_when_not_regenerated(self):
        """Existing (not regenerated) plan keeps the displayed/downloaded
        assignments even if they differ from the optimal auto assignment."""
        r = _run_js(f"""
        const r = resolveUploadAssignments({_TWO_ZONES}, {_TWO_DOCKS}, [1, 0], false, false);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [1, 0])  # preserved, NOT recomputed to [0,1]
        self.assertEqual(r["missingLabels"], [])

    def test_auto_recomputes_when_regenerated(self):
        """A regenerated plan recomputes, overriding stale current assignments."""
        r = _run_js(f"""
        const r = resolveUploadAssignments({_TWO_ZONES}, {_TWO_DOCKS}, [1, 0], false, true);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [0, 1])  # recomputed optimal
        self.assertEqual(r["missingLabels"], [])

    def test_auto_recomputes_when_current_incomplete_not_regenerated(self):
        """Incomplete current assignments force a recompute even when the plan
        was not regenerated (the first-click-with-no-prior-assign case)."""
        r = _run_js(f"""
        const r = resolveUploadAssignments({_TWO_ZONES}, {_TWO_DOCKS}, [0, null], false, false);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [0, 1])
        self.assertEqual(r["missingLabels"], [])

    def test_manual_mode_preserves_current(self):
        """Manual mode keeps the user's in-range assignments verbatim,
        regardless of regeneration."""
        r = _run_js(f"""
        const r = resolveUploadAssignments({_TWO_ZONES}, {_TWO_DOCKS}, [1, 0], true, true);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [1, 0])
        self.assertEqual(r["missingLabels"], [])

    def test_manual_mode_shorter_than_zones_flags_missing(self):
        """A manual assignment array shorter than the zone count leaves the
        trailing zones unassigned and reported."""
        r = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}], set_index: 0},
          {track: [{lat:41, lon:44}], set_index: 0},
          {track: [{lat:42, lon:44}], set_index: 0},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];
        const r = resolveUploadAssignments(zones, fallbackLocations, [0, 0], true, false);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [0, 0, None])
        self.assertEqual(r["missingLabels"], ["S1U3"])

    def test_manual_mode_out_of_range_index_is_missing(self):
        """A stale index pointing past the DOCK list is nulled and flagged."""
        r = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}], set_index: 0},
          {track: [{lat:41, lon:44}], set_index: 0},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];   // only index 0 valid
        const r = resolveUploadAssignments(zones, fallbackLocations, [0, 5], true, false);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [0, None])
        self.assertEqual(r["missingLabels"], ["S1U2"])

    def test_auto_regenerated_trackless_zone_is_missing(self):
        """autoAssignFallbackLocations returns null for a zone with no track endpoint;
        the helper reports it rather than uploading a null target."""
        r = _run_js("""
        const zones = [
          {track: [], set_index: 0},
          {track: [{lat:40, lon:44}], set_index: 0},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];
        const r = resolveUploadAssignments(zones, fallbackLocations, [], false, true);
        console.log(JSON.stringify(r));
        """)
        self.assertIsNone(r["assignments"][0])
        self.assertEqual(r["assignments"][1], 0)
        self.assertEqual(r["missingLabels"], ["S1U1"])

    def test_more_zones_than_docks_all_assigned(self):
        """Sharing is allowed: every zone gets a (possibly shared) DOCK, none missing."""
        r = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}]},
          {track: [{lat:40.1, lon:44}]},
          {track: [{lat:40.2, lon:44}]},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];
        const r = resolveUploadAssignments(zones, fallbackLocations, [], false, true);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(r["assignments"], [0, 0, 0])
        self.assertEqual(r["missingLabels"], [])

    def test_labels_respect_set_index(self):
        """Zone labels reset the UAV counter per set_index."""
        r = _run_js("""
        const zones = [
          {track: [], set_index: 0},
          {track: [], set_index: 0},
          {track: [], set_index: 1},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];
        const r = resolveUploadAssignments(zones, fallbackLocations, [], false, true);
        console.log(JSON.stringify(r.missingLabels));
        """)
        self.assertEqual(r, ["S1U1", "S1U2", "S2U1"])


if __name__ == "__main__":
    unittest.main()
