"""Tests for useTrackWpMarkers.js and makeTrackWpIcon selection differentiation."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_ICONS_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "constants", "icons.js",
))

_HOOK_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks", "useTrackWpMarkers.js",
))


def _strip_es_modules(src):
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


_ICONS_SRC = _strip_es_modules(
    open(_ICONS_PATH, encoding="utf-8").read()
)

_HOOK_SRC = open(_HOOK_PATH, encoding="utf-8").read()


def _run_node(expr):
    script = _ICONS_SRC + f"\nconsole.log(JSON.stringify({expr}));"
    result = run_node(script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


class TestMakeTrackWpIconSelected(unittest.TestCase):
    """makeTrackWpIcon produces distinct SVGs for selected vs unselected."""

    def test_unselected_has_fill_opacity(self):
        svg = _run_node("makeTrackWpIcon('#ff0000', 16, false)")
        self.assertIn("fill-opacity", svg)

    def test_selected_no_fill_opacity(self):
        svg = _run_node("makeTrackWpIcon('#ff0000', 20, true)")
        self.assertNotIn("fill-opacity", svg)

    def test_selected_thick_white_stroke(self):
        svg = _run_node("decodeURIComponent(makeTrackWpIcon('#ff0000', 20, true))")
        self.assertIn('stroke="white"', svg)
        self.assertIn('stroke-width="2"', svg)

    def test_unselected_thin_stroke(self):
        svg = _run_node("decodeURIComponent(makeTrackWpIcon('#ff0000', 16))")
        self.assertIn('stroke-width="1"', svg)

    def test_uses_diamond_shape(self):
        """Track WP icons use polygon (diamond), not circle like vertex points."""
        svg = _run_node("decodeURIComponent(makeTrackWpIcon('#ff0000', 16))")
        self.assertIn('<polygon', svg)
        self.assertNotIn('<circle', svg)

    def test_different_icons(self):
        sel = _run_node("makeTrackWpIcon('#ff0000', 20, true)")
        unsel = _run_node("makeTrackWpIcon('#ff0000', 16, false)")
        self.assertNotEqual(sel, unsel)

    def test_default_is_unselected(self):
        default = _run_node("makeTrackWpIcon('#ff0000', 16)")
        explicit = _run_node("makeTrackWpIcon('#ff0000', 16, false)")
        self.assertEqual(default, explicit)


class TestTrackWpMarkersHookSource(unittest.TestCase):
    """Source-level checks on useTrackWpMarkers.js."""

    def test_accepts_simDockWps_param(self):
        self.assertIn("simDockWps", _HOOK_SRC)

    def test_uses_callback_property(self):
        self.assertIn("CallbackProperty", _HOOK_SRC)

    def test_differentiates_selected(self):
        """Icon call passes selected boolean."""
        self.assertIn("makeTrackWpIcon(colorHex, size, selected)", _HOOK_SRC)

    def test_selected_size_larger(self):
        """Selected dots use a larger size constant."""
        m_def = re.search(r"DOT_SIZE\s*=\s*(\d+)", _HOOK_SRC)
        m_sel = re.search(r"DOT_SIZE_SELECTED\s*=\s*(\d+)", _HOOK_SRC)
        self.assertIsNotNone(m_def)
        self.assertIsNotNone(m_sel)
        self.assertGreater(int(m_sel.group(1)), int(m_def.group(1)))

    def test_selection_key_in_deps(self):
        """Effect depends on selectionKey for re-render on toggle."""
        self.assertIn("selectionKey", _HOOK_SRC)


if __name__ == "__main__":
    unittest.main()
