"""Tests for the Vision-tab per-class preset editor pure JS helpers.

Covers the override-buffer semantics the "Min confirm pixels" / "Target
altitude" editor relies on: editing one field must preserve a class's other
preset fields, the accumulated buffer is exactly what Apply flushes to
PUT /api/vision-profiles/{profile} as ``dock_presets``, and an empty buffer
(Reset / profile switch) makes the grid show the catalog values unchanged.
"""
import json
import os
import re
import unittest

from tests.gcs.js_runner import run_node


_SETTINGS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "settings",
))


def _strip_es_modules(src):
    """Remove ES module syntax so Node.js can eval the helper code."""
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


_JS_PATH = os.path.join(_SETTINGS_DIR, "visionTabUtils.js")
_JS_CODE = _strip_es_modules(open(_JS_PATH, encoding="utf-8").read())


def _run_js(script):
    """Run a JS snippet against the helpers via Node.js; return parsed JSON."""
    result = run_node(_JS_CODE + "\n" + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestApplyPresetEdit(unittest.TestCase):
    def test_edit_min_pixel_preserves_sibling_fields(self):
        """Editing min_pixel_size must keep the class's altitude_m + label."""
        result = _run_js("""
            const catalog = { dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' } };
            const overrides = applyPresetEdit(catalog, {}, 'dock', 'min_pixel_size', 5);
            console.log(JSON.stringify(overrides));
        """)
        self.assertEqual(
            result["dock"],
            {"altitude_m": 120, "min_pixel_size": 5, "label": "Dock"},
        )

    def test_edit_altitude_preserves_min_pixel(self):
        """Editing altitude_m must keep the class's min_pixel_size + label."""
        result = _run_js("""
            const catalog = { dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' } };
            const overrides = applyPresetEdit(catalog, {}, 'dock', 'altitude_m', 300);
            console.log(JSON.stringify(overrides));
        """)
        self.assertEqual(
            result["dock"],
            {"altitude_m": 300, "min_pixel_size": 45, "label": "Dock"},
        )

    def test_second_edit_same_class_keeps_prior_edit(self):
        """A second field edit on a class must not drop the first edit."""
        result = _run_js("""
            const catalog = { dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' } };
            let o = applyPresetEdit(catalog, {}, 'dock', 'min_pixel_size', 5);
            o = applyPresetEdit(catalog, o, 'dock', 'altitude_m', 300);
            console.log(JSON.stringify(o));
        """)
        self.assertEqual(
            result["dock"],
            {"altitude_m": 300, "min_pixel_size": 5, "label": "Dock"},
        )

    def test_apply_payload_carries_complete_dock_preset(self):
        """The accumulated buffer IS the Apply payload (dock_presets); the
        edited dock preset is carried complete."""
        result = _run_js("""
            const catalog = {
                dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' },
            };
            const o = applyPresetEdit(catalog, {}, 'dock', 'min_pixel_size', 5);
            console.log(JSON.stringify(o));
        """)
        self.assertEqual(set(result.keys()), {"dock"})
        self.assertEqual(result["dock"]["min_pixel_size"], 5)
        self.assertEqual(result["dock"]["altitude_m"], 120)
        self.assertEqual(result["dock"]["label"], "Dock")

    def test_edit_does_not_mutate_input_buffer(self):
        """applyPresetEdit returns a new buffer without mutating the prior one."""
        result = _run_js("""
            const catalog = { dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' } };
            const prev = {};
            const next = applyPresetEdit(catalog, prev, 'dock', 'min_pixel_size', 5);
            console.log(JSON.stringify({ prevKeys: Object.keys(prev), same: prev === next }));
        """)
        self.assertEqual(result["prevKeys"], [])
        self.assertFalse(result["same"])


class TestMergeDisplayPresets(unittest.TestCase):
    def test_overrides_win_and_override_only_class_appears(self):
        result = _run_js("""
            const catalog = { dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' } };
            const overrides = {
                dock: { altitude_m: 120, min_pixel_size: 5, label: 'Dock' },
                extra: { min_pixel_size: 9 },
            };
            console.log(JSON.stringify(mergeDisplayPresets(catalog, overrides)));
        """)
        self.assertEqual(result["dock"]["min_pixel_size"], 5)
        self.assertEqual(result["dock"]["altitude_m"], 120)
        self.assertEqual(result["extra"], {"min_pixel_size": 9})

    def test_empty_buffer_shows_catalog_unchanged(self):
        """Reset / profile switch discards the buffer -> grid reverts to catalog."""
        result = _run_js("""
            const catalog = {
                dock: { altitude_m: 120, min_pixel_size: 45, label: 'Dock' },
            };
            console.log(JSON.stringify(mergeDisplayPresets(catalog, {})));
        """)
        self.assertEqual(result, {
            "dock": {"altitude_m": 120, "min_pixel_size": 45, "label": "Dock"},
        })


if __name__ == "__main__":
    unittest.main()
