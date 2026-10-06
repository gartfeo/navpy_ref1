"""Tests for armBtnColor utility in FlightModeColumn."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re
import json


_COMPONENT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "FlightModeColumn.jsx",
))


def _strip_es_modules(src):
    """Remove ES module import/export and JSX so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        if re.match(r"^export\s+default\s", stripped):
            break  # stop before the React component
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_COLORS_JS = """
const colors = {
  success: '#4caf50',
  error: '#f44336',
  border: '#2a3f5f',
  textBright: '#ffffff',
  accent: '#00d2ff',
  textDim: '#8899aa',
};
"""

_JS_SRC = _COLORS_JS + _strip_es_modules(
    open(_COMPONENT_PATH, encoding="utf-8").read()
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


C_SUCCESS = '#4caf50'
C_ERROR = '#f44336'


class TestArmBtnColor(unittest.TestCase):
    """armBtnColor returns red when prearm fails, green otherwise."""

    def test_prearm_false_returns_error(self):
        self.assertEqual(_run_js("console.log(armBtnColor(false));"), C_ERROR)

    def test_prearm_true_returns_success(self):
        self.assertEqual(_run_js("console.log(armBtnColor(true));"), C_SUCCESS)

    def test_prearm_null_returns_success(self):
        self.assertEqual(_run_js("console.log(armBtnColor(null));"), C_SUCCESS)

    def test_prearm_undefined_returns_success(self):
        self.assertEqual(_run_js("console.log(armBtnColor(undefined));"), C_SUCCESS)


class TestFormatPrearmWarning(unittest.TestCase):
    """Pre-arm warning formatter translates known locale keys."""

    def test_prearm_key_uses_translation(self):
        out = _run_js("""
const calls = [];
const t = (key, options) => {
  calls.push({ key, options: options ?? null });
  return `translated:${key}`;
};
const result = formatPrearmWarning('prearm.throttleNotZero', t);
console.log(JSON.stringify({ result, calls }));
""")
        data = json.loads(out)
        self.assertEqual(data["result"], "translated:prearm.throttleNotZero")
        self.assertEqual(data["calls"], [
            {"key": "prearm.throttleNotZero", "options": None}
        ])

    def test_prearm_object_uses_key_and_options(self):
        out = _run_js("""
const warning = { key: 'prearm.batteryLow', pct: 12 };
const calls = [];
const t = (key, options) => {
  calls.push({ key, options });
  return `${key}:${options.pct}`;
};
const result = formatPrearmWarning(warning, t);
console.log(JSON.stringify({ result, calls }));
""")
        data = json.loads(out)
        self.assertEqual(data["result"], "prearm.batteryLow:12")
        self.assertEqual(data["calls"], [
            {
                "key": "prearm.batteryLow",
                "options": {"key": "prearm.batteryLow", "pct": 12},
            }
        ])

    def test_plain_autopilot_text_stays_raw(self):
        out = _run_js("""
const result = formatPrearmWarning('Hardware safety switch', () => 'unexpected');
console.log(result);
""")
        self.assertEqual(out, "Hardware safety switch")

    def test_malformed_object_renders_empty_string(self):
        out = _run_js("""
const result = formatPrearmWarning({ text: 'missing key' }, () => 'unexpected');
console.log(JSON.stringify(result));
""")
        self.assertEqual(json.loads(out), "")


class TestPrearmWarningLayout(unittest.TestCase):
    """Pre-arm warning labels must wrap inside the narrow flight-mode column."""

    def test_warning_rows_allow_wrapping(self):
        src = open(_COMPONENT_PATH, encoding="utf-8").read()
        self.assertIn("whiteSpace: 'normal'", src)
        self.assertIn("overflowWrap: 'break-word'", src)
        self.assertNotIn("textOverflow: 'ellipsis'", src)


if __name__ == "__main__":
    unittest.main()
