"""Tests for utils/accelCal.js — pure accel-calibration state logic.

Loads the real source, strips ES module syntax, and evaluates it in Node.js so
the reducer/constants are tested directly (not a re-implementation).
"""
import unittest
import subprocess
import json
import os
import re


_SRC_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "accelCal.js",
))


def _strip_es_modules(src):
    out = []
    for line in src.split("\n"):
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        if re.match(r"^export\s+default\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_SRC_JS = _strip_es_modules(open(_SRC_FILE, encoding="utf-8").read())


def _run_js(script):
    code = _SRC_JS + "\n" + script
    result = subprocess.run(
        ["node", "-e", code],
        capture_output=True, text=True, timeout=10,
    )
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestPositions(unittest.TestCase):
    def test_pos_key_by_code(self):
        out = _run_js("console.log(JSON.stringify(ACCEL_CAL_POS_KEY_BY_CODE));")
        self.assertEqual(json.loads(out), {
            "1": "level", "2": "left", "3": "right",
            "4": "noseDown", "5": "noseUp", "6": "back",
        })

    def test_six_positions_in_order(self):
        out = _run_js("console.log(ACCEL_CAL_POSITIONS.map(p => p.code).join(','));")
        self.assertEqual(out, "1,2,3,4,5,6")


class TestInitCalState(unittest.TestCase):
    def test_level(self):
        out = _run_js("console.log(JSON.stringify(initCalState('level')));")
        self.assertEqual(json.loads(out), {
            "active": True, "mode": "level", "step": None,
            "prompt": "", "result": None, "busy": True,
        })


class TestReduce(unittest.TestCase):
    def test_prompt_while_idle_opens_session(self):
        out = _run_js("""
            const s = reduceAccelCalStep(null, {status:'prompt', step:2, prompt_text:'left'});
            console.log(JSON.stringify(s));
        """)
        s = json.loads(out)
        self.assertTrue(s["active"])
        self.assertEqual(s["mode"], "full")
        self.assertEqual(s["step"], 2)
        self.assertEqual(s["prompt"], "left")

    def test_prompt_updates_active_session(self):
        out = _run_js("""
            const a = initCalState('full');
            const b = reduceAccelCalStep(a, {status:'prompt', step:4, prompt_text:'nose down'});
            console.log(b.step, b.prompt, b.busy);
        """)
        self.assertEqual(out, "4 nose down false")

    def test_success_closes_session(self):
        out = _run_js("""
            const a = initCalState('full');
            const b = reduceAccelCalStep(a, {status:'success', step:null, prompt_text:'ok'});
            console.log(b.active, b.result, b.step);
        """)
        self.assertEqual(out, "false success null")

    def test_failed_closes_session(self):
        out = _run_js("""
            const a = initCalState('full');
            const b = reduceAccelCalStep(a, {status:'failed', step:null, prompt_text:''});
            console.log(b.active, b.result);
        """)
        self.assertEqual(out, "false failed")

    def test_result_while_idle_ignored(self):
        out = _run_js("""
            const s = reduceAccelCalStep(null, {status:'success', step:null, prompt_text:''});
            console.log(JSON.stringify(s));
        """)
        self.assertEqual(json.loads(out), None)


if __name__ == "__main__":
    unittest.main()
