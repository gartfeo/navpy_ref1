"""Tests for loadStage.js — per-vehicle connect/load stage computation.

The load stage is what makes UAV loading traceable (Phase C): a vehicle coming
online reads connecting → downloading → ready instead of popping in already done.
It is derived from BACKEND telemetry (link_ok + is_probing) so it works in
auto-connect mode, where the frontend never runs the /mission download.
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

_JS_PATH = os.path.join(_UTILS_DIR, "loadStage.js")


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


_JS_CODE = _strip_es_modules(open(_JS_PATH, encoding="utf-8").read())


def _run_js(snippet):
    full = _JS_CODE + "\n" + snippet
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestComputeLoadStage(unittest.TestCase):
    def test_no_vehicle_is_ready(self):
        r = _run_js("console.log(JSON.stringify(computeLoadStage(null, new Set())));")
        self.assertEqual(r, "ready")

    def test_no_link_is_connecting(self):
        r = _run_js("""
        const v = { sys_id: 1, link_ok: false, is_probing: false };
        console.log(JSON.stringify(computeLoadStage(v, new Set())));
        """)
        self.assertEqual(r, "connecting")

    def test_pending_vehicle_is_connecting(self):
        """Discovered but not-yet-telemetry vehicles render as loading cards."""
        r = _run_js("""
        const v = { sys_id: 1, pending: true, link_ok: false };
        console.log(JSON.stringify(computeLoadStage(v, new Set())));
        """)
        self.assertEqual(r, "connecting")

    def test_pending_vehicle_can_show_download_stage(self):
        r = _run_js("""
        const v = { sys_id: 1, pending: true, link_ok: false };
        console.log(JSON.stringify(computeLoadStage(v, new Set([1]))));
        """)
        self.assertEqual(r, "downloading")

    def test_link_ok_probing_is_downloading(self):
        """Backend probe in flight — the auto-connect case with no frontend download."""
        r = _run_js("""
        const v = { sys_id: 1, link_ok: true, is_probing: true };
        console.log(JSON.stringify(computeLoadStage(v, new Set())));
        """)
        self.assertEqual(r, "downloading")

    def test_link_ok_frontend_download_is_downloading(self):
        """Frontend-driven download (manual connect / Download Plan)."""
        r = _run_js("""
        const v = { sys_id: 7, link_ok: true, is_probing: false };
        console.log(JSON.stringify(computeLoadStage(v, new Set([7]))));
        """)
        self.assertEqual(r, "downloading")

    def test_link_ok_settled_is_ready(self):
        r = _run_js("""
        const v = { sys_id: 1, link_ok: true, is_probing: false };
        console.log(JSON.stringify(computeLoadStage(v, new Set())));
        """)
        self.assertEqual(r, "ready")

    def test_no_link_takes_precedence_over_probing(self):
        """A link problem is the more fundamental state than an in-flight probe."""
        r = _run_js("""
        const v = { sys_id: 1, link_ok: false, is_probing: true };
        console.log(JSON.stringify(computeLoadStage(v, new Set([1]))));
        """)
        self.assertEqual(r, "connecting")

    def test_undefined_downloading_set_is_safe(self):
        r = _run_js("""
        const v = { sys_id: 1, link_ok: true, is_probing: false };
        console.log(JSON.stringify(computeLoadStage(v, undefined)));
        """)
        self.assertEqual(r, "ready")


class TestDeriveLoadStagesLatch(unittest.TestCase):
    """The ready-latch: once ready, a transient link drop stays ready; a vehicle
    that leaves the list is pruned so a reconnect replays the stages."""

    _HELPER = """
    const run = (list, dl, prev) => {
      const r = deriveLoadStages(list, new Set(dl), new Set(prev));
      return { stages: Object.fromEntries(r.stages), ready: [...r.ready] };
    };
    const V = (over) => ({ sys_id: 1, link_ok: true, is_probing: false, ...over });
    """

    def test_latch_holds_through_link_blip(self):
        r = _run_js(self._HELPER + """
        const f1 = run([V()], [], []);                       // ready
        const f2 = run([V({link_ok:false})], [], f1.ready);  // link blip, was ready
        console.log(JSON.stringify({ s1: f1.stages['1'], s2: f2.stages['1'], ready1: f1.ready }));
        """)
        self.assertEqual(r["s1"], "ready")
        self.assertEqual(r["ready1"], [1])
        self.assertEqual(r["s2"], "ready")  # latched — NOT reverted to 'connecting'

    def test_leaves_list_prunes_latch_then_reconnect_replays(self):
        r = _run_js(self._HELPER + """
        const f1 = run([V()], [], []);                                   // ready -> ready set [1]
        const f2 = run([], [], f1.ready);                                // vehicle gone
        const f3 = run([V({is_probing:true})], [], f2.ready);            // reconnect, probing
        console.log(JSON.stringify({ ready1: f1.ready, ready2: f2.ready, s3: f3.stages['1'] }));
        """)
        self.assertEqual(r["ready1"], [1])
        self.assertEqual(r["ready2"], [])          # pruned on leave
        self.assertEqual(r["s3"], "downloading")   # replays (latch was cleared)

    def test_downloading_not_latched_until_ready(self):
        r = _run_js(self._HELPER + """
        const f1 = run([V({is_probing:true})], [], []);          // downloading
        const f2 = run([V({link_ok:false})], [], f1.ready);      // then link drops before ever ready
        console.log(JSON.stringify({ s1: f1.stages['1'], ready1: f1.ready, s2: f2.stages['1'] }));
        """)
        self.assertEqual(r["s1"], "downloading")
        self.assertEqual(r["ready1"], [])         # never ready yet -> not latched
        self.assertEqual(r["s2"], "connecting")   # so a real link problem shows through

    def test_redownload_after_ready_shows_downloading(self):
        """A *real* re-download after ready (manual "Download Plan" re-arms
        downloadingSysIds) must surface progress, NOT be pinned back to 'ready'.
        The latch suppresses only a post-ready `connecting` link blip; a genuine
        `downloading` is not suppressed. Regression for the over-broad latch that
        forced any post-ready stage to 'ready'."""
        r = _run_js(self._HELPER + """
        const f1 = run([V()], [], []);              // ready -> ready set [1]
        const f2 = run([V()], [1], f1.ready);       // "Download Plan" re-arms downloadingSysIds
        console.log(JSON.stringify({ s1: f1.stages['1'], ready1: f1.ready, s2: f2.stages['1'], ready2: f2.ready }));
        """)
        self.assertEqual(r["s1"], "ready")
        self.assertEqual(r["ready1"], [1])
        self.assertEqual(r["s2"], "downloading")   # re-download surfaces, not latched to 'ready'
        self.assertEqual(r["ready2"], [])          # un-latched while genuinely downloading


class TestIsInitializing(unittest.TestCase):
    """isInitializing gates which card sections are hidden while a UAV loads."""

    def test_connecting_and_downloading_are_initializing(self):
        r = _run_js("""
        console.log(JSON.stringify({
          connecting: isInitializing('connecting'),
          downloading: isInitializing('downloading'),
        }));
        """)
        self.assertTrue(r["connecting"])
        self.assertTrue(r["downloading"])

    def test_ready_is_not_initializing(self):
        r = _run_js("console.log(JSON.stringify(isInitializing('ready')));")
        self.assertFalse(r)

    def test_unknown_stage_is_not_initializing(self):
        """Undefined/null/empty → false, so the full card shows by default; the
        hiding only kicks in when we positively know the UAV is still loading."""
        r = _run_js("""
        console.log(JSON.stringify({
          undef: isInitializing(undefined),
          nul: isInitializing(null),
          empty: isInitializing(''),
        }));
        """)
        self.assertFalse(r["undef"])
        self.assertFalse(r["nul"])
        self.assertFalse(r["empty"])


class TestProgressPct(unittest.TestCase):
    """progressPct drives LoadStageRow's numeric "12/45 waypoints" + progress bar."""

    def test_null_without_a_pair(self):
        r = _run_js("""
        console.log(JSON.stringify({
          undef: progressPct(undefined),
          nul: progressPct(null),
          notArray: progressPct(42),
        }));
        """)
        self.assertIsNone(r["undef"])
        self.assertIsNone(r["nul"])
        self.assertIsNone(r["notArray"])

    def test_null_when_total_not_yet_known(self):
        """Before MISSION_COUNT arrives there's no total — no premature 0%/NaN."""
        r = _run_js("console.log(JSON.stringify(progressPct([0, 0])));")
        self.assertIsNone(r)

    def test_partial_progress(self):
        r = _run_js("console.log(JSON.stringify(progressPct([12, 45])));")
        self.assertEqual(r, 27)  # round(12/45*100) = 27

    def test_complete_is_100(self):
        r = _run_js("console.log(JSON.stringify(progressPct([45, 45])));")
        self.assertEqual(r, 100)

    def test_clamped_to_100_even_if_current_exceeds_total(self):
        r = _run_js("console.log(JSON.stringify(progressPct([50, 45])));")
        self.assertEqual(r, 100)


if __name__ == "__main__":
    unittest.main()
