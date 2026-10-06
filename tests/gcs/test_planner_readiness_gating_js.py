"""Source-level tests for planner readiness gating in frontend hooks."""
import os
import unittest


def _read(*parts):
    path = os.path.normpath(os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "gcs", "frontend", "src",
        *parts,
    ))
    with open(path, encoding="utf-8") as f:
        return f.read()


class TestPlannerReadyThreading(unittest.TestCase):
    def test_app_threads_planner_ready_to_orchestrator_and_map(self):
        source = _read("App.jsx")
        self.assertIn("plannerReady,", source)
        self.assertIn("plannerReady={plannerReady}", source)
        self.assertIn("usePlanningOrchestrator({", source)

    def test_cesium_map_forwards_planner_ready_to_coverage(self):
        # App passes plannerReady into CesiumMap; guard against CesiumMap
        # dropping it (the exact gap this fix closes) by asserting it is both
        # destructured from props and forwarded as the final useCoverageLayer arg.
        source = _read("components", "map", "CesiumMap.jsx")
        # destructured as its own prop (order-independent of siblings)...
        self.assertIn("\n  plannerReady,", source)
        # ...and forwarded as the final arg of the coverage hook call
        self.assertIn("viewerReady, plannerReady);", source)

    def test_orchestrator_threads_planner_ready_to_hooks(self):
        source = _read("hooks", "usePlanningOrchestrator.js")
        self.assertIn("settingsVersion, plannerReady,", source)
        self.assertIn("plannerReady,\n    setFallbackLocationAssignments", source)
        self.assertIn("handleSaveSettings,\n    plannerReady,", source)
        self.assertIn("localAnalyze, localGenerate,\n    plannerReady,", source)


class TestPlannerReadyGuards(unittest.TestCase):
    def test_plan_generation_noops_until_ready(self):
        source = _read("hooks", "usePlanGeneration.js")
        self.assertIn("if (!plannerReady) { setAnalysis(null); return; }", source)
        self.assertIn("if (!plannerReady) return null;", source)
        self.assertIn("if (!plannerReady) return;\n    if (phaseRef.current !== PHASES.PLANNING) return;", source)

    def test_plan_drag_noops_until_ready(self):
        source = _read("hooks", "usePlanDrag.js")
        self.assertIn("if (!plannerReady) return;", source)
        self.assertIn("if (!plannerReady || !uavCountLocked || polygon.length < 3)", source)

    def test_plan_persistence_skips_analysis_until_ready(self):
        source = _read("hooks", "usePlanPersistence.js")
        self.assertIn("if (plannerReady && resolvedPoly && resolvedPoly.length >= 3", source)

    def test_coverage_layer_hides_until_ready(self):
        source = _read("components", "map", "hooks", "useCoverageLayer.js")
        self.assertIn("if (!plannerReady) {", source)
        self.assertIn("const showLive = plannerReady && (showCoverage || showVision);", source)


if __name__ == "__main__":
    unittest.main()
