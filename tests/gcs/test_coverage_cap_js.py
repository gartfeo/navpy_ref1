"""Tests for coverage entity cap constant in useCoverageLayer.js."""
import unittest
import os
import re


_COVERAGE_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks", "useCoverageLayer.js",
))


class TestCoverageEntityCap(unittest.TestCase):
    def setUp(self):
        with open(_COVERAGE_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_max_coverage_entities_defined(self):
        """MAX_COVERAGE_ENTITIES constant should be defined."""
        match = re.search(r"const\s+MAX_COVERAGE_ENTITIES\s*=\s*(\d+)", self.source)
        self.assertIsNotNone(match, "MAX_COVERAGE_ENTITIES not found in source")
        self.assertEqual(int(match.group(1)), 500)

    def test_fifo_eviction_logic_present(self):
        """The FIFO eviction logic should reference MAX_COVERAGE_ENTITIES."""
        self.assertIn("MAX_COVERAGE_ENTITIES", self.source)
        # Should have a while loop that removes oldest entities
        self.assertIn("while (total > MAX_COVERAGE_ENTITIES)", self.source)

    def test_scratch_variable_optimization(self):
        """lerpPositions should use scratch array to avoid per-frame allocations."""
        match = re.search(r"function\s+lerpPositions\s*\([^)]*scratch", self.source)
        self.assertIsNotNone(match, "lerpPositions should accept a scratch parameter")


if __name__ == "__main__":
    unittest.main()
