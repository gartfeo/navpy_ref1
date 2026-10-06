"""Tests for the build-time manifest URL version: Node.js logic + build wiring."""
import os
import subprocess
import unittest

_FRONTEND = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend",
))

_NODE_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "frontend", "test_manifest_version.mjs",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as f:
        return f.read()


class TestManifestVersionLogic(unittest.TestCase):
    def test_node_logic(self):
        result = subprocess.run(
            ["node", _NODE_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, f"Node test failed:\n{result.stderr}")


class TestManifestVersionWiring(unittest.TestCase):
    def test_vite_build_versions_the_manifest_link(self):
        config = _read("vite.config.js")
        self.assertIn("import { manifestVersionPlugin } from './manifestVersion.js'", config)
        self.assertIn("manifestVersionPlugin(PUBLIC_DIR)", config)

    def test_index_html_has_the_link_the_plugin_rewrites(self):
        self.assertIn('<link rel="manifest" href="/manifest.json">', _read("index.html"))


if __name__ == "__main__":
    unittest.main()
