"""Tests for the TopBar fullscreen toggle: Node.js logic + source-level wiring."""
import json
import os
import subprocess
import unittest

_FRONTEND_SRC = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))

_NODE_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "frontend", "test_fullscreen_logic.mjs",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND_SRC, *parts), encoding="utf-8") as f:
        return f.read()


class TestFullscreenLogic(unittest.TestCase):
    """Run the Node.js test for the fullscreen helpers."""

    def test_node_logic(self):
        result = subprocess.run(
            ["node", _NODE_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, f"Node test failed:\n{result.stderr}")


class TestTopBarWiresFullscreen(unittest.TestCase):
    """The toggle lives in the TopBar and only shows where fullscreen works."""

    def setUp(self):
        self.src = _read("components", "TopBar.jsx")

    def test_uses_hook(self):
        self.assertIn("import useFullscreen from '../hooks/useFullscreen'", self.src)
        self.assertIn("const fullscreen = useFullscreen()", self.src)

    def test_button_gated_on_support_and_calls_toggle(self):
        block = self.src[self.src.index("{fullscreen.supported && ("):]
        block = block[:block.index("</button>")]
        self.assertIn("onClick={fullscreen.toggle}", block)
        self.assertIn("topBar.exitFullscreen", block)
        self.assertIn("topBar.fullscreen", block)

    def test_hook_tracks_fullscreenchange(self):
        hook = _read("hooks", "useFullscreen.js")
        self.assertIn("addEventListener('fullscreenchange'", hook)
        self.assertIn("removeEventListener('fullscreenchange'", hook)

    def test_installed_app_reenters_on_click_not_pointerdown(self):
        """Re-entry must follow a completed click: hiding the bars shifts the
        layout, and a pointerdown trigger could move the tap onto another
        control (E-STOP, force launch)."""
        hook = _read("hooks", "useFullscreen.js")
        self.assertIn("useRef(supported && isInstalledApp(window))", hook)
        self.assertNotIn("pointerdown", hook.replace("click, not pointerdown", ""))
        # Leaving with the button disarms re-entry.
        self.assertIn(
            "autoEnterRef.current = !isFullscreenActive(document) && installedRef.current",
            hook,
        )

    def test_reentry_listens_in_bubble_phase(self):
        """A capture-phase listener would call requestFullscreen before the
        tapped control's handler and consume the tap's user activation, so a
        Load plan / parameter import tap could not open its file chooser."""
        hook = _read("hooks", "useFullscreen.js")
        self.assertIn("document.addEventListener('click', onClick);", hook)
        self.assertIn("document.removeEventListener('click', onClick);", hook)
        self.assertNotIn("onClick, true", hook)

    def test_one_request_at_a_time(self):
        """The button tap reaches both the button handler and the document
        listener while the first request is pending."""
        hook = _read("hooks", "useFullscreen.js")
        self.assertIn("if (pendingRef.current) return;", hook)
        self.assertIn("pendingRef.current = false;", hook)


class TestFullscreenLocales(unittest.TestCase):
    def test_every_locale_has_both_labels(self):
        for name in ("en.json", "hy.json"):
            top_bar = json.loads(_read("locales", name))["topBar"]
            for key in ("fullscreen", "exitFullscreen"):
                self.assertTrue(top_bar.get(key), f"{name} topBar.{key} missing")


if __name__ == "__main__":
    unittest.main()
