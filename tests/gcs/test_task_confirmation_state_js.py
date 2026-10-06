"""Run the Node taskConfirmationState reducer test + source-level export checks."""
import os
import subprocess
import unittest

_NODE_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_task_confirmation_state.js",
))

_SRC = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
    "hooks", "taskConfirmationState.js",
))


class TestTaskConfirmationStateNode(unittest.TestCase):
    """Run the Node.js reducer test against the real ESM module."""

    def test_node_reducer(self):
        result = subprocess.run(
            ["node", _NODE_TEST],
            capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node test failed:\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}",
        )


class TestSourceExports(unittest.TestCase):
    """Guard the public reducer/selector API surface at the source level."""

    def setUp(self):
        with open(_SRC, encoding="utf-8") as f:
            self.src = f.read()

    def test_exports_present(self):
        for name in [
            "confirmRequest", "confirmImage", "confirmResponse",
            "disarmAfterGuided", "confirmReset", "expireDecided", "hasPendingConfirm",
            "canCancel", "visibleConfirmEntries",
        ]:
            self.assertIn(f"export function {name}", self.src)

    def test_vehicle_abort_removed(self):
        self.assertNotIn("export function vehicleAbort", self.src)


class TestRoundUidPlumbedThroughTheUi(unittest.TestCase):
    """The confirm round uid has to survive the whole round trip: WS payload
    -> reducer state -> POST body, and it must key the card so a bounded
    re-ask of the same (sysId, taskId) remounts with a fresh timeout guard."""

    def _read(self, *parts):
        path = os.path.normpath(os.path.join(
            os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src", *parts,
        ))
        with open(path, encoding="utf-8") as f:
            return f.read()

    def test_reducer_stores_the_round_uid(self):
        self.assertIn("roundUid: data.round_uid", self._read("hooks", "taskConfirmationState.js"))

    def test_response_post_echoes_the_round_uid(self):
        src = self._read("hooks", "useTaskConfirmation.js")
        self.assertIn("entry.roundUid", src)
        self.assertIn("round_uid: roundUid", src)

    def test_card_key_includes_the_round_uid(self):
        """Both renderers: a card keyed only by sysId-taskId is not remounted
        for a re-ask, so its timeout guard and in-progress press-and-hold
        would carry across rounds."""
        for parts in (("..", "src", "App.jsx"),
                      ("components", "sidebar", "MonitoringSidebar.jsx")):
            self.assertIn("${entry.taskId}-${entry.roundUid", self._read(*parts), parts[-1])

    def test_long_press_resets_per_round(self):
        src = self._read("components", "TaskConfirmCard.jsx")
        self.assertIn("resetKey: roundKey", src)


if __name__ == "__main__":
    unittest.main()
