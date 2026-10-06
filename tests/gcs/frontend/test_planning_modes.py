"""Tests for planningModes.js pure helpers.

Runs test_planning_modes_logic.js via Node.js subprocess.
"""
import subprocess
from pathlib import Path

import pytest

JS_TEST = Path(__file__).parent / "test_planning_modes_logic.js"


def test_planning_modes():
    """Run the JS planning modes logic tests via Node and assert PASS."""
    result = subprocess.run(
        ["node", str(JS_TEST)],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, f"JS tests failed:\n{result.stdout}\n{result.stderr}"
    assert "PASS" in result.stdout, f"Expected PASS in output:\n{result.stdout}"
