"""Runner for upload retry frontend JS logic tests via Node subprocess."""
import subprocess
import sys
from pathlib import Path

import pytest

JS_TEST = Path(__file__).parent / "test_upload_retry_logic.js"


def test_upload_retry_logic():
    """Run the JS logic tests via Node and assert PASS."""
    result = subprocess.run(
        ["node", str(JS_TEST)],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, f"JS tests failed:\n{result.stdout}\n{result.stderr}"
    assert "PASS" in result.stdout, f"Expected PASS in output:\n{result.stdout}"
