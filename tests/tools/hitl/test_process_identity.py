"""tools/hitl/process_identity.py: the creation time a ready line names is the start time the runner reads back."""
from __future__ import annotations

import os
import re

import pytest

from tools.hitl.process_identity import created_filetime, own_identity
from tools.tuning import approach_handles


@pytest.mark.skipif(os.name != "nt", reason="the creation time is read on Windows only")
def test_the_creation_time_equals_the_start_time_powershell_reads_back():
    assert approach_handles.windows_process(os.getpid()) == str(created_filetime())


def test_the_identity_names_the_pid_and_the_creation_time():
    match = re.fullmatch(r"pid (\d+), started (\d+)", own_identity())
    assert match and int(match.group(1)) == os.getpid()
    assert (int(match.group(2)) > 0) is (os.name == "nt")
