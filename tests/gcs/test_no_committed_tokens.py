"""The repository is public: no access tokens may be committed to tracked files."""

import re
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
# JWT (e.g. Cesium Ion tokens) and common provider key shapes.
_TOKEN = re.compile(
    rb"eyJ[A-Za-z0-9_-]{10,}\.eyJ[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}"
    rb"|ghp_[A-Za-z0-9]{30,}|AKIA[0-9A-Z]{16}|-----BEGIN [A-Z ]*PRIVATE KEY-----"
)
_SKIP_SUFFIXES = {".pt", ".hgt", ".glb", ".png", ".jpg", ".bin", ".BIN", ".tlog"}


def test_no_access_tokens_in_tracked_text_files():
    files = subprocess.run(
        ["git", "ls-files", "-z"], cwd=REPO, capture_output=True, check=True
    ).stdout.split(b"\0")
    hits = []
    for name in filter(None, files):
        path = REPO / name.decode()
        if path.suffix in _SKIP_SUFFIXES or not path.is_file():
            continue
        if _TOKEN.search(path.read_bytes()):
            hits.append(name.decode())
    assert not hits, f"access token committed in: {hits}"
