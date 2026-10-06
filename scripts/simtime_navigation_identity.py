"""Record the actual executable and source files used by each experiment."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess


def source_hashes(root: Path) -> dict[str, str]:
    paths = sorted((root / "scripts").glob("*simtime*navigation*.py"))
    paths += [root / "scripts" / name for name in (
        "simtime_step_artifacts.py", "simtime_navigation_config.py",
        "eval_param_file.py", "eval_param_float32.py")]
    paths += sorted((root / "src/navpy").rglob("*.py"))
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(set(paths))}


def record_identity(root: Path, firmware: str, case: Path, peer_python: str) -> None:
    binary = subprocess.run(["wsl.exe", "--exec", "sha256sum",
                             f"{firmware}/build/sitl/bin/arduplane"],
                            check=True, capture_output=True, text=True, timeout=20)
    firmware_head = subprocess.run(["wsl.exe", "--exec", "git", "-C", firmware,
                                    "rev-parse", "HEAD"], check=True,
                                   capture_output=True, text=True, timeout=20)
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, check=True,
                          capture_output=True, text=True, timeout=10)
    versions = subprocess.run(["wsl.exe", "--exec", peer_python, "-c",
        "import sys,json,importlib.metadata as m;print(json.dumps({'python':sys.version,'packages':"
        "{n:m.version(n) for n in ('numpy','pymap3d','geopy','opencv-python-headless','pymavlink')}}))"],
        check=True, capture_output=True, text=True, timeout=20)
    payload = {"navpy_head": head.stdout.strip(), "firmware_head": firmware_head.stdout.strip(),
               "binary_sha256": binary.stdout.split()[0], "peer_python": peer_python,
               "source_sha256": source_hashes(root), "runtime": json.loads(versions.stdout),
               "defaults_sha256": hashlib.sha256((case / "navigation-defaults.parm").read_bytes()).hexdigest()}
    with (case / "identity.json").open("x") as output:
        json.dump(payload, output, indent=2)
