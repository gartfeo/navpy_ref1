"""The bundle deploy_handheld.py copies to the handheld: HEAD's deployed paths,
a fresh frontend build and a DEPLOYED file."""
from __future__ import annotations

import io
import shutil
import subprocess
import tarfile
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
# What the handheld runs. Anything else in the repository stays on the PC.
PATHS = ("src", "scripts/termux", "pyproject.toml")
FRONTEND = Path("src/gcs/frontend")
DIST = FRONTEND / "dist"


class DeployError(RuntimeError):
    pass


def git(*args: str, repo: Path = REPO) -> str:
    # rstrip only: a status line starts with a meaningful space.
    return subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True,
                          text=True).stdout.rstrip()


def uncommitted(repo: Path = REPO) -> list[str]:
    """Changed or untracked files in the deployed paths; ignored files do not count."""
    out = git("status", "--porcelain", "--untracked-files=all", "--", *PATHS, repo=repo)
    return [line for line in out.splitlines() if line]


def build_frontend(repo: Path = REPO) -> None:
    frontend = repo / FRONTEND
    if not (frontend / "node_modules").is_dir():
        raise DeployError(f"No node_modules in {frontend}; run npm ci there first.")
    npm = shutil.which("npm")
    if npm is None:
        raise DeployError("npm is not on PATH.")
    subprocess.run([npm, "run", "build"], cwd=frontend, check=True)


def _normalize(info: tarfile.TarInfo) -> tarfile.TarInfo:
    # Windows reports 0666/0777; the handheld gets ordinary modes.
    info.mode = 0o755 if info.isdir() else 0o644
    info.uid = info.gid = 0
    info.uname = info.gname = ""
    return info


def deployed_text(repo: Path = REPO) -> str:
    return (f"commit {git('rev-parse', 'HEAD', repo=repo)}\n"
            f"subject {git('log', '-1', '--format=%s', repo=repo)}\n"
            f"built {datetime.now(timezone.utc).isoformat(timespec='seconds')}\n")


def build_bundle(out: Path, repo: Path = REPO) -> None:
    """Writes HEAD's deployed paths, the frontend build and a DEPLOYED file to out."""
    dist = repo / DIST
    if not (dist / "index.html").is_file():
        raise DeployError(f"No frontend build at {dist}.")
    # LF endings whatever core.autocrlf says: the handheld runs bash and Python.
    archive = subprocess.run(
        ["git", "-c", "core.autocrlf=false", "archive", "--format=tar", "HEAD", *PATHS],
        cwd=repo, check=True, capture_output=True).stdout
    with tarfile.open(out, "w:gz") as bundle:
        with tarfile.open(fileobj=io.BytesIO(archive)) as source:
            for member in source:
                if member.isfile():
                    bundle.addfile(member, source.extractfile(member))
                elif member.isdir():
                    bundle.addfile(member)
        bundle.add(dist, arcname=DIST.as_posix(), filter=_normalize)
        text = deployed_text(repo).encode()
        info = _normalize(tarfile.TarInfo("DEPLOYED"))
        info.size = len(text)
        info.mtime = int(time.time())
        bundle.addfile(info, io.BytesIO(text))
