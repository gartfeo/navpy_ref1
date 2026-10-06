"""Deploy the GCS to the handheld (SIYI UniRC 10 Pro) over adb.

Packs the committed repository at HEAD with a fresh frontend build, copies it
into Termux, and runs scripts/termux/update_gcs.sh from the new copy. That
script rebuilds the full-screen app when its sources changed, swaps the copy
in (keeping the old one as ~/navpy.prev), and restarts the backend if it was
running, rolling back when the new one does not come up. Then the app is
installed from here with adb install -r when the handheld's build differs
from the installed app, and restarted if it was open, so it loads the new page.

    python scripts/deploy_handheld.py                  # deploy HEAD
    python scripts/deploy_handheld.py --out bundle.tgz # only build the bundle

Needs a Termux build that allows run-as (the GitHub build does).
"""
from __future__ import annotations

import argparse
import io
import os
import re
import secrets
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
# What the handheld runs. Anything else in the repository stays on the PC.
PATHS = ("src", "scripts/termux", "pyproject.toml")
FRONTEND = Path("src/gcs/frontend")
DIST = FRONTEND / "dist"

TERMUX = "com.termux"
PREFIX = "/data/data/com.termux/files/usr"
HOME = "/data/data/com.termux/files/home"
NEW_COPY = f"{HOME}/navpy.new"
# One deploy at a time. Termux has no flock; Android's toybox does, and the
# remote PATH holds only Termux's bin.
DEPLOY_LOCK = f"{HOME}/.navpy-deploy.lock"
FLOCK = "/system/bin/flock"
# Where scripts/termux/build_apk.sh writes the app.
APK_ON_DEVICE = f"{HOME}/.gcs/apk/aas-gcs.apk"
APP = "com.aas.gcs"
APP_ACTIVITY = f"{APP}/.MainActivity"
# update_gcs.sh ends with one of these lines and exit codes; the code alone
# could come from tar or bash failing before the updater decided anything.
UPDATED = (0, "update result: updated")
APP_BUILD_FAILED = (2, "update result: app build failed")
NOT_MARKED = (3, "update result: updated, but not marked checked")
_SHA256 = re.compile(r"\b[0-9a-f]{64}\b")

# run-as starts a bare shell: give it the Termux environment. Its SELinux
# context makes termux-exec route every exec through the system linker, which
# breaks some tools; run-as can exec app files directly, so turn that off.
_PREAMBLE = f"""set -e
export PREFIX={PREFIX} HOME={HOME} PATH={PREFIX}/bin TMPDIR={PREFIX}/tmp LANG=en_US.UTF-8
export TERMUX_EXEC__SYSTEM_LINKER_EXEC__MODE=disable
for f in "$PREFIX/lib/libtermux-exec-ld-preload.so" "$PREFIX/lib/libtermux-exec.so"; do
    if [ -f "$f" ]; then export LD_PRELOAD="$f"; break; fi
done
cd "$HOME"
"""


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


def _sha256(output: bytes) -> str | None:
    found = _SHA256.search(output.decode(errors="replace"))
    return found.group(0) if found else None


class Handheld:
    def __init__(self, adb: str, serial: str | None) -> None:
        self.base = [adb, *(["-s", serial] if serial else [])]

    def _adb(self, *args: str, check: bool = False) -> subprocess.CompletedProcess:
        return subprocess.run([*self.base, *args], capture_output=True, check=check)

    def upload(self, local: Path, remote: str) -> None:
        with local.open("rb") as data:
            subprocess.run([*self.base, "exec-in", "run-as", TERMUX, "sh", "-c", f"cat > '{remote}'"],
                           stdin=data, check=True)

    def download(self, remote: str, local: Path) -> None:
        with local.open("wb") as data:
            subprocess.run([*self.base, "exec-out", "run-as", TERMUX, "cat", remote],
                           stdout=data, check=True)

    def run(self, script: str) -> tuple[int, list[str]]:
        """Runs script in Termux's bash, echoes its output, returns its exit code and lines."""
        proc = subprocess.Popen([*self.base, "shell", "-T", "run-as", TERMUX, f"{PREFIX}/bin/bash", "-s"],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        # Bytes, not a text pipe: on Windows that would send CRLF to bash.
        proc.stdin.write((_PREAMBLE + script).encode())
        proc.stdin.close()
        lines = []
        for raw in proc.stdout:
            line = raw.decode(errors="replace").rstrip("\r\n")
            print(line, flush=True)
            lines.append(line)
        return proc.wait(), lines

    def app_running(self) -> bool:
        return bool(self._adb("shell", "pidof", APP).stdout.strip())

    def termux_sha256(self, path: str) -> str | None:
        return _sha256(self._adb("exec-out", "run-as", TERMUX, f"{PREFIX}/bin/sha256sum", path).stdout)

    def installed_apk_sha256(self) -> str | None:
        """The installed app's file, which adb install copies byte for byte."""
        found = re.search(rb"package:(\S+)", self._adb("shell", "pm", "path", APP).stdout)
        if not found:
            return None
        return _sha256(self._adb("shell", "sha256sum", found.group(1).decode()).stdout)

    def install(self, apk: Path) -> None:
        subprocess.run([*self.base, "install", "-r", str(apk)], check=True)

    def restart_app(self) -> None:
        self._adb("shell", "am", "force-stop", APP, check=True)
        self._adb("shell", "am", "start", "-n", APP_ACTIVITY, check=True)


def deploy(handheld: Handheld, bundle: Path) -> None:
    if handheld.run("true\n")[0] != 0:
        raise DeployError("Cannot run commands in Termux over adb; run-as needs Termux's GitHub build "
                          "(see the guide, or use --out).")
    app_was_open = handheld.app_running()
    print(f"copying {bundle.stat().st_size / 1e6:.1f} MB to the handheld")
    # Its own name, so a deploy started meanwhile cannot overwrite it.
    on_device = f"{HOME}/navpy-update-{secrets.token_hex(4)}.tgz"
    handheld.upload(bundle, on_device)
    # The lock on fd 9 is taken before the shared navpy.new is touched and
    # held until this shell exits; update_gcs.sh keeps it from the backend.
    code, lines = handheld.run(
        f"exec 9> '{DEPLOY_LOCK}'\n"
        f"if ! {FLOCK} -n 9; then\n"
        f"    rm -f '{on_device}'\n"
        f"    echo 'another deploy is running on the handheld; nothing was changed' >&2\n"
        f"    exit 1\n"
        f"fi\n"
        # Bundles left by interrupted deploys. A deploy that has uploaded but
        # not yet taken the lock then fails at tar, before it changes anything.
        f"for f in '{HOME}'/navpy-update-*.tgz; do [ \"$f\" = '{on_device}' ] || rm -f \"$f\"; done\n"
        f"rm -rf '{NEW_COPY}'\nmkdir '{NEW_COPY}'\n"
        f"tar -xzf '{on_device}' -C '{NEW_COPY}'\nrm -f '{on_device}'\n"
        f"bash '{NEW_COPY}/scripts/termux/update_gcs.sh'\n")
    outcome = (code, lines[-1] if lines else "")
    if outcome not in (UPDATED, APP_BUILD_FAILED, NOT_MARKED):
        raise DeployError(f"The handheld update failed (exit {code}); see its output above.")
    # After a failed build the APK on the handheld may be half-written: keep
    # the installed app. Otherwise comparing files, not "was it rebuilt",
    # also retries an install that failed.
    if outcome == UPDATED:
        built = handheld.termux_sha256(APK_ON_DEVICE)
        installed = handheld.installed_apk_sha256()
        if built and installed and built != installed:
            with tempfile.TemporaryDirectory() as tmp:
                local = Path(tmp) / "aas-gcs.apk"
                handheld.download(APK_ON_DEVICE, local)
                print("installing the app built on the handheld")
                handheld.install(local)
        elif built and not installed:
            print(f"The app is not installed; install {APK_ON_DEVICE} to use it (see the guide).")
    # The new backend runs either way, so the open app must load its page.
    if app_was_open:
        print("restarting the app so it loads the new page")
        handheld.restart_app()
    if outcome == APP_BUILD_FAILED:
        raise DeployError("The backend is updated, but the app build failed; the installed app is "
                          "unchanged. Deploy again to retry the build.")
    if outcome == NOT_MARKED:
        raise DeployError("The backend is updated and running, but its copy could not be marked "
                          "VERIFIED (see above; check free space). The next deploy keeps it as the "
                          "backup only if its backend is running and healthy then.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--adb", default=os.environ.get("ADB", "adb"), help="adb executable (default: $ADB or adb)")
    parser.add_argument("-s", "--serial", help="device serial, when more than one is attached")
    parser.add_argument("--out", type=Path, help="write the bundle here and stop (no adb)")
    args = parser.parse_args(argv)
    # Keep this script's lines in order with the output of adb and npm.
    sys.stdout.reconfigure(line_buffering=True)
    try:
        dirty = uncommitted()
        if dirty:
            raise DeployError("Commit or discard these first; the handheld gets HEAD only:\n  "
                              + "\n  ".join(dirty))
        print(f"deploying {git('log', '-1', '--format=%h %s')}")
        build_frontend()
        if args.out:
            build_bundle(args.out)
            print(f"wrote {args.out}")
            return 0
        with tempfile.TemporaryDirectory() as tmp:
            bundle = Path(tmp) / "navpy-update.tgz"
            build_bundle(bundle)
            deploy(Handheld(args.adb, args.serial), bundle)
    except (DeployError, subprocess.CalledProcessError) as error:
        print(f"deploy failed: {error}", file=sys.stderr)
        return 1
    print("deployed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
