"""Deploying to the handheld: the PC-side bundle and the handheld-side swap."""
from __future__ import annotations

import io
import os
import shutil
import subprocess
import tarfile
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import scripts.deploy_handheld as deploy

_UPDATE = Path(deploy.REPO) / "scripts" / "termux" / "update_gcs.sh"
_HEALTHY = '{"status":"ok","vehicles_connected":0,"unavailable_routes":[]}'


def _git(repo: Path, *args: str) -> None:
    subprocess.run(["git", "-c", "user.name=t", "-c", "user.email=t@t", *args], cwd=repo,
                   check=True, capture_output=True)


def _write(path: Path, text: str, newline: str = "\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline=newline) as f:
        f.write(text)


def _bash() -> str | None:
    """Git's bash; on Windows the one on PATH may be the WSL launcher."""
    found = shutil.which("bash")
    if os.name != "nt":
        return found
    git = shutil.which("git")
    if git:
        for candidate in (Path(git).parents[1] / "bin" / "bash.exe",
                          Path(git).parents[1] / "usr" / "bin" / "bash.exe"):
            if candidate.is_file():
                return str(candidate)
    return None if not found or "system32" in found.lower() else found


class TestBundle(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name)
        _git(self.repo, "init", "-q")
        _git(self.repo, "config", "core.autocrlf", "true")
        _write(self.repo / ".gitignore", "dist/\n")
        _write(self.repo / "pyproject.toml", "[project]\n")
        _write(self.repo / "scripts" / "termux" / "start_gcs.sh", "echo start\n")
        _write(self.repo / "scripts" / "other.py", "print('pc only')\n")
        _write(self.repo / "src" / "gcs" / "backend" / "main.py", "app = 1\n")
        _write(self.repo / "src" / "gcs" / "frontend" / "dist" / "index.html", "<html>", newline="\r\n")
        _git(self.repo, "add", "-A")
        _git(self.repo, "commit", "-q", "-m", "first")
        self.head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=self.repo, check=True,
                                   capture_output=True, text=True).stdout.strip()

    def tearDown(self):
        self.tmp.cleanup()

    def test_bundle_holds_head_the_build_and_its_version(self):
        out = self.repo / "bundle.tgz"
        deploy.build_bundle(out, repo=self.repo)
        with tarfile.open(out) as bundle:
            files = {m.name: m for m in bundle.getmembers() if m.isfile()}
            self.assertEqual(set(files), {
                "pyproject.toml", "scripts/termux/start_gcs.sh", "src/gcs/backend/main.py",
                "src/gcs/frontend/dist/index.html", "DEPLOYED"})
            # LF on the handheld even with core.autocrlf=true on the PC.
            self.assertEqual(bundle.extractfile(files["scripts/termux/start_gcs.sh"]).read(),
                             b"echo start\n")
            self.assertEqual(bundle.extractfile(files["src/gcs/backend/main.py"]).read(), b"app = 1\n")
            version = bundle.extractfile(files["DEPLOYED"]).read().decode()
            self.assertTrue(version.startswith(f"commit {self.head}\nsubject first\nbuilt "))
            self.assertEqual(files["src/gcs/frontend/dist/index.html"].mode, 0o644)

    def test_only_uncommitted_deployed_files_block_a_deploy(self):
        self.assertEqual(deploy.uncommitted(self.repo), [])
        _write(self.repo / "scripts" / "other.py", "print('changed')\n")
        _write(self.repo / "src" / "gcs" / "frontend" / "dist" / "index.html", "<html>new")
        self.assertEqual(deploy.uncommitted(self.repo), [])
        _write(self.repo / "src" / "gcs" / "backend" / "main.py", "app = 2\n")
        _write(self.repo / "src" / "gcs" / "backend" / "new.py", "")
        self.assertEqual(deploy.uncommitted(self.repo),
                         [" M src/gcs/backend/main.py", "?? src/gcs/backend/new.py"])


class _FakePopen:
    """Records what Handheld.run writes to adb's stdin."""
    sent = b""

    def __init__(self, *args, **kwargs):
        self.stdin = self
        self.stdout = io.BytesIO(b"one\ntwo\n")
        self.returncode = 0

    def write(self, data):
        _FakePopen.sent += data

    def close(self):
        pass

    def wait(self):
        return 0


class TestHandheldRun(unittest.TestCase):
    def test_the_remote_script_reaches_bash_with_lf_endings(self):
        """A text pipe on Windows turns every newline into CRLF."""
        _FakePopen.sent = b""
        with mock.patch.object(deploy.subprocess, "Popen", _FakePopen), \
                mock.patch("builtins.print"):
            code, lines = deploy.Handheld("adb", None).run("v=abc\necho $v\n")
        self.assertEqual((code, lines), (0, ["one", "two"]))
        self.assertIn(b"v=abc\necho $v\n", _FakePopen.sent)
        self.assertNotIn(b"\r", _FakePopen.sent)


_RESULTS = {0: ["update result: updated"], 2: ["update result: app build failed"],
            3: ["update result: updated, but not marked checked"]}


class _FakeHandheld:
    def __init__(self, built="b1", installed="b1", app_running=True, update_code=0,
                 update_lines=None, preflight_code=0):
        self.built, self.installed, self.running = built, installed, app_running
        self.update_code, self.preflight_code = update_code, preflight_code
        self.update_lines = _RESULTS.get(update_code, []) if update_lines is None else update_lines
        self.calls = []
        self.update_script = ""
        self.uploads = []

    def run(self, script):
        if "update_gcs.sh" not in script:
            return self.preflight_code, []
        self.calls.append("update")
        self.update_script = script
        return self.update_code, ["stopping backend (pid 1)", *self.update_lines]

    def upload(self, local, remote):
        self.calls.append("upload")
        self.uploads.append(remote)

    def app_running(self):
        return self.running

    def termux_sha256(self, path):
        return self.built

    def installed_apk_sha256(self):
        return self.installed

    def download(self, remote, local):
        self.calls.append("download")

    def install(self, apk):
        self.calls.append("install")

    def restart_app(self):
        self.calls.append("restart_app")


class TestDeploySteps(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.bundle = Path(self.tmp.name) / "b.tgz"
        self.bundle.write_bytes(b"x")
        self.print = mock.patch("builtins.print")
        self.print.start()

    def tearDown(self):
        self.print.stop()
        self.tmp.cleanup()

    def test_an_app_that_differs_from_the_build_is_installed_and_restarted(self):
        handheld = _FakeHandheld(built="new", installed="old")
        deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update", "download", "install", "restart_app"])

    def test_an_app_that_matches_the_build_is_only_restarted_for_the_new_page(self):
        handheld = _FakeHandheld()
        deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update", "restart_app"])

    def test_an_app_that_was_not_open_is_not_started(self):
        handheld = _FakeHandheld(built="new", installed="old", app_running=False)
        deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update", "download", "install"])

    def test_an_app_that_is_not_installed_is_left_alone(self):
        handheld = _FakeHandheld(built="new", installed=None, app_running=False)
        deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update"])

    def test_a_failed_or_rolled_back_update_installs_and_restarts_nothing(self):
        handheld = _FakeHandheld(built="new", installed="old", update_code=1)
        with self.assertRaises(deploy.DeployError):
            deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update"])

    def test_a_failed_app_build_still_restarts_the_app_for_the_new_backend(self):
        """Codex P2: the new backend runs, so the open app must load its page;
        the APK on the handheld may be half-written, so it is not installed."""
        handheld = _FakeHandheld(built="broken", installed="old", update_code=2)
        with self.assertRaises(deploy.DeployError):
            deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update", "restart_app"])

    def test_a_copy_that_could_not_be_marked_still_restarts_the_app(self):
        """Codex P2: the new backend runs, so the app loads its page, but the
        deploy is not reported as a success and nothing is installed."""
        handheld = _FakeHandheld(built="new", installed="old", update_code=3)
        with self.assertRaises(deploy.DeployError):
            deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update", "restart_app"])

    def test_exit_2_without_the_updater_result_is_a_failure(self):
        """Codex P2: tar and bash also exit 2 (a damaged bundle, a syntax
        error) before the updater decides anything; that is no partial update."""
        handheld = _FakeHandheld(built="new", installed="old", update_code=2, update_lines=[])
        with self.assertRaises(deploy.DeployError):
            deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update"])

    def test_exit_0_without_the_updater_result_is_a_failure(self):
        handheld = _FakeHandheld(built="new", installed="old", update_code=0, update_lines=[])
        with self.assertRaises(deploy.DeployError):
            deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, ["upload", "update"])

    def test_deploys_to_one_handheld_run_one_at_a_time(self):
        """Codex P1 (round 6): two overlapping updaters could each see the
        same healthy copy, and the second deleted it after the first had
        moved it to the backup. The lock is taken before the shared staging
        folder is touched, and each deploy uploads its own bundle."""
        first, second = _FakeHandheld(), _FakeHandheld()
        deploy.deploy(first, self.bundle)
        deploy.deploy(second, self.bundle)
        script = first.update_script
        self.assertIn("flock -n 9", script)
        self.assertLess(script.index("flock -n 9"), script.index(deploy.NEW_COPY))
        self.assertNotEqual(first.uploads, second.uploads)

    def test_bundles_left_by_interrupted_deploys_are_removed_under_the_lock(self):
        """Codex P2 (round 7): each deploy uploads under a new name, so a
        bundle left by an interrupted upload was never removed."""
        handheld = _FakeHandheld()
        deploy.deploy(handheld, self.bundle)
        script = handheld.update_script
        self.assertIn("navpy-update-*.tgz", script)
        self.assertLess(script.index("flock -n 9"), script.index("navpy-update-*.tgz"))

    def test_no_run_as_stops_before_copying(self):
        handheld = _FakeHandheld(preflight_code=1)
        with self.assertRaises(deploy.DeployError):
            deploy.deploy(handheld, self.bundle)
        self.assertEqual(handheld.calls, [])


@unittest.skipUnless(_bash(), "bash is not installed")
class TestUpdateScript(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.home = root / "home"
        self.live = self.home / "navpy"
        self.new = self.home / "navpy.new"
        self.prev = self.home / "navpy.prev"
        self.stubs = root / "stubs"
        self.jar = root / "android.jar"
        _write(self.jar, "")
        self.tree(self.live, "old")
        self.tree(self.new, "new")
        # Git Bash has neither pgrep nor setsid; the handheld's curl is faked
        # too and logs its arguments. It answers for the copy named in
        # $HOME/bad_version (default "new") with $HOME/new_health: DOWN =
        # refused, HANG = no reply (curl gives up only with --max-time). Git
        # Bash runs a file from PATH only if it starts with #!.
        stubs = {
            "pgrep": '[ -f "$HOME/backend.pid" ] && cat "$HOME/backend.pid"\nexit 0\n',
            "setsid": 'exec "$@"\n',
            "curl": ('echo "$*" >> "$HOME/curl_args"\n'
                     'bad="$(cat "$HOME/bad_version" 2>/dev/null || echo new)"\n'
                     'if [ -f "$HOME/new_health" ] && grep -qx "commit $bad" "$HOME/navpy/DEPLOYED"; then\n'
                     '  case "$(cat "$HOME/new_health")" in\n'
                     '    DOWN) exit 7 ;;\n'
                     '    HANG) exit 28 ;;\n'
                     '  esac\n'
                     '  cat "$HOME/new_health"; exit 0\n'
                     f"fi\necho '{_HEALTHY}'\n"),
            "aapt2": "exit 0\n",
            # The lock is busy while $HOME/lock_busy exists.
            "flock": 'echo "$*" >> "$HOME/flock_args"\n[ -f "$HOME/lock_busy" ] && exit 1\nexit 0\n',
        }
        for name, body in stubs.items():
            _write(self.stubs / name, "#!/bin/sh\n" + body)

    def tearDown(self):
        unkillable = self.home / "unkillable"
        if unkillable.is_file():  # the fake backend that refused KILL
            subprocess.run([_bash(), "-c", f"kill -KILL {unkillable.read_text().strip()}"],
                           capture_output=True, timeout=30)
        self.tmp.cleanup()

    @staticmethod
    def tree(path: Path, version: str, java: str = "class Main {}\n",
             build: str = '[ -f "$HOME/build_fails" ] && exit 3\necho BUILT\n'):
        _write(path / "DEPLOYED", f"commit {version}\nsubject {version} change\n")
        _write(path / "src" / "gcs" / "requirements-termux.txt", "fastapi\n")
        _write(path / "src" / "gcs" / "android" / "Main.java", java)
        _write(path / "src" / "gcs" / "frontend" / "public" / "aas-icon-maskable-512.png", "icon")
        _write(path / "scripts" / "termux" / "build_apk.sh", build)
        _write(path / "scripts" / "termux" / "start_gcs.sh", 'echo started >> "$HOME/started"\n')

    def deployed(self, path: Path) -> str:
        return (path / "DEPLOYED").read_text().splitlines()[0]

    def update(self, before: str = "") -> subprocess.CompletedProcess:
        env = dict(os.environ, HOME=str(self.home), GCS_DATA_DIR=(self.home / ".gcs").as_posix(),
                   ANDROID_JAR=self.jar.as_posix(), STUBS=self.stubs.as_posix(), GCS_FLOCK="flock",
                   # Nothing listens on port 1, should the real curl ever run.
                   GCS_PORT="1", GCS_START_TIMEOUT_S="3", GCS_STOP_TIMEOUT_S="2")
        # Windows cannot move a folder while bash holds a script open inside
        # it, so run a copy from outside the new copy and name the folder.
        runner = self.stubs.parent / "update_gcs.sh"
        shutil.copy(_UPDATE, runner)
        # A "C:/..." entry would split PATH at its colon.
        # kill is a bash builtin, so an exported function stands in for it: it
        # ignores KILL for the pid in $HOME/unkillable, like a hung process.
        script = ('stubs="$STUBS"; if command -v cygpath >/dev/null; then stubs="$(cygpath -u "$stubs")"; fi\n'
                  'kill() { if [ "$1" = -KILL ] && [ "$2" = "$(cat "$HOME/unkillable" 2>/dev/null)" ]; '
                  'then return 0; fi; command kill "$@"; }\n'
                  'export -f kill\n'
                  f'chmod +x "$stubs"/*; export PATH="$stubs:$PATH"; {before}\n'
                  f'bash "{runner.as_posix()}" "{self.new.as_posix()}"')
        return subprocess.run([_bash(), "-c", script], env=env, capture_output=True, text=True,
                              timeout=60)

    @staticmethod
    def running(ignores_term: bool = False) -> str:
        # A fake backend. Its output goes nowhere, so a script that exits
        # before stopping it fails the test instead of holding the pipe open.
        body = "trap '' TERM; while :; do sleep 1; done" if ignores_term else "sleep 300"
        return f'bash -c "{body}" >/dev/null 2>&1 & echo $! > "$HOME/backend.pid"'

    def test_swaps_the_copy_and_keeps_the_previous_one(self):
        done = self.update()
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertEqual(done.stdout.splitlines()[-1], "update result: updated")
        self.assertIn("installed: commit old subject old change", done.stdout)
        self.assertIn("backend was not running", done.stdout)
        self.assertEqual(self.deployed(self.live), "commit new")
        self.assertEqual(self.deployed(self.prev), "commit old")
        self.assertFalse(self.new.exists())

    def test_a_built_app_is_not_rebuilt(self):
        first = self.update()
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        self.assertIn("BUILT", first.stdout)
        self.assertTrue((self.home / ".gcs" / "apk" / "sources.sha256").is_file())
        self.tree(self.new, "new2")
        second = self.update()
        self.assertEqual(second.returncode, 0, second.stdout + second.stderr)
        self.assertNotIn("BUILT", second.stdout)
        self.assertIn("app sources unchanged since the last build", second.stdout)
        self.tree(self.new, "new3", java="class Main { int x; }\n")
        self.assertIn("BUILT", self.update().stdout)

    def test_a_failed_app_build_is_retried_by_the_next_deploy(self):
        """Codex P2: the retry compared sources with the copy it had already
        installed, so it never built the app again."""
        # The build fails for a reason outside the sources (a tool, the disk).
        changed = "class Main { int x; }\n"
        self.tree(self.new, "new", java=changed)
        _write(self.home / "build_fails", "")
        failed = self.update()
        # 2: the copy is updated, only the app build failed.
        self.assertEqual(failed.returncode, 2, failed.stdout + failed.stderr)
        self.assertEqual(failed.stdout.splitlines()[-1], "update result: app build failed")
        self.assertEqual(self.deployed(self.live), "commit new")
        (self.home / "build_fails").unlink()
        self.tree(self.new, "new", java=changed)
        retried = self.update()
        self.assertEqual(retried.returncode, 0, retried.stdout + retried.stderr)
        self.assertIn("BUILT", retried.stdout)

    def test_a_failed_build_forgets_the_last_good_one(self):
        """Codex P2: build_apk.sh may overwrite the APK before it fails, so the
        stamp of the previous build must not survive and skip the next one."""
        self.assertIn("BUILT", self.update().stdout)
        self.tree(self.new, "b", java="class Main { int x; }\n")
        _write(self.home / "build_fails", "")
        self.assertEqual(self.update().returncode, 2)
        (self.home / "build_fails").unlink()
        self.tree(self.new, "a")  # the sources of the first, successful build
        again = self.update()
        self.assertEqual(again.returncode, 0, again.stdout + again.stderr)
        self.assertIn("BUILT", again.stdout)

    def test_the_app_is_built_before_the_backend_stops(self):
        done = self.update(before=self.running())
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertLess(done.stdout.index("BUILT"), done.stdout.index("stopping backend"))

    def test_changed_requirements_ask_for_the_pip_step(self):
        _write(self.new / "src" / "gcs" / "requirements-termux.txt", "fastapi\nhttpx\n")
        done = self.update()
        self.assertIn("WARNING: src/gcs/requirements-termux.txt changed", done.stdout)

    def test_a_running_backend_is_stopped_and_started_again(self):
        done = self.update(before=self.running())
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        pid = (self.home / "backend.pid").read_text().strip()
        self.assertIn(f"stopping backend (pid {pid})", done.stdout)
        self.assertIn(f"backend up: {_HEALTHY}", done.stdout)
        self.assertEqual((self.home / "started").read_text().split(), ["started"])
        self.assertTrue((self.live / "VERIFIED").is_file())

    def test_a_backend_that_ignores_term_is_killed(self):
        done = self.update(before=self.running(ignores_term=True))
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertIn("sending KILL", done.stdout)
        self.assertEqual(self.deployed(self.live), "commit new")

    def test_missing_api_routes_roll_back_and_a_retry_keeps_the_working_copy(self):
        """Codex P2 + P1: /health 200 with unavailable_routes passed, and a
        retry after a failed start deleted the last working copy."""
        _write(self.home / "new_health", '{"status":"ok","unavailable_routes":["params"]}')
        for _ in range(2):
            done = self.update(before=self.running())
            self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
            self.assertIn("unavailable_routes", done.stderr)
            self.assertIn("rolled back to commit old", done.stdout)
            self.assertEqual(self.deployed(self.live), "commit old")
            self.assertEqual(self.deployed(self.home / "navpy.failed"), "commit new")
            self.tree(self.new, "new")
        self.assertEqual((self.home / "started").read_text().split(), ["started"] * 4)

    def test_a_backend_that_never_answers_rolls_back(self):
        _write(self.home / "new_health", "DOWN")
        done = self.update(before=self.running())
        self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
        self.assertIn("did not answer /health", done.stderr)
        self.assertIn(f"backend up: {_HEALTHY}", done.stdout)
        self.assertEqual(self.deployed(self.live), "commit old")

    def test_every_health_request_has_a_time_limit(self):
        """Codex P1: a backend that accepts the connection and never replies
        held curl, so the deadline was never checked and nothing rolled back."""
        _write(self.home / "new_health", "HANG")
        done = self.update(before=self.running())
        self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
        self.assertEqual(self.deployed(self.live), "commit old")
        calls = (self.home / "curl_args").read_text().splitlines()
        self.assertTrue(calls)
        self.assertTrue(all("--max-time" in call for call in calls), calls)

    def test_an_unchecked_copy_never_replaces_the_checked_backup(self):
        """Codex P1: a copy deployed while the backend was stopped was never
        checked, yet the next deploy moved it over the working backup."""
        checked = self.update(before=self.running())
        self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)
        (self.home / "backend.pid").unlink()  # the operator stops the backend
        for version in ("b", "c"):
            self.tree(self.new, version)
            done = self.update()
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertEqual(self.deployed(self.live), "commit c")
        self.assertEqual(self.deployed(self.prev), "commit new")

    def test_a_working_copy_started_by_hand_is_not_lost(self):
        """Codex P1: the first deploy's backup is unchecked; the operator then
        starts the new copy by hand. The next deploy dropped that working copy
        for the unchecked backup, and its rollback restored the wrong one."""
        self.update()  # backend stopped: old becomes the backup, new is unchecked
        self.tree(self.new, "c")
        _write(self.home / "bad_version", "c")
        _write(self.home / "new_health", "DOWN")
        done = self.update(before=self.running())  # the operator started "new"
        self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
        self.assertEqual(self.deployed(self.live), "commit new")
        self.assertIn("rolled back to commit new", done.stdout)

    def test_a_healthy_copy_that_cannot_be_marked_is_still_kept(self):
        """Codex P1 (round 5): the running copy answered /health, but writing
        its VERIFIED file failed, so it was deleted for the unchecked backup."""
        self.update()  # backend stopped: old becomes the backup, new is unchecked
        (self.live / "VERIFIED").mkdir()  # writing the file fails: it is a folder
        self.tree(self.new, "c")
        _write(self.home / "bad_version", "c")
        _write(self.home / "new_health", "DOWN")
        done = self.update(before=self.running())  # the operator started "new"
        self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
        self.assertEqual(self.deployed(self.live), "commit new")
        self.assertIn("rolled back to commit new", done.stdout)

    def test_an_update_is_refused_while_another_holds_the_lock(self):
        """Codex P1 (round 7): run by hand, the updater took no lock, so it
        could overlap a PC deploy and delete the backup the other had made."""
        _write(self.home / "lock_busy", "")
        done = self.update()
        self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
        self.assertIn("another update is running", done.stderr)
        self.assertEqual((self.home / "flock_args").read_text().split(), ["-n", "9"])
        self.assertEqual(self.deployed(self.live), "commit old")
        self.assertFalse(self.prev.exists())
        self.assertTrue(self.new.is_dir())

    def test_the_started_backend_does_not_hold_the_deploy_lock(self):
        """The deploy holds its lock on fd 9; a backend that inherited it would
        keep it for as long as it runs and refuse every later deploy."""
        _write(self.new / "scripts" / "termux" / "start_gcs.sh",
               'if { : >&9; } 2>/dev/null; then echo open; else echo closed; fi > "$HOME/fd9"\n')
        done = self.update(before=self.running() + '; exec 9> "$HOME/deploy.lock"')
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        fd9 = self.home / "fd9"
        for _ in range(50):  # the backend is started in the background
            if fd9.is_file() and fd9.read_text().strip():
                break
            time.sleep(0.1)
        self.assertEqual(fd9.read_text().strip(), "closed")

    def test_unchecked_deploys_never_replace_the_backup(self):
        """Codex P1: with the backend stopped nothing is checked; a second
        unchecked deploy deleted the backup, possibly the only working copy.
        The backup now changes only to a copy that was seen working."""
        for version in ("b", "c"):
            self.tree(self.new, version)
            done = self.update()
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertEqual(self.deployed(self.prev), "commit old")
        self.assertEqual(self.deployed(self.live), "commit c")

    def test_a_copy_that_cannot_be_marked_is_not_reported_as_updated(self):
        """Codex P2: the VERIFIED write failed inside an if condition, where
        set -e does not apply, and the PC was told "updated"."""
        (self.new / "VERIFIED").mkdir()  # writing the file fails: it is a folder
        done = self.update(before=self.running())
        self.assertEqual(done.returncode, 3, done.stdout + done.stderr)
        self.assertIn("could not mark", done.stderr)
        self.assertEqual(done.stdout.splitlines()[-1], "update result: updated, but not marked checked")
        self.assertEqual(self.deployed(self.live), "commit new")

    def test_a_new_backend_that_survives_kill_blocks_the_rollback_start(self):
        """Two backends must never run: if the failed one cannot be stopped,
        the previous copy is put back but not started."""
        _write(self.new / "scripts" / "termux" / "start_gcs.sh", 'echo started >> "$HOME/started"\nbash -c "trap \'\' TERM; while :; do sleep 1; done" >/dev/null 2>&1 &\necho $! > "$HOME/backend.pid"; echo $! > "$HOME/unkillable"\n')
        _write(self.home / "new_health", "DOWN")
        done = self.update(before=self.running())
        self.assertEqual(done.returncode, 1, done.stdout + done.stderr)
        self.assertIn("still running after KILL", done.stderr)
        self.assertIn("previous copy was not started", done.stderr)
        self.assertEqual(self.deployed(self.live), "commit old")
        self.assertEqual((self.home / "started").read_text().split(), ["started"])

    def test_an_interrupted_swap_keeps_the_backup(self):
        """Stopped between its two moves, the update left no ~/navpy; the
        retry must not delete ~/navpy.prev, the only working copy."""
        self.live.rename(self.prev)
        done = self.update()
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertEqual(self.deployed(self.prev), "commit old")
        self.assertEqual(self.deployed(self.live), "commit new")

    def test_refuses_to_run_from_the_installed_copy(self):
        shutil.copy(_UPDATE, self.live / "scripts" / "termux" / "update_gcs.sh")
        env = dict(os.environ, HOME=str(self.home))
        done = subprocess.run([_bash(), (self.live / "scripts" / "termux" / "update_gcs.sh").as_posix()],
                              env=env, capture_output=True, text=True, timeout=60)
        self.assertEqual(done.returncode, 1)
        self.assertIn("Run this script from the new copy", done.stderr)
        self.assertEqual(self.deployed(self.live), "commit old")


if __name__ == "__main__":
    unittest.main()
