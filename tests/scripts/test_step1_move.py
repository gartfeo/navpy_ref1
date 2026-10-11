"""The step-1 move tool end to end, on a tiny git repository per test.

The synthetic plan moves navpy/legacy/* into core/ and src/gcs over three
commits: c03 a deep kernel file (new packages get empty inits), c04 an old
package init into a folder c03 already gave an empty one (that init gives
way; the old folder keeps an empty one while it holds files), c05 the rest
(the emptied old folders go, caches included). The real worktree is never
touched.
"""

from __future__ import annotations

import hashlib
import importlib.util
import io
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"


def _load(name: str) -> ModuleType:
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


for _name in ("step1_manifest", "step1_rewrite_source", "step1_rewrite_maps",
              "step1_rewrite_strings", "step1_rewrite_imports", "step1_rewrite",
              "step1_leftovers_segments", "step1_leftovers", "step1_move_plan",
              "step1_move_files", "step1_move_tree"):
    _load(_name)
move = _load("step1_move")

MANIFEST = "".join(f"{line}\n" for line in (
    "src\tfinal_dest\tstep1_dest\taction\tvision_path\tnote",
    "__init__.py\t__init__.py\t__init__.py\tstay\tn\troot",
    "legacy/__init__.py\tdelete\tdelete\tdelete\tn\tdissolves",
    "legacy/alpha.py\tcore/kernel/sub/alpha.py\tcore/kernel/sub/alpha.py\tmove\tn\tx",
    "legacy/kpkg/__init__.py\tcore/kernel/__init__.py\tcore/kernel/__init__.py\tmove\tn\tx",
    "legacy/kpkg/helper.py\tgcs:backend/helper.py\tgcs:backend/helper.py\tgcs\tn\tx",
    "legacy/beta.py\tcore/ports/beta.py\tcore/ports/beta.py\tmove\tn\tx",
))
COMMITS = "commit\tstep1_prefix\nc03\tcore/kernel/sub/\nc04\tcore/kernel/\nc05\tcore/ports/\nc05\tgcs:backend/\n"
CONSUMER = (
    "from navpy.legacy.alpha import A\r\n"
    "from navpy.legacy import alpha, beta\r\n"
    "TARGET = (\r\n"
    "    \"navpy.legacy.\"\r\n"
    "    \"alpha.A\"\r\n"
    ")\r\n"
    "PATH = \"src/navpy/legacy/alpha.py\"\r\n"
)
TOOLING = "OLD = 'navpy.legacy.alpha'\n"
FILES = {
    "docs/design/step1/manifest.tsv": MANIFEST,
    "docs/design/step1/commits.tsv": COMMITS,
    "src/navpy/__init__.py": "",
    "src/navpy/legacy/__init__.py": '"""Emptied by c01; goes with its folder."""\n',
    "src/navpy/legacy/alpha.py": "A = 1\n",
    "src/navpy/legacy/beta.py": "from .alpha import A\r\nB = A + 1\r\n",
    "src/navpy/legacy/kpkg/__init__.py": "KERNEL = 1\n",
    "src/navpy/legacy/kpkg/helper.py": "from navpy.legacy.kpkg import KERNEL\n",
    "src/gcs/__init__.py": "",
    "src/gcs/backend/__init__.py": "",
    "tests/test_consumer.py": CONSUMER,
    "tests/scripts/test_step1_fixture.py": TOOLING,
}


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True,
                          text=True).stdout


def _repo(tmp_path: Path, extra: dict[str, str] | None = None) -> Path:
    repo = tmp_path / "repo"
    for path, text in {**FILES, **(extra or {})}.items():
        (repo / path).parent.mkdir(parents=True, exist_ok=True)
        (repo / path).write_bytes(text.encode("utf-8"))
    _git(repo, "init", "-q")
    _git(repo, "config", "core.autocrlf", "false")
    _commit(repo, "base")
    return repo


def _commit(repo: Path, message: str) -> None:
    _git(repo, "add", "-A")
    _git(repo, "-c", "user.name=t", "-c", "user.email=t@example.com", "commit", "-qm", message)


def _run(repo: Path, *args: str) -> tuple[int, str]:
    out = io.StringIO()
    code = move.main([*args, "--repo", str(repo)], out)
    return code, out.getvalue()


def _read(repo: Path, path: str) -> str:
    return (repo / path).read_bytes().decode("utf-8")


def _snapshot(repo: Path) -> dict[str, str]:
    return {path.relative_to(repo).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in repo.rglob("*") if path.is_file() and ".git" not in path.parts}


def test_three_commits_move_files_rewrite_references_and_clear_old_folders(tmp_path: Path) -> None:
    repo = _repo(tmp_path)

    code, out = _run(repo, "--commit", "c03")

    assert code == 0, out
    assert (repo / "src/navpy/core/kernel/sub/alpha.py").is_file()
    assert not (repo / "src/navpy/legacy/alpha.py").exists()
    for init in ("core", "core/kernel", "core/kernel/sub"):
        assert _read(repo, f"src/navpy/{init}/__init__.py") == ""
    assert _read(repo, "tests/test_consumer.py") == (
        "from navpy.core.kernel.sub.alpha import A\r\n"
        "from navpy.core.kernel.sub import alpha\r\n"
        "from navpy.legacy import beta\r\n"
        "TARGET = (\r\n"
        "    \"navpy.core.kernel.sub.\"\r\n"
        "    \"alpha.A\"\r\n"
        ")\r\n"
        "PATH = \"src/navpy/core/kernel/sub/alpha.py\"\r\n"
    )
    assert _read(repo, "src/navpy/legacy/beta.py") == (
        "from navpy.core.kernel.sub.alpha import A\r\nB = A + 1\r\n")
    assert _read(repo, "tests/scripts/test_step1_fixture.py") == TOOLING
    status = _git(repo, "status", "--porcelain", "--untracked-files=all").splitlines()
    assert status and all(line[1] == " " for line in status), status  # all of it staged
    _commit(repo, "c03")

    code, out = _run(repo, "--commit", "c04")

    assert code == 0, out
    assert _read(repo, "src/navpy/core/kernel/__init__.py") == "KERNEL = 1\n"
    assert _read(repo, "src/navpy/legacy/kpkg/__init__.py") == ""
    assert _read(repo, "src/navpy/legacy/kpkg/helper.py") == "from navpy.core.kernel import KERNEL\n"
    _commit(repo, "c04")
    for cache in ("src/navpy/legacy/__pycache__", "src/navpy/legacy/kpkg/__pycache__"):
        (repo / cache).mkdir()
        (repo / cache / "x.cpython-311.pyc").write_bytes(b"\0")

    code, out = _run(repo, "--commit", "c05")

    assert code == 0, out
    assert not (repo / "src/navpy/legacy").exists()
    assert _read(repo, "src/gcs/backend/helper.py") == "from navpy.core.kernel import KERNEL\n"
    assert _read(repo, "src/navpy/core/ports/__init__.py") == ""
    assert "from navpy.core.ports import beta\r\n" in _read(repo, "tests/test_consumer.py")
    assert "remove src/navpy/legacy/kpkg\n" in out and "remove src/navpy/legacy\n" in out
    assert "D  src/navpy/legacy/__init__.py" in _git(repo, "status", "--porcelain").splitlines()


def test_dry_run_prints_the_plan_and_touches_nothing(tmp_path: Path) -> None:
    repo = _repo(tmp_path)
    before = _snapshot(repo)

    code, out = _run(repo, "--commit", "c03", "--dry-run")

    assert code == 0, out
    assert "move src/navpy/legacy/alpha.py -> src/navpy/core/kernel/sub/alpha.py\n" in out
    assert "create src/navpy/core/kernel/sub/__init__.py\n" in out
    assert "rewrite tests/test_consumer.py\n" in out
    assert "rewrite src/navpy/legacy/beta.py\n" in out
    assert _snapshot(repo) == before
    assert _git(repo, "status", "--porcelain") == ""


def test_the_script_runs_by_path(tmp_path: Path) -> None:
    repo = _repo(tmp_path)

    result = subprocess.run(
        [sys.executable, str(SCRIPTS / "step1_move.py"), "--commit", "c03", "--dry-run",
         "--repo", str(repo)],
        capture_output=True, text=True, timeout=60, check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "dry run" in result.stdout


def test_a_leftover_fails_the_run_until_fixed_by_hand(tmp_path: Path) -> None:
    built = 'ROOT = Path(__file__).parents[1]\nDATA = ROOT / "src" / "navpy" / "legacy" / "alpha.py"\n'
    repo = _repo(tmp_path, {"tests/test_paths.py": built})

    code, out = _run(repo, "--commit", "c03", "--dry-run")

    assert code == 1
    assert "tests/test_paths.py:2: segment-built path: navpy/legacy/alpha.py" in out

    code, out = _run(repo, "--commit", "c03")

    assert code == 1
    assert "tests/test_paths.py:2: segment-built path: navpy/legacy/alpha.py" in out
    assert (repo / "src/navpy/core/kernel/sub/alpha.py").is_file()
    assert _run(repo, "--check")[0] == 1
    (repo / "tests/test_paths.py").write_text(
        'DATA = ROOT / "src" / "navpy" / "core" / "kernel" / "sub" / "alpha.py"\n', encoding="utf-8")
    assert _run(repo, "--check") == (0, "no leftovers\n")


def test_vendored_files_are_never_edited(tmp_path: Path) -> None:
    vendored = "import navpy.legacy.alpha\n"
    digest = hashlib.sha256(vendored.encode("utf-8")).hexdigest().upper()
    guard = f'VENDORED_SHA256 = {{\n    "scripts/vendored.py":\n        "{digest}",\n}}\n'
    repo = _repo(tmp_path, {"scripts/vendored.py": vendored,
                            "tests/guards/solid_architecture_guard.py": guard})

    code, out = _run(repo, "--commit", "c03")

    assert code == 1
    assert "scripts/vendored.py: vendored, needs 1 edits by hand" in out
    assert _read(repo, "scripts/vendored.py") == vendored


def test_a_commit_that_cannot_run_touches_nothing(tmp_path: Path) -> None:
    repo = _repo(tmp_path)
    assert _run(repo, "--commit", "c03")[0] == 0
    _commit(repo, "c03")
    before = _snapshot(repo)

    code, out = _run(repo, "--commit", "c03")

    assert code == 2
    assert "src/navpy/legacy/alpha.py: already moved" in out
    assert _snapshot(repo) == before
    assert _run(repo, "--commit", "c99") == (2, "step1_move: no manifest row moves in c99\n")


def test_an_occupied_destination_stops_the_move(tmp_path: Path) -> None:
    repo = _repo(tmp_path, {"src/navpy/core/kernel/__init__.py": "REAL = 1\n"})
    _run(repo, "--commit", "c03")
    _commit(repo, "c03")

    code, out = _run(repo, "--commit", "c04")

    assert code == 2
    assert "src/navpy/core/kernel/__init__.py: exists and is not an empty package init" in out
    assert (repo / "src/navpy/legacy/kpkg/__init__.py").is_file()


def test_a_folder_the_move_empties_keeps_hidden_files_and_refuses(tmp_path: Path) -> None:
    """Only bytecode caches and package inits go with an emptied old folder:
    a hidden file there stops the commit before anything moves."""
    notes = "src/navpy/legacy/kpkg/.local/notes.txt"
    repo = _repo(tmp_path)
    for commit in ("c03", "c04"):
        assert _run(repo, "--commit", commit)[0] == 0
        _commit(repo, commit)
    (repo / notes).parent.mkdir()
    (repo / notes).write_text("keep me\n", encoding="utf-8")
    before = _snapshot(repo)

    code, out = _run(repo, "--commit", "c05")

    assert code == 2, out
    assert f"{notes}: left in a folder this commit empties" in out
    assert _snapshot(repo) == before


def test_folder_removal_deletes_only_caches_and_inits(tmp_path: Path) -> None:
    files = _load("step1_move_files")
    repo = _repo(tmp_path)
    folder = repo / "src/navpy/legacy"
    for name in ("alpha.py", "beta.py", "kpkg/helper.py"):
        (folder / name).unlink()
    (folder / "kpkg/__init__.py").write_text("", encoding="utf-8")
    (folder / "__pycache__").mkdir()
    (folder / "__pycache__/alpha.cpython-311.pyc").write_bytes(b"\0")
    (folder / "stray.pyc").write_bytes(b"\0")
    (folder / ".hidden").mkdir()
    (folder / ".hidden/keep.txt").write_text("keep\n", encoding="utf-8")
    delete_rows = frozenset({"src/navpy/legacy/__init__.py"})
    before = _snapshot(repo)

    with pytest.raises(RuntimeError, match=r"legacy/\.hidden/keep\.txt"):
        files.remove_folders(repo, ["src/navpy/legacy"], delete_rows)
    assert _snapshot(repo) == before

    (folder / ".hidden/keep.txt").unlink()
    files.remove_folders(repo, ["src/navpy/legacy"], delete_rows)
    assert not folder.exists()
