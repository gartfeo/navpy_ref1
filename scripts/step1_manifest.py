"""Shared reader of the refactor step-1 plan data in docs/design/step1/.

The step-1 guards and the move tool read the plan only through this module:
manifest.tsv (one row per file under src/navpy), splits.json, reexports.tsv,
exceptions.tsv and commits.tsv. Manifest paths are relative to src/navpy; a
"gcs:<path>" home means src/gcs/<path> and "delete" means the file goes.
Only the loaders and current_location read the repository; the rest is pure.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

Stage = Literal["step1", "final"]
STEP1_DATA = Path("docs", "design", "step1")
NAVPY_ROOT = "src/navpy/"
GCS_ROOT = "src/gcs/"
GCS_HOME = "gcs:"
DELETE = "delete"
MOVING_ACTIONS = frozenset({"move", "split", "gcs"})
ACTIONS = MOVING_ACTIONS | {"delete", "stay"}
# Files at the src/navpy root that stay there as the composition root.
ROOT_APP_FILES = frozenset({"__init__.py", "main.py", "main_gui.py"})
MANIFEST_COLUMNS = ("src", "final_dest", "step1_dest", "action", "vision_path", "note")
REEXPORT_COLUMNS = ("src", "line", "name", "imported_from", "why")
EXCEPTION_COLUMNS = ("guard", "file", "imports", "from_layer", "to_layer", "where",
                     "type_checking", "fix_step", "why", "old_src")
COMMIT_COLUMNS = ("commit", "step1_prefix")


@dataclass(frozen=True)
class ManifestRow:
    src: str
    final_dest: str
    step1_dest: str
    action: str
    vision_path: bool
    note: str

    def dest(self, stage: Stage) -> str:
        return {"step1": self.step1_dest, "final": self.final_dest}[stage]


@dataclass(frozen=True)
class SplitPart:
    symbols: tuple[str, ...]
    final: str
    step1: str
    vision_path: bool

    def dest(self, stage: Stage) -> str:
        return {"step1": self.step1, "final": self.final}[stage]


@dataclass(frozen=True)
class Reexport:
    src: str
    line: int
    name: str
    imported_from: str
    why: str


@dataclass(frozen=True)
class GuardException:
    guard: str
    file: str
    imports: str
    from_layer: str
    to_layer: str
    where: str
    type_checking: bool
    fix_step: str
    why: str
    old_src: str


@dataclass(frozen=True)
class CommitPrefix:
    commit: str
    step1_prefix: str


@dataclass(frozen=True)
class Layer:
    """kind: products, core, plugins, sim, app, gcs, deleted or old (still in
    the old tree); part: the mission, core folder or plugin family."""

    kind: str
    part: str = ""

    def __str__(self) -> str:
        return f"{self.kind}/{self.part}" if self.part else self.kind


def load_manifest(repo_root: Path) -> tuple[ManifestRow, ...]:
    rows: list[ManifestRow] = []
    seen: set[str] = set()
    for record in _read_tsv(repo_root / STEP1_DATA / "manifest.tsv", MANIFEST_COLUMNS):
        if record["src"] in seen:
            raise ValueError(f"manifest.tsv: duplicate src {record['src']}")
        if record["action"] not in ACTIONS:
            raise ValueError(f"manifest.tsv: {record['src']}: unknown action {record['action']!r}")
        seen.add(record["src"])
        rows.append(ManifestRow(
            record["src"], record["final_dest"], record["step1_dest"], record["action"],
            _flag(record["vision_path"], "y", "n"), record["note"],
        ))
    return tuple(rows)


def load_splits(repo_root: Path) -> dict[str, tuple[SplitPart, ...]]:
    """Old src -> the parts split out of it (the rest stays in the manifest row)."""
    text = (repo_root / STEP1_DATA / "splits.json").read_text(encoding="utf-8")
    return {
        src: tuple(
            SplitPart(tuple(part["symbols"]), part["final"], part["step1"],
                      _flag(part["vision_path"], "y", "n"))
            for part in entry["parts"]
        )
        for src, entry in json.loads(text).items()
    }


def load_reexports(repo_root: Path) -> tuple[Reexport, ...]:
    return tuple(
        Reexport(r["src"], int(r["line"]), r["name"], r["imported_from"], r["why"])
        for r in _read_tsv(repo_root / STEP1_DATA / "reexports.tsv", REEXPORT_COLUMNS)
    )


def load_exceptions(repo_root: Path) -> tuple[GuardException, ...]:
    return tuple(
        GuardException(
            r["guard"], r["file"], r["imports"], r["from_layer"], r["to_layer"], r["where"],
            _flag(r["type_checking"], "yes", "no"), r["fix_step"], r["why"], r["old_src"],
        )
        for r in _read_tsv(repo_root / STEP1_DATA / "exceptions.tsv", EXCEPTION_COLUMNS)
    )


def load_commits(repo_root: Path) -> tuple[CommitPrefix, ...]:
    return tuple(
        CommitPrefix(r["commit"], r["step1_prefix"])
        for r in _read_tsv(repo_root / STEP1_DATA / "commits.tsv", COMMIT_COLUMNS)
    )


def repo_path(manifest_path: str) -> str | None:
    """Repo-relative path of a manifest src or home; None for "delete"."""
    if manifest_path == DELETE:
        return None
    if manifest_path.startswith(GCS_HOME):
        return GCS_ROOT + manifest_path[len(GCS_HOME):]
    return NAVPY_ROOT + manifest_path


def module_name(repo_relative: str) -> str | None:
    """Dotted module of a repo-relative .py path under src/ (a package's
    __init__.py names the package); None for any other file."""
    if not repo_relative.startswith("src/") or not repo_relative.endswith(".py"):
        return None
    parts = repo_relative[len("src/"):-len(".py")].split("/")
    if parts[-1] == "__init__":
        parts.pop()
    return ".".join(parts)


def path_map(rows: tuple[ManifestRow, ...], stage: Stage) -> dict[str, str | None]:
    """Repo-relative old path -> path at stage (None: deleted) for every row."""
    return {NAVPY_ROOT + row.src: repo_path(row.dest(stage)) for row in rows}


def module_map(rows: tuple[ManifestRow, ...], stage: Stage) -> dict[str, str | None]:
    """Old dotted module -> module at stage (None: deleted) for every .py row.
    A split row maps to the module that keeps the rest; see split_symbol_map."""
    result: dict[str, str | None] = {}
    for old, new in path_map(rows, stage).items():
        old_module = module_name(old)
        if old_module is not None:
            result[old_module] = None if new is None else module_name(new)
    return result


def split_symbol_map(
    splits: dict[str, tuple[SplitPart, ...]], stage: Stage
) -> dict[tuple[str, str], str]:
    """(old dotted module, symbol) -> module of the split part defining it."""
    result: dict[tuple[str, str], str] = {}
    for src, parts in splits.items():
        old_module = _navpy_module(src)
        for part in parts:
            for symbol in part.symbols:
                result[(old_module, symbol)] = _navpy_module(part.dest(stage))
    return result


def current_location(repo_root: Path, row: ManifestRow) -> str | None:
    """Repo-relative path of the row's file now: its old path until it has
    moved, then its step-1 home; None when neither exists (deleted).

    The old path wins when both exist: c02 makes empty package inits at new
    homes before the rows that bring the real ones move."""
    for candidate in (NAVPY_ROOT + row.src, repo_path(row.step1_dest)):
        if candidate is not None and (repo_root / candidate).is_file():
            return candidate
    return None


def layer_of(path: str) -> Layer:
    """Layer of a path under src/navpy (also a manifest "gcs:" or "delete" home)."""
    path = path.replace("\\", "/")
    if path.startswith("src/"):
        raise ValueError(f"layer_of takes a path under src/navpy, got {path!r}")
    if path == DELETE:
        return Layer("deleted")
    if path.startswith(GCS_HOME):
        return Layer("gcs")
    parts = path.split("/")
    if len(parts) == 1:
        return Layer("app") if parts[0] in ROOT_APP_FILES else Layer("old")
    if parts[0] in ("products", "core", "plugins"):
        return Layer(parts[0], parts[1] if len(parts) > 2 else "")
    if parts[0] in ("sim", "app"):
        return Layer(parts[0])
    return Layer("old")


def allowed(importer: Layer, imported: Layer) -> bool:
    """May a file in `importer` import one in `imported`? The layout's import
    rules; an end still in the old tree is not judged until it lands (True)."""
    if importer.kind in ("app", "deleted", "gcs", "old") or imported.kind == "old":
        return True
    if imported.kind in ("app", "gcs", "deleted"):
        return False
    neutral = imported.kind == "core" and imported.part in ("kernel", "ports")
    if importer.kind == "core":
        if importer.part == "kernel":
            return imported == Layer("core", "kernel")
        return neutral if importer.part == "ports" else imported.kind == "core"
    if importer.kind == "plugins":
        return neutral or imported == importer
    if importer.kind == "products":
        return imported.kind == "core" or imported == importer
    return importer.kind == "sim" and imported.kind in ("core", "plugins", "sim")


def commit_of(row: ManifestRow, prefixes: tuple[CommitPrefix, ...]) -> str | None:
    """Commit moving the row: longest commits.tsv prefix of its step-1 home.
    None for rows that do not move (delete, stay) or match no prefix."""
    if row.action not in MOVING_ACTIONS:
        return None
    matches = [p for p in prefixes if row.step1_dest.startswith(p.step1_prefix)]
    if not matches:
        return None
    return max(matches, key=lambda p: len(p.step1_prefix)).commit


def rows_for_commit(
    rows: tuple[ManifestRow, ...], prefixes: tuple[CommitPrefix, ...], commit: str
) -> tuple[ManifestRow, ...]:
    return tuple(row for row in rows if commit_of(row, prefixes) == commit)


def _navpy_module(manifest_path: str) -> str:
    module = module_name(NAVPY_ROOT + manifest_path)
    if module is None:
        raise ValueError(f"not a Python module path: {manifest_path!r}")
    return module


def _flag(value: str, true: str, false: str) -> bool:
    if value not in (true, false):
        raise ValueError(f"expected {true!r} or {false!r}, got {value!r}")
    return value == true


def _read_tsv(path: Path, columns: tuple[str, ...]) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle, delimiter="\t")
        if tuple(reader.fieldnames or ()) != columns:
            raise ValueError(f"{path.name}: columns {reader.fieldnames}, expected {list(columns)}")
        rows: list[dict[str, str]] = []
        for record in reader:
            if None in record or None in record.values():
                raise ValueError(f"{path.name}:{reader.line_num}: expected {len(columns)} fields")
            rows.append({key: value.strip() for key, value in record.items()})
    return rows
