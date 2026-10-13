"""Raw-byte artifact comparison and the wall-mode canonical view.

Deterministic artifacts compare as raw bytes: the same file-name set and equal
contents, no canonicalizer. Host observations are excluded by name only
(``*_host*`` files, BIN logs). Wall mode keeps truthful host values in its
artifacts, so only its comparison view canonicalizes: the run directory
becomes ``<run_dir>`` and each sysid becomes its UAV ordinal.
"""

from __future__ import annotations

import fnmatch
from typing import Any


#: Host observations, never compared (step 5 of the deterministic eval design).
HOST_ONLY_GLOBS = ("*_host*", "*.BIN", "*.bin")
RUN_DIR_TOKEN = "<run_dir>"
_SYSID_FIELDS = frozenset({
    "sys_id", "sys_ids", "owner_sys_id", "expected_sys_ids", "launch_order",
})


def is_host_only(relative_path: str) -> bool:
    name = relative_path.rsplit("/", 1)[-1]
    return any(fnmatch.fnmatchcase(name, pattern) for pattern in HOST_ONLY_GLOBS)


def without_host_only(inventory: dict[str, str]) -> dict[str, str]:
    """The compared part of a run-relative ``path -> sha256`` inventory."""
    return {path: digest for path, digest in inventory.items() if not is_host_only(path)}


def compare_inventories(left: dict[str, str], right: dict[str, str]) -> dict:
    """Byte equality per file; equal only with the same names and contents."""
    shared = sorted(left.keys() & right.keys())
    differing = [name for name in shared if left[name] != right[name]]
    only_left = sorted(left.keys() - right.keys())
    only_right = sorted(right.keys() - left.keys())
    return {
        "status": "equal" if not (differing or only_left or only_right) else "different",
        "compared_files": len(shared),
        "differing_files": differing,
        "only_in_reference": only_left,
        "only_in_repeat": only_right,
        "excluded_globs": list(HOST_ONLY_GLOBS),
    }


def canonical_view(value: Any, sysid_ordinals: dict[int, int], run_dir: str) -> Any:
    """The wall comparison view: run-dir prefix and sysids made run-independent."""
    def ordinal(sys_id: Any) -> Any:
        if type(sys_id) is int and sys_id in sysid_ordinals:
            return f"uav{sysid_ordinals[sys_id]}"
        return sys_id

    def walk(item: Any, field: str | None) -> Any:
        if isinstance(item, dict):
            return {_key(key): walk(child, key) for key, child in item.items()}
        if isinstance(item, list):
            return [walk(child, field) for child in item]
        if field in _SYSID_FIELDS:
            return ordinal(item)
        if isinstance(item, str) and run_dir:
            return _run_relative(item)
        return item

    def _run_relative(text: str) -> str:
        # Only the directory itself or a path inside it, never a sibling
        # such as ``<run_dir>-other``.
        if text == run_dir:
            return RUN_DIR_TOKEN
        for separator in ("/", "\\"):
            if text.startswith(run_dir + separator):
                return RUN_DIR_TOKEN + text[len(run_dir):]
        return text

    def _key(key: str) -> str:
        if key.isdigit() and int(key) in sysid_ordinals:
            return f"uav{sysid_ordinals[int(key)]}"
        return key

    return walk(value, None)


__all__ = [
    "HOST_ONLY_GLOBS",
    "RUN_DIR_TOKEN",
    "canonical_view",
    "compare_inventories",
    "is_host_only",
    "without_host_only",
]
