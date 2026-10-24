"""Media runtime check of a HITL host (hitl_final_approach.md section 9, stage 1 step 7).

Checks that this Python can load the pinned GStreamer and its bindings, and that the elements the encoded video loop
needs (steps 8, 9) exist, come from that GStreamer and can be created. Installs nothing. A run record freezes a passing
result (approach_records.py).

    python -E -s -m tools.hitl.check_media_runtime --host pc --root <GStreamer root> --installers <MSI folder>

Run from the repository root. It imports only the standard library and media_profiles.py, so it runs from a separate
Python that matches the bindings: the official 1.26 MSVC installers ship PyGObject for one Python version (the
gi\\_gi*.pyd tag), not necessarily navpy's. The bindings are loaded process-locally (sys.path and
os.add_dll_directory), never through PATH, PYTHONPATH or the registry; the code that later streams video must load
them the same way (the result's launch field).

It checks the installers' official SHA-256 sums, the layout and a binding matching this interpreter, and imports gi
and Gst only when layout and binding hold. Then: gi, its extension and the core DLL loaded from the install, the
pinned Gst version, and each element found, from the install's plugin folder at the pinned version, and created.
Exit 0 on a pass, 1 with every blocker listed.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib.machinery
import json
import os
import platform
import subprocess
import sys
import sysconfig
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ContextManager, Optional

from tools.hitl.media_profiles import (CORE_DLL, PC_PLATFORM, PROFILES, ROOT_ENV, HostProfile, Layout, layout_of,
                                       loadable_bindings, origin_problems)

SCHEMA_VERSION = 1


@dataclass(frozen=True)
class Interpreter:
    """The Python running the check, as far as loading the bindings depends on it."""
    version: str
    implementation: str
    platform: str
    extension_suffixes: tuple[str, ...]
    executable: str


def this_interpreter() -> Interpreter:
    return Interpreter(version=platform.python_version(), implementation=platform.python_implementation(),
                       platform=sysconfig.get_platform(),
                       extension_suffixes=tuple(importlib.machinery.EXTENSION_SUFFIXES), executable=sys.executable)


# A loader yields (gi, Gst, path of the loaded core DLL) with Gst initialised; it may raise ImportError, OSError or
# ValueError.
Loader = Callable[[Layout], ContextManager[tuple[Any, Any, Optional[str]]]]


def loaded_dll_path(name: str) -> Optional[str]:
    """The file this process loaded as DLL name, or None (Windows only)."""
    import ctypes
    from ctypes import wintypes
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetModuleHandleW.argtypes, kernel32.GetModuleHandleW.restype = [wintypes.LPCWSTR], wintypes.HMODULE
    kernel32.GetModuleFileNameW.argtypes = [wintypes.HMODULE, wintypes.LPWSTR, wintypes.DWORD]
    handle = kernel32.GetModuleHandleW(name)
    if not handle:
        return None
    buffer = ctypes.create_unicode_buffer(32768)
    return buffer.value if kernel32.GetModuleFileNameW(handle, buffer, len(buffer)) else None


@contextlib.contextmanager
def load_gst(layout: Layout) -> Iterator[tuple[Any, Any, Optional[str]]]:
    """Import gi and Gst from layout's install, process-locally. The DLL directory stays added until the caller is
    done with Gst."""
    with contextlib.ExitStack() as stack:
        stack.enter_context(os.add_dll_directory(str(layout.bin)))
        sys.path.insert(0, str(layout.site_packages))
        stack.callback(sys.path.remove, str(layout.site_packages))
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        yield gi, Gst, loaded_dll_path(CORE_DLL)


def check_installers(profile: HostProfile, folder: Optional[Path]) -> tuple[dict[str, str], list[str]]:
    """Each pinned installer's SHA-256 as found in folder, and every missing or differing one."""
    if folder is None:
        return {}, ["--installers is required: the installers' SHA-256 must match the official sums"]
    found: dict[str, str] = {}
    problems = []
    for name, expected in sorted(profile.installers.items()):
        path = Path(folder) / name
        if not path.is_file():
            problems.append(f"installer {name} is not in {folder}")
            continue
        with path.open("rb") as handle:
            found[name] = hashlib.file_digest(handle, "sha256").hexdigest()
        if found[name] != expected:
            problems.append(f"installer {name} SHA-256 {found[name]} is not the official {expected}")
    return found, problems


def check_layout(layout: Layout) -> list[str]:
    return [f"{what} {path} does not exist" for what, path in
            (("GStreamer DLL folder", layout.bin), ("Python bindings folder", layout.gi),
             ("plugin folder", layout.plugins)) if not path.is_dir()]


def check_binding(layout: Layout, interpreter: Interpreter) -> tuple[list[str], list[str]]:
    """The gi extension files this interpreter can load, and why none fits. Import stays the final proof."""
    problems = []
    if interpreter.implementation != "CPython" or interpreter.platform != PC_PLATFORM:
        problems.append(f"this Python is {interpreter.implementation} {interpreter.platform}; the official MSVC "
                        f"bindings need CPython {PC_PLATFORM}")
    present = sorted(p.name for p in layout.gi.glob("_gi.*") if p.is_file()) if layout.gi.is_dir() else []
    loadable = loadable_bindings(present, list(interpreter.extension_suffixes))
    if not present:
        problems.append(f"no gi extension (_gi.*) in {layout.gi}")
    elif not loadable:
        problems.append(f"the bindings are {', '.join(present)}; Python {interpreter.version} loads only "
                        f"_gi{{{', '.join(interpreter.extension_suffixes)}}}: run this check, and the video code, "
                        f"from a Python that matches the bindings")
    return loadable, problems


def _element(gst: Any, name: str, profile: HostProfile) -> tuple[Optional[dict], list]:
    factory = gst.ElementFactory.find(name)
    if factory is None:
        return None, [f"element {name} is not registered"]
    plugin = factory.get_plugin()
    if plugin is None:
        return None, [f"element {name} has no plugin"]
    found = {"plugin": plugin.get_name(), "version": plugin.get_version(), "filename": plugin.get_filename(),
             "rank": int(factory.get_rank())}
    problems = []
    if found["version"] != profile.version_text:
        problems.append(f"element {name} plugin {found['plugin']} is version {found['version']}, not "
                        f"{profile.version_text}")
    if gst.ElementFactory.make(name, None) is None:
        problems.append(f"element {name} is registered but cannot be created")
    return found, problems


def inspect_gst(gi: Any, gst: Any, core_dll: Optional[str], profile: HostProfile, layout: Layout,
                names: Sequence[str]) -> tuple[dict[str, Any], dict[str, Any], list[str]]:
    """GStreamer's identity and each element's origin, with every problem."""
    version = tuple(int(part) for part in gst.version())
    gstreamer = {"version": ".".join(map(str, version[:3])), "nano": version[3],
                 "gi_file": getattr(gi, "__file__", None),
                 "gi_extension_file": getattr(getattr(gi, "_gi", None), "__file__", None), "core_dll": core_dll,
                 "pygobject_version": getattr(gi, "__version__", None)}
    if version != (*profile.gst_version, 0):
        problems = [f"GStreamer is {'.'.join(map(str, version))}, not the pinned release {profile.version_text}"]
        return gstreamer, {}, problems + origin_problems(layout, gstreamer, {})
    elements: dict[str, Any] = {}
    problems = []
    for name in names:
        found, element_problems = _element(gst, name, profile)
        if found is not None:
            elements[name] = found
        problems.extend(element_problems)
    return gstreamer, elements, origin_problems(layout, gstreamer, elements) + problems


def gpu_driver() -> str:
    """The NVIDIA GPU and driver nvidia-smi reports, or 'none' where it cannot."""
    try:
        result = subprocess.run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
                                capture_output=True, text=True, timeout=30, check=False)
    except (OSError, subprocess.SubprocessError):
        return "none"
    return result.stdout.strip() if result.returncode == 0 and result.stdout.strip() else "none"


def check_runtime(profile: HostProfile, *, root: Optional[Path], installers: Optional[Path], encoder: str,
                  decoder: str, interpreter: Interpreter, loader: Loader = load_gst,
                  driver: Callable[[], str] = gpu_driver) -> dict[str, Any]:
    """The host's media runtime result (see the module docstring); verdict 'pass' only without blockers."""
    blockers: list[str] = []
    if encoder not in profile.encoders:
        blockers.append(f"encoder {encoder} is not one of {', '.join(profile.encoders)}")
    if decoder not in profile.decoders:
        blockers.append(f"decoder {decoder} is not one of {', '.join(profile.decoders)}")
    can_import = not blockers
    found_installers, problems = check_installers(profile, installers)
    blockers += problems                        # independent of the import: its diagnostics still run
    layout: Optional[Layout] = None
    loadable: list[str] = []
    if root is None:
        blockers.append(f"no GStreamer root: pass --root or set {ROOT_ENV}")
        can_import = False
    else:
        layout = layout_of(root)
        layout_problems = check_layout(layout)
        loadable, binding_problems = check_binding(layout, interpreter)
        blockers += layout_problems + binding_problems
        can_import = can_import and not layout_problems and not binding_problems

    gstreamer: dict[str, Any] = {}
    elements: dict[str, Any] = {}
    if can_import:
        try:
            with loader(layout) as (gi, gst, core_dll):
                gstreamer, elements, problems = inspect_gst(gi, gst, core_dll, profile, layout,
                                                            [*profile.elements, encoder, decoder])
                blockers += problems
        except (ImportError, OSError, ValueError) as exc:
            blockers.append(f"importing gi/Gst failed: {type(exc).__name__}: {exc}")

    return {
        "schema_version": SCHEMA_VERSION,
        "host": profile.name,
        "verdict": "blocked" if blockers else "pass",
        "blockers": blockers,
        "python": {"version": interpreter.version, "implementation": interpreter.implementation,
                   "platform": interpreter.platform, "extension_suffixes": list(interpreter.extension_suffixes),
                   "executable": interpreter.executable, "gi_extensions": loadable},
        "gstreamer": {"pinned": profile.version_text, "root": None if layout is None else str(layout.root),
                      **gstreamer},
        "launch": None if layout is None else {"dll_directory": str(layout.bin),
                                               "sys_path": str(layout.site_packages)},
        "installers": found_installers,
        "selected": {"encoder": encoder, "decoder": decoder},
        "elements": elements,
        "gpu_driver": driver(),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", choices=sorted(PROFILES), required=True)
    parser.add_argument("--root", type=Path, help=f"GStreamer root (default ${ROOT_ENV})")
    parser.add_argument("--installers", type=Path, help="folder holding the official installers")
    parser.add_argument("--encoder", help="nvh265enc (default) or x265enc")
    parser.add_argument("--decoder", help="nvh265dec (default), d3d11h265dec or avdec_h265")
    parser.add_argument("--out", type=Path, help="also write the result here; refuses an existing file")
    args = parser.parse_args(argv)
    profile = PROFILES[args.host]
    root = args.root or (Path(os.environ[ROOT_ENV]) if os.environ.get(ROOT_ENV) else None)
    result = check_runtime(profile, root=root, installers=args.installers,
                           encoder=args.encoder or profile.encoders[0], decoder=args.decoder or profile.decoders[0],
                           interpreter=this_interpreter())
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.out is not None:
        try:
            with args.out.open("x", encoding="utf-8", newline="\n") as handle:
                handle.write(text)
        except FileExistsError:
            print(f"refused: {args.out} exists", file=sys.stderr)
            return 1
    sys.stdout.write(text)
    for blocker in result["blockers"]:
        print(f"BLOCKED: {blocker}", file=sys.stderr)
    return 0 if result["verdict"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
