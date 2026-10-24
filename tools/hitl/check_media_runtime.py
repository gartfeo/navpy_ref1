"""Media runtime check of a HITL host (hitl_final_approach.md section 9, stage 1 step 7).

Checks that this Python loads the pinned GStreamer and its bindings from the pinned distro packages, that the elements
the encoded video loop needs (steps 8, 9) exist there and can be created, and that the selected encoder and decoder
pass real frames (media_roundtrip.py). Installs nothing. A run record freezes a passing result (approach_records.py).

    /usr/bin/python3 -E -s -m tools.hitl.check_media_runtime --host pc-wsl --out <private json>

Run from the repository root inside the WSL distro, with the distro's own Python: its python3-gi is the binding. It
imports only the standard library and its two sibling modules, so navpy's dependencies are not needed.

It checks each pinned package's installed version and its files against the package database (dpkg --verify), the
layout and a binding matching this interpreter, and imports gi and Gst only when layout and binding hold. Then: the
pinned Gst version, each element found and created, every loaded file (gi, its extension, the core library, the
plugins) inside the layout and owned by a pinned package at its version, and the roundtrip.
Exit 0 on a pass, 1 with every blocker listed.
"""
from __future__ import annotations

import argparse
import importlib.machinery
import json
import platform
import subprocess
import sys
import sysconfig
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from tools.hitl import media_roundtrip
from tools.hitl.media_profiles import (CORE_LIBRARY, PROFILES, HostProfile, loadable_bindings, loaded_file_problems,
                                       loaded_files, origin_problems, package_problems)

SCHEMA_VERSION = 2
DPKG_TIMEOUT_S = 120            # dpkg --verify hashes every file of the pinned packages


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


# A command runner returns (exit code, stdout, stderr); a loader returns (gi, Gst, loaded core library path) with Gst
# initialised and may raise ImportError, OSError or ValueError; a roundtrip returns (counts, problems).
Command = Callable[[Sequence[str]], tuple[int, str, str]]
Loader = Callable[[], tuple[Any, Any, Optional[str]]]
Roundtrip = Callable[[Any, str, str], tuple[dict[str, Any], list[str]]]


def run_command(args: Sequence[str]) -> tuple[int, str, str]:
    try:
        result = subprocess.run(list(args), capture_output=True, text=True, timeout=DPKG_TIMEOUT_S, check=False)
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, "", f"{type(exc).__name__}: {exc}"
    return result.returncode, result.stdout, result.stderr


def mapped_files(maps: Path = Path("/proc/self/maps")) -> list[str]:
    """The shared objects and typelibs this process has mapped."""
    try:
        lines = maps.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    files = {line.split(maxsplit=5)[5] for line in lines if len(line.split(maxsplit=5)) == 6}
    return sorted(f for f in files if f.startswith("/") and (".so" in Path(f).name or f.endswith(".typelib")))


def loaded_library_path(name: str, maps: Path = Path("/proc/self/maps")) -> Optional[str]:
    """The file this process mapped for library name (any version suffix), or None."""
    return next((file for file in mapped_files(maps) if Path(file).name.startswith(name)), None)


def loaded_inventory() -> list[str]:
    """Every shared object and typelib mapped now, and the file of every imported gi module (bindings, overrides)."""
    modules = {getattr(module, "__file__", None) for name, module in list(sys.modules.items())
               if name == "gi" or name.startswith("gi.")}
    return sorted(set(mapped_files()) | {file for file in modules if isinstance(file, str)})


def load_gst() -> tuple[Any, Any, Optional[str]]:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    return gi, Gst, loaded_library_path(CORE_LIBRARY)


def check_packages(profile: HostProfile, command: Command) -> tuple[dict[str, str], bool, list[str]]:
    """The pinned packages' installed versions, whether dpkg --verify found their files unchanged, and why not.
    Versions are compared by origin_problems."""
    names = sorted(profile.packages)
    code, out, _ = command(["dpkg-query", "-W", "-f=${Package}\t${Version}\t${db:Status-Abbrev}\n", *names])
    found = {}
    for line in out.splitlines():
        fields = line.split("\t")
        if len(fields) == 3 and fields[2].startswith("ii"):
            found[fields[0]] = fields[1]
    code, out, err = command(["dpkg", "--verify", *names])
    if code == 0 and not out.strip():
        return found, True, []
    changed = (out.strip() or err.strip() or f"exit {code}").splitlines()
    return found, False, [f"dpkg --verify of the pinned packages reports: {'; '.join(changed[:10])}"]


def file_owners(files: Sequence[str], command: Command) -> dict[str, str]:
    """The package owning each file, from dpkg-query -S ('pkg:arch: /path'); files nobody owns are left out."""
    if not files:
        return {}
    _, out, _ = command(["dpkg-query", "-S", *sorted(set(files))])
    owners = {}
    for line in out.splitlines():
        owner, separator, file = line.partition(": ")
        if separator and "," not in owner and not owner.startswith("diversion"):
            owners[file.strip()] = owner.split(":")[0]
    return owners


def bindings_in(folder: str) -> Optional[list[str]]:
    """The gi extension files (_gi.*) in folder, or None when there is no such folder."""
    path = Path(folder)
    return sorted(p.name for p in path.glob("_gi.*") if p.is_file()) if path.is_dir() else None


def check_binding(profile: HostProfile, interpreter: Interpreter,
                  listed: Optional[list[str]]) -> tuple[list[str], list[str]]:
    """The gi extension files (listed: those in the bindings folder) this interpreter can load, and why none fits.
    Import stays the final proof."""
    problems = []
    if interpreter.implementation != "CPython" or interpreter.platform != profile.platform:
        problems.append(f"this Python is {interpreter.implementation} {interpreter.platform}; the distro bindings "
                        f"need CPython {profile.platform}")
    gi_folder, present = profile.layout.gi, listed or []
    loadable = loadable_bindings(present, list(interpreter.extension_suffixes))
    if listed is None:
        problems.append(f"Python bindings folder {gi_folder} does not exist")
    elif not present:
        problems.append(f"no gi extension (_gi.*) in {gi_folder}")
    elif not loadable:
        problems.append(f"the bindings are {', '.join(present)}; Python {interpreter.version} loads only "
                        f"_gi{{{', '.join(interpreter.extension_suffixes)}}}: run this check, and the video code, "
                        f"with the distro's own python3")
    return loadable, problems


def _element(gst: Any, name: str) -> tuple[Optional[dict], list]:
    factory = gst.ElementFactory.find(name)
    if factory is None:
        return None, [f"element {name} is not registered"]
    plugin = factory.get_plugin()
    if plugin is None:
        return None, [f"element {name} has no plugin"]
    found = {"plugin": plugin.get_name(), "version": plugin.get_version(), "filename": plugin.get_filename(),
             "rank": int(factory.get_rank())}
    if gst.ElementFactory.make(name, None) is None:
        return found, [f"element {name} is registered but cannot be created"]
    return found, []


def inspect_gst(gi: Any, gst: Any, core_library: Optional[str], profile: HostProfile,
                names: Sequence[str]) -> tuple[dict[str, Any], dict[str, Any], list[str]]:
    """GStreamer's identity and each element, with every problem; origins are checked by the caller."""
    version = tuple(int(part) for part in gst.version())
    gstreamer = {"version": ".".join(map(str, version[:3])), "nano": version[3],
                 "gi_file": getattr(gi, "__file__", None),
                 "gi_extension_file": getattr(getattr(gi, "_gi", None), "__file__", None),
                 "core_library": core_library, "pygobject_version": getattr(gi, "__version__", None)}
    if version != (*profile.gst_version, 0):
        return gstreamer, {}, [f"GStreamer is {'.'.join(map(str, version))}, not the pinned release "
                               f"{profile.version_text}"]
    elements: dict[str, Any] = {}
    problems = []
    for name in names:
        found, element_problems = _element(gst, name)
        if found is not None:
            elements[name] = found
        problems.extend(element_problems)
    return gstreamer, elements, problems


def gpu_driver() -> str:
    """The NVIDIA GPU and driver nvidia-smi reports, or 'none' where it cannot."""
    code, out, _ = run_command(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
    return out.strip() if code == 0 and out.strip() else "none"


def check_runtime(profile: HostProfile, *, encoder: str, decoder: str, interpreter: Interpreter,
                  loader: Loader = load_gst, command: Command = run_command,
                  roundtrip: Roundtrip = media_roundtrip.roundtrip,
                  list_bindings: Callable[[str], Optional[list[str]]] = bindings_in,
                  inventory: Callable[[], list[str]] = loaded_inventory,
                  driver: Callable[[], str] = gpu_driver) -> dict[str, Any]:
    """The host's media runtime result (see the module docstring); verdict 'pass' only without blockers."""
    blockers: list[str] = []
    if encoder not in profile.encoders:
        blockers.append(f"encoder {encoder} is not one of {', '.join(profile.encoders)}")
    if decoder not in profile.decoders:
        blockers.append(f"decoder {decoder} is not one of {', '.join(profile.decoders)}")
    can_import = not blockers
    packages, verified, problems = check_packages(profile, command)
    blockers += problems + package_problems(profile, packages)     # independent of the import: its diagnostics run
    loadable, binding_problems = check_binding(profile, interpreter, list_bindings(profile.layout.gi))
    blockers += binding_problems
    can_import = can_import and not binding_problems

    gstreamer: dict[str, Any] = {}
    elements: dict[str, Any] = {}
    owners: dict[str, str] = {}
    loaded: list[str] = []
    exchanged: dict[str, Any] = dict(media_roundtrip.NOTHING_EXCHANGED)
    if can_import:
        try:
            gi, gst, core_library = loader()
            gstreamer, elements, problems = inspect_gst(gi, gst, core_library, profile,
                                                        [*profile.elements, encoder, decoder])
            files = [file for _, file in loaded_files(gstreamer, elements) if isinstance(file, str)]
            owners = file_owners(files, command)
            origins = origin_problems(profile, owners, gstreamer, elements)
            blockers += problems + origins
            if not problems and not origins:
                exchanged, problems = roundtrip(gst, encoder, decoder)
                blockers += problems
            loaded = inventory()                # after the roundtrip: what the encoder and decoder pulled in too
            owners.update(file_owners(loaded, command))
            blockers += loaded_file_problems(profile, owners, loaded)
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
        "gstreamer": {"pinned": profile.version_text, **gstreamer},
        "packages": packages,
        "package_files_verified": verified,
        "file_packages": owners,
        "loaded_files": loaded,
        "selected": {"encoder": encoder, "decoder": decoder},
        "elements": elements,
        "roundtrip": exchanged,
        "gpu_driver": driver(),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", choices=sorted(PROFILES), required=True)
    parser.add_argument("--encoder", help="nvcudah265enc (default) or x265enc")
    parser.add_argument("--decoder", help="nvh265dec (default) or avdec_h265")
    parser.add_argument("--out", type=Path, help="also write the result here; refuses an existing file")
    args = parser.parse_args(argv)
    profile = PROFILES[args.host]
    result = check_runtime(profile, encoder=args.encoder or profile.encoders[0],
                           decoder=args.decoder or profile.decoders[0], interpreter=this_interpreter())
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
