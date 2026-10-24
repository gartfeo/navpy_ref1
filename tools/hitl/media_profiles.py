"""What a HITL host's media runtime must hold (check_media_runtime.py): the pinned GStreamer, the distro packages that
supply it and where they install, the elements the encoded video loop needs and the encoder/decoder choices. Also the
origin rules a passing result must keep, shared by the check and the run record (approach_records.py). Standard
library only.

Only the PC is profiled: stage 1 runs navpy's video in the PC's WSL Ubuntu-24.04 distro, from Ubuntu's archive.
Windows Smart App Control refuses the unsigned bindings of the official Windows installers (hitl_final_approach.md
step 7). The Orin (JetPack GStreamer 1.20 and python3-gi) gets its profile, with its apt versions, when stage 2
installs it.
"""
from __future__ import annotations

import posixpath
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Any

CORE_LIBRARY = "libgstreamer-1.0.so"         # the loaded file is this name plus its soname/version suffix
LOADED_FILES = ("gi_file", "gi_extension_file", "core_library")
# The Windows GPU driver's libraries as WSL mounts them (libcuda, libnvidia-encode, libnvcuvid); no package owns them,
# and the result records the driver version (nvidia-smi).
WSL_DRIVER_FOLDER = "/usr/lib/wsl"


@dataclass(frozen=True)
class Layout:
    """Where the distro packages put the Python bindings, the core library and the plugins."""
    gi: str
    libraries: str
    plugins: str


@dataclass(frozen=True)
class HostProfile:
    """What one host must load: the pinned GStreamer, the packages supplying it at exact versions, their layout, the
    interpreter platform, the elements every run uses and the encoder/decoder choices."""
    name: str
    gst_version: tuple[int, int, int]
    packages: Mapping[str, str]         # package -> exact installed version
    layout: Layout
    platform: str
    elements: tuple[str, ...]
    encoders: tuple[str, ...]
    decoders: tuple[str, ...]

    @property
    def version_text(self) -> str:
        return ".".join(map(str, self.gst_version))


PROFILES: Mapping[str, HostProfile] = {
    # Ubuntu 24.04 (noble) archive packages as installed 2026-10-24; a security update changes a version and needs a
    # new pin, so a run never mixes runtimes silently.
    # The ZR10 streams HEVC over RTSP: the PC encodes (appsrc -> encoder -> parser -> payloader -> rtspclientsink)
    # and the receiver decodes (rtspsrc -> depayloader -> parser -> decoder -> videoconvert -> appsink).
    # nvcudah265enc/nvh265dec are NVENC/NVDEC; the older nvh265enc refuses its preset with this driver (597.13).
    # x265enc and avdec_h265 are software alternatives that steps 8 and 9 must measure before use.
    "pc-wsl": HostProfile(
        name="pc-wsl", gst_version=(1, 24, 2),
        packages={
            "libgstreamer1.0-0": "1.24.2-1ubuntu0.1",
            "gstreamer1.0-plugins-base": "1.24.2-1ubuntu0.5",
            "gstreamer1.0-plugins-good": "1.24.2-1ubuntu1.8",
            "gstreamer1.0-plugins-bad": "1.24.2-1ubuntu4",
            "gstreamer1.0-libav": "1.24.1-1build1",
            "gstreamer1.0-rtsp": "1.24.2-1",
            "python3-gi": "3.48.2-1",
            "python3-gst-1.0": "1.24.1-1",
            "gir1.2-gstreamer-1.0": "1.24.2-1ubuntu0.1",
            "gir1.2-gst-plugins-base-1.0": "1.24.2-1ubuntu0.5",
            # The support libraries the elements load (found by the loaded-file check, 2026-10-24).
            "libgstreamer-plugins-base1.0-0": "1.24.2-1ubuntu0.5",
            "libgstreamer-plugins-bad1.0-0": "1.24.2-1ubuntu4",
            "libgstreamer-gl1.0-0": "1.24.2-1ubuntu0.5",
            "libgstrtspserver-1.0-0": "1.24.2-1",
        },
        layout=Layout(gi="/usr/lib/python3/dist-packages/gi", libraries="/usr/lib/x86_64-linux-gnu",
                      plugins="/usr/lib/x86_64-linux-gnu/gstreamer-1.0"),
        platform="linux-x86_64",
        elements=("appsrc", "appsink", "rtspsrc", "rtph265depay", "rtph265pay", "h265parse", "rtspclientsink",
                  "videoconvert"),
        encoders=("nvcudah265enc", "x265enc"), decoders=("nvh265dec", "avdec_h265")),
}


def upstream_version(package_version: str) -> str:
    """The upstream part of a Debian version: '1:1.24.1-1build1' -> '1.24.1'."""
    return package_version.split(":", 1)[-1].rsplit("-", 1)[0]


def inside(path: Any, folder: str) -> bool:
    """path (absolute POSIX text) is folder or below it."""
    if not isinstance(path, str) or not path.startswith("/"):
        return False
    path, base = PurePosixPath(posixpath.normpath(path)), PurePosixPath(folder)
    return path == base or base in path.parents


def cpython_suffixes(version: str) -> list[str]:
    """The extension suffixes CPython of this version ('3.12.3') imports on x86-64 Linux (importlib.machinery
    EXTENSION_SUFFIXES): ['.cpython-312-x86_64-linux-gnu.so', '.abi3.so', '.so']."""
    major, minor = version.split(".")[:2]
    return [f".cpython-{major}{minor}-x86_64-linux-gnu.so", ".abi3.so", ".so"]


def loadable_bindings(names: list[str], extension_suffixes: list[str]) -> list[str]:
    """The gi extension file names an interpreter with these extension suffixes can import."""
    return [name for name in names if any(name == "_gi" + suffix for suffix in extension_suffixes)]


def loaded_files(gstreamer: Mapping[str, Any], elements: Mapping[str, Any]) -> list[tuple[str, Any]]:
    """(field path, file) of every file a result says it loaded: gi, its extension, the core library, the plugins."""
    return ([(f"gstreamer.{field}", gstreamer.get(field)) for field in LOADED_FILES]
            + [(f"elements.{name}.filename", element.get("filename")) for name, element in sorted(elements.items())])


def package_problems(profile: HostProfile, packages: Mapping[str, Any], path: str = "") -> list[str]:
    """Each pinned package not installed at its version, and each package that is not pinned."""
    problems = [f"{path}packages.{name} is {packages.get(name) or 'not installed'}, not the pinned {version}"
                for name, version in sorted(profile.packages.items()) if packages.get(name) != version]
    return problems + [f"{path}packages.{name} is not a pinned package" for name in sorted(packages)
                       if name not in profile.packages]


def origin_problems(profile: HostProfile, file_packages: Mapping[str, Any], gstreamer: Mapping[str, Any],
                    elements: Mapping[str, Any], path: str = "") -> list[str]:
    """Each loaded file outside the layout or not owned by a pinned package, and each plugin whose version is not
    its package's."""
    layout = profile.layout
    problems = []
    folders = {"gstreamer.gi_file": layout.gi, "gstreamer.gi_extension_file": layout.gi,
               "gstreamer.core_library": layout.libraries}
    for field, file in loaded_files(gstreamer, elements):
        folder = folders.get(field, layout.plugins)
        if not inside(file, folder):
            problems.append(f"{path}{field} {file} is not in {folder}")
        owner = file_packages.get(file) if isinstance(file, str) else None
        if owner not in profile.packages:
            problems.append(f"{path}{field} {file} is not from a pinned package (owner {owner or 'none'})")
    for name, element in sorted(elements.items()):
        owner = file_packages.get(element.get("filename"))
        if owner in profile.packages and element.get("version") != upstream_version(profile.packages[owner]):
            problems.append(f"{path}elements.{name}.version {element.get('version')} is not {owner}'s "
                            f"{upstream_version(profile.packages[owner])}")
    return problems


def is_media_file(file: str) -> bool:
    """A GStreamer library or plugin, a Gst typelib, or a file of any gi package folder: gi.overrides extends its
    path over every gi/overrides on sys.path, so bindings and overrides need not sit in the layout's folder."""
    path = PurePosixPath(file)
    return (path.name.startswith("libgst") or (path.name.startswith("Gst") and path.name.endswith(".typelib"))
            or "gi" in path.parts[:-1])


def loaded_file_problems(profile: HostProfile, file_packages: Mapping[str, Any], loaded: list,
                         path: str = "") -> list[str]:
    """Each file the process loaded that no package owns (outside the WSL driver folder), and each media file that
    no pinned package owns: a library, typelib or override taken from LD_LIBRARY_PATH, GI_TYPELIB_PATH or a copy."""
    problems = []
    for file in loaded:
        owner = file_packages.get(file)
        if is_media_file(file) and owner not in profile.packages:
            problems.append(f"{path}loaded_files: {file} is not from a pinned package (owner {owner or 'none'})")
        elif owner is None and not inside(file, WSL_DRIVER_FOLDER):
            problems.append(f"{path}loaded_files: {file} is not from a distro package")
    return problems
