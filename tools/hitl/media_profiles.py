"""What a HITL host's media runtime must hold (check_media_runtime.py): the pinned GStreamer, its official installers
and install layout, the elements the encoded video loop needs and the encoder/decoder choices. Also the origin rules
a passing result must keep, shared by the check and the run record (approach_records.py). Standard library only.

Only the PC is profiled: stage 1 runs navpy's video there. The Orin (JetPack GStreamer 1.20 and python3-gi) gets its
profile, with its apt versions and hashes, when stage 2 installs it.
"""
from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT_ENV = "GSTREAMER_1_0_ROOT_MSVC_X86_64"     # the variable the official MSVC x86_64 installer sets
PC_PLATFORM = "win-amd64"
CORE_DLL = "gstreamer-1.0-0.dll"


@dataclass(frozen=True)
class HostProfile:
    """What one host must load: the pinned GStreamer, its installers, the elements every run uses and the
    encoder/decoder choices."""
    name: str
    gst_version: tuple[int, int, int]
    installers: Mapping[str, str]       # official installer file name -> SHA-256
    elements: tuple[str, ...]
    encoders: tuple[str, ...]
    decoders: tuple[str, ...]

    @property
    def version_text(self) -> str:
        return ".".join(map(str, self.gst_version))


PROFILES: Mapping[str, HostProfile] = {
    # Official 1.26.11 MSVC x86_64 runtime and devel installers; sums from
    # gstreamer.freedesktop.org/data/pkg/windows/1.26.11/msvc/<name>.sha256sum (2026-10-23).
    # The ZR10 streams HEVC over RTSP: the PC encodes (appsrc -> encoder -> parser -> payloader -> rtspclientsink)
    # and the receiver decodes (rtspsrc -> depayloader -> parser -> decoder -> videoconvert -> appsink).
    # nvh265enc/nvh265dec are NVENC/NVDEC; x265enc, d3d11h265dec and avdec_h265 are alternatives that steps 8 and 9
    # must measure before use.
    "pc": HostProfile(
        name="pc", gst_version=(1, 26, 11),
        installers={
            "gstreamer-1.0-msvc-x86_64-1.26.11.msi":
                "31cbc21fa0950b5c1e79c80959b2799805cb05a7a35953a13a9f790776137605",
            "gstreamer-1.0-devel-msvc-x86_64-1.26.11.msi":
                "af0a0720692052aaf5fb0cf2847ef34a87016ef0f914328d516f049e6c86f587",
        },
        elements=("appsrc", "appsink", "rtspsrc", "rtph265depay", "rtph265pay", "h265parse", "rtspclientsink",
                  "videoconvert"),
        encoders=("nvh265enc", "x265enc"), decoders=("nvh265dec", "d3d11h265dec", "avdec_h265")),
}


@dataclass(frozen=True)
class Layout:
    """Where an official MSVC install keeps its DLLs, Python bindings and plugins."""
    root: Path
    bin: Path
    site_packages: Path
    gi: Path
    plugins: Path


def layout_of(root: Path) -> Layout:
    site_packages = Path(root) / "lib" / "site-packages"
    return Layout(Path(root), Path(root) / "bin", site_packages, site_packages / "gi",
                  Path(root) / "lib" / "gstreamer-1.0")


def inside(path: Any, folder: Path) -> bool:
    """path (text) is folder or below it, compared as Windows compares paths."""
    if not isinstance(path, str) or not path:
        return False
    path, folder = (Path(os.path.normcase(os.path.abspath(p))) for p in (path, folder))
    return path == folder or folder in path.parents


def cpython_pc_suffix(version: str) -> str:
    """The extension suffix CPython of this version ('3.13.9') imports on the PC, e.g. '.cp313-win_amd64.pyd'."""
    major, minor = version.split(".")[:2]
    return f".cp{major}{minor}-{PC_PLATFORM.replace('-', '_')}.pyd"


def loadable_bindings(names: list[str], extension_suffixes: list[str]) -> list[str]:
    """The gi extension file names an interpreter with these extension suffixes can import."""
    return [name for name in names if any(name == "_gi" + suffix for suffix in extension_suffixes)]


def origin_problems(layout: Layout, gstreamer: Mapping[str, Any], elements: Mapping[str, Any],
                    path: str = "") -> list[str]:
    """Each loaded file that is not from the install under layout: gi, its extension, the core DLL, the plugins."""
    loaded = (("gi_file", layout.gi), ("gi_extension_file", layout.gi), ("core_dll", layout.bin))
    problems = [f"{path}gstreamer.{field} {gstreamer.get(field)} is not in {folder}"
                for field, folder in loaded if not inside(gstreamer.get(field), folder)]
    problems += [f"{path}elements.{name}.filename {element.get('filename')} is not in {layout.plugins}"
                 for name, element in sorted(elements.items()) if not inside(element.get("filename"), layout.plugins)]
    return problems
