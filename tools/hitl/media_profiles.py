"""What each HITL host's media runtime must hold (check_media_runtime.py): the pinned GStreamer, the elements the
encoded video loop needs, the encoder/decoder choices and, on the PC, the official installers and install layout.
Standard library only, like the check."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

ROOT_ENV = "GSTREAMER_1_0_ROOT_MSVC_X86_64"     # the variable the official MSVC x86_64 installer sets
PC_PLATFORM = "win-amd64"


@dataclass(frozen=True)
class HostProfile:
    """What one host must load: the pinned GStreamer, the elements every run uses and the encoder/decoder choices."""
    name: str
    gst_version: tuple[int, int, int]
    elements: tuple[str, ...]
    encoders: tuple[str, ...]
    decoders: tuple[str, ...]
    installers: Mapping[str, str]       # official installer file name -> SHA-256; empty where the OS supplies it
    windows_install: bool               # an official MSVC install under a root folder

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
        elements=("appsrc", "appsink", "rtspsrc", "rtph265depay", "rtph265pay", "h265parse", "rtspclientsink",
                  "videoconvert"),
        encoders=("nvh265enc", "x265enc"), decoders=("nvh265dec", "d3d11h265dec", "avdec_h265"),
        installers={
            "gstreamer-1.0-msvc-x86_64-1.26.11.msi":
                "31cbc21fa0950b5c1e79c80959b2799805cb05a7a35953a13a9f790776137605",
            "gstreamer-1.0-devel-msvc-x86_64-1.26.11.msi":
                "af0a0720692052aaf5fb0cf2847ef34a87016ef0f914328d516f049e6c86f587",
        },
        windows_install=True),
    # JetPack's GStreamer and python3-gi (stage 2); its apt versions and hashes are pinned when stage 2 installs it.
    "orin": HostProfile(
        name="orin", gst_version=(1, 20, 3),
        elements=("appsrc", "appsink", "rtspsrc", "rtph265depay", "h265parse", "videoconvert"),
        encoders=(), decoders=("nvv4l2decoder",), installers={}, windows_install=False),
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
