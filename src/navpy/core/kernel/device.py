"""Shared GPU/CPU device-resolution helpers for the vision package.

Single source of truth so the YOLO detector, deep-search detector, and the
tracker ReID backends resolve "auto"/CUDA/CPU the same way and treat *any*
non-negative GPU index as CUDA (not just index 0).
"""
from __future__ import annotations

from typing import Union

DeviceT = Union[str, int]


def cuda_available() -> bool:
    """True if a CUDA device is usable. Never raises (torch may be absent)."""
    try:
        import torch
        return bool(torch.cuda.is_available())
    except Exception:
        return False


def resolve_auto_device(device: DeviceT) -> DeviceT:
    """Resolve "auto" to GPU index 0 (if CUDA) or "cpu". Pass through anything else."""
    if device == "auto":
        return 0 if cuda_available() else "cpu"
    return device


def uses_cuda_device(device: DeviceT) -> bool:
    """True if ``device`` denotes a CUDA device.

    Accepts any non-negative integer index (e.g. ``1`` for a second GPU), a
    numeric string (``"0"``, ``"1"``), or a ``"cuda[:N]"`` string. ``bool`` is
    explicitly excluded so ``True``/``False`` are never mistaken for an index.
    """
    if isinstance(device, bool):
        return False
    if isinstance(device, int):
        return device >= 0
    s = str(device).strip().lower()
    if s.isdigit():
        return True
    return s.startswith("cuda")


def resolve_reid_device(config_device: DeviceT, detector_device: DeviceT) -> str:
    """Resolve a ReID embedder device to a torch device string ("cuda:N"/"cpu").

    ``config_device == "auto"`` inherits the detector's resolved device.
    """
    device = detector_device if config_device == "auto" else config_device
    device = resolve_auto_device(device)
    if not uses_cuda_device(device):
        return "cpu"
    if isinstance(device, int):
        return f"cuda:{device}"
    s = str(device).strip().lower()
    if s.startswith("cuda"):
        return s
    return f"cuda:{int(s)}"


def free_cuda_cache(device: DeviceT) -> None:
    """Release cached CUDA memory if ``device`` is a CUDA device.

    Safe under any failure (torch missing, CUDA gone) — close() paths must
    never raise on teardown.
    """
    if not uses_cuda_device(device):
        return
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:
        pass
