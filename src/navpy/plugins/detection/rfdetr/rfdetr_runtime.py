"""RF-DETR weights run through the ``rfdetr`` package (pip rfdetr 1.12.0, as gimbal-controller; HITL step 23b).

``rfdetr`` is not a navpy dependency yet (the owner approves that): it is imported only when this runtime is built,
in an environment that has it. The runtime takes the codec's normalised (B, 3, S, S) blob and returns the model's
raw boxes (B, Q, 4: cx cy w h, normalised) and logits (B, Q, 91); pre- and post-processing are navpy's
(rfdetr_codec.py), so the model's own ``predict()`` is not used. On CUDA the forward pass runs under FP16 autocast
(gimbal-controller's live RF-DETR-L is a TensorRT FP16 engine, its parity in docs/CAR-MOT-LIVE.md); the outputs are
returned as float32.
"""
from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np

from navpy.core.kernel.device import DeviceT, cuda_available, free_cuda_cache, resolve_auto_device, uses_cuda_device

# rfdetr's model class per variant.
_VARIANT_CLASSES = {"large": "RFDETRLarge"}


class RfDetrRuntime(Protocol):
    device: DeviceT

    def infer(self, blob: np.ndarray) -> tuple[np.ndarray, np.ndarray]: ...

    def close(self) -> None: ...


class TorchRfDetrRuntime:
    """The weights in rfdetr's own torch model, evaluated in eval mode without gradients."""

    def __init__(self, weights: str, variant: str, device: DeviceT = "auto") -> None:
        if not Path(weights).is_file():
            raise ValueError(f"the RF-DETR weights are missing: {weights}")
        import rfdetr
        import torch
        self._torch = torch
        resolved = resolve_auto_device(device)
        if uses_cuda_device(resolved) and not cuda_available():
            resolved = "cpu"
        self.device = resolved
        model = getattr(rfdetr, _VARIANT_CLASSES[variant])(pretrain_weights=str(weights))
        # rfdetr builds on its own default device; the module is moved to the resolved one explicitly.
        self._target = torch.device(_torch_device(resolved))
        self._module = model.model.model.to(self._target)
        self._module.eval()

    def infer(self, blob: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self._module is None:
            raise RuntimeError("the RF-DETR runtime is closed")
        with self._torch.inference_mode(), self._torch.autocast(
                self._target.type, dtype=self._torch.float16, enabled=self._target.type == "cuda"):
            out = self._module(self._torch.from_numpy(np.ascontiguousarray(blob)).to(self._target))
        if isinstance(out, tuple):
            boxes, logits = out[0], out[1]
        else:
            boxes, logits = out["pred_boxes"], out["pred_logits"]
        return boxes.float().cpu().numpy(), logits.float().cpu().numpy()

    def close(self) -> None:
        self._module = None
        free_cuda_cache(self.device)


def _torch_device(device: DeviceT) -> str:
    """A CUDA index (an int or a numeric string, as navpy's device settings allow) -> ``cuda:N``."""
    if isinstance(device, int) or (isinstance(device, str) and device.isdigit()):
        return f"cuda:{int(device)}"
    return str(device)


__all__ = ["RfDetrRuntime", "TorchRfDetrRuntime"]
