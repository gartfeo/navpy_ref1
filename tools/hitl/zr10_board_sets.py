"""Rig board-calibration sessions (session.json + one saved candidate fit): load, refit, score a lens."""
from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

FIT_FLAGS = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3


def board(pattern: Sequence[int]) -> np.ndarray:
    cols, rows = pattern
    return np.array([[x, y, 0.0] for y in range(rows) for x in range(cols)], np.float32)


def load_session(folder: Path) -> tuple[dict, list[np.ndarray], list[np.ndarray]]:
    s = json.loads((folder / "session.json").read_text(encoding="utf-8"))
    imgs = [np.asarray(v["corners"], np.float32).reshape(-1, 1, 2) for v in s["views"]]
    return s, [board(s["pattern"]) for _ in imgs], imgs


def fit(objs: list, imgs: list, size: tuple[int, int], fx0: float) -> dict[str, float]:
    k0 = np.array([[fx0, 0, size[0] / 2], [0, fx0, size[1] / 2], [0, 0, 1]], float)
    _, k, d, _, _ = cv2.calibrateCamera(objs, imgs, size, k0, None, flags=FIT_FLAGS | cv2.CALIB_USE_INTRINSIC_GUESS)
    d = d.ravel()
    return {"fx": k[0, 0], "fy": k[1, 1], "cx": k[0, 2], "cy": k[1, 2], "k1": d[0], "k2": d[1]}


def candidate(folder: Path) -> dict:
    files = sorted(folder.glob("candidate-*.json"))
    if len(files) != 1:
        raise ValueError(f"{folder.name}: expected one saved fit, found {len(files)}")
    return json.loads(files[0].read_text(encoding="utf-8"))


def score(row: Mapping[str, Any], objs: list, imgs: list) -> dict[str, float]:
    """Held-out board projection: each view's pose solved with the row's lens fixed, per-corner error in pixels."""
    k = np.array([[row["fx"], 0, row["cx"]], [0, row["fy"], row["cy"]], [0, 0, 1]], float)
    d = np.asarray(row["dist"], float)
    errors = []
    for o, m in zip(objs, imgs):
        _, rv, tv = cv2.solvePnP(o, m, k, d)
        p, _ = cv2.projectPoints(o, rv, tv, k, d)
        errors.append(np.linalg.norm(p.reshape(-1, 2) - m.reshape(-1, 2), axis=1))
    e = np.concatenate(errors)
    return {"p95_px": round(float(np.percentile(e, 95)), 3), "max_px": round(float(e.max()), 3)}
