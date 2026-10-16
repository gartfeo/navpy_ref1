"""Small derivative-free optimisers (numpy only; scipy is not a project dependency).

- nelder_mead: Nelder and Mead (1965) simplex, used for low-dimensional model fits.
- cmaes: CMA-ES (Hansen, "The CMA Evolution Strategy: A Tutorial", 2016), used for the
  gain search; it evaluates a whole population per call so the batched controller
  simulation in ap_controller.py runs once per generation.
"""
from __future__ import annotations

from typing import Callable

import numpy as np


def nelder_mead(f: Callable[[np.ndarray], float], x0: np.ndarray, step: np.ndarray, max_iter: int = 400,
                tol: float = 1e-9) -> tuple[np.ndarray, float]:
    """Minimise f(x) for a 1-D array x. Returns (x_best, f_best)."""
    x0 = np.asarray(x0, dtype=float)
    n = len(x0)
    simplex = [x0] + [x0 + np.eye(n)[i] * np.asarray(step, dtype=float)[i] for i in range(n)]
    values = [f(x) for x in simplex]
    for _ in range(max_iter):
        order = np.argsort(values)
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        if abs(values[-1] - values[0]) <= tol * (abs(values[0]) + tol):
            break
        centroid = np.mean(simplex[:-1], axis=0)
        xr = centroid + (centroid - simplex[-1])
        fr = f(xr)
        if fr < values[0]:
            xe = centroid + 2.0 * (centroid - simplex[-1])
            fe = f(xe)
            simplex[-1], values[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < values[-2]:
            simplex[-1], values[-1] = xr, fr
        else:
            xc = centroid + 0.5 * (simplex[-1] - centroid)
            fc = f(xc)
            if fc < values[-1]:
                simplex[-1], values[-1] = xc, fc
            else:
                best = simplex[0]
                simplex = [best] + [best + 0.5 * (x - best) for x in simplex[1:]]
                values = [values[0]] + [f(x) for x in simplex[1:]]
    i = int(np.argmin(values))
    return simplex[i], values[i]


def cmaes(f_batch: Callable[[np.ndarray], np.ndarray], x0: np.ndarray, sigma0: float, max_gen: int = 60,
          popsize: int | None = None, seed: int = 0, lower: np.ndarray | None = None,
          upper: np.ndarray | None = None) -> tuple[np.ndarray, float, list[float]]:
    """Minimise f over a box. f_batch takes an (lambda, n) array and returns (lambda,) costs.

    Returns (x_best, f_best, history) where history holds the best cost per generation.
    """
    rng = np.random.default_rng(seed)
    x0 = np.asarray(x0, dtype=float)
    n = len(x0)
    lam = popsize or 4 + int(3 * np.log(n))
    mu = lam // 2
    w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    w /= w.sum()
    mueff = 1.0 / np.sum(w ** 2)
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n)
    cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff)
    cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0.0, np.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chi_n = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
    mean, sigma = x0.copy(), sigma0
    pc, ps = np.zeros(n), np.zeros(n)
    B, D, C = np.eye(n), np.ones(n), np.eye(n)
    lo = -np.inf * np.ones(n) if lower is None else np.asarray(lower, dtype=float)
    hi = np.inf * np.ones(n) if upper is None else np.asarray(upper, dtype=float)
    best_x, best_f, history = mean.copy(), np.inf, []
    for gen in range(max_gen):
        z = rng.standard_normal((lam, n))
        y = z @ (B * D).T
        x = np.clip(mean + sigma * y, lo, hi)
        y = (x - mean) / sigma                       # keep the update consistent with the clipped samples
        fx = np.asarray(f_batch(x), dtype=float)
        order = np.argsort(fx)
        if fx[order[0]] < best_f:
            best_f, best_x = float(fx[order[0]]), x[order[0]].copy()
        history.append(best_f)
        y_sel = y[order[:mu]]
        y_w = w @ y_sel
        mean = mean + sigma * y_w
        c_inv_sqrt = B @ np.diag(1 / D) @ B.T
        ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * (c_inv_sqrt @ y_w)
        hsig = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) / chi_n < 1.4 + 2 / (n + 1)
        pc = (1 - cc) * pc + hsig * np.sqrt(cc * (2 - cc) * mueff) * y_w
        C = ((1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * C)
             + cmu * (y_sel.T * w) @ y_sel)
        sigma *= np.exp((cs / damps) * (np.linalg.norm(ps) / chi_n - 1))
        C = np.triu(C) + np.triu(C, 1).T
        evals, B = np.linalg.eigh(C)
        D = np.sqrt(np.maximum(evals, 1e-20))
        if sigma * D.max() < 1e-6:
            break
    return best_x, best_f, history
