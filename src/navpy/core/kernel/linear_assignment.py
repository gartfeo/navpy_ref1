"""Dependency-light linear assignment algorithms shared across domains."""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


def hungarian(cost: np.ndarray) -> List[Tuple[int, int]]:
    """
    Solve the linear assignment problem using the Hungarian algorithm.

    Args:
        cost: NxM cost matrix where cost[i,j] is cost of assigning row i to column j

    Returns:
        List of (row, col) assignment pairs that minimize total cost
    """
    cost = cost.copy()
    n, m = cost.shape
    size = max(n, m)
    pad_val = float(cost.max() + 1.0) if cost.size else 1.0
    pad = np.full((size, size), pad_val, dtype=np.float32)
    pad[:n, :m] = cost
    cost = pad

    cost -= cost.min(axis=1, keepdims=True)
    cost -= cost.min(axis=0, keepdims=True)

    mask = np.zeros((size, size), dtype=np.int8)  # 1=star,2=prime
    row_cov = np.zeros(size, dtype=bool)
    col_cov = np.zeros(size, dtype=bool)

    for r in range(size):
        for c in range(size):
            if cost[r, c] == 0 and not row_cov[r] and not col_cov[c]:
                mask[r, c] = 1
                row_cov[r] = True
                col_cov[c] = True

    row_cov[:] = False
    col_cov[:] = False

    def cover_cols_with_stars() -> None:
        col_cov[:] = False
        for c in range(size):
            if np.any(mask[:, c] == 1):
                col_cov[c] = True

    def find_uncovered_zero() -> tuple[int, int] | None:
        for r in range(size):
            if row_cov[r]:
                continue
            for c in range(size):
                if col_cov[c]:
                    continue
                if cost[r, c] == 0:
                    return r, c
        return None

    def find_star_in_row(r: int) -> int | None:
        cs = np.where(mask[r] == 1)[0]
        return int(cs[0]) if cs.size else None

    def find_star_in_col(c: int) -> int | None:
        rs = np.where(mask[:, c] == 1)[0]
        return int(rs[0]) if rs.size else None

    def find_prime_in_row(r: int) -> int | None:
        cs = np.where(mask[r] == 2)[0]
        return int(cs[0]) if cs.size else None

    def augment(path: list[tuple[int, int]]) -> None:
        for rr, cc in path:
            if mask[rr, cc] == 1:
                mask[rr, cc] = 0
            elif mask[rr, cc] == 2:
                mask[rr, cc] = 1

    cover_cols_with_stars()

    while col_cov.sum() < size:
        z = find_uncovered_zero()
        while z is None:
            uncovered_rows = ~row_cov
            uncovered_cols = ~col_cov
            minval = np.min(cost[uncovered_rows][:, uncovered_cols])
            cost[uncovered_rows, :] -= minval
            cost[:, col_cov] += minval
            z = find_uncovered_zero()

        r, c = z
        mask[r, c] = 2

        star_c = find_star_in_row(r)
        if star_c is None:
            path = [(r, c)]
            cur_c = c
            while True:
                star_r = find_star_in_col(cur_c)
                if star_r is None:
                    break
                path.append((star_r, cur_c))
                prime_c = find_prime_in_row(star_r)
                path.append((star_r, prime_c))
                cur_c = prime_c

            augment(path)
            mask[mask == 2] = 0
            row_cov[:] = False
            col_cov[:] = False
            cover_cols_with_stars()
        else:
            row_cov[r] = True
            col_cov[star_c] = False

    assignments = []
    for r in range(n):
        cs = np.where(mask[r] == 1)[0]
        if cs.size:
            c = int(cs[0])
            if c < m:
                assignments.append((r, c))
    return assignments


__all__ = ["hungarian"]
