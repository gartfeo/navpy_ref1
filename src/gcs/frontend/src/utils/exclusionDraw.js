import { closestEdgeIndex } from './geo';

// Two clicks of a browser double-click land on the same pixel, so they resolve
// to the same lat/lon. Drop a vertex that coincides with an existing anchor
// (well below any deliberate vertex spacing) so double-click doesn't leave a
// zero-length spur on the ring. ~0.1 m in degrees.
export const COINCIDENT_EPS_DEG = 1e-6;

function _coincident(a, b) {
  return !!a && !!b
    && Math.abs(a.lat - b.lat) < COINCIDENT_EPS_DEG
    && Math.abs(a.lon - b.lon) < COINCIDENT_EPS_DEG;
}

/**
 * Keep-out drawing click reducer — same logic as the search-zone drawing:
 * the ring CLOSES as soon as it has 3 vertices, and every later click inserts
 * into the nearest edge of the (already-closed) active ring.
 *
 * State shape: {
 *   draft:      [{lat,lon}, ...]  in-progress open points (length 0..2),
 *   rings:      [[{lat,lon},...], ...]  committed keep-out rings,
 *   activeRing: number|null      index into rings still being shaped by clicks,
 * }
 *
 * Transitions on click:
 * - activeRing != null → insert the click into the nearest edge of that ring.
 * - draft has 0-1 pts  → append to draft.
 * - draft has 2 pts    → third click closes the ring: commit [d0, d1, click]
 *                        to rings and make it the active ring (draft empties).
 * Coincident clicks (double-click's second hit) are ignored everywhere.
 *
 * The result additionally carries `inserted: {ring, index} | null` — the
 * active-ring vertex this click created, so the caller can REVOKE it when the
 * click turns out to be the first half of a finishing double-click (Cesium
 * fires both LEFT_CLICKs before LEFT_DOUBLE_CLICK; without revocation every
 * "double-click to stop shaping" would deform the committed ring).
 *
 * Pure — returns a new state; never mutates the input.
 */
export function nextExclusionClickState(state, latlon) {
  const { draft = [], rings = [], activeRing = null } = state || {};
  if (!latlon) return { draft, rings, activeRing, inserted: null };

  if (activeRing != null && rings[activeRing]) {
    const ring = rings[activeRing];
    if (ring.some((v) => _coincident(v, latlon))) {
      return { draft, rings, activeRing, inserted: null };
    }
    const insertAt = closestEdgeIndex(ring, latlon) + 1;
    const nextRing = [...ring.slice(0, insertAt), { lat: latlon.lat, lon: latlon.lon }, ...ring.slice(insertAt)];
    const nextRings = rings.map((r, i) => (i === activeRing ? nextRing : r));
    return { draft, rings: nextRings, activeRing, inserted: { ring: activeRing, index: insertAt } };
  }

  if (draft.some((v) => _coincident(v, latlon))) {
    return { draft, rings, activeRing, inserted: null };
  }

  if (draft.length < 2) {
    return { draft: [...draft, { lat: latlon.lat, lon: latlon.lon }], rings, activeRing, inserted: null };
  }

  // Third point — the ring closes NOW (same as the zone polygon) and becomes
  // the active ring so further clicks reshape it via nearest-edge insertion.
  // Not reported as `inserted`: the closing vertex of a final double-click is
  // the intended corner (matching the old draft-finish behavior).
  const ring = [...draft, { lat: latlon.lat, lon: latlon.lon }];
  return { draft: [], rings: [...rings, ring], activeRing: rings.length, inserted: null };
}

/**
 * Remove a just-inserted vertex (revocation of a double-click's first click).
 * Validates ring/index/position so a concurrent ring removal can't delete the
 * wrong vertex; returns the input rings unchanged when anything mismatches or
 * the removal would break the >= 3 minimum. Pure.
 */
export function revokeInsertedVertex(rings, inserted) {
  if (!inserted) return rings;
  const ring = rings?.[inserted.ring];
  if (!ring || ring.length <= 3) return rings;
  const v = ring[inserted.index];
  if (!v || !_coincident(v, inserted.latlon)) return rings;
  return rings.map((r, i) => (
    i === inserted.ring ? [...r.slice(0, inserted.index), ...r.slice(inserted.index + 1)] : r
  ));
}

/**
 * Adjust the active-ring index after deleting ring `removedIndex`.
 * Deleting the active ring ends editing; deleting an earlier ring shifts it.
 */
export function activeRingAfterRemove(activeRing, removedIndex) {
  if (activeRing == null) return null;
  if (removedIndex === activeRing) return null;
  return removedIndex < activeRing ? activeRing - 1 : activeRing;
}
