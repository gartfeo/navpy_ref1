// Pure helpers for operator edits of the inclusion-fence polygon.
//
// The fence is auto-derived (fenceInclusion) until the operator's first edit
// gesture; from then on the mission holds `fenceCustomVertices` and the
// auto-derivation stops (reset via "Auto" in the sidebar). Every helper takes
// `custom` (the operator's ring or null) plus `auto` (the derived ring) and
// returns the next custom ring — seeding from `auto` on the first edit.

/** Minimum vertices for a valid inclusion polygon (matches the backend). */
export const MIN_FENCE_RING = 3;

function _seed(custom, auto) {
  const base = custom ?? auto;
  return base ? base.map((p) => ({ lat: p.lat, lon: p.lon })) : null;
}

/** Move vertex `index` to `latlon`. Returns the next custom ring (or null). */
export function fenceVertexDrag(custom, auto, index, latlon) {
  const ring = _seed(custom, auto);
  if (!ring || index < 0 || index >= ring.length || !latlon) return custom ?? null;
  ring[index] = { lat: latlon.lat, lon: latlon.lon };
  return ring;
}

/** Insert a vertex after `afterIndex` (midpoint handle). */
export function fenceMidpointInsert(custom, auto, afterIndex, latlon) {
  const ring = _seed(custom, auto);
  if (!ring || afterIndex < 0 || afterIndex >= ring.length || !latlon) return custom ?? null;
  ring.splice(afterIndex + 1, 0, { lat: latlon.lat, lon: latlon.lon });
  return ring;
}

/** Delete vertex `index`, keeping at least MIN_FENCE_RING vertices. */
export function fenceVertexDelete(custom, auto, index) {
  const ring = _seed(custom, auto);
  if (!ring || ring.length <= MIN_FENCE_RING || index < 0 || index >= ring.length) {
    return custom ?? null;
  }
  ring.splice(index, 1);
  return ring;
}
