// ---------------------------------------------------------------------------
// Rectangle search-area drawing: three clicks make a rectangle at any heading.
// Click 1 and 2 are the ends of one side (that side sets the heading); click 3
// sets the width, as its perpendicular distance from that side. The result is
// a plain 4-corner polygon, the same shape the free polygon draw produces.
// ---------------------------------------------------------------------------

import { METERS_PER_DEG, offsetLatLon } from './geo';

/**
 * Shortest side length (m) a drawn rectangle may have. Shorter means the two
 * clicks landed on (nearly) one spot, e.g. the two clicks of a double-click,
 * and the click is ignored rather than making a degenerate zone.
 */
export const MIN_RECT_SIDE_M = 1;

// North/east metres of b relative to origin a (flat earth at a's latitude).
function toLocal(a, b) {
  return {
    n: (b.lat - a.lat) * METERS_PER_DEG,
    e: (b.lon - a.lon) * METERS_PER_DEG * Math.cos(a.lat * Math.PI / 180),
  };
}

/** Length (m) of the first side a-b. */
export function sideLength(a, b) {
  const { n, e } = toLocal(a, b);
  return Math.hypot(n, e);
}

/**
 * Four corners [a, b, b + w, a + w] of the rectangle with side a-b whose
 * opposite side passes through c (c's signed perpendicular distance from a-b
 * is the width). Returns null when either side is shorter than MIN_RECT_SIDE_M.
 */
export function rectangleCorners(a, b, c) {
  const ab = toLocal(a, b);
  const len = Math.hypot(ab.n, ab.e);
  if (len < MIN_RECT_SIDE_M) return null;
  // Unit normal to a-b (a-b rotated by +90 deg in the north/east plane).
  const un = -ab.e / len;
  const ue = ab.n / len;
  const ac = toLocal(a, c);
  const width = ac.n * un + ac.e * ue;
  if (Math.abs(width) < MIN_RECT_SIDE_M) return null;
  const dn = width * un;
  const de = width * ue;
  return [
    { lat: a.lat, lon: a.lon },
    { lat: b.lat, lon: b.lon },
    offsetLatLon(b.lat, b.lon, dn, de),
    offsetLatLon(a.lat, a.lon, dn, de),
  ];
}

/**
 * Preview ring for the pointer at `cursor` while a rectangle is drafted from
 * `draft` (the clicks so far): after one click the side to the pointer, after
 * two the full rectangle. [] when there is nothing to preview.
 */
export function rectanglePreview(draft, cursor) {
  if (!cursor || !draft) return [];
  if (draft.length === 1) return [draft[0], cursor];
  if (draft.length === 2) return rectangleCorners(draft[0], draft[1], cursor) || [draft[0], draft[1]];
  return [];
}
