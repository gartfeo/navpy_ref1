/**
 * Max positions stored per trail. When exceeded, the older part of the
 * trail is thinned instead of dropped, so the whole flight stays visible.
 */
export const MAX_TRAIL_LENGTH = 2000;

/** Newest positions kept at full resolution when the trail is thinned. */
export const RECENT_FULL_RES = 200;

/**
 * Minimum squared-degree distance to add a new trail point.
 * ~2 m at equator: (2 / 111320)^2 ≈ 3.2e-10
 */
export const MIN_TRAIL_STEP_DEG2 = 3.2e-10;

/** Squared distance between two lon/lat points (degrees). */
export function distDeg2(a, b) {
  const dx = a.lon - b.lon;
  const dy = a.lat - b.lat;
  return dx * dx + dy * dy;
}

/**
 * Accumulate a position into a trail array, respecting distance and length limits.
 * Returns the (mutated) trail array.
 */
export function accumulate(trail, pt) {
  if (trail.length === 0) {
    trail.push(pt);
    return trail;
  }
  if (distDeg2(trail[trail.length - 1], pt) < MIN_TRAIL_STEP_DEG2) {
    return trail;
  }
  trail.push(pt);
  if (trail.length > MAX_TRAIL_LENGTH) {
    thin(trail);
  }
  return trail;
}

/** Metres per degree of latitude. */
const M_PER_DEG = 111320;

/** Project lon/lat/alt points to local metres around the first point. */
function toLocalMetres(pts) {
  const lat0 = pts[0].lat;
  const lon0 = pts[0].lon;
  const kx = Math.cos((lat0 * Math.PI) / 180) * M_PER_DEG;
  return pts.map((p) => [(p.lon - lon0) * kx, (p.lat - lat0) * M_PER_DEG, p.alt ?? 0]);
}

/**
 * Distance (m) from b to segment a-c in 3D. Zero for points on a straight,
 * level leg; positive for turns, climbs/descents, and the tip of an
 * out-and-back reversal (which lies beyond the segment ends).
 */
function segDist(a, b, c) {
  const ux = c[0] - a[0], uy = c[1] - a[1], uz = c[2] - a[2];
  const vx = b[0] - a[0], vy = b[1] - a[1], vz = b[2] - a[2];
  const len2 = ux * ux + uy * uy + uz * uz;
  let t = len2 > 0 ? (vx * ux + vy * uy + vz * uz) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const dx = vx - t * ux, dy = vy - t * uy, dz = vz - t * uz;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Simplify pts down to `target` points (Visvalingam-Whyatt elimination with
 * the Douglas-Peucker error measure): repeatedly drop the interior point that
 * lies closest, in 3D, to the segment joining its current neighbours.
 * Straight legs collapse to their ends; turns, loops, reversals and
 * climbs/descents survive. Endpoints are always kept. Returns a new array.
 */
export function simplify(pts, target) {
  const n = pts.length;
  if (n <= target || n <= 2) return pts.slice();
  const m = toLocalMetres(pts);
  const prev = new Int32Array(n);
  const next = new Int32Array(n);
  const err = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    prev[i] = i - 1;
    next[i] = i + 1;
  }
  err[0] = Infinity;
  err[n - 1] = Infinity;
  for (let i = 1; i < n - 1; i++) err[i] = segDist(m[i - 1], m[i], m[i + 1]);
  for (let left = n; left > target; left--) {
    let min = 0;
    for (let i = next[0]; i < n - 1; i = next[i]) {
      if (err[i] < err[min]) min = i;
    }
    const p = prev[min];
    const q = next[min];
    next[p] = q;
    prev[q] = p;
    if (p > 0) err[p] = segDist(m[prev[p]], m[p], m[q]);
    if (q < n - 1) err[q] = segDist(m[p], m[q], m[next[q]]);
  }
  const out = [];
  for (let i = 0; i < n; i = next[i]) out.push(pts[i]);
  return out;
}

/**
 * Halve the older part of the trail with shape-preserving simplification,
 * keeping the newest RECENT_FULL_RES points untouched. The recent boundary
 * point is included so the join between the two parts keeps its shape.
 * Mutates and returns trail.
 */
export function thin(trail) {
  const oldEnd = trail.length - RECENT_FULL_RES;
  if (oldEnd <= 2) return trail;
  // Simplify old part plus the first recent point (kept as an endpoint).
  const kept = simplify(trail.slice(0, oldEnd + 1), Math.ceil(oldEnd / 2) + 1);
  kept.pop();
  trail.splice(0, oldEnd, ...kept);
  return trail;
}
