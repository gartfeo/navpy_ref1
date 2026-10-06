// ---------------------------------------------------------------------------
// Flat-earth geo utilities
// ---------------------------------------------------------------------------

export const METERS_PER_DEG = 111320;

/** Flat-earth distance in meters between two {lat, lon} points. */
export function flatDist(a, b) {
  const dlat = (b.lat - a.lat) * METERS_PER_DEG;
  const dlon = (b.lon - a.lon) * METERS_PER_DEG * Math.cos(a.lat * Math.PI / 180);
  return Math.sqrt(dlat * dlat + dlon * dlon);
}

/** Flat-earth squared distance (avoids sqrt for comparisons). */
export function flatDistSq(a, b) {
  const dlat = (b.lat - a.lat) * METERS_PER_DEG;
  const dlon = (b.lon - a.lon) * METERS_PER_DEG * Math.cos(a.lat * Math.PI / 180);
  return dlat * dlat + dlon * dlon;
}

/** Offset a lat/lon by north/east meters. Returns {lat, lon}. */
export function offsetLatLon(lat, lon, dn, de) {
  return {
    lat: lat + dn / METERS_PER_DEG,
    lon: lon + de / (METERS_PER_DEG * Math.cos(lat * Math.PI / 180)),
  };
}

/** Project point onto segment [a, b]. Returns { lat, lon, t, distSq }. */
export function projectPointOnSegment(pt, a, b) {
  const dx = b.lat - a.lat, dy = b.lon - a.lon;
  const lenSq = dx * dx + dy * dy;
  let t = 0;
  if (lenSq > 0) {
    t = Math.max(0, Math.min(1, ((pt.lat - a.lat) * dx + (pt.lon - a.lon) * dy) / lenSq));
  }
  const lat = a.lat + t * dx, lon = a.lon + t * dy;
  const distSq = (pt.lat - lat) ** 2 + (pt.lon - lon) ** 2;
  return { lat, lon, t, distSq };
}

// ---------------------------------------------------------------------------
// Convex hull
// ---------------------------------------------------------------------------

/**
 * Compute convex hull of points using Graham scan.
 * @param {Array<{lat: number, lon: number}>} points
 * @returns {Array<{lat: number, lon: number}>} hull vertices in CCW order
 */
export function convexHull(points) {
  if (points.length < 3) return [...points];

  // Deduplicate
  const seen = new Set();
  const pts = points.filter((p) => {
    const key = `${p.lat},${p.lon}`;
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });
  if (pts.length < 3) return [...pts];

  // Find bottom-most (min lat), leftmost as tiebreaker
  let pivot = 0;
  for (let i = 1; i < pts.length; i++) {
    if (pts[i].lat < pts[pivot].lat ||
        (pts[i].lat === pts[pivot].lat && pts[i].lon < pts[pivot].lon)) {
      pivot = i;
    }
  }
  [pts[0], pts[pivot]] = [pts[pivot], pts[0]];
  const p0 = pts[0];

  // Cross product of vectors (o→a) and (o→b)
  const cross = (o, a, b) =>
    (a.lon - o.lon) * (b.lat - o.lat) - (a.lat - o.lat) * (b.lon - o.lon);

  const distSq = (a, b) => (a.lat - b.lat) ** 2 + (a.lon - b.lon) ** 2;

  // Sort by polar angle from pivot
  pts.sort((a, b) => {
    if (a === p0) return -1;
    if (b === p0) return 1;
    const c = cross(p0, a, b);
    if (c !== 0) return -c; // CCW → negative cross means a comes first
    return distSq(p0, a) - distSq(p0, b);
  });

  const stack = [pts[0], pts[1]];
  for (let i = 2; i < pts.length; i++) {
    while (stack.length > 1 && cross(stack[stack.length - 2], stack[stack.length - 1], pts[i]) <= 0) {
      stack.pop();
    }
    stack.push(pts[i]);
  }
  return stack;
}

/**
 * Index i of the closed-ring edge (i, i+1) closest to the given point.
 * Shared by the zone drawing hook and keep-out drawing so a click on a
 * finished ring inserts the new vertex into the nearest edge.
 * @param {Array<{lat:number, lon:number}>} verts - ring vertices (>= 2)
 * @param {{lat:number, lon:number}} pt
 * @returns {number} edge start index
 */
export function closestEdgeIndex(verts, pt) {
  let bestDist = Infinity;
  let bestIdx = 0;
  for (let i = 0; i < verts.length; i++) {
    const j = (i + 1) % verts.length;
    const d = projectPointOnSegment(pt, verts[i], verts[j]).distSq;
    if (d < bestDist) {
      bestDist = d;
      bestIdx = i;
    }
  }
  return bestIdx;
}

/**
 * Find the nearest point on a polygon's boundary to a given point.
 * Projects the point onto each edge segment and returns the closest result.
 * @param {{lat: number, lon: number}} pt - Query point
 * @param {Array<{lat: number, lon: number}>} polygon - Closed polygon vertices
 * @returns {{lat: number, lon: number}} Nearest point on the polygon edge
 */
export function nearestPointOnPolygonEdge(pt, polygon) {
  if (!polygon || polygon.length < 2) return { lat: pt.lat, lon: pt.lon };

  let bestDist = Infinity;
  let bestPt = { lat: polygon[0].lat, lon: polygon[0].lon };

  for (let i = 0; i < polygon.length; i++) {
    const proj = projectPointOnSegment(pt, polygon[i], polygon[(i + 1) % polygon.length]);
    if (proj.distSq < bestDist) {
      bestDist = proj.distSq;
      bestPt = { lat: proj.lat, lon: proj.lon };
    }
  }
  return bestPt;
}

// ---------------------------------------------------------------------------
// Polygon outward offset (geofence)
// ---------------------------------------------------------------------------

// Default outward margin of the inclusion fence hull, meters. Kept LARGE on
// purpose: a fixed-wing loiters (RTL / mission-end / waiting) on a circle of
// radius WP_LOITER_RAD — ~627 m in the SITL/demo config — and climbs out a few
// hundred metres before turning back, all while the fence auto-enables at
// takeoff altitude. The margin must clear the loiter radius or the fence trips
// on any orbit/RTL. Validated in SITL: with a 500 m margin a UAV breached at
// takeoff; at 900 m all 3 UAVs stayed inside (max observed excursion 702 m > the
// 627 m loiter, well under the margin). Room to fly comes from THIS margin; size
// it to the aircraft's WP_LOITER_RAD via the fence slider.
export const DEFAULT_FENCE_OFFSET_M = 900;

// Default takeoff-round radius: a small ring at the launch for immediate
// lift-off dispersion only. Climb-out / loiter coverage is the fence margin.
export const DEFAULT_TAKEOFF_ROUND_M = 40;

// Confirmation-orbit / loiter radius used for fence coverage + conflict checks.
// Matches the aircraft's loiter (WP_LOITER_RAD ~627 m here); the real
// per-mission R_nav_min is navigation-side. Conservative so orbit-vs-keepout
// conflicts aren't under-reported.
export const DEFAULT_ORBIT_RADIUS_M = 600;

// A sharp corner's bisector scale (1/cos(half-angle)) blows up as the interior
// angle approaches 0; clamp cos so a spike vertex can't fling the fence point
// arbitrarily far.
const _MIN_BISECTOR_COS = 0.25;

// --- internal flat-earth-meters helpers (shared by expandPolygon + fence merge) ---

/** Signed area×2 (shoelace) of an {x,y} ring. CCW → positive. */
function _signedArea2XY(pts) {
  let a2 = 0;
  const n = pts.length;
  for (let i = 0; i < n; i++) {
    const a = pts[i], b = pts[(i + 1) % n];
    a2 += a.x * b.y - b.x * a.y;
  }
  return a2;
}

/**
 * Offset an {x,y} ring outward by ``offsetMeters`` using a per-vertex
 * angle-bisector offset. Winding-agnostic (uses the signed area so the outward
 * normal points away from the interior regardless of vertex order). Returns new
 * {x,y} points. A sharp spike's bisector scale is clamped by _MIN_BISECTOR_COS.
 */
function _offsetRingXY(pts, offsetMeters) {
  const n = pts.length;
  if (n < 3 || !offsetMeters) return pts.map((p) => ({ x: p.x, y: p.y }));
  const sign = _signedArea2XY(pts) > 0 ? 1 : -1;
  const edgeNormal = (a, b) => {
    let dx = b.x - a.x, dy = b.y - a.y;
    const len = Math.hypot(dx, dy) || 1;
    dx /= len; dy /= len;
    return { nx: sign * dy, ny: -sign * dx };
  };
  const out = [];
  for (let i = 0; i < n; i++) {
    const prev = pts[(i - 1 + n) % n];
    const cur = pts[i];
    const next = pts[(i + 1) % n];
    const n1 = edgeNormal(prev, cur);
    const n2 = edgeNormal(cur, next);
    let bx = n1.nx + n2.nx, by = n1.ny + n2.ny;
    const blen = Math.hypot(bx, by);
    if (blen < 1e-9) {
      // Nearly-straight-back spike — fall back to the outgoing edge normal.
      bx = n2.nx; by = n2.ny;
    } else {
      bx /= blen; by /= blen;
    }
    let cosHalf = bx * n1.nx + by * n1.ny;
    if (cosHalf < _MIN_BISECTOR_COS) cosHalf = _MIN_BISECTOR_COS;
    const d = offsetMeters / cosHalf;
    out.push({ x: cur.x + bx * d, y: cur.y + by * d });
  }
  return out;
}

/**
 * Expand a polygon outward by a perpendicular distance, producing a larger
 * polygon whose every edge is offset ``offsetMeters`` outward from the source
 * edge (angle-bisector offset). Used to derive the inclusion geofence from the
 * search polygon. Winding-agnostic (handles CW and CCW). Returns lat/lon copies.
 *
 * @param {Array<{lat:number, lon:number}>} polygon
 * @param {number} offsetMeters - outward distance in meters (>0)
 * @returns {Array<{lat:number, lon:number}>}
 */
export function expandPolygon(polygon, offsetMeters) {
  const n = polygon?.length ?? 0;
  if (n < 3 || !offsetMeters) {
    return polygon ? polygon.map((p) => ({ lat: p.lat, lon: p.lon })) : [];
  }
  // Local flat-earth meters frame (x = east, y = north) about the mean latitude.
  const latRef = polygon.reduce((s, p) => s + p.lat, 0) / n;
  const cosLat = Math.cos(latRef * Math.PI / 180) || 1;
  const pts = polygon.map((p) => ({
    x: p.lon * METERS_PER_DEG * cosLat,
    y: p.lat * METERS_PER_DEG,
  }));
  return _offsetRingXY(pts, offsetMeters).map((q) => ({
    lat: q.y / METERS_PER_DEG,
    lon: q.x / (METERS_PER_DEG * cosLat),
  }));
}

// ---------------------------------------------------------------------------
// Inclusion geofence (zone + takeoff round + corridor + confirmation orbits)
// ---------------------------------------------------------------------------

/** Even-odd (ray-cast) point-in-ring test on {x,y} points. True = strictly in. */
function _pointInRingXY(pt, poly) {
  let inside = false;
  const n = poly.length;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const a = poly[i], b = poly[j];
    const denom = (b.y - a.y) || 1e-12;
    const crosses = ((a.y > pt.y) !== (b.y > pt.y)) &&
      (pt.x < (b.x - a.x) * (pt.y - a.y) / denom + a.x);
    if (crosses) inside = !inside;
  }
  return inside;
}

function _orient(a, b, c) {
  const v = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return v > 1e-7 ? 1 : (v < -1e-7 ? -1 : 0);
}
function _onSeg(a, b, c) {
  return Math.min(a.x, b.x) - 1e-7 <= c.x && c.x <= Math.max(a.x, b.x) + 1e-7 &&
         Math.min(a.y, b.y) - 1e-7 <= c.y && c.y <= Math.max(a.y, b.y) + 1e-7;
}
/** Segment intersection, counting collinear-overlap and endpoint touches. */
function _segsIntersect(p1, p2, p3, p4) {
  const o1 = _orient(p1, p2, p3), o2 = _orient(p1, p2, p4);
  const o3 = _orient(p3, p4, p1), o4 = _orient(p3, p4, p2);
  if (o1 !== o2 && o3 !== o4) return true;
  if (o1 === 0 && _onSeg(p1, p2, p3)) return true;
  if (o2 === 0 && _onSeg(p1, p2, p4)) return true;
  if (o3 === 0 && _onSeg(p3, p4, p1)) return true;
  if (o4 === 0 && _onSeg(p3, p4, p2)) return true;
  return false;
}
/** Nearest point on segment [a,b] to pt, in {x,y}. */
function _nearestOnSegmentXY(pt, a, b) {
  const dx = b.x - a.x, dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  let t = len2 > 0 ? ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  return { x: a.x + t * dx, y: a.y + t * dy };
}

/**
 * N points evenly spaced around a circle of ``radiusMeters`` about ``center``.
 * Used to fold the takeoff round and confirmation orbit into the fence hull.
 */
export function circlePointsLL(center, radiusMeters, n = 16) {
  if (!center || !(radiusMeters > 0)) return [];
  const cosLat = Math.cos(center.lat * Math.PI / 180) || 1;
  const out = [];
  for (let i = 0; i < n; i++) {
    const a = (2 * Math.PI * i) / n;
    out.push({
      lat: center.lat + (radiusMeters * Math.cos(a)) / METERS_PER_DEG,
      lon: center.lon + (radiusMeters * Math.sin(a)) / (METERS_PER_DEG * cosLat),
    });
  }
  return out;
}

// --- corridor tube grafting (spur merge) -----------------------------------
// Validated in the original merge implementation, resurrected for the corridor
// tube: the corridor must contribute a corridor-SHAPED arm to the fence, not
// inflate the whole convex hull toward a distant launch point.

function _sub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
function _unit(v) { const L = Math.hypot(v.x, v.y) || 1; return { x: v.x / L, y: v.y / L }; }
function _leftNormal(d) { return { x: -d.y, y: d.x }; } // 90° CCW

/**
 * Left/right offset rails of an open polyline, each vertex pushed ±``half``
 * along the local (miter) normal. Travelling A→end, ``left`` is +90° of travel.
 */
function _corridorSides(C, half) {
  const n = C.length;
  const left = [], right = [];
  for (let i = 0; i < n; i++) {
    let nx = 0, ny = 0;
    if (i > 0) { const ln = _leftNormal(_unit(_sub(C[i], C[i - 1]))); nx += ln.x; ny += ln.y; }
    if (i < n - 1) { const ln = _leftNormal(_unit(_sub(C[i + 1], C[i]))); nx += ln.x; ny += ln.y; }
    const L = Math.hypot(nx, ny) || 1;
    nx /= L; ny /= L;
    left.push({ x: C[i].x + nx * half, y: C[i].y + ny * half });
    right.push({ x: C[i].x - nx * half, y: C[i].y - ny * half });
  }
  return { left, right };
}

/**
 * Graft one takeoff corridor into the fence ring as a thin out-and-back "spur"
 * at the ring edge nearest the corridor's zone end: down one rail from the
 * ring to the launch, a cap across the launch, back up the other rail. The
 * outward offset pass then widens it into a full tube (half-width = margin)
 * and pushes the cap past the launch. ``flip`` swaps which rail is the down
 * side; the correct choice depends on the ring winding, so the caller tries
 * both and keeps the one that validates.
 */
function _spliceCorridorXY(ring, isBase, pathXY, half, flip) {
  const nearEnd = pathXY[pathXY.length - 1];
  const n = ring.length;
  let best = null;
  for (let i = 0; i < n; i++) {
    const k = (i + 1) % n;
    if (!isBase[i] || !isBase[k]) continue; // only attach to original base edges
    const np = _nearestOnSegmentXY(nearEnd, ring[i], ring[k]);
    const d2 = (np.x - nearEnd.x) ** 2 + (np.y - nearEnd.y) ** 2;
    if (!best || d2 < best.d2) best = { i, np, d2 };
  }
  if (!best) return null;
  // Centerline from the ring boundary outward to the launch point.
  const C = [{ x: best.np.x, y: best.np.y }, ...pathXY.slice().reverse()];
  if (C.length < 2) return null;
  const { left, right } = _corridorSides(C, half);
  const down = flip ? left : right;
  const up = flip ? right : left;
  const inserted = down.concat(up.slice().reverse());
  const j = best.i;
  return {
    ring: ring.slice(0, j + 1).concat(inserted, ring.slice(j + 1)),
    isBase: isBase.slice(0, j + 1).concat(inserted.map(() => false), isBase.slice(j + 1)),
  };
}

/**
 * Inclusion geofence covering everything the UAVs actually touch:
 *
 * - the search zone,
 * - each ASSIGNED POI's confirmation-orbit circle (orbit radius; the
 *   outward margin then adds on top → circle of orbit + margin),
 * - each takeoff corridor as a corridor-shaped TUBE arm (half-width = margin,
 *   launch cap covering the takeoff round) — grafted into the ring rather
 *   than dumped into the hull, so a distant launch doesn't balloon the fence
 *   into the huge hull triangle between launch and zone.
 *
 * Base area = convexHull(zone ∪ POI orbit circles), corridors grafted as
 * spurs, then the whole ring offset outward by ``marginM`` (widening each spur
 * to a full tube). The result is accepted only when it is a simple polygon
 * that passes the segment-level ``fenceCoversPlan`` check; otherwise it falls
 * back to the convex hull of everything (always simple, always containing) —
 * the vehicle can never be handed a fence that leaves its flight path outside.
 * Still ONE polygon (ArduPilot AND's multiple inclusion polygons — see
 * docs/decisions/geofence-takeoff-corridor).
 *
 * @param {Array<{lat,lon}>} zone - search polygon
 * @param {Array<Array<{lat,lon}>>} transitPaths - one [launch, ...corridor] per set
 * @param {Array<{lat,lon}>} pois - ASSIGNED fallback locations + simulated POIs
 * @param {{marginM?:number, takeoffRadiusM?:number, orbitRadiusM?:number}} opts
 * @returns {Array<{lat,lon}>}
 */
export function fenceInclusion(zone, transitPaths, pois, opts = {}) {
  const marginM = opts.marginM ?? DEFAULT_FENCE_OFFSET_M;
  const takeoffR = opts.takeoffRadiusM ?? 0;
  const orbitR = opts.orbitRadiusM ?? 0;

  // Area contributors: the zone plus each POI's confirmation orbit.
  const basePts = [];
  for (const p of (zone || [])) basePts.push({ lat: p.lat, lon: p.lon });
  for (const t of (pois || [])) {
    if (!t) continue;
    basePts.push({ lat: t.lat, lon: t.lon });
    if (orbitR > 0) basePts.push(...circlePointsLL(t, orbitR));
  }
  const paths = (transitPaths || [])
    .map((p) => (p || []).filter(Boolean))
    .filter((p) => p.length > 0);

  // Always-safe fallback: convex hull of everything (corridor points and
  // takeoff rounds included), expanded — the pre-tube behavior.
  const hullFallback = () => {
    const pts = [...basePts];
    for (const path of paths) {
      path.forEach((p, i) => {
        pts.push({ lat: p.lat, lon: p.lon });
        if (i === 0 && takeoffR > 0) pts.push(...circlePointsLL(p, takeoffR));
      });
    }
    if (pts.length < 3) return pts.map((p) => ({ lat: p.lat, lon: p.lon }));
    return expandPolygon(convexHull(pts), marginM);
  };

  if (basePts.length < 3 || paths.length === 0 || !marginM) return hullFallback();

  const baseHull = convexHull(basePts);
  if (baseHull.length < 3) return hullFallback();

  // Flat-earth meters frame about the base's mean latitude.
  const latRef = baseHull.reduce((s, p) => s + p.lat, 0) / baseHull.length;
  const cosLat = Math.cos(latRef * Math.PI / 180) || 1;
  const toXY = (p) => ({ x: p.lon * METERS_PER_DEG * cosLat, y: p.lat * METERS_PER_DEG });
  const toLL = (q) => ({ lat: q.y / METERS_PER_DEG, lon: q.x / (METERS_PER_DEG * cosLat) });
  const pathsXY = paths.map((p) => p.map(toXY));

  for (const flip of [false, true]) {
    let ring = baseHull.map(toXY);
    let isBase = ring.map(() => true);
    // Thin spur; the outward offset pass widens it to a tube of half-width
    // marginM (and pushes the launch cap out by marginM).
    const half = Math.max(1, marginM * 0.05);
    let ok = true;
    for (const pathXY of pathsXY) {
      const res = _spliceCorridorXY(ring, isBase, pathXY, half, flip);
      if (!res) { ok = false; break; }
      ring = res.ring; isBase = res.isBase;
    }
    if (!ok) break;
    const fence = _offsetRingXY(ring, marginM).map(toLL);
    if (isSimpleRing(fence)
        && fenceCoversPlan(fence, zone, transitPaths, pois, opts)) {
      return fence;
    }
  }
  return hullFallback();
}

// ---------------------------------------------------------------------------
// Exclusion-conflict analysis (does the flight path enter a keep-out?)
// ---------------------------------------------------------------------------

/** Distance (m) from point to segment [a,b], all in {x,y} meters. */
function _distPointSegXY(pt, a, b) {
  const q = _nearestOnSegmentXY(pt, a, b);
  return Math.hypot(pt.x - q.x, pt.y - q.y);
}

/** Does segment [a,b] touch or enter polygon poly? (all {x,y}) */
function _segIntersectsPolyXY(a, b, poly) {
  if (_pointInRingXY(a, poly) || _pointInRingXY(b, poly)) return true;
  const n = poly.length;
  for (let i = 0; i < n; i++) {
    if (_segsIntersect(a, b, poly[i], poly[(i + 1) % n])) return true;
  }
  return false;
}

/** Does circle(center, r) touch or enter polygon poly? ({x,y}, r meters) */
function _circleIntersectsPolyXY(center, r, poly) {
  if (_pointInRingXY(center, poly)) return true; // circle centre inside keep-out
  const n = poly.length;
  for (let i = 0; i < n; i++) {
    const a = poly[i], b = poly[(i + 1) % n];
    if (Math.hypot(center.x - a.x, center.y - a.y) <= r) return true; // vertex in circle
    if (_distPointSegXY(center, a, b) <= r) return true;              // edge within r
  }
  return false;
}

/**
 * Report where the planned flight enters an exclusion keep-out so the operator
 * can move things or replan. Detect-and-warn only — no auto-rerouting. Checks
 * corridor legs, scan-track legs, and confirmation-orbit circles against each
 * exclusion polygon. Returns a de-duplicated list of
 * ``{kind:'corridor'|'track'|'orbit', label, exclusionIndex}``.
 *
 * @param {{corridorPaths?:Array<Array<{lat,lon}>>, tracks?:Array<Array<{lat,lon}>>,
 *          deliveryPoints?:Array<{lat,lon}>, orbitRadiusM?:number,
 *          exclusions?:Array<Array<{lat,lon}>>}} input
 */
export function analyzeExclusionConflicts(input) {
  const { corridorPaths = [], tracks = [], deliveryPoints = [], orbitRadiusM = 0, exclusions = [] } = input || {};
  const excl = (exclusions || []).filter((e) => e && e.length >= 3);
  if (excl.length === 0) return [];

  const ref = excl[0][0];
  const cosLat = Math.cos(ref.lat * Math.PI / 180) || 1;
  const toXY = (p) => ({ x: p.lon * METERS_PER_DEG * cosLat, y: p.lat * METERS_PER_DEG });
  const exclXY = excl.map((e) => e.map(toXY));

  const conflicts = [];
  const checkLegs = (path, kind, label) => {
    const xy = (path || []).filter(Boolean).map(toXY);
    for (let i = 0; i + 1 < xy.length; i++) {
      exclXY.forEach((poly, ei) => {
        if (_segIntersectsPolyXY(xy[i], xy[i + 1], poly)) {
          conflicts.push({ kind, label, exclusionIndex: ei });
        }
      });
    }
  };
  corridorPaths.forEach((p, i) => checkLegs(p, 'corridor', `set ${i + 1}`));
  tracks.forEach((t, i) => checkLegs(t, 'track', `UAV ${i + 1}`));
  if (orbitRadiusM > 0) {
    (deliveryPoints || []).filter(Boolean).forEach((o, i) => {
      const c = toXY(o);
      exclXY.forEach((poly, ei) => {
        if (_circleIntersectsPolyXY(c, orbitRadiusM, poly)) {
          conflicts.push({ kind: 'orbit', label: `delivery location ${i + 1}`, exclusionIndex: ei });
        }
      });
    });
  }

  const seen = new Set();
  return conflicts.filter((c) => {
    const k = `${c.kind}|${c.label}|${c.exclusionIndex}`;
    if (seen.has(k)) return false;
    seen.add(k);
    return true;
  });
}

/**
 * Even-odd point-in-polygon on {lat,lon} points (lon→x, lat→y). Matches the
 * ArduPilot ``Polygon_outside`` inclusion test. Exposed for fence validation.
 */
export function pointInPolygon(pt, polygon) {
  return _pointInRingXY(
    { x: pt.lon, y: pt.lat },
    (polygon || []).map((p) => ({ x: p.lon, y: p.lat })),
  );
}

/**
 * Is the {lat,lon} ring a simple polygon (no non-adjacent edges crossing)?
 * Segment intersection is affine-invariant, so testing in raw lat/lon is
 * exact. An operator can drag a fence/keep-out vertex across the ring; a
 * self-intersecting ring uploads fine but ArduPilot's even-odd test flips the
 * crossover pockets, so the UI must warn.
 */
export function isSimpleRing(polygon) {
  const poly = (polygon || []).map((p) => ({ x: p.lon, y: p.lat }));
  const n = poly.length;
  if (n < 3) return false;
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      // Skip edges sharing a vertex (adjacent, including the wrap pair).
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      if (_segsIntersect(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n])) return false;
    }
  }
  return true;
}

/** Does open polyline `pts` cross any edge of ring `fenceXY`? ({x,y} space) */
function _polylineCrossesRingXY(pts, fenceXY, closed) {
  const m = fenceXY.length;
  const last = closed ? pts.length : pts.length - 1;
  for (let i = 0; i < last; i++) {
    const a = pts[i], b = pts[(i + 1) % pts.length];
    for (let j = 0; j < m; j++) {
      if (_segsIntersect(a, b, fenceXY[j], fenceXY[(j + 1) % m])) return true;
    }
  }
  return false;
}

/**
 * Does a (possibly operator-edited) inclusion fence still contain everything
 * the UAVs touch? Checks the same coverage set `fenceInclusion` guarantees:
 * zone vertices, each launch's takeoff round, corridor points, and each fallback location's
 * confirmation orbit. Point containment alone is only sufficient for a CONVEX
 * fence; a hand-edited ring can be concave, so every flight SEGMENT is also
 * checked against the fence edges (endpoints inside + no edge crossing ⇒ the
 * whole leg is inside a simple ring). Segments covered: the zone boundary,
 * each corridor leg, and the takeoff/orbit circle chords. Used to warn — not
 * block — when a custom fence would leave part of the flight path outside
 * (breach → RTL hazard).
 *
 * @param {Array<{lat,lon}>} fence - inclusion polygon to validate
 * @param {Array<{lat,lon}>} zone - search polygon
 * @param {Array<Array<{lat,lon}>>} transitPaths - one [launch, ...corridor] per set
 * @param {Array<{lat,lon}>} deliveryPoints
 * @param {{takeoffRadiusM?:number, orbitRadiusM?:number}} opts
 * @returns {boolean} true when the fence covers the whole flight path
 */
export function fenceCoversPlan(fence, zone, transitPaths, deliveryPoints, opts = {}) {
  if (!fence || fence.length < 3) return false;
  const takeoffR = opts.takeoffRadiusM ?? 0;
  const orbitR = opts.orbitRadiusM ?? 0;

  const pts = [];
  const segments = []; // {pts: [{lat,lon},...], closed}
  const zoneRing = (zone || []).filter(Boolean);
  if (zoneRing.length > 0) {
    pts.push(...zoneRing);
    if (zoneRing.length >= 2) segments.push({ pts: zoneRing, closed: zoneRing.length >= 3 });
  }
  for (const path of (transitPaths || [])) {
    const leg = (path || []).filter(Boolean);
    if (leg.length === 0) continue;
    pts.push(...leg);
    if (leg.length >= 2) segments.push({ pts: leg, closed: false });
    if (takeoffR > 0) {
      const ring = circlePointsLL(leg[0], takeoffR);
      pts.push(...ring);
      segments.push({ pts: ring, closed: true });
    }
  }
  for (const o of (deliveryPoints || [])) {
    if (!o) continue;
    pts.push(o);
    if (orbitR > 0) {
      const ring = circlePointsLL(o, orbitR);
      pts.push(...ring);
      segments.push({ pts: ring, closed: true });
    }
  }

  if (!pts.every((p) => pointInPolygon(p, fence))) return false;

  const fenceXY = fence.map((p) => ({ x: p.lon, y: p.lat }));
  for (const seg of segments) {
    const xy = seg.pts.map((p) => ({ x: p.lon, y: p.lat }));
    if (_polylineCrossesRingXY(xy, fenceXY, seg.closed)) return false;
  }
  return true;
}

/**
 * Compute the mission waypoint offset — the number of mission items before
 * the first scanning track NAV_WAYPOINT.
 *
 * Mission layout:
 *   Home(0), Takeoff(1), [corridor NAV_WPs], [metadata DOs], Track0...
 *
 * Non-corridor (distributed/delta): corridor points are prepended as NAV_WPs,
 *   corridor backbone is NOT in metadata.
 * Corridor search pattern: corridor backbone is in metadata, NOT as NAV_WPs.
 *
 * @param {string} searchPattern - "distributed" | "corridor"
 * @param {number} polygonLen - outer planning polygon vertex count
 * @param {number} corridorLen - corridor waypoint count
 * @param {boolean} hasLaunchPoint - whether a launch point exists
 * @returns {number} wpOffset
 */
export function missionWpOffset(searchPattern, polygonLen, corridorLen, hasLaunchPoint) {
  const isCorrSearchPattern = searchPattern === 'corridor';
  const corrNavCount = isCorrSearchPattern ? 0 : corridorLen;
  const corrMetaCount = isCorrSearchPattern ? corridorLen : 0;
  // +1 for mandatory fallback location metadata (META_FALLBACK_DELIVERY_LOCATION)
  const metaCount = polygonLen + corrMetaCount + (hasLaunchPoint ? 1 : 0) + 1;
  return 2 + corrNavCount + metaCount;
}
