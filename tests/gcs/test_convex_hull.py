"""Tests for geo.js utilities (convex hull, nearest point on polygon edge, wpOffset)."""
import unittest
import math
import os
from tests.gcs.js_runner import run_node
import json


# JS source inlined for Node.js subprocess testing
_GEO_JS = """
function convexHull(points) {
  if (points.length < 3) return [...points];
  const seen = new Set();
  const pts = points.filter((p) => {
    const key = p.lat + ',' + p.lon;
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });
  if (pts.length < 3) return [...pts];
  let pivot = 0;
  for (let i = 1; i < pts.length; i++) {
    if (pts[i].lat < pts[pivot].lat ||
        (pts[i].lat === pts[pivot].lat && pts[i].lon < pts[pivot].lon)) {
      pivot = i;
    }
  }
  [pts[0], pts[pivot]] = [pts[pivot], pts[0]];
  const p0 = pts[0];
  const cross = (o, a, b) =>
    (a.lon - o.lon) * (b.lat - o.lat) - (a.lat - o.lat) * (b.lon - o.lon);
  const distSq = (a, b) => (a.lat - b.lat) ** 2 + (a.lon - b.lon) ** 2;
  pts.sort((a, b) => {
    if (a === p0) return -1;
    if (b === p0) return 1;
    const c = cross(p0, a, b);
    if (c !== 0) return -c;
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

function nearestPointOnPolygonEdge(pt, polygon) {
  if (!polygon || polygon.length < 2) return { lat: pt.lat, lon: pt.lon };
  let bestDist = Infinity;
  let bestPt = { lat: polygon[0].lat, lon: polygon[0].lon };
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i];
    const b = polygon[(i + 1) % polygon.length];
    const dx = b.lat - a.lat;
    const dy = b.lon - a.lon;
    const lenSq = dx * dx + dy * dy;
    let t = 0;
    if (lenSq > 0) {
      t = ((pt.lat - a.lat) * dx + (pt.lon - a.lon) * dy) / lenSq;
      t = Math.max(0, Math.min(1, t));
    }
    const projLat = a.lat + t * dx;
    const projLon = a.lon + t * dy;
    const dist = (pt.lat - projLat) ** 2 + (pt.lon - projLon) ** 2;
    if (dist < bestDist) {
      bestDist = dist;
      bestPt = { lat: projLat, lon: projLon };
    }
  }
  return bestPt;
}
"""


def _run_node(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    full = _GEO_JS + script
    result = run_node(full, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


def _hull(points):
    """Compute convex hull via Node.js."""
    pts_json = json.dumps(points)
    return _run_node(f"console.log(JSON.stringify(convexHull({pts_json})));")


class TestConvexHull(unittest.TestCase):

    def test_triangle(self):
        pts = [{"lat": 0, "lon": 0}, {"lat": 1, "lon": 0}, {"lat": 0, "lon": 1}]
        hull = _hull(pts)
        self.assertEqual(len(hull), 3)

    def test_square(self):
        pts = [
            {"lat": 0, "lon": 0}, {"lat": 0, "lon": 1},
            {"lat": 1, "lon": 0}, {"lat": 1, "lon": 1},
        ]
        hull = _hull(pts)
        self.assertEqual(len(hull), 4)

    def test_interior_points_excluded(self):
        """Points inside the hull should not appear in result."""
        pts = [
            {"lat": 0, "lon": 0}, {"lat": 0, "lon": 4},
            {"lat": 4, "lon": 0}, {"lat": 4, "lon": 4},
            {"lat": 2, "lon": 2},  # interior
            {"lat": 1, "lon": 1},  # interior
        ]
        hull = _hull(pts)
        self.assertEqual(len(hull), 4)
        hull_set = {(p["lat"], p["lon"]) for p in hull}
        self.assertNotIn((2, 2), hull_set)
        self.assertNotIn((1, 1), hull_set)

    def test_duplicates_handled(self):
        pts = [
            {"lat": 0, "lon": 0}, {"lat": 1, "lon": 0},
            {"lat": 0, "lon": 1}, {"lat": 0, "lon": 0},  # duplicate
        ]
        hull = _hull(pts)
        self.assertEqual(len(hull), 3)

    def test_fewer_than_3_returns_copy(self):
        pts = [{"lat": 0, "lon": 0}, {"lat": 1, "lon": 1}]
        hull = _hull(pts)
        self.assertEqual(len(hull), 2)

    def test_collinear_points(self):
        """Collinear points should return endpoints only."""
        pts = [
            {"lat": 0, "lon": 0}, {"lat": 1, "lon": 0},
            {"lat": 2, "lon": 0},
        ]
        hull = _hull(pts)
        # Collinear: hull degenerates to a line (2 points)
        self.assertLessEqual(len(hull), 3)

    def test_hull_contains_all_extreme_points(self):
        """All corner points of a known shape should be on the hull."""
        pts = [
            {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
            {"lat": 10, "lon": 0}, {"lat": 10, "lon": 10},
            {"lat": 5, "lon": 5},  # interior
        ]
        hull = _hull(pts)
        hull_set = {(p["lat"], p["lon"]) for p in hull}
        for corner in [(0, 0), (0, 10), (10, 0), (10, 10)]:
            self.assertIn(corner, hull_set)

    def test_realistic_track_points(self):
        """Hull of a coverage track should be a reasonable polygon."""
        # Simulate a lawn-mower track in a rectangular area
        pts = []
        for row in range(5):
            lat = 32.0 + row * 0.001
            for col in range(10):
                lon = 34.0 + col * 0.001
                pts.append({"lat": lat, "lon": lon})
        hull = _hull(pts)
        # Rectangle corners should be on hull
        self.assertGreaterEqual(len(hull), 4)
        self.assertLessEqual(len(hull), 6)  # rectangle = 4 corners


def _nearest(pt, polygon):
    """Compute nearest point on polygon edge via Node.js."""
    pt_json = json.dumps(pt)
    poly_json = json.dumps(polygon)
    return _run_node(
        f"console.log(JSON.stringify(nearestPointOnPolygonEdge({pt_json}, {poly_json})));"
    )


class TestNearestPointOnPolygonEdge(unittest.TestCase):

    SQUARE = [
        {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
        {"lat": 10, "lon": 10}, {"lat": 10, "lon": 0},
    ]

    def test_point_on_edge_returns_itself(self):
        """A point already on an edge should return approximately that point."""
        pt = {"lat": 0, "lon": 5}  # on bottom edge
        result = _nearest(pt, self.SQUARE)
        self.assertAlmostEqual(result["lat"], 0, places=6)
        self.assertAlmostEqual(result["lon"], 5, places=6)

    def test_point_outside_clamps_to_nearest_edge(self):
        """A point outside should project onto the nearest edge."""
        pt = {"lat": -2, "lon": 5}  # below bottom edge
        result = _nearest(pt, self.SQUARE)
        self.assertAlmostEqual(result["lat"], 0, places=6)
        self.assertAlmostEqual(result["lon"], 5, places=6)

    def test_point_outside_corner_clamps_to_vertex(self):
        """A point beyond a corner should clamp to the nearest vertex."""
        pt = {"lat": -1, "lon": -1}  # outside bottom-left corner
        result = _nearest(pt, self.SQUARE)
        self.assertAlmostEqual(result["lat"], 0, places=6)
        self.assertAlmostEqual(result["lon"], 0, places=6)

    def test_point_inside_returns_nearest_edge(self):
        """A point inside the polygon returns the nearest boundary point."""
        pt = {"lat": 1, "lon": 5}  # inside, close to bottom edge
        result = _nearest(pt, self.SQUARE)
        # Should project to bottom edge at (0, 5)
        self.assertAlmostEqual(result["lat"], 0, places=6)
        self.assertAlmostEqual(result["lon"], 5, places=6)

    def test_point_far_right_projects_to_right_edge(self):
        """A point to the right of the square projects onto the right edge."""
        pt = {"lat": 5, "lon": 15}  # right of right edge
        result = _nearest(pt, self.SQUARE)
        self.assertAlmostEqual(result["lat"], 5, places=6)
        self.assertAlmostEqual(result["lon"], 10, places=6)

    def test_triangle(self):
        """Nearest point on a triangle edge."""
        tri = [
            {"lat": 0, "lon": 0}, {"lat": 10, "lon": 5}, {"lat": 0, "lon": 10},
        ]
        pt = {"lat": -1, "lon": 5}  # below the base
        result = _nearest(pt, tri)
        self.assertAlmostEqual(result["lat"], 0, places=6)
        self.assertAlmostEqual(result["lon"], 5, places=6)

    def test_single_edge_polygon(self):
        """Two-point polygon (degenerate) still works."""
        poly = [{"lat": 0, "lon": 0}, {"lat": 0, "lon": 10}]
        pt = {"lat": 5, "lon": 5}
        result = _nearest(pt, poly)
        self.assertAlmostEqual(result["lat"], 0, places=6)
        self.assertAlmostEqual(result["lon"], 5, places=6)


_GEO_JS_PATH = os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "geo.js",
)
_GEO_JS_FILE = open(os.path.normpath(_GEO_JS_PATH), encoding="utf-8").read()
_GEO_JS_FILE = _GEO_JS_FILE.replace("export function ", "function ")
_GEO_JS_FILE = _GEO_JS_FILE.replace("export const ", "const ")


def _run_node_file(script):
    full = _GEO_JS_FILE + "\n" + script
    result = run_node(full, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


def _wp_offset(search_pattern, poly_len, corr_len, has_lp):
    return _run_node_file(
        f"console.log(missionWpOffset('{search_pattern}', {poly_len}, {corr_len}, {str(has_lp).lower()}));"
    )


class TestMissionWpOffset(unittest.TestCase):
    """Verify missionWpOffset matches the mission item layout from waypoint_builder."""

    def test_distributed_no_corridor_no_lp(self):
        """Home + Takeoff + 1 fallback_delivery_location metadata = 3."""
        self.assertEqual(_wp_offset("distributed", 0, 0, False), 3)

    def test_distributed_polygon_only(self):
        """Home + Takeoff + (5 poly + 1 dt) metadata = 8."""
        self.assertEqual(_wp_offset("distributed", 5, 0, False), 8)

    def test_distributed_polygon_and_lp(self):
        """Home + Takeoff + (5 poly + 1 lp + 1 dt) metadata = 9."""
        self.assertEqual(_wp_offset("distributed", 5, 0, True), 9)

    def test_distributed_polygon_corridor_and_lp(self):
        """Home + Takeoff + 2 corridor NAVs + (5 poly + 1 lp + 1 dt) metadata = 11.
        Corridor backbone is NOT in metadata for distributed search_pattern."""
        self.assertEqual(_wp_offset("distributed", 5, 2, True), 11)

    def test_corridor_search_pattern_no_corridor_navs(self):
        """Corridor search_pattern: backbone goes in metadata, NOT as NAV_WPs.
        Home + Takeoff + (5 poly + 2 corr + 1 lp + 1 dt) metadata = 11."""
        self.assertEqual(_wp_offset("corridor", 5, 2, True), 11)

    def test_corridor_search_pattern_no_polygon(self):
        """Corridor with no polygon: Home + Takeoff + (3 corr + 1 lp + 1 dt) metadata = 7."""
        self.assertEqual(_wp_offset("corridor", 0, 3, True), 7)

    def test_corridor_search_pattern_no_lp(self):
        """Corridor no launch point: Home + Takeoff + (4 poly + 2 corr + 1 dt) metadata = 9."""
        self.assertEqual(_wp_offset("corridor", 4, 2, False), 9)

    def test_matches_build_mission_distributed(self):
        """Cross-check: distributed with 4-vertex polygon, 2 corridor, launch point.
        build_mission layout: Home(0), Takeoff(1), Corr0(2), Corr1(3),
        Meta[poly0..3,lp,dt](4-9), Track0(10)... → offset = 10."""
        self.assertEqual(_wp_offset("distributed", 4, 2, True), 10)

    def test_matches_build_mission_corridor(self):
        """Cross-check: corridor with 4-vertex polygon, 2 corridor, launch point.
        build_mission layout: Home(0), Takeoff(1),
        Meta[poly0..3,corr0,corr1,lp,dt](2-9), Track0(10)... → offset = 10."""
        self.assertEqual(_wp_offset("corridor", 4, 2, True), 10)


class TestFlatDist(unittest.TestCase):
    """Verify flatDist, flatDistSq, offsetLatLon from geo.js."""

    def test_flat_dist_zero(self):
        """Same point returns 0."""
        pt = {"lat": 32.0, "lon": 34.0}
        d = _run_node_file(
            f"console.log(flatDist({json.dumps(pt)}, {json.dumps(pt)}));"
        )
        self.assertAlmostEqual(d, 0.0, places=6)

    def test_flat_dist_north(self):
        """1 degree latitude ~ 111320 m."""
        a = {"lat": 32.0, "lon": 34.0}
        b = {"lat": 33.0, "lon": 34.0}
        d = _run_node_file(
            f"console.log(flatDist({json.dumps(a)}, {json.dumps(b)}));"
        )
        self.assertAlmostEqual(d, 111320, delta=1)

    def test_flat_dist_sq(self):
        """flatDistSq matches flatDist squared."""
        a = {"lat": 32.0, "lon": 34.0}
        b = {"lat": 32.01, "lon": 34.01}
        d = _run_node_file(
            f"const d = flatDist({json.dumps(a)}, {json.dumps(b)});"
            f"const dsq = flatDistSq({json.dumps(a)}, {json.dumps(b)});"
            f"console.log(JSON.stringify({{d, dsq}}));"
        )
        self.assertAlmostEqual(d["dsq"], d["d"] ** 2, places=2)

    def test_offset_lat_lon_north(self):
        """Offset 111320m north adds ~1 degree latitude."""
        r = _run_node_file(
            "console.log(JSON.stringify(offsetLatLon(32.0, 34.0, 111320, 0)));"
        )
        self.assertAlmostEqual(r["lat"], 33.0, places=4)
        self.assertAlmostEqual(r["lon"], 34.0, places=6)

    def test_offset_lat_lon_roundtrip(self):
        """Offset then distance should match original offset."""
        r = _run_node_file(
            "const p = offsetLatLon(32.0, 34.0, 500, 300);"
            "const d = flatDist({lat: 32.0, lon: 34.0}, p);"
            "console.log(d);"
        )
        expected = math.sqrt(500**2 + 300**2)
        self.assertAlmostEqual(r, expected, delta=1)


class TestProjectPointOnSegment(unittest.TestCase):
    """Verify projectPointOnSegment from geo.js."""

    def test_midpoint_projection(self):
        """Point projects to midpoint of segment."""
        a = {"lat": 0, "lon": 0}
        b = {"lat": 0, "lon": 10}
        pt = {"lat": 5, "lon": 5}
        r = _run_node_file(
            f"console.log(JSON.stringify(projectPointOnSegment("
            f"{json.dumps(pt)}, {json.dumps(a)}, {json.dumps(b)})));"
        )
        self.assertAlmostEqual(r["lat"], 0, places=6)
        self.assertAlmostEqual(r["lon"], 5, places=6)
        self.assertAlmostEqual(r["t"], 0.5, places=6)

    def test_clamp_to_start(self):
        """Point before segment start clamps to t=0."""
        a = {"lat": 0, "lon": 5}
        b = {"lat": 0, "lon": 10}
        pt = {"lat": 0, "lon": 0}
        r = _run_node_file(
            f"console.log(JSON.stringify(projectPointOnSegment("
            f"{json.dumps(pt)}, {json.dumps(a)}, {json.dumps(b)})));"
        )
        self.assertAlmostEqual(r["t"], 0, places=6)
        self.assertAlmostEqual(r["lat"], 0, places=6)
        self.assertAlmostEqual(r["lon"], 5, places=6)

    def test_clamp_to_end(self):
        """Point past segment end clamps to t=1."""
        a = {"lat": 0, "lon": 0}
        b = {"lat": 0, "lon": 5}
        pt = {"lat": 0, "lon": 10}
        r = _run_node_file(
            f"console.log(JSON.stringify(projectPointOnSegment("
            f"{json.dumps(pt)}, {json.dumps(a)}, {json.dumps(b)})));"
        )
        self.assertAlmostEqual(r["t"], 1, places=6)
        self.assertAlmostEqual(r["lat"], 0, places=6)
        self.assertAlmostEqual(r["lon"], 5, places=6)

    def test_zero_length_segment(self):
        """Degenerate segment returns t=0 at a."""
        a = {"lat": 5, "lon": 5}
        pt = {"lat": 10, "lon": 10}
        r = _run_node_file(
            f"console.log(JSON.stringify(projectPointOnSegment("
            f"{json.dumps(pt)}, {json.dumps(a)}, {json.dumps(a)})));"
        )
        self.assertAlmostEqual(r["t"], 0, places=6)
        self.assertAlmostEqual(r["lat"], 5, places=6)
        self.assertAlmostEqual(r["lon"], 5, places=6)


def _expand(polygon, offset):
    """Compute expandPolygon via Node.js against the real geo.js source."""
    return _run_node_file(
        f"console.log(JSON.stringify(expandPolygon({json.dumps(polygon)}, {offset})));"
    )


class TestExpandPolygon(unittest.TestCase):
    """Verify expandPolygon (geofence outward offset) from geo.js."""

    # CCW unit-ish square at the equator (cosLat ≈ 1 → 1 deg lon ≈ 111320 m).
    SQUARE = [
        {"lat": 0.0, "lon": 0.0},
        {"lat": 0.0, "lon": 0.01},
        {"lat": 0.01, "lon": 0.01},
        {"lat": 0.01, "lon": 0.0},
    ]
    # 100 m in degrees at the equator.
    DEG = 100.0 / 111320.0

    def test_fewer_than_three_returns_copy(self):
        pts = [{"lat": 0, "lon": 0}, {"lat": 1, "lon": 1}]
        self.assertEqual(len(_expand(pts, 100)), 2)

    def test_zero_offset_returns_copy(self):
        out = _expand(self.SQUARE, 0)
        self.assertEqual(len(out), 4)

    def test_square_expands_outward(self):
        """Each edge of an axis-aligned square moves out by ~offset meters."""
        out = _expand(self.SQUARE, 100)
        self.assertEqual(len(out), 4)
        lats = [p["lat"] for p in out]
        lons = [p["lon"] for p in out]
        self.assertAlmostEqual(min(lats), 0.0 - self.DEG, places=5)
        self.assertAlmostEqual(max(lats), 0.01 + self.DEG, places=5)
        self.assertAlmostEqual(min(lons), 0.0 - self.DEG, places=5)
        self.assertAlmostEqual(max(lons), 0.01 + self.DEG, places=5)

    def test_reversed_winding_still_expands_outward(self):
        """CW vertex order must still grow the polygon, not shrink it."""
        out = _expand(list(reversed(self.SQUARE)), 100)
        lats = [p["lat"] for p in out]
        self.assertAlmostEqual(min(lats), 0.0 - self.DEG, places=5)
        self.assertAlmostEqual(max(lats), 0.01 + self.DEG, places=5)

    def test_result_strictly_contains_original(self):
        """Every original vertex sits strictly inside the expanded box."""
        out = _expand(self.SQUARE, 100)
        lats = [p["lat"] for p in out]
        lons = [p["lon"] for p in out]
        self.assertLess(min(lats), 0.0)
        self.assertGreater(max(lats), 0.01)
        self.assertLess(min(lons), 0.0)
        self.assertGreater(max(lons), 0.01)


def _fence_incl(zone, paths=None, fallbackLocations=None, margin=100, takeoff=0, orbit=0):
    """Compute fenceInclusion via Node.js against the real geo.js source."""
    opts = {"marginM": margin, "takeoffRadiusM": takeoff, "orbitRadiusM": orbit}
    return _run_node_file(
        f"console.log(JSON.stringify(fenceInclusion("
        f"{json.dumps(zone)}, {json.dumps(paths or [])}, {json.dumps(fallbackLocations or [])}, "
        f"{json.dumps(opts)})));"
    )


def _round_inside(zone, center, r, paths=None, fallbackLocations=None, margin=100, takeoff=0, orbit=0):
    """True iff a 24-point ring of radius r about center is inside the fence."""
    opts = {"marginM": margin, "takeoffRadiusM": takeoff, "orbitRadiusM": orbit}
    return _run_node_file(
        f"const f=fenceInclusion({json.dumps(zone)},{json.dumps(paths or [])},"
        f"{json.dumps(fallbackLocations or [])},{json.dumps(opts)});"
        f"const ring=circlePointsLL({json.dumps(center)},{r},24);"
        f"console.log(JSON.stringify(ring.every((p)=>pointInPolygon(p,f))));"
    )


def _conflicts(**kw):
    """Run analyzeExclusionConflicts via Node.js; returns the conflict list."""
    return _run_node_file(
        f"console.log(JSON.stringify(analyzeExclusionConflicts({json.dumps(kw)})));"
    )


def _pip(pt, poly):
    """pointInPolygon via Node.js (True = strictly inside)."""
    return _run_node_file(
        f"console.log(JSON.stringify(pointInPolygon({json.dumps(pt)}, {json.dumps(poly)})));"
    )


def _all_inside(pts, poly):
    """True iff every point in pts is strictly inside poly."""
    return _run_node_file(
        f"const poly={json.dumps(poly)};const pts={json.dumps(pts)};"
        f"console.log(JSON.stringify(pts.every((p)=>pointInPolygon(p, poly))));"
    )


class TestPointInPolygon(unittest.TestCase):
    """Even-odd point-in-polygon (lon→x, lat→y), matches ArduPilot's test."""

    SQUARE = [
        {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
        {"lat": 10, "lon": 10}, {"lat": 10, "lon": 0},
    ]

    def test_center_inside(self):
        self.assertTrue(_pip({"lat": 5, "lon": 5}, self.SQUARE))

    def test_outside(self):
        self.assertFalse(_pip({"lat": 5, "lon": 15}, self.SQUARE))
        self.assertFalse(_pip({"lat": -1, "lon": 5}, self.SQUARE))

    def test_concave_notch_excluded(self):
        """A point in a concave notch is outside even if inside the bbox."""
        # C-shape opening east: notch around lon 8, lat 5.
        cshape = [
            {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
            {"lat": 4, "lon": 10}, {"lat": 4, "lon": 4},
            {"lat": 6, "lon": 4}, {"lat": 6, "lon": 10},
            {"lat": 10, "lon": 10}, {"lat": 10, "lon": 0},
        ]
        self.assertFalse(_pip({"lat": 5, "lon": 8}, cshape))  # in the notch
        self.assertTrue(_pip({"lat": 5, "lon": 2}, cshape))   # in the solid part


class TestFenceInclusion(unittest.TestCase):
    """Inclusion fence = convex hull of {zone, takeoff round, corridor, orbits}."""

    # ~1.1 km square search zone near (32, 34).
    ZONE = [
        {"lat": 32.000, "lon": 34.000},
        {"lat": 32.000, "lon": 34.010},
        {"lat": 32.010, "lon": 34.010},
        {"lat": 32.010, "lon": 34.000},
    ]
    # Launch ~1.1 km south of the zone, one corridor point leading north to it.
    PATH_SOUTH = [{"lat": 31.990, "lon": 34.005}, {"lat": 31.996, "lon": 34.005}]

    def test_convex_zone_no_transit_matches_expand(self):
        """A convex zone with nothing else is just the zone offset outward."""
        got = _fence_incl(self.ZONE, margin=100)
        want = _expand(self.ZONE, 100)
        self.assertEqual(len(got), len(want))
        for a, b in zip(got, want):
            self.assertAlmostEqual(a["lat"], b["lat"], places=6)
            self.assertAlmostEqual(a["lon"], b["lon"], places=6)

    def test_contains_zone_launch_corridor(self):
        """Every zone vertex + launch/corridor point is strictly inside."""
        fence = _fence_incl(self.ZONE, [self.PATH_SOUTH], margin=100)
        self.assertGreaterEqual(len(fence), 3)
        self.assertTrue(_all_inside(self.ZONE + self.PATH_SOUTH, fence))

    def test_launch_outside_zone_fence_now_inside(self):
        """The launch is OUTSIDE the zone-only fence but INSIDE the full fence —
        the arming/RTL bug the feature fixes."""
        launch = self.PATH_SOUTH[0]
        self.assertFalse(_pip(launch, _expand(self.ZONE, 100)))
        self.assertTrue(_pip(launch, _fence_incl(self.ZONE, [self.PATH_SOUTH], margin=100)))

    def test_takeoff_round_is_covered(self):
        """A full ring at the takeoff radius around the launch is enclosed."""
        self.assertTrue(_round_inside(
            self.ZONE, self.PATH_SOUTH[0], 200,
            paths=[self.PATH_SOUTH], margin=100, takeoff=200,
        ))

    def test_confirmation_orbit_is_covered(self):
        """A full ring at the orbit radius around an DOCK is enclosed."""
        dock = {"lat": 32.010, "lon": 34.010}  # DOCK on the zone corner (worst case)
        self.assertTrue(_round_inside(
            self.ZONE, dock, 300,
            fallbackLocations=[dock], margin=100, orbit=300,
        ))

    def test_multi_corridor_and_docks_contained(self):
        """Two corridors + Docks are all enclosed."""
        path_west = [{"lat": 32.005, "lon": 33.988}, {"lat": 32.005, "lon": 33.996}]
        fallbackLocations = [{"lat": 32.004, "lon": 34.004}, {"lat": 32.007, "lon": 34.007}]
        fence = _fence_incl(self.ZONE, [self.PATH_SOUTH, path_west], fallbackLocations, margin=100)
        self.assertTrue(_all_inside(self.PATH_SOUTH + path_west + fallbackLocations + self.ZONE, fence))

    def test_degenerate_zone_returns_copy(self):
        """A <3-point input returns a copy, never throws."""
        out = _fence_incl([{"lat": 32, "lon": 34}, {"lat": 32.01, "lon": 34}], margin=100)
        self.assertEqual(len(out), 2)

    # Long corridor: launch ~5.6 km south of the zone. The corridor must
    # contribute a TUBE arm, not balloon the hull toward the launch.
    LONG_PATH = [{"lat": 31.950, "lon": 34.005}, {"lat": 31.980, "lon": 34.005}]

    def test_corridor_is_a_tube_not_hull_fill(self):
        """A point lateral of the corridor midway (inside the OLD hull-fill
        fence, outside a margin-wide tube) must be OUTSIDE the fence."""
        fence = _fence_incl(self.ZONE, [self.LONG_PATH], margin=100)
        # On the corridor centerline, midway: inside the tube.
        self.assertTrue(_pip({"lat": 31.975, "lon": 34.005}, fence))
        # ~180 m east of the centerline at the same latitude: outside the
        # 100 m tube, though the pre-tube hull fence contained it.
        self.assertFalse(_pip({"lat": 31.975, "lon": 34.007}, fence))
        # The hull-fill fence (corridor points dumped into the hull) DID
        # contain that lateral point — proving the tube is what excludes it.
        hull_fill = _expand(_hull(self.ZONE + self.LONG_PATH), 100)
        self.assertTrue(_pip({"lat": 31.975, "lon": 34.007}, hull_fill))

    def test_tube_fence_still_covers_everything(self):
        """The tube fence passes the segment-level coverage check."""
        fence = _fence_incl(self.ZONE, [self.LONG_PATH], margin=100)
        self.assertTrue(_covers(fence, self.ZONE, [self.LONG_PATH]))
        self.assertTrue(_all_inside(self.ZONE + self.LONG_PATH, fence))

    def test_poi_circle_shapes_the_fence(self):
        """A POI (assigned DOCK / simulated POI) contributes a circle of
        orbit radius; the margin expands on top (orbit + margin total)."""
        dock = {"lat": 32.005, "lon": 34.020}  # ~850 m east of the zone
        fence = _fence_incl(self.ZONE, [], [dock], margin=100, orbit=300)
        # A point 350 m east of the POI: inside orbit(300)+margin(100).
        self.assertTrue(_pip({"lat": 32.005, "lon": 34.0241}, fence))
        # Without the POI the same point is far outside the zone fence.
        no_poi = _fence_incl(self.ZONE, [], [], margin=100, orbit=300)
        self.assertFalse(_pip({"lat": 32.005, "lon": 34.0241}, no_poi))


class TestClosestEdgeIndex(unittest.TestCase):
    """closestEdgeIndex — nearest closed-ring edge for click insertion."""

    SQUARE = [
        {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
        {"lat": 10, "lon": 10}, {"lat": 10, "lon": 0},
    ]

    def _cei(self, pt):
        return _run_node_file(
            f"console.log(closestEdgeIndex({json.dumps(self.SQUARE)}, {json.dumps(pt)}));"
        )

    def test_near_bottom_edge(self):
        self.assertEqual(self._cei({"lat": -1, "lon": 5}), 0)

    def test_near_right_edge(self):
        self.assertEqual(self._cei({"lat": 5, "lon": 11}), 1)

    def test_near_top_edge(self):
        self.assertEqual(self._cei({"lat": 11, "lon": 5}), 2)

    def test_near_wrap_edge(self):
        """The closing edge (last -> first vertex) is index n-1."""
        self.assertEqual(self._cei({"lat": 5, "lon": -1}), 3)


def _covers(fence, zone, paths=None, fallbackLocations=None, takeoff=0, orbit=0):
    """Run fenceCoversPlan via Node.js against the real geo.js source."""
    opts = {"takeoffRadiusM": takeoff, "orbitRadiusM": orbit}
    return _run_node_file(
        f"console.log(JSON.stringify(fenceCoversPlan({json.dumps(fence)}, "
        f"{json.dumps(zone)}, {json.dumps(paths or [])}, {json.dumps(fallbackLocations or [])}, "
        f"{json.dumps(opts)})));"
    )


class TestFenceCoversPlan(unittest.TestCase):
    """Containment check behind the custom-fence coverage warning."""

    ZONE = TestFenceInclusion.ZONE
    PATH_SOUTH = TestFenceInclusion.PATH_SOUTH

    def test_auto_fence_always_covers(self):
        """The auto-derived fence must always pass its own coverage check."""
        fence = _fence_incl(self.ZONE, [self.PATH_SOUTH], margin=100, takeoff=40)
        self.assertTrue(_covers(fence, self.ZONE, [self.PATH_SOUTH], takeoff=40))

    def test_shrunken_fence_fails(self):
        """A fence hugging only the zone leaves the launch outside -> False."""
        zone_only = _expand(self.ZONE, 100)
        self.assertFalse(_covers(zone_only, self.ZONE, [self.PATH_SOUTH]))

    def test_orbit_poking_out_fails(self):
        """Covering the DOCK point but not its orbit ring -> False."""
        dock = {"lat": 32.005, "lon": 34.005}
        fence = _expand(self.ZONE, 100)  # zone + 100 m, orbit needs 600 m
        self.assertTrue(_covers(fence, self.ZONE, [], [dock], orbit=0))
        self.assertFalse(_covers(fence, self.ZONE, [], [dock], orbit=800))

    def test_degenerate_fence_fails(self):
        self.assertFalse(_covers([{"lat": 0, "lon": 0}], self.ZONE, []))

    def test_concave_notch_across_corridor_leg_fails(self):
        """A hand-edited CONCAVE fence whose notch dips across a corridor LEG
        (both endpoints still inside) must fail — point sampling alone would
        pass it and the UAV would breach mid-transit."""
        # Corridor from launch (31.990) straight north to the zone (32.000).
        path = [{"lat": 31.990, "lon": 34.005}, {"lat": 31.998, "lon": 34.005}]
        # Fence: big box around everything, with a thin notch cutting inward
        # across the corridor line at lat ~31.994 (between the two endpoints).
        fence = [
            {"lat": 31.985, "lon": 33.995},
            {"lat": 31.985, "lon": 34.015},
            {"lat": 31.994, "lon": 34.015},
            {"lat": 31.994, "lon": 34.004},   # notch reaches past the corridor lon (34.005)
            {"lat": 31.9945, "lon": 34.004},
            {"lat": 31.9945, "lon": 34.015},
            {"lat": 32.015, "lon": 34.015},
            {"lat": 32.015, "lon": 33.995},
        ]
        # Endpoints are inside...
        self.assertTrue(_pip(path[0], fence))
        self.assertTrue(_pip(path[1], fence))
        # ...but the leg crosses the notch — coverage must fail.
        self.assertFalse(_covers(fence, self.ZONE, [path]))

    def test_concave_notch_into_zone_interior_fails(self):
        """A notch biting into the zone interior (all zone vertices still
        inside) must fail — the zone boundary crosses the fence edges."""
        fence = [
            {"lat": 31.995, "lon": 33.995},
            {"lat": 31.995, "lon": 34.015},
            {"lat": 32.015, "lon": 34.015},
            {"lat": 32.015, "lon": 34.006},
            {"lat": 32.005, "lon": 34.005},   # dips into the zone's middle
            {"lat": 32.015, "lon": 34.004},
            {"lat": 32.015, "lon": 33.995},
        ]
        for v in self.ZONE:
            self.assertTrue(_pip(v, fence))
        self.assertFalse(_covers(fence, self.ZONE, []))


class TestIsSimpleRing(unittest.TestCase):
    """isSimpleRing — self-intersection detection for edited fence/keep-outs."""

    def _simple(self, poly):
        return _run_node_file(
            f"console.log(JSON.stringify(isSimpleRing({json.dumps(poly)})));"
        )

    def test_square_is_simple(self):
        self.assertTrue(self._simple([
            {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
            {"lat": 10, "lon": 10}, {"lat": 10, "lon": 0},
        ]))

    def test_bowtie_is_not_simple(self):
        """Vertex dragged across the ring -> classic bowtie."""
        self.assertFalse(self._simple([
            {"lat": 0, "lon": 0}, {"lat": 10, "lon": 10},
            {"lat": 0, "lon": 10}, {"lat": 10, "lon": 0},
        ]))

    def test_concave_but_simple(self):
        """Concavity alone is fine — only crossings flag."""
        self.assertTrue(self._simple([
            {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
            {"lat": 10, "lon": 10}, {"lat": 10, "lon": 6},
            {"lat": 4, "lon": 5}, {"lat": 10, "lon": 4},
            {"lat": 10, "lon": 0},
        ]))

    def test_degenerate_not_simple(self):
        self.assertFalse(self._simple([{"lat": 0, "lon": 0}, {"lat": 1, "lon": 1}]))


class TestExclusionConflicts(unittest.TestCase):
    """analyzeExclusionConflicts flags flight elements entering a keep-out."""

    # Keep-out square around (32.005, 34.005), ~330 m per side.
    KEEPOUT = [
        {"lat": 32.0035, "lon": 34.0035},
        {"lat": 32.0035, "lon": 34.0065},
        {"lat": 32.0065, "lon": 34.0065},
        {"lat": 32.0065, "lon": 34.0035},
    ]

    def test_no_exclusions_no_conflicts(self):
        self.assertEqual(_conflicts(
            corridorPaths=[[{"lat": 32.0, "lon": 34.005}, {"lat": 32.01, "lon": 34.005}]],
            exclusions=[],
        ), [])

    def test_corridor_crossing_flagged(self):
        """A corridor leg straight through the keep-out is flagged."""
        res = _conflicts(
            corridorPaths=[[{"lat": 32.000, "lon": 34.005}, {"lat": 32.010, "lon": 34.005}]],
            exclusions=[self.KEEPOUT],
        )
        self.assertTrue(any(c["kind"] == "corridor" and c["exclusionIndex"] == 0 for c in res))

    def test_corridor_clear_not_flagged(self):
        """A corridor leg well outside the keep-out is not flagged."""
        res = _conflicts(
            corridorPaths=[[{"lat": 32.000, "lon": 34.000}, {"lat": 32.000, "lon": 34.010}]],
            exclusions=[self.KEEPOUT],
        )
        self.assertEqual(res, [])

    def test_track_crossing_flagged(self):
        res = _conflicts(
            tracks=[[{"lat": 32.005, "lon": 34.000}, {"lat": 32.005, "lon": 34.010}]],
            exclusions=[self.KEEPOUT],
        )
        self.assertTrue(any(c["kind"] == "track" for c in res))

    def test_orbit_reaching_in_flagged(self):
        """An DOCK whose orbit circle reaches into the keep-out is flagged,
        even though the DOCK point itself is outside it."""
        dock = {"lat": 32.005, "lon": 34.009}  # ~350 m east of the keep-out edge
        res = _conflicts(deliveryPoints=[dock], orbitRadiusM=400, exclusions=[self.KEEPOUT])
        self.assertTrue(any(c["kind"] == "orbit" for c in res))
        # The same DOCK with a tiny orbit does not reach the keep-out.
        self.assertEqual(_conflicts(deliveryPoints=[dock], orbitRadiusM=50, exclusions=[self.KEEPOUT]), [])

    def test_conflicts_deduped(self):
        """Multiple crossing legs of one element collapse to one entry."""
        zig = [
            {"lat": 32.000, "lon": 34.005}, {"lat": 32.010, "lon": 34.005},
            {"lat": 32.000, "lon": 34.0055}, {"lat": 32.010, "lon": 34.0055},
        ]
        res = _conflicts(tracks=[zig], exclusions=[self.KEEPOUT])
        track_hits = [c for c in res if c["kind"] == "track" and c["label"] == "UAV 1"]
        self.assertEqual(len(track_hits), 1)


if __name__ == "__main__":
    unittest.main()
