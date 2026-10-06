/**
 * Basic polygon geometry — area, centroid, inset, edge angle, line intersection, polyline offset.
 */

import { RAD2DEG } from './projection.js';

export function polygonAreaM2(pts) {
  const n = pts.length;
  if (n < 3) return 0;
  let area = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    area += pts[i][0] * pts[j][1];
    area -= pts[j][0] * pts[i][1];
  }
  return Math.abs(area) / 2;
}

export function polygonCentroid(pts) {
  const n = pts.length;
  let cx = 0, cy = 0;
  for (const p of pts) { cx += p[0]; cy += p[1]; }
  return [cx / n, cy / n];
}

export function insetPolygon(pts, insetM) {
  if (insetM <= 0 || pts.length < 3) return pts;
  const n = pts.length;
  // Winding: signed area > 0 is CCW (interior lies to the left of each edge).
  let signed = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    signed += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1];
  }
  const inward = signed >= 0 ? 1 : -1;
  // Offset each edge inward by insetM along its (unit) normal — a true polygon
  // offset (miter join), not a radial scale toward the centroid (which distorted
  // the shape and gave a wrong, direction-dependent inset).
  const oLines = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const [ax, ay] = pts[i];
    const [bx, by] = pts[j];
    const dx = bx - ax, dy = by - ay;
    const len = Math.hypot(dx, dy);
    if (len < 1e-9) continue;
    const nx = (-dy / len) * insetM * inward;
    const ny = (dx / len) * insetM * inward;
    oLines.push([[ax + nx, ay + ny], [bx + nx, by + ny]]);
  }
  const m = oLines.length;
  if (m < 3) return pts;
  // Each inset vertex is the intersection of the two adjacent offset edges.
  const result = [];
  for (let i = 0; i < m; i++) {
    const pt = lineIntersect(oLines[(i - 1 + m) % m], oLines[i]);
    result.push(pt || oLines[i][0]);
  }
  if (result.length < 3) return pts;
  // Reject an over-inset/degenerate result — an inset must SHRINK the polygon
  // while keeping its winding. When the swath margin exceeds the polygon the
  // miter offset can flip or balloon the shape; callers treat "returns the
  // original polygon" as "cannot inset".
  let sa = 0;
  for (let i = 0; i < result.length; i++) {
    const j = (i + 1) % result.length;
    sa += result[i][0] * result[j][1] - result[j][0] * result[i][1];
  }
  if (Math.sign(sa) !== Math.sign(signed) || Math.abs(sa) < 1e-6
      || Math.abs(sa) >= Math.abs(signed)) return pts;
  return result;
}

export function longestEdgeAngle(pts) {
  let bestLen = 0, bestAngle = 0;
  const n = pts.length;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const dx = pts[j][0] - pts[i][0];
    const dy = pts[j][1] - pts[i][1];
    const len = Math.hypot(dx, dy);
    if (len > bestLen) {
      bestLen = len;
      bestAngle = Math.atan2(dy, dx) * RAD2DEG;
    }
  }
  return bestAngle;
}

export function lineIntersect(line1, line2) {
  const [[x1, y1], [x2, y2]] = line1;
  const [[x3, y3], [x4, y4]] = line2;
  const denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (Math.abs(denom) < 1e-12) return null;
  const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
  return [x1 + t * (x2 - x1), y1 + t * (y2 - y1)];
}

export function offsetPolyline(polyline, offsetDist, side) {
  if (polyline.length < 2 || offsetDist === 0) return [...polyline];
  const d = offsetDist * side;
  const n = polyline.length;
  const oLines = [];
  for (let i = 0; i < n - 1; i++) {
    const [ax, ay] = polyline[i];
    const [bx, by] = polyline[i + 1];
    const dx = bx - ax, dy = by - ay;
    const segLen = Math.hypot(dx, dy);
    if (segLen < 1e-9) { oLines.push([[ax, ay], [bx, by]]); continue; }
    const nx = -dy / segLen * d, ny = dx / segLen * d;
    oLines.push([[ax + nx, ay + ny], [bx + nx, by + ny]]);
  }
  if (!oLines.length) return [...polyline];
  const result = [oLines[0][0]];
  for (let i = 0; i < oLines.length - 1; i++) {
    const pt = lineIntersect(oLines[i], oLines[i + 1]);
    result.push(pt || oLines[i][1]);
  }
  result.push(oLines[oLines.length - 1][1]);
  return result;
}
