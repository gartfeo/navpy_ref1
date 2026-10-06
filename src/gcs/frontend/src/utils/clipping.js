/**
 * Sutherland-Hodgman polygon clipping and equal-area partition.
 */

import { polygonAreaM2 } from './geometry.js';

export function clipPolygonByLine(polygon, p1, p2) {
  if (!polygon.length) return [];
  const inside = (p) =>
    (p2[0] - p1[0]) * (p[1] - p1[1]) - (p2[1] - p1[1]) * (p[0] - p1[0]) >= 0;
  const intersect = (a, b) => {
    const dax = b[0] - a[0], day = b[1] - a[1];
    const dpx = p2[0] - p1[0], dpy = p2[1] - p1[1];
    const denom = dax * dpy - day * dpx;
    if (Math.abs(denom) < 1e-12) return a;
    const t = ((p1[0] - a[0]) * dpy - (p1[1] - a[1]) * dpx) / denom;
    return [a[0] + t * dax, a[1] + t * day];
  };
  const output = [];
  const n = polygon.length;
  for (let i = 0; i < n; i++) {
    const curr = polygon[i];
    const nxt = polygon[(i + 1) % n];
    const cIn = inside(curr);
    const nIn = inside(nxt);
    if (cIn) {
      output.push(curr);
      if (!nIn) output.push(intersect(curr, nxt));
    } else if (nIn) {
      output.push(intersect(curr, nxt));
    }
  }
  return output;
}

export function clipPolygonByStrip(polygon, axisAngleRad, stripMin, stripMax) {
  const cosA = Math.cos(axisAngleRad), sinA = Math.sin(axisAngleRad);
  const perpX = -sinA, perpY = cosA;
  let result = polygon;
  // min bound
  result = clipPolygonByLine(result,
    [perpX * stripMin, perpY * stripMin],
    [perpX * stripMin + cosA, perpY * stripMin + sinA]);
  if (!result.length) return [];
  // max bound (reversed direction)
  return clipPolygonByLine(result,
    [perpX * stripMax + cosA, perpY * stripMax + sinA],
    [perpX * stripMax, perpY * stripMax]);
}

export function projectAlongPerp(pts, angleRad) {
  const perpX = -Math.sin(angleRad), perpY = Math.cos(angleRad);
  return pts.map((p) => p[0] * perpX + p[1] * perpY);
}

export function partitionPolygon(polygon, nParts, angleRad) {
  if (nParts <= 1) return [polygon];
  if (polygonAreaM2(polygon) < 1) return [polygon];

  const zones = [];
  let remaining = polygon;
  for (let i = 0; i < nParts - 1; i++) {
    const partsLeft = nParts - i;
    const targetArea = polygonAreaM2(remaining) / partsLeft;
    const rProjs = projectAlongPerp(remaining, angleRad);
    const rMin = Math.min(...rProjs);
    let lo = rMin, hi = Math.max(...rProjs);

    for (let iter = 0; iter < 50; iter++) {
      const mid = (lo + hi) / 2;
      const leftPart = clipPolygonByStrip(remaining, angleRad, rMin, mid);
      if (polygonAreaM2(leftPart) < targetArea) lo = mid;
      else hi = mid;
    }

    const split = (lo + hi) / 2;
    const zone = clipPolygonByStrip(remaining, angleRad, rMin, split);
    if (zone.length) zones.push(zone);
    remaining = clipPolygonByStrip(remaining, angleRad, split, Math.max(...rProjs));
    if (!remaining.length) break;
  }
  if (remaining.length) zones.push(remaining);
  return zones;
}
