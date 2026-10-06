/**
 * Launch zone computation — polygon offset with round-join corners.
 */

import { polygonToMeters, polygonToLatLon } from './projection.js';

export function computeLaunchZone(polygonLL, bufferM) {
  if (!polygonLL || polygonLL.length < 3) return [[], 0];
  if (bufferM > 0) bufferM = Math.max(100, bufferM);
  const [ptsM, refLat] = polygonToMeters(polygonLL);
  const n = ptsM.length;
  // Ensure CCW winding
  let area = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    area += ptsM[i][0] * ptsM[j][1] - ptsM[j][0] * ptsM[i][1];
  }
  const work = area < 0 ? [...ptsM].reverse() : ptsM;
  // Offset each edge
  const offsetLines = [];
  const edgeEndVertex = [];
  for (let i = 0; i < work.length; i++) {
    const j = (i + 1) % work.length;
    const ex = work[j][0] - work[i][0], ey = work[j][1] - work[i][1];
    const elen = Math.hypot(ex, ey);
    if (elen < 1e-9) continue;
    const nx = ey / elen, ny = -ex / elen;
    offsetLines.push([
      [work[i][0] + nx * bufferM, work[i][1] + ny * bufferM],
      [work[j][0] + nx * bufferM, work[j][1] + ny * bufferM],
    ]);
    edgeEndVertex.push(work[j]);
  }
  if (offsetLines.length < 3) return [[], 0];
  // Round join at every corner — arc at radius r around each original vertex.
  const r = Math.abs(bufferM);
  const offsetPts = [];
  const m = offsetLines.length;
  for (let i = 0; i < m; i++) {
    const j = (i + 1) % m;
    const orig = edgeEndVertex[i];
    const arcStart = offsetLines[i][1];
    const arcEnd = offsetLines[j][0];
    const a0 = Math.atan2(arcStart[1] - orig[1], arcStart[0] - orig[0]);
    const a1 = Math.atan2(arcEnd[1] - orig[1], arcEnd[0] - orig[0]);
    const cross = (arcStart[0] - orig[0]) * (arcEnd[1] - orig[1]) -
                  (arcStart[1] - orig[1]) * (arcEnd[0] - orig[0]);
    let delta = a1 - a0;
    if (cross < 0) { if (delta > 0) delta -= 2 * Math.PI; }
    else { if (delta < 0) delta += 2 * Math.PI; }
    const nArc = Math.max(2, Math.round(Math.abs(delta) / (Math.PI / 8)));
    for (let k = 0; k <= nArc; k++) {
      const angle = a0 + (k / nArc) * delta;
      offsetPts.push([orig[0] + r * Math.cos(angle), orig[1] + r * Math.sin(angle)]);
    }
  }
  return [polygonToLatLon(offsetPts, refLat), bufferM];
}
