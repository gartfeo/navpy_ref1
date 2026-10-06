/**
 * Equirectangular projection helpers — lat/lon ↔ meters.
 */

export const R = 6371000.0;
export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;

export function toMeters(lat, lon, refLat) {
  return [
    lon * DEG2RAD * R * Math.cos(refLat * DEG2RAD),
    lat * DEG2RAD * R,
  ];
}

export function toLatLon(x, y, refLat) {
  return [
    y / R * RAD2DEG,
    x / (R * Math.cos(refLat * DEG2RAD)) * RAD2DEG,
  ];
}

export function polygonToMeters(polygon) {
  const refLat = polygon.reduce((s, p) => s + p[0], 0) / polygon.length;
  const pts = polygon.map((p) => toMeters(p[0], p[1], refLat));
  return [pts, refLat];
}

export function polygonToLatLon(ptsM, refLat) {
  return ptsM.map((p) => toLatLon(p[0], p[1], refLat));
}
