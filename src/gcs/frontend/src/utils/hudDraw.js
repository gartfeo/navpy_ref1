/** Pure helpers shared by HUD canvas components. */

export function normalizeHeading(deg) {
  return ((deg % 360) + 360) % 360;
}

const CARDINAL = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };

export function headingLabel(deg) {
  const n = normalizeHeading(Math.round(deg));
  if (CARDINAL[n]) return CARDINAL[n];
  return String(n).padStart(3, '0');
}

export function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

export function fmtVal(val, decimals = 0) {
  if (val == null || Number.isNaN(val)) return '--';
  return Number(val).toFixed(decimals);
}
