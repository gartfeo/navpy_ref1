import { MIN_DETECT_PIXELS, MIN_CONFIRM_PIXELS } from './plannerConfig.js';

const _finite = (x) => Number.isFinite(x);

/**
 * Pure detect-far scan geometry for a gimbal + zoom camera.
 *
 * The OPERATOR chooses the camera pitch; this returns the altitude at which the FAR
 * (top) FOV edge grazes the ground at the selected target's detection range R_det:
 *   altitude = R_det·sin(θ − FOVv/2),   reach = R_det·cos(θ − FOVv/2)
 * where θ = |boresightDeg| is the pitch depression. No clamping here — the caller
 * applies the altitude floor / ceiling and the recognition cap (see {@link confirmRange}),
 * so the pitch always follows the operator and only the altitude is bounded.
 *
 * @param {object}  p
 * @param {number}  p.fy            vertical focal length at the scan zoom, pixels
 * @param {number}  p.imageHeight   image height, pixels
 * @param {number}  p.targetSizeM   selected (smallest) target characteristic diagonal, m
 * @param {number} [p.boresightDeg=45] camera pitch depression (deg); non-finite → 45
 * @param {number} [p.detectPx]     detection pixel gate (default MIN_DETECT_PIXELS)
 * @param {number} [p.zoom=1]       scan zoom level, returned as-is
 * @returns {{altitudeM:number, reachM:number, pitchDeg:number, zoom:number,
 *            rDetM:number, fovVDeg:number, farDepDeg:number} | null}
 *          null on invalid input, or if the far edge points at/above the horizon.
 */
export function optimizeScanGeometry({
  fy,
  imageHeight,
  targetSizeM,
  boresightDeg = 45,
  detectPx = MIN_DETECT_PIXELS,
  zoom = 1,
}) {
  if (![fy, imageHeight, targetSizeM, detectPx].every((x) => _finite(x) && x > 0)) {
    return null;
  }
  const theta = (_finite(boresightDeg) ? Math.abs(boresightDeg) : 45) * Math.PI / 180;
  const rDet = (fy * targetSizeM) / detectPx;
  const fovV = 2 * Math.atan(imageHeight / (2 * fy)); // rad — vertical FOV
  const farDep = theta - fovV / 2; // far-edge (top FOV ray) depression
  if (!(farDep > 0)) return null;  // far edge at/above the horizon → no ground intersection
  return {
    altitudeM: rDet * Math.sin(farDep),
    reachM: rDet * Math.cos(farDep),
    pitchDeg: (-theta * 180) / Math.PI,
    zoom,
    rDetM: rDet,
    fovVDeg: (fovV * 180) / Math.PI,
    farDepDeg: (farDep * 180) / Math.PI,
  };
}

/**
 * Max-zoom confirmation range: the slant range at which a target of size `sizeM` reaches
 * `recognizePx` pixels at the camera's MAX zoom (`fyMax`). The optimizer caps the scan
 * altitude at this so the operator can always confirm/ID by zooming in. Returns Infinity
 * (no cap) when the inputs are missing/invalid.
 */
export function confirmRange(fyMax, sizeM, recognizePx) {
  if (!(fyMax > 0) || !(sizeM > 0) || !(recognizePx > 0)) return Infinity;
  return (fyMax * sizeM) / recognizePx;
}

/**
 * Factor that rescales a confirm-range base (fy·minClassSize/MIN_CONFIRM_PIXELS) to a
 * target's DETECTION range (fy·targetSize/MIN_DETECT_PIXELS). Used by the camera-
 * calculator diagram so it shows detection reach, not confirm-of-smallest.
 */
export function detectRangeScale(targetSizeM, minClassSizeM) {
  return (targetSizeM / minClassSizeM) * (MIN_CONFIRM_PIXELS / MIN_DETECT_PIXELS);
}

// ---------------------------------------------------------------------------
// Fixed-camera group-pitch model
//
// A multi-camera fixed rig is controlled by two GROUP parameters:
//   - center: the mean camera pitch (deg). Moving it rigidly shifts every camera.
//   - spread: the angular gap between ADJACENT cameras (deg). Widening it fans the
//             cameras apart around the center.
// Each camera sits in a slot i (0 = lowest pitch .. n-1 = highest):
//   pitch_i = center + spread * (i - (n-1)/2)
// For a single camera spread is 0 and the center IS that camera's pitch. The pure
// math lives here so the React layer only wires sliders; slot ORDER (which device
// owns which slot) is the caller's concern, kept stable by device name.
// ---------------------------------------------------------------------------

const _num = (x) => typeof x === 'number' && Number.isFinite(x);
const _clampNum = (x, lo, hi) => Math.min(Math.max(x, lo), hi);

/**
 * Derive {center, spread} from a list of per-camera pitches (deg). center is the
 * mean; spread is the even-fan adjacent spacing (max-min)/(n-1). Order-independent.
 * Returns null on empty / non-finite input.
 *
 * This is an even-fan BEST FIT: for n > 2 cameras whose stored pitches are NOT
 * evenly spaced, re-distributing {center, spread} does not reproduce the inputs
 * (only the mean and the overall min..max span are preserved). The fan model has
 * just two degrees of freedom by design. For n <= 2 the fit is exact. Callers
 * therefore must NOT silently re-distribute on panel open; they derive
 * {center, spread} for the slider thumbs but leave the stored per-camera pitches
 * untouched until the operator actually moves a group slider.
 */
export function groupPitchFromList(pitches) {
  if (!Array.isArray(pitches) || pitches.length === 0) return null;
  if (!pitches.every(_num)) return null;
  const n = pitches.length;
  const center = pitches.reduce((s, p) => s + p, 0) / n;
  const spread = n > 1 ? (Math.max(...pitches) - Math.min(...pitches)) / (n - 1) : 0;
  return { center, spread };
}

/**
 * Even-fan pitch (deg) for each of n slots: slot 0 lowest .. slot n-1 highest.
 * Negative spread is treated as 0. Returns null on invalid input.
 */
export function distributeGroupPitch(center, spread, n) {
  if (!_num(center) || !_num(spread) || !Number.isInteger(n) || n < 1) return null;
  const s = Math.max(0, spread);
  const out = [];
  for (let i = 0; i < n; i += 1) out.push(center + s * (i - (n - 1) / 2));
  return out;
}

/**
 * Clamp group center+spread so EVERY distributed camera pitch stays within the
 * profile [minPitch, maxPitch] envelope. Spread is first capped at the widest fan
 * that fits; the center is then constrained so neither the lowest nor the highest
 * slot leaves the envelope. Returns {center, spread, pitches} or null on bad input.
 */
export function clampGroupPitch(center, spread, n, minPitch, maxPitch) {
  if (![center, spread, minPitch, maxPitch].every(_num)) return null;
  if (!Number.isInteger(n) || n < 1 || minPitch > maxPitch) return null;
  if (n <= 1) {
    const c = _clampNum(center, minPitch, maxPitch);
    return { center: c, spread: 0, pitches: [c] };
  }
  const maxSpread = (maxPitch - minPitch) / (n - 1);
  const s = _clampNum(spread, 0, maxSpread);
  const minOffset = -(n - 1) / 2;
  const maxOffset = (n - 1) / 2;
  const centerMin = minPitch - minOffset * s;   // keeps the lowest slot >= minPitch
  const centerMax = maxPitch - maxOffset * s;   // keeps the highest slot <= maxPitch
  const c = _clampNum(center, centerMin, centerMax);
  return { center: c, spread: s, pitches: distributeGroupPitch(c, s, n) };
}

/**
 * Vertical FOV (degrees) from focal length fy and image height (pixels):
 * fovV = 2 * atan(H / (2 * fy)). Returns null when fy or height is non-positive /
 * non-finite, so callers fail closed rather than drawing a bogus cone.
 */
export function fovVDegFromFy(fy, imageHeight) {
  if (!_num(fy) || !_num(imageHeight) || fy <= 0 || imageHeight <= 0) return null;
  return 2 * Math.atan(imageHeight / (2 * fy)) * 180 / Math.PI;
}

/**
 * Mission altitude (m) for a fixed multi-camera rig at one dock class.
 *
 * Each camera entry is {fy, fyMax, pitchDeg}. A GROUND-FACING camera (negative
 * pitch) yields the altitude at which its boresight ground point sits at the
 * confirm slant range fy*size/minPx, i.e. altitude = confirmSlant * sin(depression).
 * That candidate is bounded above by the max-zoom recognition range
 * confirmRange(fyMax, size, minPx) and the airspace ceiling maxAlt. The mission
 * altitude is the MOST RESTRICTIVE (lowest) candidate over the ground-facing
 * cameras, floored at minAlt. Cameras at/above the horizon (e.g. an "up"-tilted
 * camera in a dual rig) are ignored. Returns null when NO camera faces the ground
 * (fail closed — the caller keeps the prior altitude rather than writing a bogus one).
 */
export function fixedMissionAltitude({ cameras, sizeM, minPx, minAlt, maxAlt }) {
  if (!Array.isArray(cameras) || cameras.length === 0) return null;
  if (![sizeM, minPx, minAlt, maxAlt].every(_num)) return null;
  if (sizeM <= 0 || minPx <= 0) return null;
  let best = Infinity;
  for (const cam of cameras) {
    if (!cam || !_num(cam.fy) || !_num(cam.pitchDeg) || cam.fy <= 0) continue;
    const depRad = -cam.pitchDeg * Math.PI / 180;   // depression below horizon
    if (depRad <= 0) continue;                       // at/above horizon — no ground confirm
    const confirmSlant = cam.fy * sizeM / minPx;
    let alt = confirmSlant * Math.sin(depRad);
    const cap = Math.min(maxAlt, confirmRange(cam.fyMax, sizeM, minPx));
    if (alt > cap) alt = cap;
    if (alt < best) best = alt;
  }
  if (!Number.isFinite(best)) return null;
  return Math.max(minAlt, Math.round(best));
}

/**
 * Mean vertical FOV (deg) over cameras `[{fy, imageHeight}]`, skipping entries
 * with non-positive/non-finite optics. Returns null when none are usable.
 *
 * Used to express the multi-camera control as an edge-to-edge OVERLAP instead of
 * a raw boresight separation: with `spread` the angle between adjacent camera
 * boresights, the FOV edges overlap by `overlap = meanFovV - spread` (positive =
 * the views intersect, negative = an uncovered gap, 0 = edges exactly touch).
 * Because it folds in each camera's FOV, the overlap automatically tightens as a
 * camera zooms in (narrower FOV) even when the boresights don't move.
 */
export function meanFovVDeg(cameras) {
  if (!Array.isArray(cameras) || cameras.length === 0) return null;
  const vals = [];
  for (const c of cameras) {
    const f = c ? fovVDegFromFy(c.fy, c.imageHeight) : null;
    if (f != null) vals.push(f);
  }
  if (!vals.length) return null;
  return vals.reduce((s, v) => s + v, 0) / vals.length;
}
