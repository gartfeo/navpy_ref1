import { getCameraConfig } from '../../../utils/planner';
import { offsetLatLon } from '../../../utils/geo';

export const COVERAGE_SAMPLE_DIST = 50; // meters between accumulated footprint samples
const MAX_PROJ_FACTOR = 8; // fallback slant-range cap when no detector range is available
const MIN_DOWN_COMPONENT = 0.001;

/**
 * Whether a camera's FOV reaches the ground within its detection range: the
 * steepest (lower) FOV edge must point below the horizon AND hit the ground
 * inside `maxDetectDist`. SHARED by the settings DetectionRangeDiagram and the
 * live map footprint so a camera that does not look at the ground (e.g. a
 * near-horizontal forward camera) leaves NO footprint in either view. Uses the
 * camera's nominal pitch (level vehicle) so the classification is stable and
 * does not flicker with live vehicle attitude.
 */
export function cameraHitsGround(pitchDeg, fovVRad, altitude, maxDetectDist) {
  if (!(altitude > 0) || !(maxDetectDist > 0)) return false;
  const lowerElev = (pitchDeg * Math.PI / 180) - fovVRad / 2;
  if (lowerElev >= 0) return false;            // steepest ray is at/above the horizon
  const groundSlant = altitude / Math.sin(-lowerElev);
  return groundSlant <= maxDetectDist;
}

// Camera mount derived values - recomputed when settings change
export function _mountConstants(camConfig) {
  const cam = camConfig || getCameraConfig();
  const p = cam.pitchDeg * Math.PI / 180;
  return { fovH: cam.fovH, fovV: cam.fovV, cosP: Math.cos(p), sinP: Math.sin(p) };
}

function _frustumCorners(fovH, fovV) {
  const tanH = Math.tan(fovH / 2);
  const tanV = Math.tan(fovV / 2);
  return [
    { cx: -tanH, cy: -tanV, cz: 1 }, // top-left
    { cx:  tanH, cy: -tanV, cz: 1 }, // top-right
    { cx:  tanH, cy:  tanV, cz: 1 }, // bottom-right
    { cx: -tanH, cy:  tanV, cz: 1 }, // bottom-left
  ];
}

function _vehicleToNedMatrix(headingDeg, pitchDeg, rollDeg) {
  const heading = headingDeg * Math.PI / 180;
  const pitch = pitchDeg * Math.PI / 180;
  const roll = rollDeg * Math.PI / 180;

  const ch = Math.cos(heading), sh = Math.sin(heading);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll), sr = Math.sin(roll);

  return [
    [ch*cp, ch*sp*sr - sh*cr, ch*sp*cr + sh*sr],
    [sh*cp, sh*sp*sr + ch*cr, sh*sp*cr - ch*sr],
    [-sp,   cp*sr,            cp*cr],
  ];
}

function _axisMatrix(axis, angleRad) {
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  if (axis === 'X') {
    return [
      [1, 0, 0],
      [0, c, -s],
      [0, s, c],
    ];
  }
  if (axis === 'Y') {
    return [
      [c, 0, s],
      [0, 1, 0],
      [-s, 0, c],
    ];
  }
  if (axis === 'Z') {
    return [
      [c, -s, 0],
      [s, c, 0],
      [0, 0, 1],
    ];
  }
  return null;
}

function _matMul(a, b) {
  return [
    [
      a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0],
      a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1],
      a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2],
    ],
    [
      a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0],
      a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1],
      a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2],
    ],
    [
      a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0],
      a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1],
      a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2],
    ],
  ];
}

function _matVec(m, v) {
  return [
    m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
    m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
    m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
  ];
}

function _eulerFromAttitudeList(attitude, sequence) {
  if (!Array.isArray(attitude) || attitude.length !== 3) return null;
  const pitch = Number(attitude[0]);
  const yaw = Number(attitude[1]);
  const roll = Number(attitude[2]);
  if (![pitch, yaw, roll].every(Number.isFinite)) return null;

  const values = { X: roll, Y: pitch, Z: yaw };
  const axes = String(sequence || '').split('');
  if (axes.length !== 3) return null;
  const angles = [];
  for (const rawAxis of axes) {
    const axis = rawAxis.toUpperCase();
    if (!(axis in values)) return null;
    angles.push(values[axis]);
  }
  return angles;
}

function _eulerMatrixFromAttitudeList(attitude, sequence) {
  const seq = String(sequence || '');
  const eulerDeg = _eulerFromAttitudeList(attitude, seq);
  if (!eulerDeg) return null;

  let axes = seq;
  let angles = eulerDeg;
  if (seq === seq.toLowerCase()) {
    axes = seq.split('').reverse().join('');
    angles = [...eulerDeg].reverse();
  }

  let result = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  for (let i = 0; i < axes.length; i += 1) {
    const axisMatrix = _axisMatrix(axes[i].toUpperCase(), angles[i] * Math.PI / 180);
    if (!axisMatrix) return null;
    result = _matMul(result, axisMatrix);
  }
  return result;
}

function _quaternionWxyzToMatrix(q) {
  if (!Array.isArray(q) || q.length !== 4) return null;
  if (!q.every((value) => typeof value === 'number' && Number.isFinite(value))) return null;
  const w = q[0], x = q[1], y = q[2], z = q[3];
  return [
    [
      1 - 2*(y*y + z*z),
      2*(x*y - z*w),
      2*(x*z + y*w),
    ],
    [
      2*(x*y + z*w),
      1 - 2*(x*x + z*z),
      2*(y*z - x*w),
    ],
    [
      2*(x*z - y*w),
      2*(y*z + x*w),
      1 - 2*(x*x + y*y),
    ],
  ];
}

function _validRangeCap(value, alt) {
  if (typeof value === 'number' && Number.isFinite(value) && value > 0) return value;
  return MAX_PROJ_FACTOR * alt;
}

function _rayNorm(ray) {
  return Math.sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]);
}

function _horizontalNorm(ray) {
  return Math.sqrt(ray[0]*ray[0] + ray[1]*ray[1]);
}

function _groundRangeFromSlant(maxSlantRange, alt) {
  const groundRangeSq = maxSlantRange*maxSlantRange - alt*alt;
  if (!Number.isFinite(groundRangeSq) || groundRangeSq <= 0) return null;
  return Math.sqrt(groundRangeSq);
}

function _projectRayToGroundRange(ray, maxGroundRange) {
  const horizontal = _horizontalNorm(ray);
  if (!Number.isFinite(horizontal) || horizontal <= 0) return [0, 0];
  const scale = maxGroundRange / horizontal;
  return [ray[0] * scale, ray[1] * scale];
}

function _projectDetectionEnvelopeRay(ray, alt, maxSlantRange) {
  const nedN = ray[0];
  const nedE = ray[1];
  const nedD = ray[2];
  const norm = _rayNorm(ray);
  if (!Number.isFinite(norm) || norm <= 0) return null;

  const maxGroundRange = _groundRangeFromSlant(maxSlantRange, alt);
  if (maxGroundRange == null) return null;

  if (nedD <= MIN_DOWN_COMPONENT) {
    return _projectRayToGroundRange(ray, maxGroundRange);
  }

  const groundScale = alt / nedD;
  const groundSlantRange = groundScale * norm;
  if (groundSlantRange > maxSlantRange) {
    return _projectRayToGroundRange(ray, maxGroundRange);
  }

  return [nedN * groundScale, nedE * groundScale];
}

function _clipRaysBelowHorizon(rays, minDown) {
  // Sutherland-Hodgman clip of the FOV-corner ray loop against the half-space
  // nedD >= minDown (the part of the frustum that looks below the horizon). A
  // convex set clipped by a half-plane stays convex, so the projected polygon is
  // ALWAYS simple — no bow-tie even for a near-horizontal, rolled camera whose
  // corners are nearly collinear (where a fixed-order 4-corner clamp would
  // self-intersect). The vertex count varies (3..5); the live footprint is then
  // padded to a constant length (see padFootprintLoop) so the animation still
  // never snaps.
  const out = [];
  const n = rays.length;
  for (let i = 0; i < n; i += 1) {
    const a = rays[i];
    const b = rays[(i + 1) % n];
    const da = a[2] - minDown;
    const db = b[2] - minDown;
    const aIn = da >= 0;
    const bIn = db >= 0;
    if (aIn) out.push(a);
    if (aIn !== bIn) {
      const t = da / (da - db);
      out.push([
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
      ]);
    }
  }
  return out;
}

function _projectDetectionEnvelopeRays(lat, lon, alt, rays, maxSlantRange) {
  const visible = _clipRaysBelowHorizon(rays, MIN_DOWN_COMPONENT);
  if (visible.length < 3) return null;   // FOV sees no usable ground patch

  const result = [];
  for (const ray of visible) {
    const projected = _projectDetectionEnvelopeRay(ray, alt, maxSlantRange);
    if (!projected) return null;
    result.push(offsetLatLon(lat, lon, projected[0], projected[1]));
  }
  return result;
}

/**
 * Pad a footprint loop to exactly `n` vertices by repeating the last vertex
 * (coincident, zero-length edges — invisible and they cannot self-intersect).
 * The Sutherland-Hodgman clip yields a VARYING vertex count (3..5) as a corner
 * crosses the horizon during a roll, which makes the interpolating live
 * animation snap (the turn jitter). Padding to a CONSTANT length removes the
 * snap while keeping the polygon SIMPLE (clip) and its real corners SHARP
 * (unlike resampling, which rounded them). Returns the loop unchanged when it
 * already has >= n vertices.
 */
export function padFootprintLoop(loop, n) {
  if (!Array.isArray(loop) || loop.length === 0 || loop.length >= n) return loop;
  const out = loop.slice();
  const last = loop[loop.length - 1];
  while (out.length < n) out.push(last);
  return out;
}

function _staticCameraToBodyMatrix(mount) {
  return [
    [0, mount.sinP, mount.cosP],
    [1, 0, 0],
    [0, mount.cosP, -mount.sinP],
  ];
}

function _projectCameraFrame(
  lat, lon, alt, headingDeg, pitchDeg, rollDeg, camConfig, cameraToVehicle,
) {
  const vehicleToNed = _vehicleToNedMatrix(headingDeg, pitchDeg, rollDeg);
  const cameraToNed = _matMul(vehicleToNed, cameraToVehicle);
  const rays = _frustumCorners(camConfig.fovH, camConfig.fovV)
    .map((corner) => _matVec(cameraToNed, [corner.cx, corner.cy, corner.cz]));
  return _projectDetectionEnvelopeRays(
    lat,
    lon,
    alt,
    rays,
    _validRangeCap(camConfig.maxDetectDist, alt),
  );
}

export function computeLiveGimbalDetectionEnvelope(
  lat, lon, alt, headingDeg, pitchDeg, rollDeg, camConfig, liveGimbalTelemetry,
) {
  if (alt == null || alt < 10 || !liveGimbalTelemetry) return null;
  const cam = camConfig || getCameraConfig();
  const setupMatrix = _eulerMatrixFromAttitudeList(cam.setup_att, cam.setup_seq);
  const gimbalToVehicle = _quaternionWxyzToMatrix(liveGimbalTelemetry.q);
  if (!setupMatrix || !gimbalToVehicle) return null;

  return _projectCameraFrame(
    lat, lon, alt, headingDeg, pitchDeg, rollDeg,
    cam,
    _matMul(gimbalToVehicle, setupMatrix),
  );
}

export function computeStaticCameraDetectionEnvelope(
  lat, lon, alt, headingDeg, pitchDeg, rollDeg, camConfig,
) {
  if (alt == null || alt < 10) return null;
  const cam = camConfig || getCameraConfig();
  const mount = _mountConstants(cam);
  return _projectCameraFrame(
    lat, lon, alt, headingDeg, pitchDeg, rollDeg,
    cam,
    _staticCameraToBodyMatrix(mount),
  );
}

export function computeCameraDetectionEnvelope(
  lat, lon, alt, headingDeg, pitchDeg, rollDeg, camConfig,
) {
  return computeStaticCameraDetectionEnvelope(
    lat, lon, alt, headingDeg, pitchDeg, rollDeg, camConfig,
  );
}

export const computeLiveGimbalCameraFootprint = computeLiveGimbalDetectionEnvelope;
export const computeStaticCameraFootprint = computeStaticCameraDetectionEnvelope;
export const computeCameraFootprint = computeCameraDetectionEnvelope;
