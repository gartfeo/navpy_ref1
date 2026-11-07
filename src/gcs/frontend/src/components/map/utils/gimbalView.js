/**
 * The vehicle window's Synthetic view through the gimbal camera
 * (docs/design/gcs_video_streams.md, section 5 "Synthetic view"; the owner,
 * 2026-11-07): where live gimbal telemetry exists the view looks where the
 * camera looks, with its field of view; else through the nose. Display only:
 * never a navigation input. The same telemetry and mount math as the map's
 * live camera footprint (gimbalTelemetry.js, cameraFootprint.js).
 */
import { cameraToNedMatrix, liveGimbalCameraToVehicle } from './cameraFootprint';
import { resolveLiveGimbalCameraConfig, selectGimbalTelemetry } from './gimbalTelemetry';

const RAD2DEG = 180 / Math.PI;

/**
 * Turned from the nose view by more than this (axis or roll about it), the
 * HUD's horizon (the aircraft's attitude) would visibly disagree with the
 * picture's: at the Synthetic picture's 179 px height and a 60 degree view,
 * 5 degrees is about 15 px.
 */
export const GIMBAL_ON_AXIS_RAD = 5 / RAD2DEG;

/**
 * The first device of the active vision profile with live gimbal telemetry.
 *
 * @param {object|null} vehicle - the store's snapshot of the UAV.
 * @param {object[]} devices - plannerConfig getDeviceConfigs().
 * @returns {{cameraToVehicle: number[][], fovH: number, fovV: number}|null}
 */
export function liveGimbalView(vehicle, devices) {
  for (const device of devices || []) {
    const telemetry = selectGimbalTelemetry(vehicle, device);
    if (!telemetry) continue;
    const cameraToVehicle = liveGimbalCameraToVehicle(device, telemetry);
    const config = resolveLiveGimbalCameraConfig(device, telemetry) ?? telemetry;
    if (cameraToVehicle) return { cameraToVehicle, fovH: config.fovH, fovV: config.fovV };
  }
  return null;
}

/** Camera frame (x right, y down, z optical axis) looking along the nose, in body FRD. */
const NOSE_MOUNT = [[0, 0, 1], [1, 0, 0], [0, 1, 0]];

/**
 * Cesium's frustum.fov for a camera's horizontal field of view: Cesium reads
 * fov as horizontal only when the canvas is wider than tall, else as vertical.
 *
 * @param {number} fovH - radians.
 * @param {number} aspectRatio - canvas width over height.
 */
export function cesiumFov(fovH, aspectRatio) {
  if (!(aspectRatio > 0) || aspectRatio >= 1) return fovH;
  return 2 * Math.atan(Math.tan(fovH / 2) / aspectRatio);
}

/**
 * The Cesium camera orientation looking through the gimbal camera.
 *
 * @param {{heading: number, pitch: number, roll: number}} nose - the vehicle's
 *   attitude as the nose view's camera orientation (radians; fpvPose.js
 *   noseOrientation: heading from north, pitch up, roll right wing down).
 * @param {number[][]} cameraToVehicle - liveGimbalView's matrix.
 * @returns {{heading: number, pitch: number, roll: number, offNose: number}|null}
 *   radians; offNose is the camera's rotation from the nose view, its roll
 *   about the optical axis included.
 */
export function gimbalOrientation(nose, cameraToVehicle) {
  const deg = [nose.heading * RAD2DEG, nose.pitch * RAD2DEG, nose.roll * RAD2DEG];
  const m = cameraToNedMatrix(...deg, cameraToVehicle);
  // Columns: the camera's right (x), down (y) and optical axis (z) in NED.
  const f = [m[0][2], m[1][2], m[2][2]];
  const r = [m[0][0], m[1][0], m[2][0]];
  const fNorm = Math.hypot(...f);
  if (!(fNorm > 0)) return null;
  const heading = Math.atan2(f[1], f[0]);
  const pitch = Math.asin(Math.max(-1, Math.min(1, -f[2] / fNorm)));
  // Roll: the camera's right against the level right for this heading; a
  // positive roll tips the right side down (NED +D).
  const levelRight = [-Math.sin(heading), Math.cos(heading), 0];
  // The unrolled camera's down: forward x level right (NED: N x E = D).
  const fu = f.map((v) => v / fNorm);
  const down0 = [
    fu[1] * levelRight[2] - fu[2] * levelRight[1],
    fu[2] * levelRight[0] - fu[0] * levelRight[2],
    fu[0] * levelRight[1] - fu[1] * levelRight[0],
  ];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const roll = Math.atan2(dot(r, down0), dot(r, levelRight));
  // The rotation angle of NOSE_MOUNT^T * cameraToVehicle: acos((trace - 1) / 2).
  let trace = 0;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) trace += NOSE_MOUNT[i][j] * cameraToVehicle[i][j];
  const offNose = Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2)));
  return { heading, pitch, roll, offNose };
}
