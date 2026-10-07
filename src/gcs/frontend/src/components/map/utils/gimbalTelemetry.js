import { MIN_DETECT_PIXELS, getDockDetectSize } from '../../../utils/plannerConfig.js';

const GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME = 32;
const GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME = 64;
const QUATERNION_MIN_NORM = 1e-12;

function normalizeQuaternionWxyz(q) {
  if (!Array.isArray(q) || q.length !== 4) return null;
  if (!q.every((value) => typeof value === 'number' && Number.isFinite(value))) return null;
  const norm = Math.hypot(q[0], q[1], q[2], q[3]);
  if (!Number.isFinite(norm) || norm <= QUATERNION_MIN_NORM) return null;
  return q.map((value) => value / norm);
}

function normalizeDeviceId(value) {
  if (!Number.isInteger(value)) return null;
  return value;
}

function normalizePositiveNumber(value) {
  if (typeof value !== 'number' || !Number.isFinite(value) || value <= 0) return null;
  return value;
}

function normalizeFovRad(value) {
  const fov = normalizePositiveNumber(value);
  if (fov == null || fov >= Math.PI) return null;
  return fov;
}

function usesLiveGimbalFootprint(camConfig) {
  const source = camConfig?.footprintSource;
  if (source != null) return source === 'live-gimbal';
  return camConfig?.gimbal_device_id != null;
}

export function normalizeGimbalTelemetry(telemetry) {
  if (!telemetry || telemetry.stale !== false) return null;

  const deviceId = normalizeDeviceId(telemetry.device_id);
  if (deviceId == null) return null;

  if (typeof telemetry.flags !== 'number' || !Number.isFinite(telemetry.flags)) return null;
  if ((telemetry.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME) === 0) return null;
  if ((telemetry.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME) !== 0) return null;

  if (typeof telemetry.failure_flags !== 'number' || !Number.isFinite(telemetry.failure_flags)) {
    return null;
  }
  if (telemetry.failure_flags !== 0) return null;

  const q = normalizeQuaternionWxyz(telemetry.q);
  if (!q) return null;

  if (telemetry.optics_stale !== false) return null;
  const fovH = normalizeFovRad(telemetry.fov_h_rad);
  const fovV = normalizeFovRad(telemetry.fov_v_rad);
  if (fovH == null || fovV == null) return null;

  const zoomLevel = telemetry.zoom_level == null
    ? null
    : normalizePositiveNumber(telemetry.zoom_level);
  if (telemetry.zoom_level != null && zoomLevel == null) return null;

  return {
    device_id: deviceId,
    q,
    flags: telemetry.flags,
    failure_flags: telemetry.failure_flags,
    fovH,
    fovV,
    zoomLevel,
  };
}

/** Select usable live gimbal telemetry for a device config from a vehicle snapshot. */
export function selectGimbalTelemetry(vehicle, camConfig) {
  if (!usesLiveGimbalFootprint(camConfig)) return null;
  const deviceId = normalizeDeviceId(camConfig?.gimbal_device_id);
  if (deviceId == null || !vehicle?.gimbals) return null;

  const telemetry = vehicle.gimbals[String(deviceId)];
  if (!telemetry) return null;

  const live = normalizeGimbalTelemetry(telemetry);
  if (!live || live.device_id !== deviceId) return null;
  return live;
}

/** Stable signature for coverage sampling when gimbal attitude changes in place. */
export function gimbalTelemetrySignature(telemetry) {
  if (!telemetry) return 'none';
  return [
    telemetry.device_id,
    telemetry.flags,
    telemetry.failure_flags,
    telemetry.fovH,
    telemetry.fovV,
    telemetry.zoomLevel ?? 'nozoom',
    ...telemetry.q,
  ].join(':');
}

export function computeLiveDetectRangeFromFov(fovVRad, imageHeight) {
  const fovV = normalizeFovRad(fovVRad);
  const height = normalizePositiveNumber(imageHeight);
  if (fovV == null || height == null) return null;
  const fy = height / (2 * Math.tan(fovV / 2));
  if (!Number.isFinite(fy) || fy <= 0) return null;
  return fy * getDockDetectSize() / MIN_DETECT_PIXELS;
}

export function resolveLiveGimbalCameraConfig(camConfig, telemetry) {
  if (!camConfig || !telemetry) return null;
  const imageHeight = normalizePositiveNumber(camConfig.imageHeight);
  if (imageHeight == null) return null;
  const maxDetectDist = computeLiveDetectRangeFromFov(telemetry.fovV, imageHeight);
  if (maxDetectDist == null) return null;
  return {
    ...camConfig,
    fovH: telemetry.fovH,
    fovV: telemetry.fovV,
    maxDetectDist,
    zoomLevel: telemetry.zoomLevel,
  };
}
