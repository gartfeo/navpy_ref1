/**
 * Planner configuration — mutable module-level state, mirrors gcs/backend/config.py.
 */

import { DEG2RAD } from './projection.js';

let CAMERA_FX = 2252.628;
let CAMERA_FY = 2262.151;
let CAMERA_IMAGE_WIDTH = 1920;
let CAMERA_IMAGE_HEIGHT = 1080;
let CAMERA_PITCH_DEG = -35.0;
let FOV_HORIZONTAL_DEG = 2 * Math.atan(CAMERA_IMAGE_WIDTH / (2 * CAMERA_FX)) * (180 / Math.PI);
let FOV_VERTICAL_DEG = 2 * Math.atan(CAMERA_IMAGE_HEIGHT / (2 * CAMERA_FY)) * (180 / Math.PI);
let OVERLAP_FRACTION = 0.1;
// Demo-mode manual route offset (m). null = auto (camera-derived spacing).
let TRACK_SPACING_OVERRIDE_M = null;
let CRUISE_SPEED_MS = 22.0;
let FLIGHT_BUDGET_KM = 80.0;
let SAFETY_RESERVE_KM = 20.0;
let MIN_LAUNCH_ZONE_BUFFER_KM = 5.0;
let USABLE_FLIGHT_KM = FLIGHT_BUDGET_KM - SAFETY_RESERVE_KM - MIN_LAUNCH_ZONE_BUFFER_KM;
let ALTITUDE_SEPARATION_M = 20.0;
let UAVS_PER_SET = 3;

let DOCK_PRESETS = {
  small: { altitude_m: 150.0 },
  medium: { altitude_m: 200.0 },
  large: { altitude_m: 250.0 },
};

// Multi-device config from backend profile catalog.
let DEVICE_CONFIGS = null;

// Detection: MIN_DETECT_PIXELS for the raw detector gate.
// Confirmation: MIN_CONFIRM_PIXELS for conservative operator/settings previews.
export const MIN_DETECT_PIXELS = 8;
export const MIN_CONFIRM_PIXELS = 20;

// Physical characteristic size per class: the bbox DIAGONAL sqrt(w^2+h^2)
// in metres. The BACKEND owns these — the GCS catalog's top-level
// `detector_class_dimensions` (GET /api/vision-profiles) is the single source of truth.
// `configurePlanner` rebuilds CLASS_DETECT_SIZES / MIN_CLASS_SIZE from it.
//
// The values below are only a before-fetch fallback so pure helpers
// (computeMaxDetectDist, resolveSmallestDetectClassSize) don't return NaN
// before the catalog arrives. The planner UI is gated on `plannerReady`, so
// these defaults are not used for any operator-visible computation; they
// mirror the backend (navpy.modules.vision.vision_profiles.get_class_detect_size)
// purely to keep early calls numerically sane.
const FALLBACK_CLASS_DETECT_SIZES = { 0: 4.3012, 1: 3.9051, 2: 3.9051, 3: 4.7434, 4: 1.8682 };

// Mutable: populated from the backend `detector_class_dimensions` by configurePlanner.
let CLASS_DETECT_SIZES = { ...FALLBACK_CLASS_DETECT_SIZES };
let MIN_CLASS_SIZE = Math.min(...Object.values(CLASS_DETECT_SIZES)); // Person

// True once the backend `detector_class_dimensions` has populated CLASS_DETECT_SIZES.
// Until then the values above are the before-fetch fallback and must NOT drive
// operator-authoritative computations (e.g. ProfileSelector altitude optimize).
let DETECTOR_CLASS_DIMENSIONS_CONFIGURED = false;

export const DOCK_CLASS_TO_DETECT_ID = { small: 4, medium: 0, large: 0 };
export const DEFAULT_IMGSZ = 640;         // kept for backward compat

/** Per-class characteristic size (bbox diagonal) for the given detect class id. */
export function getClassDetectSize(classId) {
  return CLASS_DETECT_SIZES[classId];
}

/** Smallest characteristic size across all known detect classes (Person). */
export function getMinClassSize() {
  return MIN_CLASS_SIZE;
}

/**
 * True once the backend `detector_class_dimensions` has configured the per-class sizes.
 * Operator-authoritative computations (e.g. altitude optimization) must check
 * this and refuse to compute off the before-fetch fallback.
 */
export function isDetectorClassDimensionsConfigured() {
  return DETECTOR_CLASS_DIMENSIONS_CONFIGURED;
}

/**
 * Rebuild CLASS_DETECT_SIZES / MIN_CLASS_SIZE from the backend catalog's
 * top-level `detector_class_dimensions` block: { "0": { width_m, height_m, size_m }, ... }.
 * `size_m` is the per-class bbox diagonal — the single source of truth.
 * No-op (keeps the current fallback) when detector_class_dimensions is absent/empty.
 */
export function configureDetectorClassDimensions(detectorClassDimensions) {
  if (!detectorClassDimensions || typeof detectorClassDimensions !== 'object') return;
  const next = {};
  for (const [classId, dims] of Object.entries(detectorClassDimensions)) {
    const size = dims?.size_m;
    if (typeof size === 'number' && Number.isFinite(size)) next[Number(classId)] = size;
  }
  if (!Object.keys(next).length) return;
  CLASS_DETECT_SIZES = next;
  MIN_CLASS_SIZE = Math.min(...Object.values(next));
  DETECTOR_CLASS_DIMENSIONS_CONFIGURED = true;
}

/** Max confirm distance (slant) — worst-case (Person) at MIN_CONFIRM_PIXELS.
 * NOTE: this is the CONFIRM-range BASE. ProfileSelector scales it by
 * detectRangeScale(selectedPoiSize, MIN_CLASS_SIZE) = (size/minClass)·(20/8)
 * to get the DETECTION range of the selected POI before it reaches the
 * DetectionRangeDiagram — so the diagram already matches the live map footprint.
 * Do NOT switch this to MIN_DETECT_PIXELS: that double-counts the 20/8 factor. */
export function computeMaxDetectDist(fy, imageWidth, imageHeight, referenceHeightM, imgsz) {
  return fy * MIN_CLASS_SIZE / MIN_CONFIRM_PIXELS;
}

export function resolveSmallestDetectClassId(dockClasses) {
  const classIds = Array.isArray(dockClasses)
    ? dockClasses
      .map((dockClass) => DOCK_CLASS_TO_DETECT_ID[dockClass])
      .filter((classId) => classId in CLASS_DETECT_SIZES)
    : [];

  const candidates = classIds.length ? classIds : Object.keys(CLASS_DETECT_SIZES).map(Number);
  return candidates.reduce((smallest, classId) => (
    CLASS_DETECT_SIZES[classId] < CLASS_DETECT_SIZES[smallest] ? classId : smallest
  ), candidates[0]);
}

export function resolveSmallestDetectClassSize(dockClasses) {
  return CLASS_DETECT_SIZES[resolveSmallestDetectClassId(dockClasses)];
}

/**
 * Update planner config from backend settings object.
 * Called from useSettings when settings are fetched or changed.
 * @param {object} settings - backend settings object
 * @param {object} [profileCatalog] - vision profiles map (the catalog's `profiles` key)
 * @param {object} [detectorClassDimensions] - the catalog's top-level `detector_class_dimensions` block
 *   ({ classId: { width_m, height_m, size_m } }) — the single source for per-class
 *   characteristic sizes. Threaded through so CLASS_DETECT_SIZES is backend-owned.
 */
export function configurePlanner(settings, profileCatalog, detectorClassDimensions) {
  if (!settings) return;
  configureDetectorClassDimensions(detectorClassDimensions);
  const f = settings.flight || {};
  const c = settings.camera || {};

  if (f.cruise_speed_ms != null) CRUISE_SPEED_MS = f.cruise_speed_ms;
  if (f.flight_budget_km != null) FLIGHT_BUDGET_KM = f.flight_budget_km;
  if (f.safety_reserve_km != null) SAFETY_RESERVE_KM = f.safety_reserve_km;
  if (f.min_launch_zone_buffer_km != null) MIN_LAUNCH_ZONE_BUFFER_KM = f.min_launch_zone_buffer_km;
  if (f.overlap_fraction != null) OVERLAP_FRACTION = f.overlap_fraction;
  if (f.altitude_separation_m != null) ALTITUDE_SEPARATION_M = f.altitude_separation_m;
  if (f.uavs_per_set != null) UAVS_PER_SET = f.uavs_per_set;

  // Resolve camera pitch + intrinsics + presets from the active vision profile
  // (the single source of truth). Settings carries only the SELECTION
  // (vision_profile / vision_device / vision_zoom) — pitch_deg and
  // dock_presets live in vision_profiles.json and arrive via the catalog.
  if (profileCatalog && c.vision_profile && profileCatalog[c.vision_profile]) {
    const prof = profileCatalog[c.vision_profile];
    const selDev = prof.devices.find((d) => d.name === c.vision_device) || prof.devices[0];
    if (selDev) {
      const selZoom = c.vision_zoom || '1';
      const zData = selDev.zooms[selZoom] || selDev.zooms['1'];
      if (zData) {
        CAMERA_FX = zData.fx;
        CAMERA_FY = zData.fy;
        CAMERA_IMAGE_WIDTH = selDev.image_width;
        CAMERA_IMAGE_HEIGHT = selDev.image_height;
      }
      if (selDev.pitch_deg != null) CAMERA_PITCH_DEG = selDev.pitch_deg;
    }
    if (prof.dock_presets) {
      DOCK_PRESETS = {};
      for (const [k, v] of Object.entries(prof.dock_presets)) {
        DOCK_PRESETS[k] = { altitude_m: v.altitude_m };
      }
    }
  }

  // Recompute derived values
  USABLE_FLIGHT_KM = FLIGHT_BUDGET_KM - SAFETY_RESERVE_KM - MIN_LAUNCH_ZONE_BUFFER_KM;
  FOV_HORIZONTAL_DEG = 2 * Math.atan(CAMERA_IMAGE_WIDTH / (2 * CAMERA_FX)) * (180 / Math.PI);
  FOV_VERTICAL_DEG = 2 * Math.atan(CAMERA_IMAGE_HEIGHT / (2 * CAMERA_FY)) * (180 / Math.PI);

  // Resolve multi-device configs from the backend catalog.
  // Live coverage uses the same raw detect range the backend detector uses.
  if (profileCatalog && c.vision_profile && profileCatalog[c.vision_profile]) {
    const profile = profileCatalog[c.vision_profile];
    const selectedDevice = c.vision_device;
    const selectedZoom = c.vision_zoom || '1';
    DEVICE_CONFIGS = profile.devices.map((dev) => {
      const isPrimary = dev.name === selectedDevice;
      const zoom = isPrimary ? (dev.zooms[selectedZoom] || dev.zooms['1']) : dev.zooms['1'];
      if (!zoom) return null;
      // Per-device pitch comes from the catalog (vision_profiles.json camera_pitch).
      const pitch = dev.pitch_deg;
      const fovH = 2 * Math.atan(dev.image_width / (2 * zoom.fx));
      const fovV = 2 * Math.atan(dev.image_height / (2 * zoom.fy));
      return {
        name: dev.name,
        gimbal_device_id: dev.gimbal_device_id,
        footprintSource: dev.publishes_gimbal_telemetry === false ? 'none' : 'live-gimbal',
        imageWidth: dev.image_width,
        imageHeight: dev.image_height,
        fovH,
        fovV,
        pitchDeg: pitch,
        primary: isPrimary,
        profileDetectDist: zoom.detect_range_m,
        maxDetectDist: zoom.detect_range_m,
        setup_att: dev.setup_att,
        setup_seq: dev.setup_seq,
      };
    }).filter(Boolean);
  } else {
    DEVICE_CONFIGS = null;
  }
}

/**
 * Demo-mode manual route offset: overrides the camera-derived track spacing.
 * Pass a positive number of meters, or null to restore automatic spacing.
 */
export function setTrackSpacingOverride(meters) {
  TRACK_SPACING_OVERRIDE_M =
    typeof meters === 'number' && Number.isFinite(meters) && meters > 0 ? meters : null;
}

/** Store device configs for multi-camera coverage visualization. */
export function setDeviceConfigs(devices) { DEVICE_CONFIGS = devices; }

/** Clear device configs (manual mode). */
export function clearDeviceConfigs() { DEVICE_CONFIGS = null; }

/**
 * Return all device configs for coverage rendering.
 * Returns [] until backend-derived device configs are available.
 */
export function getDeviceConfigs() {
  if (DEVICE_CONFIGS && DEVICE_CONFIGS.length > 0) return DEVICE_CONFIGS;
  return [];
}

/** Current camera config for external consumers (e.g. CesiumMap footprint). */
export function getCameraConfig() {
  return {
    fovH: FOV_HORIZONTAL_DEG * DEG2RAD,
    fovV: FOV_VERTICAL_DEG * DEG2RAD,
    pitchDeg: CAMERA_PITCH_DEG,
  };
}

/** Return snapshot of current config values needed by track spacing computation. */
export function getConfig() {
  return {
    DOCK_PRESETS,
    FOV_HORIZONTAL_DEG,
    FOV_VERTICAL_DEG,
    CAMERA_PITCH_DEG,
    OVERLAP_FRACTION,
    TRACK_SPACING_OVERRIDE_M,
    CRUISE_SPEED_MS,
    FLIGHT_BUDGET_KM,
    SAFETY_RESERVE_KM,
    MIN_LAUNCH_ZONE_BUFFER_KM,
    USABLE_FLIGHT_KM,
    ALTITUDE_SEPARATION_M,
    UAVS_PER_SET,
    DEG2RAD,
  };
}

export function computeSets(uavCount) {
  return Math.max(1, Math.ceil(uavCount / UAVS_PER_SET));
}

export function getUavsPerSet() {
  return UAVS_PER_SET;
}
