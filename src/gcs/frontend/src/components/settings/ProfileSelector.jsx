import React, { useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { setDeviceConfigs, configurePlanner } from '../../utils/planner';
import {
  MIN_CONFIRM_PIXELS,
  DOCK_CLASS_TO_DETECT_ID,
  getClassDetectSize,
  getMinClassSize,
  isDetectorClassDimensionsConfigured,
} from '../../utils/plannerConfig.js';
import { buildCatalogDeviceInfos, buildDiagramDevices } from './profileSelectorUtils.js';

/**
 * Per-preset characteristic size derives from the preset's detect class via the
 * backend-owned CLASS_DETECT_SIZES (single source of truth). Presets no longer
 * carry `target_size_m`. Falls back to the smallest class size when unmapped.
 */
function presetTargetSize(presetName) {
  const classId = DOCK_CLASS_TO_DETECT_ID[presetName];
  return getClassDetectSize(classId) ?? getMinClassSize();
}

import { SettingsSection, inputStyle, NumericInput } from './SettingsField.jsx';
import DetectionRangeDiagram, { DEVICE_COLORS } from './DetectionRangeDiagram.jsx';
import {
  optimizeScanGeometry, detectRangeScale, confirmRange,
  groupPitchFromList, clampGroupPitch, fixedMissionAltitude, meanFovVDeg,
} from '../../utils/scanGeometry.js';

const headerStyle = { fontSize: 11, color: colors.textDim, fontWeight: 600, padding: '0 2px' };
const cellLabelStyle = { padding: '4px 10px', fontSize: 12, color: colors.text };
const cellNum = { ...inputStyle, maxWidth: 72, fontSize: 12, padding: '3px 6px' };
const cellZoom = { ...inputStyle, maxWidth: 64, fontSize: 12, padding: '3px 4px', cursor: 'pointer' };
const readonlyCell = { fontSize: 12, color: colors.textDim, padding: '3px 6px', fontVariantNumeric: 'tabular-nums' };

export default function ProfileSelector({
  draft, onApply, preSaveRef, preResetRef,
  presetOverrides = {}, setPresetOverrides, onCatalogPresets,
}) {
  const { t } = useTranslation();
  const [catalog, setCatalog] = useState(null);
  const pendingEditsRef = useRef({});
  // Frozen camera→slot order (by device name), captured once per profile from the
  // ORIGINAL catalog pitches. Reused across re-renders and post-Apply catalog
  // reloads so a camera's low/high slot identity never swaps — even when all
  // pitches become equal (spread 0) and the by-pitch sort would otherwise tie.
  const slotOrderRef = useRef({ profile: null, names: [] });
  const [localOverrides, setLocalOverrides] = useState({ profile: {}, devices: {}, zooms: {} });
  const [selectedTarget, setSelectedTarget] = useState('small');
  const [optimizeMsg, setOptimizeMsg] = useState(null);
  // Per-camera selected zoom (modal-local: the settings model persists only one
  // vision_zoom, so non-primary fixed-camera zoom lives here and feeds the diagram).
  const [deviceZooms, setDeviceZooms] = useState({});

  useEffect(() => {
    fetch('/api/vision-profiles')
      .then((r) => r.json())
      .then(setCatalog)
      .catch(() => setCatalog(null));
  }, []);

  // Auto-select default profile on first load if none set
  useEffect(() => {
    if (!catalog) return;
    const c = draft.camera || {};
    if (c.vision_profile) return;
    const defName = catalog.default_profile || Object.keys(catalog.profiles)[0];
    if (!defName) return;
    const pd = catalog.profiles[defName];
    const dev = pd?.devices[0];
    if (!dev) return;
    const zl = Object.keys(dev.zooms)[0] || '1';
    const zd = dev.zooms[zl];
    if (!zd) return;
    onApply({ vision_profile: defName, vision_device: dev.name, vision_zoom: zl });
  }, [catalog]);

  // Report the active profile's catalog presets up to the parent so the preset
  // editor + diagram render off the profile JSON (single source of truth) rather
  // than a settings copy. The parent merges these with its local edit buffer.
  useEffect(() => {
    if (!catalog || !onCatalogPresets) return;
    const c = draft.camera || {};
    const pName = c.vision_profile || catalog.default_profile || Object.keys(catalog.profiles)[0];
    onCatalogPresets(catalog.profiles[pName]?.dock_presets ?? {});
  }, [catalog, draft.camera?.vision_profile, onCatalogPresets]);

  /** Flush all pending edits to the backend on Apply. */
  useEffect(() => {
    if (!preSaveRef) return;
    preSaveRef.current = async () => {
      // Flush device-level edits sequentially (same JSON file, read-modify-write)
      const edits = pendingEditsRef.current;
      pendingEditsRef.current = {};
      let latest = null;
      for (const { profile, devName, zoom, ...fields } of Object.values(edits)) {
        try {
          const r = await fetch(`/api/vision-profiles/${encodeURIComponent(profile)}/devices/${encodeURIComponent(devName)}`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ zoom, ...fields }),
          });
          latest = await r.json();
        } catch { /* ignore */ }
      }
      // Flush profile-level overrides (envelope + optimized_altitude) and the
      // edited dock_presets together. Presets are owned by the profile JSON
      // (detector.dock_presets), NOT settings, so they persist here via the
      // profile-level endpoint which merges them per-class.
      const profBody = { ...localOverrides.profile };
      if (presetOverrides && Object.keys(presetOverrides).length > 0) {
        profBody.dock_presets = presetOverrides;
      }
      if (Object.keys(profBody).length > 0) {
        try {
          const r = await fetch(`/api/vision-profiles/${encodeURIComponent(profileName)}`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(profBody),
          });
          latest = await r.json();
        } catch { /* ignore */ }
      }
      if (latest) {
        setCatalog(latest);
        // Reconfigure planner with updated profile catalog so coverage/footprint
        // calculations use the new intrinsics without requiring a page reload.
        // Thread detector_class_dimensions so per-class sizes stay backend-owned.
        if (draft) configurePlanner(draft, latest.profiles, latest.detector_class_dimensions);
      }
      setLocalOverrides({ profile: {}, devices: {}, zooms: {} });
      if (setPresetOverrides) setPresetOverrides({});
    };
    return () => { preSaveRef.current = null; };
  });

  // Clear local overrides on Reset
  useEffect(() => {
    if (!preResetRef) return;
    preResetRef.current = () => {
      pendingEditsRef.current = {};
      setLocalOverrides({ profile: {}, devices: {}, zooms: {} });
      if (setPresetOverrides) setPresetOverrides({});
      setDeviceZooms({});
    };
    return () => { preResetRef.current = null; };
  });

  if (!catalog) return null;

  const cam = draft.camera || {};
  const profileName = cam.vision_profile || catalog.default_profile || Object.keys(catalog.profiles)[0];
  const profileData = catalog.profiles[profileName];
  const devices = profileData ? profileData.devices : [];
  const deviceName = cam.vision_device || (devices[0]?.name ?? '');
  const device = devices.find((d) => d.name === deviceName) || devices[0];
  const zoomLevels = device ? Object.keys(device.zooms) : [];
  const zoomLevel = cam.vision_zoom || zoomLevels[0] || '1';

  const profileOptions = Object.keys(catalog.profiles).map((k) => ({ value: k, label: k }));

  const buildLiveDeviceInfos = (profile, devName, zoom) =>
    buildCatalogDeviceInfos(catalog, profile, devName, zoom, 'detect_range_m');

  /** Queue a field edit for batch save on Apply. */
  const queueProfileEdit = (profile, devName, zoom, field, value) => {
    const apiField = field === 'camera_fx' ? 'fx' : field === 'camera_fy' ? 'fy' : field;
    const key = `${profile}/${devName}/${zoom}`;
    pendingEditsRef.current[key] = { ...(pendingEditsRef.current[key] || {}), profile, devName, zoom, [apiField]: value };
  };

  const applyProfile = (profile, devName, zoom) => {
    const pd = catalog.profiles[profile];
    if (!pd) return;
    const devs = pd.devices;
    const dev = devs.find((d) => d.name === devName) || devs[0];
    if (!dev) return;
    const zl = dev.zooms[zoom] ? zoom : Object.keys(dev.zooms)[0] || '1';
    if (!dev.zooms[zl]) return;

    setDeviceConfigs(buildLiveDeviceInfos(profile, devName, zl));

    // Selection only — pitch + presets live in the profile JSON (catalog), not settings.
    onApply({
      vision_profile: profile,
      vision_device: dev.name,
      vision_zoom: zl,
    });
  };

  // Diagram devices — pitch comes from the local device override (live edit)
  // falling back to the catalog device pitch (single source of truth); also
  // applies local overrides, draft pitch, and per-camera zoom.
  const diagramDevices = buildDiagramDevices(
    catalog,
    profileName,
    deviceName,
    zoomLevel,
    localOverrides.devices[deviceName]?.pitch_deg,
    localOverrides,
    deviceZooms,
  );

  // Presets come from the profile catalog (base) merged with the live edit buffer.
  const profileBasePresets = profileData?.dock_presets || {};
  const presets = {};
  for (const k of Object.keys(profileBasePresets)) {
    presets[k] = { ...profileBasePresets[k], ...(presetOverrides[k] || {}) };
  }
  for (const k of Object.keys(presetOverrides)) {
    if (!presets[k]) presets[k] = { ...presetOverrides[k] };
  }
  const presetKeys = Object.keys(presets);
  const altitude = presets[selectedTarget]?.altitude_m ?? 150;
  // Size is derived from the preset's detect class, not stored on the preset.
  const selectedTargetSize = presetTargetSize(selectedTarget);
  // Diagram shows the DETECTION range of the selected target. The base mdd uses
  // MIN_CLASS_SIZE/MIN_CONFIRM_PIXELS, so rescale to the selected target at the
  // detection gate (MIN_DETECT_PIXELS) → effective fy·selectedSize/MIN_DETECT_PIXELS.
  const mddScale = detectRangeScale(selectedTargetSize, getMinClassSize());
  const anyMultiZoom = devices.some((d) => Object.keys(d.zooms).length > 1);

  // --- Camera-class & envelope (shared by the gimbal optimizer and the fixed handlers) ---
  const anyGimbal = devices.some((d) => d.has_gimbal);
  const isFixed = !anyGimbal && devices.length > 0;     // body-fixed camera(s)
  const isMultiFixed = isFixed && devices.length > 1;   // group pitch + spread controls
  const minP = localOverrides.profile.min_pitch ?? profileData?.min_pitch ?? -60;
  const maxP = localOverrides.profile.max_pitch ?? profileData?.max_pitch ?? 20;

  // Effective per-camera zoom (modal-local). Primary defaults to the selected
  // vision_zoom; others to their first level until the operator picks one.
  const deviceEffZoom = (d) => {
    const z = deviceZooms[d.name];
    if (z && d.zooms[z]) return z;
    if (d.name === deviceName && zoomLevel && d.zooms[zoomLevel]) return zoomLevel;
    return Object.keys(d.zooms)[0] || '1';
  };
  const deviceFyAt = (d, z) => {
    const zov = localOverrides.zooms[`${d.name}/${z}`] || {};
    return zov.camera_fy ?? d.zooms[z]?.fy ?? 2000;
  };
  const deviceMaxFy = (d) => {
    const keys = Object.keys(d.zooms).map(Number).filter((n) => !Number.isNaN(n));
    return keys.length ? deviceFyAt(d, String(Math.max(...keys))) : deviceFyAt(d, deviceEffZoom(d));
  };
  const deviceCurPitch = (d) => {
    const sel = d.name === deviceName;
    const dov = localOverrides.devices[d.name] || {};
    return sel ? (cam.pitch_deg ?? dov.pitch_deg ?? d.pitch_deg) : (dov.pitch_deg ?? d.pitch_deg);
  };

  // Capture the slot order once per profile (lowest catalog pitch = slot 0, name
  // tiebreak) and reuse it; recompute only when the profile changes or it is empty.
  if (devices.length && (slotOrderRef.current.profile !== profileName || slotOrderRef.current.names.length === 0)) {
    slotOrderRef.current = {
      profile: profileName,
      names: [...devices]
        .sort((a, b) => ((a.pitch_deg ?? 0) - (b.pitch_deg ?? 0)) || (a.name < b.name ? -1 : 1))
        .map((d) => d.name),
    };
  }
  const slotDevices = slotOrderRef.current.names
    .map((n) => devices.find((d) => d.name === n))
    .filter(Boolean);
  const group = groupPitchFromList(slotDevices.map(deviceCurPitch)) || { center: -45, spread: 0 };
  const deviceImageHeight = (d) => (localOverrides.devices[d.name] || {}).image_height ?? d.image_height;

  // Express the boresight spread as an edge-to-edge OVERLAP that folds in each
  // camera's current-zoom FOV: overlap = meanFovV − spread (positive = the views
  // intersect, negative = a gap, 0 = edges touch). The overlap is the operator's
  // invariant: a per-camera zoom change holds it fixed by re-aiming the cameras
  // (see onFixedDeviceZoom), it does not recalculate the overlap.
  const meanFovV = isMultiFixed
    ? meanFovVDeg(slotDevices.map((d) => ({ fy: deviceFyAt(d, deviceEffZoom(d)), imageHeight: deviceImageHeight(d) })))
    : null;
  const maxSpread = (maxP - minP) / Math.max(1, slotDevices.length - 1);
  const groupOverlap = meanFovV != null ? meanFovV - group.spread : null;

  /** Recompute per-target altitudes for the fixed rig (lowest over ground-facing cameras). */
  const recomputeFixedPresets = (pitchByDevice = {}, zoomByDevice = {}) => {
    if (!isDetectorClassDimensionsConfigured()) return null;
    const minAlt = localOverrides.profile.min_altitude ?? profileData?.min_altitude ?? 30;
    const maxAlt = localOverrides.profile.max_altitude ?? profileData?.max_altitude ?? 2000;
    const profPresets = profileData?.dock_presets || {};
    const curPresets = draft.camera?.dock_presets || {};
    const baseKeys = Object.keys(profPresets).length > 0 ? Object.keys(profPresets) : Object.keys(curPresets);
    if (!baseKeys.length) return null;
    const cams = devices.map((d) => {
      const z = zoomByDevice[d.name] ?? deviceEffZoom(d);
      return { fy: deviceFyAt(d, z), fyMax: deviceMaxFy(d), pitchDeg: pitchByDevice[d.name] ?? deviceCurPitch(d) };
    });
    const updated = {};
    for (const k of baseKeys) {
      const pv = profPresets[k] || {};
      const dv = curPresets[k] || {};
      const size = presetTargetSize(k);
      const minPx = pv.min_pixel_size ?? dv.min_pixel_size ?? MIN_CONFIRM_PIXELS;
      const label = t(`planningSidebar.dockClasses.${k}`, { defaultValue: pv.label ?? dv.label ?? k });
      const alt = fixedMissionAltitude({ cameras: cams, sizeM: size, minPx, minAlt, maxAlt });
      updated[k] = { altitude_m: alt ?? (dv.altitude_m ?? pv.altitude_m ?? minAlt), min_pixel_size: minPx, label };
    }
    return updated;
  };

  /** Persist optimized_altitude (lowest preset) and push presets + camera fields to draft. */
  const commitFixedPresets = (updatedPresets, cameraFields = {}) => {
    if (updatedPresets && Object.keys(updatedPresets).length) {
      const lowestAlt = Math.min(...Object.values(updatedPresets).map((p) => p.altitude_m));
      setLocalOverrides((prev) => ({ ...prev, profile: { ...prev.profile, optimized_altitude: lowestAlt } }));
    }
    onApply(cameraFields, updatedPresets ?? null);
  };

  /** Apply a {deviceName: pitch} map: clamp into the envelope, persist, resize altitudes.
   *  zoomByDevice lets the caller size altitudes at a zoom not yet in deviceZooms
   *  state (async); extraCameraFields piggybacks e.g. the primary's vision_zoom. */
  const applyFixedPitches = (pitchByDevice, { zoomByDevice = {}, extraCameraFields = {} } = {}) => {
    setOptimizeMsg(null);
    const clamped = {};
    for (const d of devices) {
      if (pitchByDevice[d.name] == null) continue;
      // Keep one decimal: an odd spread fans cameras to half-degree pitches, and
      // rounding to whole degrees would drift the recovered group center.
      const p = Math.round(pitchByDevice[d.name] * 10) / 10;
      clamped[d.name] = Math.min(Math.max(p, minP), maxP);
    }
    const devOverrides = {};
    for (const [name, p] of Object.entries(clamped)) {
      devOverrides[name] = { ...(localOverrides.devices[name] || {}), pitch_deg: p };
      queueProfileEdit(profileName, name, '1', 'pitch_deg', p);
    }
    setLocalOverrides((prev) => ({ ...prev, devices: { ...prev.devices, ...devOverrides } }));
    const cameraFields = { ...extraCameraFields };
    if (clamped[deviceName] != null) cameraFields.pitch_deg = clamped[deviceName];
    commitFixedPresets(recomputeFixedPresets(clamped, zoomByDevice), cameraFields);
  };

  /** Single fixed camera: the slider sets that camera's pitch directly. */
  const onSingleFixedPitch = (d, p) => applyFixedPitches({ [d.name]: p });

  /** Multi fixed: derive every camera's pitch from the group center + spread. */
  const applyGroupPitch = (center, spread) => {
    const res = clampGroupPitch(center, spread, slotDevices.length, minP, maxP);
    if (!res) return;
    const pitchByDevice = {};
    slotDevices.forEach((d, i) => { pitchByDevice[d.name] = res.pitches[i]; });
    applyFixedPitches(pitchByDevice);
  };
  const onGroupCenter = (c) => applyGroupPitch(c, group.spread);
  // Operator sets the FOV-edge overlap; convert back to a boresight spread.
  const onGroupOverlap = (ov) => { if (meanFovV != null) applyGroupPitch(group.center, meanFovV - ov); };

  /** Per-camera zoom: modal-local; the primary also syncs vision_zoom.
   *  Multi-camera: hold the operator's OVERLAP fixed across the zoom change by
   *  re-aiming the cameras (new spread = newMeanFovV − currentOverlap) rather than
   *  letting the overlap recalculate; then resize altitudes at the new zoom. */
  const onFixedDeviceZoom = (d, z) => {
    setDeviceZooms((prev) => ({ ...prev, [d.name]: z }));
    const extraCameraFields = d.name === deviceName ? { vision_zoom: z } : {};
    if (isMultiFixed && groupOverlap != null) {
      const meanNew = meanFovVDeg(slotDevices.map((sd) => ({
        fy: deviceFyAt(sd, sd.name === d.name ? z : deviceEffZoom(sd)),
        imageHeight: deviceImageHeight(sd),
      })));
      const res = meanNew != null
        ? clampGroupPitch(group.center, meanNew - groupOverlap, slotDevices.length, minP, maxP)
        : null;
      if (res) {
        const pitchByDevice = {};
        slotDevices.forEach((sd, i) => { pitchByDevice[sd.name] = res.pitches[i]; });
        applyFixedPitches(pitchByDevice, { zoomByDevice: { [d.name]: z }, extraCameraFields });
        return;
      }
    }
    // Single camera (or fail-closed): just resize altitudes at the new zoom.
    commitFixedPresets(recomputeFixedPresets({}, { [d.name]: z }), extraCameraFields);
  };

  /** Update a profile-level field locally (saved on Apply). */
  const setProfileOverride = (field, value) => {
    setLocalOverrides((prev) => ({ ...prev, profile: { ...prev.profile, [field]: value } }));
  };

  /** Compute optimal pitch and per-target altitudes.
   *  Fixed camera: pitch = 5 - fovV/2 (upper FOV edge 5° above horizontal).
   *  Gimbal camera: run optimizer to find best pitch for search coverage.
   *  Per-target altitude = mdd * sin(-pitch), clamped to min_altitude. */
  const handleOptimize = (pitchArg, minZoomArg) => {
    // Operator-authoritative altitude optimization must use backend-sourced
    // per-class sizes (the single source of truth), never the JS before-fetch
    // fallback. Bail until the catalog has configured them.
    if (!isDetectorClassDimensionsConfigured()) return;
    const pd = catalog.profiles[profileName];
    if (!pd) return;
    const dev = pd.devices.find((d) => d.name === deviceName) || pd.devices[0];
    if (!dev) return;
    setOptimizeMsg(null);   // clear any prior message; each branch sets its own

    const minAlt = localOverrides.profile.min_altitude ?? pd.min_altitude ?? 30;
    const maxAlt = localOverrides.profile.max_altitude ?? pd.max_altitude ?? 2000;  // airspace ceiling
    const minP = localOverrides.profile.min_pitch ?? pd.min_pitch ?? -60;
    const boresight = pitchArg ?? localOverrides.devices[dev.name]?.pitch_deg ?? dev.pitch_deg ?? -45;   // operator's pitch (the device pitch field, catalog-sourced)
    const minZoom = minZoomArg ?? zoomLevel; // the device's selected zoom IS the scan min-zoom
    const dov = localOverrides.devices[dev.name] || {};
    // Always use zoom 1 for optimization — consistent base intrinsics
    const zov = localOverrides.zooms?.[`${dev.name}/1`] || {};
    const fy = zov.camera_fy ?? dev.zooms['1']?.fy ?? 2000;
    const w = dov.image_width ?? dev.image_width;
    const h = dov.image_height ?? dev.image_height;
    const fovVDeg = 2 * Math.atan(h / (2 * fy)) * 180 / Math.PI;

    let optPitch;
    const devOverrides = {};
    let smallestAlt = null;   // detect-far altitude of the smallest target (gimbal)
    let smallestSize = null;
    let visionZoom = null;
    let geom = null;          // gimbal optimizer result (for the post-loop notes)
    let fyMax = null;         // fy at max zoom — for the recognition altitude cap
    let recognitionPx = null; // operator-ID threshold for the recognition cap

    if (dev.has_gimbal) {
      // Detect-far model (gimbal + zoom): size the scan so the far FOV edge grazes
      // the ground at the SMALLEST target's detection range, trading altitude vs
      // reach (weight 0.5 = balanced) on the circle alt² + reach² = R_det². Per-class
      // altitudes follow; the planner's min over the mission's classes then makes the
      // smallest mission target govern. Scan zoom = the device's selected zoom level.
      const profPresets_g = pd.dock_presets || {};
      const curPresets_g = presetOverrides || {};
      const presetKeys_g = Object.keys(profPresets_g).length > 0 ? Object.keys(profPresets_g) : Object.keys(curPresets_g);
      smallestSize = Math.min(...presetKeys_g.map((k) => presetTargetSize(k)));
      const zovZ = localOverrides.zooms?.[`${dev.name}/${minZoom}`] || {};
      const fyCal = zovZ.camera_fy ?? dev.zooms[minZoom]?.fy;   // calibrated fy at the scan zoom
      // Clamp the pitch so the FAR FOV edge points BELOW the horizon (depression ≥ FOVv/2,
      // else the footprint floats in the air), and within the gimbal down-limit |min_pitch|.
      const fovVdeg = (fyCal > 0 && h > 0) ? 2 * Math.atan(h / (2 * fyCal)) * 180 / Math.PI : 40;
      const downLimit = Math.abs(minP);
      const depression = Math.min(downLimit, Math.max(fovVdeg / 2 + 1, Math.abs(boresight)));
      // Recognition: cap the altitude at the MAX-zoom confirm range (most demanding
      // operator-ID threshold over the presets) so the operator can confirm by zooming.
      recognitionPx = Math.max(...presetKeys_g.map((k) => {
        const pv = profPresets_g[k] || {}; const dv = curPresets_g[k] || {};
        return dv.min_pixel_size ?? pv.min_pixel_size ?? MIN_CONFIRM_PIXELS;
      }));
      const maxZoomKey = String(Math.max(...Object.keys(dev.zooms).map(Number)));
      fyMax = dev.zooms[maxZoomKey]?.fy;
      geom = optimizeScanGeometry({
        fy: fyCal, imageHeight: h, targetSizeM: smallestSize,
        boresightDeg: depression, zoom: parseFloat(minZoom),
      });
      if (!geom) {
        // Fail closed: missing calibration — write nothing.
        setOptimizeMsg(t('visionProfile.note.failClosed'));
        return;
      }
      optPitch = Math.round(geom.pitchDeg * 10) / 10;   // clamped pitch (far edge on ground, within gimbal)
      smallestAlt = geom.altitudeM;
      visionZoom = minZoom;
      // Persist the optimized gimbal pitch to the selected device's
      // gimbal.camera_pitch (vision_profiles.json, the single source of truth).
      // The fixed-camera branch already queues pitch per device in its loop;
      // the gimbal branch must do the same for the selected device or the
      // operator's pitch change never reaches the profile JSON.
      devOverrides[dev.name] = { ...(localOverrides.devices[dev.name] || {}), pitch_deg: optPitch };
      queueProfileEdit(profileName, dev.name, '1', 'pitch_deg', optPitch);
    } else {
      // Fixed: upper FOV edge 5° above horizontal — optimize ALL devices
      pd.devices.forEach((d) => {
        const dz = d.zooms['1'];
        if (!dz) return;
        const dovD = localOverrides.devices[d.name] || {};
        const zovD = localOverrides.zooms?.[`${d.name}/1`] || {};
        const devFy = zovD.camera_fy ?? dz.fy ?? 2000;
        const devH = dovD.image_height ?? d.image_height;
        const devFovV = 2 * Math.atan(devH / (2 * devFy)) * 180 / Math.PI;
        const devPitch = Math.round((5 - devFovV / 2) * 10) / 10;
        devOverrides[d.name] = { ...(localOverrides.devices[d.name] || {}), pitch_deg: devPitch };
        queueProfileEdit(profileName, d.name, '1', 'pitch_deg', devPitch);
      });
      // Use the selected device's pitch for altitude computation
      optPitch = devOverrides[dev.name]?.pitch_deg ?? Math.round((5 - fovVDeg / 2) * 10) / 10;
    }

    setLocalOverrides((prev) => ({
      ...prev,
      devices: { ...prev.devices, ...devOverrides },
    }));

    // Compute per-target altitude: mdd * sin(-pitch), clamped to min_altitude
    const profPresets = pd.dock_presets || {};
    const curPresets = presetOverrides || {};
    const baseKeys = Object.keys(profPresets).length > 0 ? Object.keys(profPresets) : Object.keys(curPresets);
    const sinP = Math.sin(-optPitch * Math.PI / 180);   // fixed-camera per-target sizing
    const updatedPresets = {};
    let ceilingClamped = false;
    let recognitionClamped = false;
    let floorClamped = false;
    for (const k of baseKeys) {
      const pv = profPresets[k] || {};
      const dv = curPresets[k] || {};
      // Size derives from the preset's detect class; it is not stored on the preset.
      const size = presetTargetSize(k);
      const minPx = dv.min_pixel_size ?? pv.min_pixel_size ?? MIN_CONFIRM_PIXELS;
      const label = t(`planningSidebar.dockClasses.${k}`, { defaultValue: dv.label ?? pv.label ?? k });
      let alt;
      if (dev.has_gimbal) {
        // Detect-far: far edge at each class's R_det → altitude scales with size. Cap
        // DOWN by the max-zoom confirm range (recognition) and the ceiling, UP by min-alt.
        const raw = smallestAlt * size / smallestSize;
        const confirm = confirmRange(fyMax, size, recognitionPx);
        const cap = Math.min(maxAlt, confirm);
        if (raw > cap) { if (confirm < maxAlt) recognitionClamped = true; else ceilingClamped = true; }
        if (raw < minAlt) floorClamped = true;
        alt = Math.max(minAlt, Math.round(Math.min(cap, raw)));
      } else {
        const mdd = fy * size / minPx;
        alt = Math.max(minAlt, Math.round(mdd * sinP));
      }
      updatedPresets[k] = { altitude_m: alt, min_pixel_size: minPx, label };
    }
    if (dev.has_gimbal && geom) {
      const notes = [];
      if (recognitionClamped) notes.push(t('visionProfile.note.recognitionCap'));
      if (ceilingClamped) notes.push(t('visionProfile.note.ceiling'));
      if (floorClamped) notes.push(t('visionProfile.note.minAltFloor'));
      setOptimizeMsg(notes.length ? notes.join(' ') : null);
    }

    // Persist optimized_altitude as the lowest target altitude (most restrictive)
    const lowestAlt = Math.min(...Object.values(updatedPresets).map((p) => p.altitude_m));
    setLocalOverrides((prev) => ({
      ...prev,
      profile: { ...prev.profile, optimized_altitude: lowestAlt },
    }));

    // Route the optimizer's per-class altitudes into the live preset buffer; it
    // is flushed to PUT /api/vision-profiles/{profile} (detector.dock_presets)
    // on Apply, NOT into the settings draft.
    if (setPresetOverrides) {
      setPresetOverrides((prev) => ({ ...prev, ...updatedPresets }));
    }
    // The (clamped) pitch is already persisted to the device via queueProfileEdit
    // and reflected live via devOverrides; only the scan zoom selection goes to
    // the settings draft here.
    if (visionZoom) onApply({ vision_zoom: visionZoom });
  };

  // Edit a device field — stored in localOverrides (live) and queued to the
  // profile JSON on Apply. pitch_deg persists to gimbal.camera_pitch via the
  // queueProfileEdit at the end of this function — never to the settings draft.
  const editField = (d, sel, devZooms, field, v) => {
    const devName = sel ? deviceName : d.name;
    const zoom = sel ? zoomLevel : (devZooms[0] || '1');
    // fx/fy are zoom-specific; pitch/W/H are device-level
    const isZoomField = field === 'camera_fx' || field === 'camera_fy';
    if (isZoomField) {
      const zoomKey = `${devName}/${zoom}`;
      setLocalOverrides((prev) => ({
        ...prev,
        zooms: { ...prev.zooms, [zoomKey]: { ...(prev.zooms[zoomKey] || {}), [field]: v } },
      }));
    } else {
      setLocalOverrides((prev) => ({
        ...prev,
        devices: { ...prev.devices, [devName]: { ...(prev.devices[devName] || {}), [field]: v } },
      }));
    }
    queueProfileEdit(profileName, devName, zoom, field, v);
  };

  return (
    <SettingsSection title={t('visionProfile.title')}>
      {/* Profile dropdown */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13 }}>
        <label style={{ color: colors.text, minWidth: 200, flexShrink: 0 }}>{t('visionProfile.profile')}</label>
        <select
          value={profileName}
          onChange={(e) => {
            const v = e.target.value;
            const pd = catalog.profiles[v];
            const dev = pd?.devices[0];
            const zl = dev ? Object.keys(dev.zooms)[0] || '1' : '1';
            pendingEditsRef.current = {};
            setLocalOverrides({ profile: {}, devices: {}, zooms: {} });
            if (setPresetOverrides) setPresetOverrides({});
            setDeviceZooms({});
            setOptimizeMsg(null);
            applyProfile(v, dev?.name ?? '', zl);
            fetch(`/api/vision-profiles/default/${encodeURIComponent(v)}`, { method: 'PUT' })
              .then((r) => r.json()).then(setCatalog).catch(() => {});
          }}
          style={{ ...inputStyle, cursor: 'pointer' }}
        >
          {profileOptions.map((o) => (
            <option key={o.value} value={o.value}>{o.label}</option>
          ))}
        </select>
      </div>

      {/* Dive pitch envelope + optimize */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 12 }}>
        <label style={{ color: colors.textDim, fontSize: 11, fontWeight: 600 }}>{t('visionProfile.divePitch')}</label>
        <label style={{ color: colors.textDim, fontSize: 11 }}>{t('visionProfile.min')}</label>
        <NumericInput
          value={localOverrides.profile.min_pitch ?? profileData?.min_pitch ?? -60}
          onChange={(v) => setProfileOverride('min_pitch', v)}
          min={-90} max={90} step={1} style={{ ...cellNum, maxWidth: 56 }}
        />
        <label style={{ color: colors.textDim, fontSize: 11 }}>{t('visionProfile.max')}</label>
        <NumericInput
          value={localOverrides.profile.max_pitch ?? profileData?.max_pitch ?? 20}
          onChange={(v) => setProfileOverride('max_pitch', v)}
          min={-90} max={90} step={1} style={{ ...cellNum, maxWidth: 56 }}
        />
        <label style={{ color: colors.textDim, fontSize: 11 }}>{t('visionProfile.minAlt')}</label>
        <NumericInput
          value={localOverrides.profile.min_altitude ?? profileData?.min_altitude ?? 30}
          onChange={(v) => setProfileOverride('min_altitude', v)}
          min={10} max={2000} step={10} style={{ ...cellNum, maxWidth: 56 }}
        />
        <label style={{ color: colors.textDim, fontSize: 11 }}>{t('visionProfile.maxAlt')}</label>
        <NumericInput
          value={localOverrides.profile.max_altitude ?? profileData?.max_altitude ?? 2000}
          onChange={(v) => setProfileOverride('max_altitude', v)}
          min={50} max={10000} step={50} style={{ ...cellNum, maxWidth: 56 }}
        />
        {/* Optimize button hidden for now — revisit later (joint pitch/zoom/altitude optimizer) */}
      </div>
      {optimizeMsg && (
        <div style={{ color: '#d08770', fontSize: 11, marginTop: 2 }}>{optimizeMsg}</div>
      )}

      {/* Group controls (multi-camera fixed rig): one pitch tilts all cameras
          together; overlap sets how their FOV edges intersect (driving the boresight
          spread, FOV-aware). Per-camera angles render read-only below. */}
      {isMultiFixed && (
        <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 12 }}>
          <label style={{ color: colors.textDim, fontSize: 11, fontWeight: 600 }}>{t('visionProfile.groupPitch')}</label>
          <input
            type="range" min={minP} max={maxP} step={1}
            value={Math.round(group.center)}
            onChange={(e) => onGroupCenter(parseFloat(e.target.value))}
            style={{ width: 110 }}
            title={t('visionProfile.groupPitchHint')}
          />
          <span style={{ fontSize: 11, color: colors.textDim, minWidth: 34 }}>{Math.round(group.center)}&deg;</span>
          <label style={{ color: colors.textDim, fontSize: 11, fontWeight: 600, marginLeft: 8 }}>{t('visionProfile.overlap')}</label>
          <input
            type="range"
            min={Math.round((meanFovV ?? 0) - maxSpread)}
            max={Math.round(meanFovV ?? maxSpread)}
            step={1}
            value={Math.round(groupOverlap ?? 0)}
            onChange={(e) => onGroupOverlap(parseFloat(e.target.value))}
            style={{ width: 110 }}
            title={t('visionProfile.overlapHint')}
          />
          <span style={{ fontSize: 11, color: colors.textDim, minWidth: 34 }}>{Math.round(groupOverlap ?? 0)}&deg;</span>
        </div>
      )}

      {/* Device / params grid */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: anyMultiZoom ? 'auto auto auto auto auto auto auto' : 'auto auto auto auto auto auto',
        gap: '4px 6px',
        alignItems: 'center',
      }}>
        <span style={headerStyle}>{t('visionProfile.device')}</span>
        {anyMultiZoom && <span style={headerStyle}>{t('visionProfile.zoom')}</span>}
        <span style={headerStyle}>{t('visionProfile.pitch')}</span>
        <span style={headerStyle}>fx</span>
        <span style={headerStyle}>fy</span>
        <span style={headerStyle}>W</span>
        <span style={headerStyle}>H</span>

        {devices.map((d, di) => {
          const sel = d.name === deviceName;
          const devZooms = Object.keys(d.zooms);
          const devZoom = deviceEffZoom(d);
          const zData = d.zooms[devZoom];
          const dov = localOverrides.devices[d.name] || {};
          const zov = localOverrides.zooms[`${d.name}/${devZoom}`] || {};
          // Pitch: live device override first, then the catalog (gimbal.camera_pitch).
          const devPitch = dov.pitch_deg ?? d.pitch_deg;
          const devFx = zov.camera_fx ?? zData?.fx ?? 0;
          const devFy = zov.camera_fy ?? zData?.fy ?? 0;
          const devW = dov.image_width ?? d.image_width;
          const devH = dov.image_height ?? d.image_height;
          const edit = (field, v) => editField(d, sel, devZooms, field, v);
          const devColor = DEVICE_COLORS[di % DEVICE_COLORS.length];

          return (
            <React.Fragment key={d.name}>
              <span style={cellLabelStyle}>
                <span style={{
                  display: 'inline-block', width: 8, height: 8, borderRadius: 2,
                  background: devColor, opacity: 0.6, marginRight: 6, verticalAlign: 'middle',
                }} />
                {d.name}
              </span>

              {anyMultiZoom && (
                devZooms.length > 1 ? (
                  <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
                    <input
                      type="range" min={0} max={devZooms.length - 1} step={1}
                      value={Math.max(0, devZooms.indexOf(devZoom))}
                      onChange={(e) => {
                        const z = devZooms[parseInt(e.target.value, 10)];
                        if (isFixed) {
                          onFixedDeviceZoom(d, z);
                        } else {
                          applyProfile(profileName, d.name, z);
                          if (sel) handleOptimize(undefined, z);
                        }
                      }}
                      style={{ width: 72 }}
                      title={isFixed ? 'Camera zoom (field of view)'
                        : (sel ? 'Scan zoom the optimizer sizes for' : undefined)}
                    />
                    <span style={{ fontSize: 11, color: colors.textDim, minWidth: 24 }}>{devZoom}x</span>
                  </div>
                ) : <span />
              )}

              {isMultiFixed ? (
                /* Multi-camera fixed rig: pitch is driven by the group sliders;
                   show each camera's resulting angle read-only (1-decimal, since an
                   odd spread produces half-degree pitches). */
                <span style={{ fontSize: 11, color: colors.textDim, minWidth: 30 }}>{Math.round((devPitch ?? -45) * 10) / 10}&deg;</span>
              ) : (
                <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
                  <input
                    type="range"
                    min={isFixed ? minP : -90}
                    max={isFixed ? maxP : 0}
                    step={1}
                    value={Math.round(devPitch ?? -45)}
                    onChange={(e) => {
                      const p = parseFloat(e.target.value);
                      if (isFixed) onSingleFixedPitch(d, p);
                      else if (sel) handleOptimize(p);
                      else edit('pitch_deg', p);
                    }}
                    style={{ width: 72 }}
                    title="Camera pitch"
                  />
                  <span style={{ fontSize: 11, color: colors.textDim, minWidth: 30 }}>{Math.round(devPitch ?? -45)}&deg;</span>
                </div>
              )}
              <span style={readonlyCell}>{Math.round(devFx)}</span>
              <span style={readonlyCell}>{Math.round(devFy)}</span>
              <span style={readonlyCell}>{devW}</span>
              <span style={readonlyCell}>{devH}</span>
            </React.Fragment>
          );
        })}
      </div>

      {/* Detection range diagram */}
      {diagramDevices.length > 0 && (
        <>
          {presetKeys.length > 0 && (
            <div style={{ display: 'flex', gap: 4, marginBottom: 4 }}>
              {presetKeys.map((key) => (
                <button
                  key={key}
                  onClick={() => setSelectedTarget(key)}
                  style={{
                    padding: '2px 10px', fontSize: 11, borderRadius: 3, cursor: 'pointer',
                    background: selectedTarget === key ? colors.accent : 'transparent',
                    color: selectedTarget === key ? '#000' : colors.textDim,
                    border: `1px solid ${selectedTarget === key ? colors.accent : colors.border}`,
                    fontWeight: selectedTarget === key ? 600 : 400,
                  }}
                >
                  {t(`planningSidebar.dockClasses.${key}`, { defaultValue: presets[key]?.label || key })} ({presets[key]?.altitude_m ?? '?'}m)
                </button>
              ))}
            </div>
          )}
          <DetectionRangeDiagram devices={diagramDevices.map((d) => ({
            ...d,
            maxDetectDist: d.maxDetectDist * mddScale,
          }))} altitude={altitude} />
        </>
      )}
    </SettingsSection>
  );
}
