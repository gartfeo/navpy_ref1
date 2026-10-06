import { useEffect, useRef } from 'react';
import { zoneColorsSolid } from '../../../styles';
import { flatDist } from '../../../utils/geo';
import { getDeviceConfigs } from '../../../utils/planner';
import {
  cameraHitsGround,
  computeLiveGimbalDetectionEnvelope,
  COVERAGE_SAMPLE_DIST,
  padFootprintLoop,
} from '../utils/cameraFootprint';
import {
  gimbalTelemetrySignature,
  resolveLiveGimbalCameraConfig,
  selectGimbalTelemetry,
} from '../utils/gimbalTelemetry';

/** Maximum accumulated coverage entities before FIFO eviction. */
const MAX_COVERAGE_ENTITIES = 500;

/** Constant vertex count the live footprint is padded to (with invisible
 * duplicate vertices) before animating. The horizon clip yields 3..5 vertices
 * as a corner crosses the horizon during a roll; a constant count means the
 * interpolating animation never hits a length mismatch and never snaps (the
 * turn jitter), while the clip keeps the shape simple and the corners sharp. */
const FOOTPRINT_RENDER_VERTICES = 6;

/** Lerp each point in a Cartesian3 array, writing into scratch to avoid allocations. */
function lerpPositions(anim, Cesium, scratch) {
  const t = Math.min((Date.now() - anim.startTime) / anim.duration, 1.0);
  return anim.fromPositions.map((from, i) => {
    if (!scratch[i]) scratch[i] = new Cesium.Cartesian3();
    return Cesium.Cartesian3.lerp(from, anim.toPositions[i], t, scratch[i]);
  });
}

/** Advance envelope animation state for a new telemetry sample. */
function advanceEnvelopeAnim(prev, toPositions, Cesium) {
  const now = Date.now();
  if (!prev || prev.fromPositions.length !== toPositions.length) {
    return { fromPositions: toPositions, toPositions, startTime: now, duration: 600, ema: 200 };
  }
  const interval = now - prev.startTime;
  const ema = prev.ema * 0.7 + interval * 0.3;
  const duration = Math.min(Math.max(ema * 3, 100), 2000);
  const t = Math.min((now - prev.startTime) / prev.duration, 1.0);
  const fromPositions = prev.fromPositions.map((from, i) =>
    Cesium.Cartesian3.lerp(from, prev.toPositions[i], t, new Cesium.Cartesian3())
  );
  return { fromPositions, toPositions, startTime: now, duration, ema };
}

function signaturesEqual(a, b) {
  if (!a || !b || a.length !== b.length) return false;
  for (let i = 0; i < a.length; i += 1) {
    if (a[i] !== b[i]) return false;
  }
  return true;
}

function liveEnvelopeInputs(vehicle, devices) {
  return devices.map((device) => {
    const telemetry = selectGimbalTelemetry(vehicle, device);
    return {
      device: resolveLiveGimbalCameraConfig(device, telemetry),
      telemetry,
    };
  });
}

function removeEntity(entityMap, animMap, viewer, key) {
  if (!entityMap[key]) return;
  try { viewer.entities.remove(entityMap[key]); } catch {}
  delete entityMap[key];
  if (animMap) delete animMap[key];
}

/**
 * Coverage accumulation + live footprint + cleanup.
 * Samples camera detection envelopes based on distance traveled and adds ground-clamped polygons.
 * Renders live detection envelopes for ALL devices in the active vision profile.
 */
export default function useCoverageLayer(cesiumRef, viewerRef, storeRef, plan, showCoverage, showVision, trackResetKey, coverageResetKey, viewerReady, plannerReady) {
  const coverageRef = useRef({});
  const liveFootprintRef = useRef({});
  const footprintAnimRef = useRef({});

  // Coverage accumulation — subscribes to store for vehicle position updates
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer || !showCoverage) return;
    // Coverage footprints depend on planner config (vision profiles + dock
    // detect size); getDeviceConfigs() stays empty until it loads. Skip
    // accumulation until ready so we never render against incomplete config.
    if (!plannerReady) {
      return;
    }

    const baseAgl = plan?.altitude_m;
    if (!baseAgl) return;

    function update() {
      const vehicleList = storeRef.current.getVehicleList();
      if (!vehicleList) return;

      const devices = getDeviceConfigs();

      vehicleList.forEach((v, i) => {
        if (v.lat == null || v.lon == null) return;
        if (v.heading == null || v.pitch == null || v.roll == null) return;
        if (v.mode !== 'AUTO') return;

        const agl = v.alt_rel != null ? v.alt_rel : (plan?.zones?.[i]?.altitude_m || baseAgl);
        const pos = { lat: v.lat, lon: v.lon };
        const cov = coverageRef.current[i] || { entities: [], lastPos: null };
        coverageRef.current[i] = cov;

        const liveInputs = liveEnvelopeInputs(v, devices);
        const gimbalSignatures = liveInputs.map(({ telemetry }) => gimbalTelemetrySignature(telemetry));
        const movedEnough = !cov.lastPos || flatDist(cov.lastPos, pos) >= COVERAGE_SAMPLE_DIST;
        const gimbalsChanged = !signaturesEqual(cov.lastAttemptedGimbalSignatures, gimbalSignatures);
        if (!movedEnough && !gimbalsChanged) return;
        cov.lastAttemptedGimbalSignatures = gimbalSignatures;

        const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];
        let anyEnvelope = false;

        for (const { device: dev, telemetry } of liveInputs) {
          if (!dev || !telemetry) continue;
          // Same ground-intersection gate as the settings diagram: a camera that
          // does not reach the ground (e.g. a near-horizontal forward camera)
          // accumulates no coverage.
          if (!cameraHitsGround(dev.pitchDeg, dev.fovV, agl, dev.maxDetectDist)) continue;
          const envelope = computeLiveGimbalDetectionEnvelope(
            v.lat, v.lon, agl, v.heading, v.pitch, v.roll, dev, telemetry,
          );
          if (!envelope) continue;
          anyEnvelope = true;

          const alpha = dev.primary ? 0.25 : 0.15;
          const positions = envelope.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));
          const entity = viewer.entities.add({
            polygon: {
              hierarchy: new Cesium.PolygonHierarchy(positions),
              material: Cesium.Color.fromCssColorString(colorStr).withAlpha(alpha),
              classificationType: Cesium.ClassificationType.TERRAIN,
            },
          });
          cov.entities.push(entity);
        }

        if (anyEnvelope) {
          cov.lastPos = pos;
        }

        // FIFO eviction: cap accumulated entities
        let total = 0;
        for (const key of Object.keys(coverageRef.current)) {
          total += (coverageRef.current[key]?.entities?.length || 0);
        }
        while (total > MAX_COVERAGE_ENTITIES) {
          let removed = false;
          for (const key of Object.keys(coverageRef.current)) {
            const c = coverageRef.current[key];
            if (c?.entities?.length > 0) {
              const old = c.entities.shift();
              try { viewer.entities.remove(old); } catch {}
              total--;
              removed = true;
              break;
            }
          }
          if (!removed) break;
        }
      });
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady, showCoverage, plan, plannerReady]);

  // Live detection envelope with smooth interpolation (CallbackProperty)
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const baseAgl = plan?.altitude_m;
    const showLive = plannerReady && (showCoverage || showVision);

    // Preserve any existing live footprints while planner config is unavailable;
    // the toggle effect hides them. Reset stays owned by the cleanup effect.
    if (!plannerReady) {
      return;
    }

    if (!showLive || !baseAgl) {
      for (const key of Object.keys(liveFootprintRef.current)) {
        try { viewer.entities.remove(liveFootprintRef.current[key]); } catch {}
      }
      liveFootprintRef.current = {};
      footprintAnimRef.current = {};
      return;
    }

    function update() {
      const vehicleList = storeRef.current.getVehicleList();
      if (!vehicleList) return;

      const devices = getDeviceConfigs();
      const anim = footprintAnimRef.current;
      const seen = new Set();

      vehicleList.forEach((v, i) => {
        const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];
        const agl = v.alt_rel != null ? v.alt_rel : (plan?.zones?.[i]?.altitude_m || baseAgl);

        devices.forEach((dev, di) => {
          const key = `${i}_${di}`;
          seen.add(key);

          if (v.lat == null || v.lon == null ||
              v.heading == null || v.pitch == null || v.roll == null) {
            removeEntity(liveFootprintRef.current, anim, viewer, key);
            return;
          }

          const liveGimbal = selectGimbalTelemetry(v, dev);
          const liveDevice = resolveLiveGimbalCameraConfig(dev, liveGimbal);
          if (!liveGimbal || !liveDevice) {
            removeEntity(liveFootprintRef.current, anim, viewer, key);
            return;
          }

          // Same ground-intersection gate as the settings diagram — no footprint
          // for a camera that does not look at the ground (matches "no footprint
          // where the settings shows no ground intersection").
          if (!cameraHitsGround(liveDevice.pitchDeg, liveDevice.fovV, agl, liveDevice.maxDetectDist)) {
            removeEntity(liveFootprintRef.current, anim, viewer, key);
            return;
          }

          const envelope = computeLiveGimbalDetectionEnvelope(
            v.lat, v.lon, agl, v.heading, v.pitch, v.roll,
            liveDevice, liveGimbal,
          );

          if (!envelope) {
            removeEntity(liveFootprintRef.current, anim, viewer, key);
            return;
          }

          // Pad to a constant vertex count (invisible duplicate vertices) so the
          // on-screen vertex count never changes as a corner crosses the horizon
          // under roll — the interpolating CallbackProperty then never hits a
          // length mismatch and never snaps (the turn jitter), while the horizon
          // clip keeps the shape simple (no bow-tie) and the real corners sharp.
          const padded = padFootprintLoop(envelope, FOOTPRINT_RENDER_VERTICES);
          const positions = padded.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));
          const outlinePositions = [...positions, positions[0]];

          // Advance animation state
          anim[key] = advanceEnvelopeAnim(anim[key], outlinePositions, Cesium);

          if (!liveFootprintRef.current[key]) {
            // Fill-only (no bright outline). The two cameras are body-fixed at
            // different depression angles, so their footprints genuinely overlap
            // under roll — that is correct geometry, matching the backend
            // geo_ref_calc that navigation uses. Drawing fills (not crossing
            // outlines) shows it honestly: on a straight leg the footprints tile
            // and simply abut; in a bank the real overlap reads as a darker
            // blended patch. No geometry is altered.
            const scratch = [];
            const fillAlpha = dev.primary ? 0.30 : 0.20;
            const entity = viewer.entities.add({
              polygon: {
                hierarchy: new Cesium.CallbackProperty(() => {
                  const a = anim[key];
                  const pts = a ? lerpPositions(a, Cesium, scratch) : outlinePositions;
                  return new Cesium.PolygonHierarchy(pts);
                }, false),
                material: Cesium.Color.fromCssColorString(colorStr).withAlpha(fillAlpha),
                classificationType: Cesium.ClassificationType.TERRAIN,
              },
            });
            liveFootprintRef.current[key] = entity;
          }
        });
      });

      for (const key of Object.keys(liveFootprintRef.current)) {
        if (!seen.has(key)) {
          removeEntity(liveFootprintRef.current, anim, viewer, key);
        }
      }
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady, showCoverage, showVision, plan, plannerReady]);

  // Coverage cleanup (mission restart or manual clear)
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    for (const key of Object.keys(coverageRef.current)) {
      const cov = coverageRef.current[key];
      if (cov?.entities) {
        cov.entities.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
      }
    }
    coverageRef.current = {};

    for (const key of Object.keys(liveFootprintRef.current)) {
      try { viewer.entities.remove(liveFootprintRef.current[key]); } catch {}
    }
    liveFootprintRef.current = {};
    footprintAnimRef.current = {};
  }, [trackResetKey, coverageResetKey]);

  // Hide/show entities when toggles change
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    for (const key of Object.keys(coverageRef.current)) {
      const cov = coverageRef.current[key];
      if (cov?.entities) {
        cov.entities.forEach((e) => { e.show = plannerReady && showCoverage; });
      }
    }
    const showLive = plannerReady && (showCoverage || showVision);
    for (const key of Object.keys(liveFootprintRef.current)) {
      liveFootprintRef.current[key].show = showLive;
    }
  }, [showCoverage, showVision, plannerReady]);
}
