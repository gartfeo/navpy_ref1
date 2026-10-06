import { useEffect, useRef } from 'react';
import { zoneColorsSolid } from '../../../styles';
import { missionWpOffset } from '../../../utils/geo';

/**
 * Track polylines + vehicle position sync.
 * Creates passed/upcoming segment pairs per zone with smooth split-point
 * interpolation between 5 Hz telemetry updates.
 */
export default function useTrackLayer(
  cesiumRef, viewerRef, entitiesRef, trackPosRef, planRef, vehiclePosRef,
  storeRef, plan, searchPattern, polygon, launchPoint, corridorPoints,
  zoneCount, showTracks, trackResetKey, phase, viewerReady
) {
  const trackResetPending = useRef(false);
  // Per-zone animation state: { [idx]: { fromFrac, toFrac, startTime } }
  const trackAnimRef = useRef({});

  // Clear track progress when restart is triggered
  useEffect(() => {
    vehiclePosRef.current = {};
    trackResetPending.current = true;
    trackAnimRef.current = {};
  }, [trackResetKey]);

  // Reset track progress when entering PLANNING so paths show bold again
  useEffect(() => {
    if (phase === 'PLANNING') {
      vehiclePosRef.current = {};
      trackAnimRef.current = {};
    }
  }, [phase]);

  // Keep vehicle progress ref in sync via store subscription
  useEffect(() => {
    if (!plan?.zones) return;

    function update() {
      const vehicleList = storeRef.current.getVehicleList();
      if (!vehicleList) return;

      if (trackResetPending.current) {
        const allReset = vehicleList.every((v) => (v?.mission_progress || 0) < 3);
        if (allReset) {
          trackResetPending.current = false;
        } else {
          return;
        }
      }

      const wpOffset = missionWpOffset(
        searchPattern, polygon?.length || 0, corridorPoints?.length || 0, !!launchPoint,
      );
      vehicleList.forEach((v, i) => {
        const mp = v?.mission_progress || 0;
        const trackIdx = mp - wpOffset;
        const trackLen = plan.zones[i]?.track?.length || 0;
        const inAuto = v?.mode === 'AUTO';
        const onTrack = inAuto && trackIdx >= 0;
        if (onTrack && v?.lat != null && v?.lon != null) {
          vehiclePosRef.current[i] = { trackIdx, lat: v.lat, lon: v.lon };
        } else if (inAuto && trackIdx < 0) {
          vehiclePosRef.current[i] = null;
        } else if (vehiclePosRef.current[i] && !inAuto) {
          vehiclePosRef.current[i] = { trackIdx: trackLen, lat: 0, lon: 0 };
        }
      });
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady, plan, searchPattern, polygon, launchPoint, corridorPoints]);

  // Track overlays (structural recreation only — CallbackProperty for positions)
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    ents.tracks.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.tracks = [];

    if (!showTracks || !zoneCount) return;

    for (let i = 0; i < zoneCount; i++) {
      const idx = i;
      const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];
      const color = Cesium.Color.fromCssColorString(colorStr);

      function progressFrac(info) {
        const trackLL = planRef.current?.zones?.[idx]?.track;
        if (!info || !trackLL || trackLL.length < 2) return -1;
        const { trackIdx, lat, lon } = info;
        const maxIdx = trackLL.length - 1;
        if (trackIdx > maxIdx) return maxIdx;
        const nextIdx = Math.min(Math.max(trackIdx, 0), maxIdx);
        const prevIdx = Math.max(nextIdx - 1, 0);
        if (prevIdx === nextIdx) return prevIdx;
        const ax = trackLL[prevIdx].lon, ay = trackLL[prevIdx].lat;
        const bx = trackLL[nextIdx].lon, by = trackLL[nextIdx].lat;
        const dx = bx - ax, dy = by - ay;
        const len2 = dx * dx + dy * dy;
        let t = len2 > 0 ? ((lon - ax) * dx + (lat - ay) * dy) / len2 : 0;
        t = Math.max(0, Math.min(1, t));
        return prevIdx + t;
      }

      /** Smooth the raw fractional progress with adaptive-duration lerp. */
      function smoothFrac(rawFrac) {
        if (rawFrac < 0) return rawFrac;
        const ta = trackAnimRef.current;
        const prev = ta[idx];
        const now = Date.now();
        if (!prev || prev.toFrac !== rawFrac) {
          // New target — capture current interpolated position as start
          let fromFrac;
          if (prev && prev.toFrac >= 0) {
            const t = Math.min((now - prev.startTime) / prev.duration, 1.0);
            fromFrac = prev.fromFrac + (prev.toFrac - prev.fromFrac) * t;
          } else {
            fromFrac = rawFrac;
          }
          // Adaptive duration: EMA-smoothed interval × 3, clamped [100, 2000]
          const interval = prev ? now - prev.startTime : 200;
          const ema = prev?.ema ? prev.ema * 0.7 + interval * 0.3 : interval;
          const duration = Math.min(Math.max(ema * 3, 100), 2000);
          ta[idx] = { fromFrac, toFrac: rawFrac, startTime: now, duration, ema };
          return fromFrac;
        }
        const t = Math.min((now - prev.startTime) / prev.duration, 1.0);
        return prev.fromFrac + (prev.toFrac - prev.fromFrac) * t;
      }

      function lerpPosition(positions, idxL, t) {
        const ci = Math.min(idxL, positions.length - 1);
        const ni = Math.min(ci + 1, positions.length - 1);
        const a = positions[ci], b = positions[ni];
        if (ci === ni) return a;
        return new Cesium.Cartesian3(
          a.x + (b.x - a.x) * t,
          a.y + (b.y - a.y) * t,
          a.z + (b.z - a.z) * t,
        );
      }

      // Passed segment (thin, dimmed)
      const passedEntity = viewer.entities.add({
        polyline: {
          positions: new Cesium.CallbackProperty(() => {
            const allPositions = trackPosRef.current[idx];
            if (!allPositions || allPositions.length < 2) return [];
            const rawFrac = progressFrac(vehiclePosRef.current[idx]);
            if (rawFrac < 0.01) return [];
            const frac = smoothFrac(rawFrac);
            if (frac < 0.01) return [];
            const last = allPositions.length - 1;
            const wholeIdx = Math.min(Math.floor(frac), last);
            const t = frac - Math.floor(frac);
            const splitPt = lerpPosition(allPositions, wholeIdx, t);
            return [...allPositions.slice(0, wholeIdx + 1), splitPt];
          }, false),
          width: 2,
          material: color.withAlpha(0.35),
        },
      });
      ents.tracks.push(passedEntity);

      // Upcoming segment (bold, vivid) — arrow material for direction indication
      const upcomingEntity = viewer.entities.add({
        polyline: {
          positions: new Cesium.CallbackProperty(() => {
            const allPositions = trackPosRef.current[idx];
            if (!allPositions || allPositions.length < 2) return [];
            const rawFrac = progressFrac(vehiclePosRef.current[idx]);
            if (rawFrac < 0) return allPositions;
            const frac = smoothFrac(rawFrac);
            if (frac < 0) return allPositions;
            const last = allPositions.length - 1;
            const wholeIdx = Math.min(Math.floor(frac), last);
            const t = frac - Math.floor(frac);
            const splitPt = lerpPosition(allPositions, wholeIdx, t);
            const remaining = allPositions.slice(wholeIdx + 1);
            if (remaining.length === 0) return [splitPt];
            return [splitPt, ...remaining];
          }, false),
          width: 8,
          material: new Cesium.PolylineArrowMaterialProperty(color.withAlpha(0.9)),
        },
      });
      ents.tracks.push(upcomingEntity);

      // Per-segment arrow polylines for direction indication
      const trackLen = planRef.current?.zones?.[idx]?.track?.length || 0;
      if (trackLen >= 2) {
        const arrowColor = color.withAlpha(0.7);
        for (let seg = 0; seg < trackLen - 1; seg++) {
          const capturedSeg = seg;
          const e = viewer.entities.add({
            polyline: {
              positions: new Cesium.CallbackProperty(() => {
                const allPositions = trackPosRef.current[idx];
                if (!allPositions || capturedSeg + 1 >= allPositions.length) return [];
                return [allPositions[capturedSeg], allPositions[capturedSeg + 1]];
              }, false),
              width: 4,
              material: new Cesium.PolylineArrowMaterialProperty(arrowColor),
            },
          });
          ents.tracks.push(e);
        }
      }
    }
  }, [zoneCount, showTracks, viewerReady]);
}
