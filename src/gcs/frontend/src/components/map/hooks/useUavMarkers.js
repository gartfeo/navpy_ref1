import { useEffect, useRef } from 'react';
import { zoneColorsSolid } from '../../../styles';
import { needsTerrainSample, sampleTerrain } from '../utils/terrainCache';
import { positionAt, orientationAt, advanceAnim, setGround } from '../utils/lerpAnim';
import { poseFinite } from '../../../utils/fpvView';
import { uavModelOrientation } from '../utils/uavOrientation';

/**
 * 3D UAV vehicle models with smooth interpolated motion.
 *
 * Each entity uses CallbackProperty for position/orientation so Cesium
 * evaluates the motion every render frame (~60 fps), producing smooth
 * movement between 5 Hz telemetry updates (dead reckoning, see lerpAnim).
 * Only a new snapshot of a UAV is a new sample (a store update for another
 * UAV must not restart its motion); a new ground height under it only moves
 * its ground (setGround), never its speed. The first sample after the UAV's
 * telemetry was not fresh (a link gap) starts a new motion at that sample,
 * as the trail starts a new run there (useUavTrails): the model does not
 * glide across the gap.
 *
 * Altitude uses terrain-sampled height + relative (AGL) altitude.
 * heightReference is incompatible with explicit orientation quaternions
 * (causes model to shift when camera rotates as terrain LOD changes).
 *
 * A snapshot whose lat, lon, altitude, heading, roll or pitch is not a finite
 * number is skipped: the model, the chase and the first-person views keep the
 * last valid pose and resume from it (no 0 fill, which used to drop a UAV
 * without altitude to the terrain).
 */
export default function useUavMarkers(cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady) {
  const terrainCacheRef = useRef({});
  const animRef = useRef({});
  // sys_id -> { v, terrainH } of the last sample, so a repeat is not re-advanced.
  const sampleRef = useRef({});
  // sys_ids whose telemetry was not fresh since their last sample.
  const staleRef = useRef(new Set());

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    function update() {
      const vehicleList = storeRef.current.getVehicleList();
      const ents = entitiesRef.current;
      const existing = ents.uavMap || {};
      const seen = new Set();
      const cache = terrainCacheRef.current;
      const anim = animRef.current;
      const samples = sampleRef.current;
      const stale = staleRef.current;
      const store = storeRef.current;
      const now = Date.now();

      if (vehicleList && vehicleList.length > 0) {
        vehicleList.forEach((v, i) => {
          const sid = v.sys_id;
          const fresh = store.isTelemetryFresh ? store.isTelemetryFresh(sid) : true;
          if (!fresh) stale.add(sid);
          if (!poseFinite(v)) {
            // Keep the existing model at its last valid pose.
            if (existing[sid]) seen.add(sid);
            return;
          }
          seen.add(sid);
          const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];

            // Target position: cached terrain height + AGL altitude
            const cached = cache[sid];
            const terrainH = cached?.terrainHeight ?? 0;
            const altRel = v.alt_rel ?? v.alt;
            const poiPos = Cesium.Cartesian3.fromDegrees(v.lon, v.lat, terrainH + altRel);

            // Async terrain sample when cache is missing, stale, or UAV moved
            if (needsTerrainSample(cached, v.lat, v.lon, now)) {
              sampleTerrain(Cesium, viewer, cache, sid, v.lat, v.lon);
            }

            // Target orientation
            const poiOri = uavModelOrientation(Cesium, poiPos, v.heading, v.roll, v.pitch);

            // Advance the animation from where the model is drawn now; the
            // sample is over home, the ground is added by the animation.
            const last = samples[sid];
            const restart = fresh && stale.has(sid);
            if (!last || last.v !== v || restart) {
              const samplePos = Cesium.Cartesian3.fromDegrees(v.lon, v.lat, altRel);
              anim[sid] = advanceAnim(restart ? undefined : anim[sid], samplePos, poiOri, Cesium, now);
              if (restart) stale.delete(sid);
            }
            if (!last || last.terrainH !== terrainH || restart) anim[sid] = setGround(anim[sid], terrainH, now);
            samples[sid] = { v, terrainH };

            if (existing[sid]) {
              existing[sid].label.text = v.name || `UAV ${sid}`;
            } else {
              // Per-entity scratch variables to avoid allocations in 60 fps callbacks
              const scratchPos = new Cesium.Cartesian3();
              const scratchOri = new Cesium.Quaternion();

              const e = viewer.entities.add({
                position: new Cesium.CallbackProperty(() => {
                  const a = anim[sid];
                  return a ? positionAt(a, Cesium, scratchPos) : scratchPos;
                }, false),
                orientation: new Cesium.CallbackProperty(() => {
                  const a = anim[sid];
                  return a ? orientationAt(a, Cesium, scratchOri) : scratchOri;
                }, false),
                model: {
                  uri: '/Peron15.glb',
                  scale: 1.0,
                  minimumPixelSize: 32,
                  maximumScale: 20,
                  color: Cesium.Color.fromCssColorString(colorStr),
                  colorBlendMode: Cesium.ColorBlendMode.MIX,
                  colorBlendAmount: 0.7,
                  silhouetteColor: Cesium.Color.fromCssColorString(colorStr),
                  silhouetteSize: 2.0,
                },
                label: {
                  text: v.name || `UAV ${sid}`,
                  font: '11px sans-serif',
                  fillColor: Cesium.Color.WHITE,
                  outlineColor: Cesium.Color.BLACK,
                  outlineWidth: 2,
                  style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                  verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
                  pixelOffset: new Cesium.Cartesian2(0, -20),
                  scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
                  pixelOffsetScaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
                  disableDepthTestDistance: Number.POSITIVE_INFINITY,
                },
              });
              existing[sid] = e;
            }
        });
      }

      // Remove entities for vehicles no longer in the list
      for (const sid of Object.keys(existing)) {
        if (!seen.has(Number(sid))) {
          try { viewer.entities.remove(existing[sid]); } catch {}
          delete existing[sid];
          delete cache[sid];
          delete anim[sid];
          delete samples[sid];
          stale.delete(Number(sid));
        }
      }

      ents.uavMap = existing;
      viewer.scene.requestRender();
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady]);
}
