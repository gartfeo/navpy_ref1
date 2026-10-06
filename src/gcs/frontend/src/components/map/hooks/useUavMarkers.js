import { useEffect, useRef } from 'react';
import { zoneColorsSolid } from '../../../styles';
import { needsTerrainSample, sampleTerrain } from '../utils/terrainCache';
import { lerpPosition, slerpOrientation, advanceAnim } from '../utils/lerpAnim';

/**
 * 3D UAV vehicle models with smooth interpolated motion.
 *
 * Each entity uses CallbackProperty for position/orientation so Cesium
 * evaluates a lerp/slerp every render frame (~60 fps), producing smooth
 * movement between 5 Hz telemetry updates.
 *
 * Altitude uses terrain-sampled height + relative (AGL) altitude.
 * heightReference is incompatible with explicit orientation quaternions
 * (causes model to shift when camera rotates as terrain LOD changes).
 */
export default function useUavMarkers(cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady) {
  const terrainCacheRef = useRef({});
  const animRef = useRef({});

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
      const now = Date.now();

      if (vehicleList && vehicleList.length > 0) {
        vehicleList.forEach((v, i) => {
          if (v.lat == null || v.lon == null) return;
          seen.add(v.sys_id);
          const sid = v.sys_id;
          const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];

            // Target position: cached terrain height + AGL altitude
            const cached = cache[sid];
            const terrainH = cached?.terrainHeight ?? 0;
            const altRel = v.alt_rel ?? v.alt ?? 0;
            const targetPos = Cesium.Cartesian3.fromDegrees(v.lon, v.lat, terrainH + altRel);

            // Async terrain sample when cache is missing, stale, or UAV moved
            if (needsTerrainSample(cached, v.lat, v.lon, now)) {
              sampleTerrain(Cesium, viewer, cache, sid, v.lat, v.lon);
            }

            // Target orientation
            const headingRad = v.heading != null ? Cesium.Math.toRadians(v.heading) : 0;
            const modelPitch = v.roll != null ? -Cesium.Math.toRadians(v.roll) : 0;
            const modelRoll = v.pitch != null ? Cesium.Math.toRadians(v.pitch) : 0;
            const vehicleHpr = new Cesium.HeadingPitchRoll(headingRad, modelPitch, modelRoll);
            const vehicleQuat = Cesium.Transforms.headingPitchRollQuaternion(targetPos, vehicleHpr);

            const modelFixHpr = new Cesium.HeadingPitchRoll(-Cesium.Math.PI, Cesium.Math.PI_OVER_TWO, 0);
            const modelFixQuat = Cesium.Quaternion.fromHeadingPitchRoll(modelFixHpr);
            const targetOri = Cesium.Quaternion.multiply(
              vehicleQuat, modelFixQuat, new Cesium.Quaternion()
            );

            // Advance animation: seamless transition from current interpolated state
            anim[sid] = advanceAnim(anim[sid], targetPos, targetOri, Cesium);

            if (existing[sid]) {
              existing[sid].label.text = v.name || `UAV ${sid}`;
            } else {
              // Per-entity scratch variables to avoid allocations in 60 fps callbacks
              const scratchPos = new Cesium.Cartesian3();
              const scratchOri = new Cesium.Quaternion();

              const e = viewer.entities.add({
                position: new Cesium.CallbackProperty(() => {
                  const a = anim[sid];
                  return a ? lerpPosition(a, Cesium, scratchPos) : scratchPos;
                }, false),
                orientation: new Cesium.CallbackProperty(() => {
                  const a = anim[sid];
                  return a ? slerpOrientation(a, Cesium, scratchOri) : scratchOri;
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
        }
      }

      ents.uavMap = existing;
      viewer.scene.requestRender();
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady]);
}
