import { useEffect, useRef } from 'react';
import { zoneColorsSolid } from '../../../styles';
import { needsTerrainSample, sampleTerrain } from '../utils/terrainCache';
import {
  attitudeSample, groundAt, groundFading, ingestVehicle, playClock, playoutMoving, playoutsOf,
  positionSample, prunePlayouts, setGround,
} from '../utils/uavPlayout';
import { backendNowMs } from '../../../utils/backendClock';
import { poseFinite } from '../../../utils/fpvView';
import { uavModelOrientation } from '../utils/uavOrientation';

/**
 * 3D UAV vehicle models with smooth motion.
 *
 * Each entity uses CallbackProperty for position/orientation so Cesium
 * evaluates the motion every render frame. The motion is a playout
 * (uavPlayout): every position fix of the 5 Hz telemetry is drawn at its own
 * time, a fixed delay behind the newest, by interpolation, so the model only
 * moves forward along the flown fixes and the trail ends at it
 * (useUavTrails). The model and the trail take each snapshot into one shared
 * playout per UAV, whichever sees it first. A new ground height under the
 * UAV only moves its ground (setGround), never its speed. The first sample
 * after a gap (the UAV's telemetry was not fresh, or it had no position)
 * starts a new playout at that sample, as the trail starts a new run there:
 * the model does not glide across the gap. Until then the sys_id is in
 * entitiesRef.current.uavGap.
 *
 * The viewer renders on request (requestRenderMode, no render on time
 * alone), and each store update requests one. Between updates the model
 * moves with no update to show it, so every frame checks (scene.preUpdate)
 * whether a model is still moving or its ground fading and then requests the
 * next render, and one more once it is at rest. Without this the map drew
 * the models only at the 5 Hz updates (todo 267).
 *
 * Altitude uses terrain-sampled height + relative (AGL) altitude.
 * heightReference is incompatible with explicit orientation quaternions
 * (causes model to shift when camera rotates as terrain LOD changes).
 *
 * A UAV gets its model with its first snapshot whose lat, lon, altitude,
 * heading, roll and pitch are finite numbers. Afterwards a fix or an
 * attitude that is not finite is skipped: the model keeps its last valid
 * position or attitude and goes on from it (no 0 fill, which used to drop a
 * UAV without altitude to the terrain).
 */
export default function useUavMarkers(cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady) {
  const terrainCacheRef = useRef({});
  // sys_id -> the ground height last given to its playout.
  const groundRef = useRef({});

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
      const grounds = groundRef.current;
      const store = storeRef.current;
      const now = Date.now();
      const ts = backendNowMs();

      if (vehicleList && vehicleList.length > 0) {
        vehicleList.forEach((v, i) => {
          const sid = v.sys_id;
          const fresh = store.isTelemetryFresh ? store.isTelemetryFresh(sid) : true;
          const pb = ingestVehicle(ents, v, fresh, ts, now);
          if (!existing[sid] && !poseFinite(v)) return;
          seen.add(sid);
          if (!pb) return;
          const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];

          // Async terrain sample when cache is missing, stale, or UAV moved
          const cached = cache[sid];
          if (Number.isFinite(v.lat) && Number.isFinite(v.lon) && needsTerrainSample(cached, v.lat, v.lon, now)) {
            sampleTerrain(Cesium, viewer, cache, sid, v.lat, v.lon);
          }
          const terrainH = cached?.terrainHeight ?? 0;
          if (!pb.ground || grounds[sid] !== terrainH) {
            setGround(pb, terrainH, now);
            grounds[sid] = terrainH;
          }

          if (existing[sid]) {
            existing[sid].label.text = v.name || `UAV ${sid}`;
            return;
          }
          // Per-entity scratch variables to avoid allocations in per-frame callbacks
          const scratchPos = new Cesium.Cartesian3();
          const scratchOri = new Cesium.Quaternion();
          // The current playout of sid (a gap replaces it).
          const playout = () => playoutsOf(entitiesRef.current)[sid];
          const drawnPosition = (p, t) => {
            const s = positionSample(p, playClock(p, t));
            return s && Cesium.Cartesian3.fromDegrees(s.lon, s.lat, s.alt + groundAt(p, t), undefined, scratchPos);
          };

          existing[sid] = viewer.entities.add({
            position: new Cesium.CallbackProperty(() => {
              const p = playout();
              return (p && drawnPosition(p, Date.now())) || scratchPos;
            }, false),
            orientation: new Cesium.CallbackProperty(() => {
              const t = Date.now();
              const p = playout();
              const a = p && attitudeSample(p, playClock(p, t));
              const pos = a && drawnPosition(p, t);
              if (!pos) return scratchOri;
              return Cesium.Quaternion.clone(uavModelOrientation(Cesium, pos, a.heading, a.roll, a.pitch), scratchOri);
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
        });
      }

      // Remove entities for vehicles no longer in the list
      for (const sid of Object.keys(existing)) {
        if (!seen.has(Number(sid))) {
          try { viewer.entities.remove(existing[sid]); } catch {}
          delete existing[sid];
          delete cache[sid];
          delete grounds[sid];
        }
      }
      prunePlayouts(ents, new Set((vehicleList || []).map((v) => v.sys_id)));

      ents.uavMap = existing;
      // A model still in a gap is not joined to the trail (useUavTrails).
      const playouts = playoutsOf(ents);
      ents.uavGap = new Set(Object.keys(existing).map(Number).filter((sid) => !playouts[sid] || playouts[sid].gap));
      viewer.scene.requestRender();
    }

    // Render while a model moves (the trail's live segment follows it), and
    // once when it comes to rest, in case frames skipped its last move.
    const restDrawn = new WeakSet();
    function onPreUpdate() {
      const now = Date.now();
      let render = false;
      for (const p of Object.values(playoutsOf(entitiesRef.current))) {
        if (playoutMoving(p, now) || groundFading(p, now)) {
          restDrawn.delete(p);
          render = true;
        } else if (!restDrawn.has(p)) {
          restDrawn.add(p);
          render = true;
        }
      }
      if (render) viewer.scene.requestRender();
    }

    update();
    const unsubscribe = storeRef.current.subscribe(update);
    // The remover holds the event itself, so it works after the viewer is destroyed.
    const removePreUpdate = viewer.scene.preUpdate.addEventListener(onPreUpdate);
    return () => {
      unsubscribe();
      removePreUpdate();
    };
  }, [viewerReady]);
}
