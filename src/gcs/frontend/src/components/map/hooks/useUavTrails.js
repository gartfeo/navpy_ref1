import { useEffect, useRef } from 'react';
import { addTrailPoint, newTrail, startRun, trailPieces, TRAIL_SEGMENTS } from '../utils/trailAccum';
import { zoneColorsSolid } from '../../../styles';

/**
 * Render the flight-path trace behind each UAV.
 *
 * Records lon/lat/alt history for every UAV in the fleet, shown or hidden
 * (older points are thinned, see trailAccum). Only fresh telemetry records
 * (store.isTelemetryFresh): cached positions held over a link gap never do,
 * and the first fresh position after a gap opens a new run. `paused` stops
 * recording for the whole fleet; resuming opens a new run. Each trace uses
 * the UAV's colour (same palette/index as its zone) and fades from faint
 * (tail) to solid (head). `show` (layer) and `hiddenSysIds` (per UAV) only
 * hide; a change of `resetKey` clears all history.
 */
export default function useUavTrails(
  cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady,
  show = true, resetKey = 0, hiddenSysIds = null, paused = false,
) {
  const trailDataRef = useRef({});
  const staleRef = useRef(new Set());
  const visRef = useRef({ show, hiddenSysIds });
  visRef.current = { show, hiddenSysIds };
  const pausedRef = useRef(paused);
  pausedRef.current = paused;

  const visibleFor = (sid) => visRef.current.show && !visRef.current.hiddenSysIds?.has(sid);

  // Clear accumulated history (Clear all / mission restart).
  useEffect(() => {
    const trailData = trailDataRef.current;
    for (const sid of Object.keys(trailData)) trailData[sid] = newTrail();
    const trailMap = entitiesRef.current?.trailMap || {};
    for (const pool of Object.values(trailMap)) {
      for (const segEnts of pool.segs) for (const e of segEnts) e.polyline.positions = [];
    }
  }, [resetKey]);

  // Resume opens a new run for every UAV.
  useEffect(() => {
    if (paused) return;
    for (const trail of Object.values(trailDataRef.current)) startRun(trail);
  }, [paused]);

  // Show/hide without losing history.
  useEffect(() => {
    const trailMap = entitiesRef.current?.trailMap || {};
    for (const [sid, pool] of Object.entries(trailMap)) {
      const vis = visibleFor(Number(sid));
      for (const segEnts of pool.segs) for (const e of segEnts) e.show = vis;
    }
  }, [show, hiddenSysIds, viewerReady]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    function addPieceEntity(pool, seg, sid) {
      const e = viewer.entities.add({
        show: visibleFor(sid),
        polyline: {
          positions: [],
          width: 2,
          material: pool.base.withAlpha((seg + 1) / TRAIL_SEGMENTS),
          clampToGround: false,
        },
      });
      pool.segs[seg].push(e);
      return e;
    }

    function update() {
      const store = storeRef.current;
      const vehicleList = store.getVehicleList();
      const ents = entitiesRef.current;
      const trailMap = ents.trailMap || {};
      const trailData = trailDataRef.current;
      const stale = staleRef.current;
      const seen = new Set();

      if (vehicleList && vehicleList.length > 0) {
        vehicleList.forEach((v, vIdx) => {
          // Fleet membership, not a valid position, keeps a UAV's history.
          seen.add(v.sys_id);
          if (v.lat == null || v.lon == null) return;
          const sid = v.sys_id;
          if (!trailData[sid]) trailData[sid] = newTrail();
          const trail = trailData[sid];

          const fresh = store.isTelemetryFresh ? store.isTelemetryFresh(sid) : true;
          if (!fresh) {
            stale.add(sid);
          } else {
            if (stale.has(sid)) {
              startRun(trail);
              stale.delete(sid);
            }
            if (!pausedRef.current) {
              addTrailPoint(trail, { lon: v.lon, lat: v.lat, alt: v.alt_rel ?? v.alt ?? 0 });
            }
          }

          // Colour follows the UAV's current list index (same as its zone);
          // re-apply when the index changes, e.g. a lower sys_id connects.
          const colorIdx = vIdx % zoneColorsSolid.length;
          if (!trailMap[sid]) {
            trailMap[sid] = {
              colorIdx: -1,
              base: null,
              segs: Array.from({ length: TRAIL_SEGMENTS }, () => []),
            };
          }
          const pool = trailMap[sid];
          if (pool.colorIdx !== colorIdx) {
            pool.base = Cesium.Color.fromCssColorString(zoneColorsSolid[colorIdx]);
            pool.colorIdx = colorIdx;
            pool.segs.forEach((segEnts, seg) => {
              for (const e of segEnts) e.polyline.material = pool.base.withAlpha((seg + 1) / TRAIL_SEGMENTS);
            });
          }

          const pieces = trailPieces(trail.runs);
          for (let seg = 0; seg < TRAIL_SEGMENTS; seg++) {
            const segPieces = pieces[seg];
            const segEnts = pool.segs[seg];
            while (segEnts.length < segPieces.length) addPieceEntity(pool, seg, sid);
            for (let k = 0; k < segEnts.length; k++) {
              const pts = segPieces[k];
              if (!pts) {
                segEnts[k].polyline.positions = [];
                continue;
              }
              const flat = [];
              for (const p of pts) flat.push(p.lon, p.lat, p.alt);
              segEnts[k].polyline.positions = Cesium.Cartesian3.fromDegreesArrayHeights(flat);
            }
          }
        });
      }

      // Remove trails for vehicles that left the fleet
      for (const sid of Object.keys(trailMap)) {
        if (!seen.has(Number(sid))) {
          for (const segEnts of trailMap[sid].segs) {
            for (const e of segEnts) {
              try { viewer.entities.remove(e); } catch {}
            }
          }
          delete trailMap[sid];
          delete trailData[sid];
          stale.delete(Number(sid));
        }
      }

      ents.trailMap = trailMap;
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady]);
}
