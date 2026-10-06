import { useEffect, useRef } from 'react';
import { accumulate } from '../utils/trailAccum';
import { zoneColorsSolid } from '../../../styles';

/** Number of polyline segments per trail for the fade gradient. */
const TRAIL_SEGMENTS = 5;

/**
 * Render the flight-path trace behind each UAV.
 *
 * Accumulates lon/lat/alt history from vehicleList telemetry for the whole
 * flight (older points are thinned, see trailAccum). Each trace uses the
 * UAV's colour (same palette/index as its zone) and fades from faint (tail)
 * to solid (head) via graduated-alpha segments. `show` hides the trace
 * without dropping history; a change of `resetKey` clears all history.
 */
export default function useUavTrails(cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady, show = true, resetKey = 0) {
  const trailDataRef = useRef({});
  const showRef = useRef(show);
  showRef.current = show;

  // Clear accumulated history (Clear button / mission restart).
  useEffect(() => {
    const trailData = trailDataRef.current;
    for (const sid of Object.keys(trailData)) trailData[sid] = [];
    const trailMap = entitiesRef.current?.trailMap || {};
    for (const segs of Object.values(trailMap)) {
      for (const e of segs) e.polyline.positions = [];
    }
  }, [resetKey]);

  // Show/hide without losing history.
  useEffect(() => {
    const trailMap = entitiesRef.current?.trailMap || {};
    for (const segs of Object.values(trailMap)) {
      for (const e of segs) e.show = show;
    }
  }, [show, viewerReady]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    function update() {
      const vehicleList = storeRef.current.getVehicleList();
      const ents = entitiesRef.current;
      const trailMap = ents.trailMap || {};
      const trailData = trailDataRef.current;
      const seen = new Set();

      if (vehicleList && vehicleList.length > 0) {
        vehicleList.forEach((v, vIdx) => {
          if (v.lat == null || v.lon == null) return;
          seen.add(v.sys_id);
          const sid = v.sys_id;
          const alt = v.alt_rel ?? v.alt ?? 0;

          if (!trailData[sid]) trailData[sid] = [];
          accumulate(trailData[sid], { lon: v.lon, lat: v.lat, alt });

          // Colour follows the UAV's current list index (same as its zone);
          // re-apply when the index changes, e.g. a lower sys_id connects.
          const colorIdx = vIdx % zoneColorsSolid.length;
          const base = Cesium.Color.fromCssColorString(zoneColorsSolid[colorIdx]);
          if (!trailMap[sid]) {
            const segEntities = [];
            for (let seg = 0; seg < TRAIL_SEGMENTS; seg++) {
              const alpha = (seg + 1) / TRAIL_SEGMENTS;
              const segColor = base.withAlpha(alpha);
              const e = viewer.entities.add({
                show: showRef.current,
                polyline: {
                  positions: [],
                  width: 2,
                  material: segColor,
                  clampToGround: false,
                },
              });
              segEntities.push(e);
            }
            segEntities.colorIdx = colorIdx;
            trailMap[sid] = segEntities;
          } else if (trailMap[sid].colorIdx !== colorIdx) {
            trailMap[sid].forEach((e, seg) => {
              e.polyline.material = base.withAlpha((seg + 1) / TRAIL_SEGMENTS);
            });
            trailMap[sid].colorIdx = colorIdx;
          }

          const pts = trailData[sid];
          const segEntities = trailMap[sid];
          const n = pts.length;
          for (let seg = 0; seg < TRAIL_SEGMENTS; seg++) {
            const start = Math.floor(seg * n / TRAIL_SEGMENTS);
            const end = Math.min(Math.floor((seg + 1) * n / TRAIL_SEGMENTS) + 1, n);
            let positions = [];
            if (end - start >= 2) {
              const flat = [];
              for (let i = start; i < end; i++) {
                const p = pts[i];
                flat.push(p.lon, p.lat, p.alt);
              }
              positions = Cesium.Cartesian3.fromDegreesArrayHeights(flat);
            }
            segEntities[seg].polyline.positions = positions;
          }
        });
      }

      // Remove trails for disconnected vehicles
      for (const sid of Object.keys(trailMap)) {
        if (!seen.has(Number(sid))) {
          const segs = trailMap[sid];
          if (Array.isArray(segs)) {
            for (const e of segs) {
              try { viewer.entities.remove(e); } catch {}
            }
          }
          delete trailMap[sid];
          delete trailData[sid];
        }
      }

      ents.trailMap = trailMap;
    }

    update();
    return storeRef.current.subscribe(update);
  }, [viewerReady]);
}
