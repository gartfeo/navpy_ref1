import { useEffect, useRef } from 'react';
import { addTrailPoint, newTrail, startRun, trailGrounds, trailHead, trailPieceFlat, trailPieces, trailUpdatePoints, TRAIL_SEGMENTS } from '../utils/trailAccum';
import { zoneColorsSolid } from '../../../styles';

/**
 * Render the flight-path trace behind each UAV.
 *
 * Records lon/lat/alt history for every UAV in the fleet, shown or hidden
 * (older points are thinned, see trailAccum): every fix of each update's
 * `path`, or the current position of an update without one (trailUpdatePoints). Only fresh telemetry records
 * (store.isTelemetryFresh): cached positions held over a link gap never do,
 * and the first fresh position after a gap opens a new run. `paused` stops
 * recording for the whole fleet; resuming opens a new run. Each trace uses
 * the UAV's colour (same palette/index as its zone) and fades from faint
 * (tail) to solid (head). `show` (layer) and `hiddenSysIds` (per UAV) only
 * hide; a change of `resetKey` clears all history.
 *
 * Heights are over home, drawn over the ground under each point, as the
 * vehicle model is drawn (terrain + alt_rel): the ellipsoid (0) until
 * `terrainReady`, then the terrain sampled at the point. A point not sampled
 * yet takes the last known ground before it; a trail with no known ground is
 * not drawn.
 *
 * The trail grows only at the 5 Hz updates while the model (useUavMarkers)
 * moves on every frame, so a live segment joins the trail's newest fix to
 * the model's drawn position. It is a polyline whose positions are a
 * CallbackProperty, which Cesium draws through a PolylineCollection and
 * re-reads every frame, as the model's position is; nothing is stored. Its
 * start is the head as the trail draws it, its end the model's own position
 * property, so both ends keep their own height rule. It is not drawn while
 * the current run is empty, the UAV's telemetry is not fresh or recording
 * is paused (the trail does not grow then, so it would bridge a gap). When
 * the UAV stops, the model comes to rest on its newest fix and the segment
 * shrinks to nothing.
 */
export default function useUavTrails(
  cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady,
  show = true, resetKey = 0, hiddenSysIds = null, paused = false,
  terrainReady = false,
) {
  const trailDataRef = useRef({});
  const staleRef = useRef(new Set());
  // sys_id -> the `path` array last recorded, so a kept path is not re-added.
  const usedPathRef = useRef({});
  const visRef = useRef({ show, hiddenSysIds });
  visRef.current = { show, hiddenSysIds };
  const pausedRef = useRef(paused);
  pausedRef.current = paused;
  const terrainReadyRef = useRef(terrainReady);
  terrainReadyRef.current = terrainReady;
  // Bumped when terrain comes or goes and on reset: older samples are stale.
  const terrainGenRef = useRef(0);
  // Redraws one UAV's trail and samples its ground; set by the drawing effect.
  const refreshRef = useRef(null);

  const visibleFor = (sid) => visRef.current.show && !visRef.current.hiddenSysIds?.has(sid);

  // Clear accumulated history (Clear all / mission restart).
  useEffect(() => {
    terrainGenRef.current += 1;
    const trailData = trailDataRef.current;
    for (const sid of Object.keys(trailData)) trailData[sid] = newTrail();
    const trailMap = entitiesRef.current?.trailMap || {};
    for (const pool of Object.values(trailMap)) {
      for (const segEnts of pool.segs) for (const e of segEnts) e.polyline.positions = [];
      pool.head = null;
    }
  }, [resetKey]);

  // Resume opens a new run for every UAV.
  useEffect(() => {
    if (paused) return;
    for (const trail of Object.values(trailDataRef.current)) startRun(trail);
  }, [paused]);

  // Terrain came or went: every trail moves to the new ground.
  useEffect(() => {
    terrainGenRef.current += 1;
    for (const [sid, trail] of Object.entries(trailDataRef.current)) {
      trail.lastGround = undefined;
      trail.sampling = null;
      for (const run of trail.runs) for (const p of run) p.ground = undefined;
      refreshRef.current?.(Number(sid));
    }
  }, [terrainReady]);

  // Show/hide without losing history.
  useEffect(() => {
    const trailMap = entitiesRef.current?.trailMap || {};
    for (const [sid, pool] of Object.entries(trailMap)) {
      const vis = visibleFor(Number(sid));
      for (const segEnts of pool.segs) for (const e of segEnts) e.show = vis;
      if (pool.live) pool.live.show = vis;
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

    const groundOf = (p) => (terrainReadyRef.current ? p.ground : 0);

    /**
     * Sample the terrain under every point of a trail not sampled yet; one
     * request at a time per trail. Points added meanwhile go in the next,
     * sent when this one fully succeeds; after a failure the next update
     * samples again.
     */
    function sampleGround(sid, trail) {
      const generation = terrainGenRef.current;
      if (!terrainReadyRef.current || trail.sampling === generation) return;
      const pts = [];
      for (const run of trail.runs) for (const p of run) if (!Number.isFinite(p.ground)) pts.push(p);
      if (pts.length === 0) return;
      trail.sampling = generation;
      const cartos = pts.map((p) => Cesium.Cartographic.fromDegrees(p.lon, p.lat));
      const apply = (sampled) => {
        // Cleared, reset or older terrain: stale.
        if (terrainGenRef.current !== generation || trailDataRef.current[sid] !== trail) return;
        trail.sampling = null;
        let complete = true;
        pts.forEach((p, i) => {
          // A rejected or non-finite sample stays unsampled.
          const height = sampled?.[i]?.height;
          if (!Number.isFinite(height)) { complete = false; return; }
          p.ground = height;
          trail.lastGround = height;
        });
        draw(sid);
        if (complete) sampleGround(sid, trail);
      };
      Promise.resolve()
        .then(() => Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, cartos))
        .then(apply, () => apply(undefined));
    }

    function draw(sid) {
      const pool = entitiesRef.current.trailMap?.[sid];
      const trail = trailDataRef.current[sid];
      if (!pool || !trail) return;
      const lastGround = terrainReadyRef.current ? trail.lastGround : 0;
      const grounds = trailGrounds(trail.runs, groundOf, lastGround);
      const pieces = trailPieces(trail.runs);
      for (let seg = 0; seg < TRAIL_SEGMENTS; seg++) {
        const segPieces = pieces[seg];
        const segEnts = pool.segs[seg];
        while (segEnts.length < segPieces.length) addPieceEntity(pool, seg, sid);
        for (let k = 0; k < segEnts.length; k++) {
          const flat = segPieces[k] ? trailPieceFlat(segPieces[k], grounds) : [];
          segEnts[k].polyline.positions = flat.length ? Cesium.Cartesian3.fromDegreesArrayHeights(flat) : [];
        }
      }
      // The live segment starts where the trail is drawn to.
      const head = trailHead(trail);
      const ground = head ? grounds.get(head) : undefined;
      pool.head = ground === undefined ? null
        : { point: head, pos: Cesium.Cartesian3.fromDegrees(head.lon, head.lat, ground + head.alt) };
    }

    /**
     * The live segment of a UAV, read every frame: its trail's drawn head to
     * its model's drawn position. Empty (not drawn) unless the head is the
     * newest point of the current run, telemetry is fresh, recording runs
     * and the model exists.
     */
    function addLiveEntity(pool, sid) {
      const modelPos = new Cesium.Cartesian3();
      const positions = [null, modelPos];
      const none = [];
      pool.live = viewer.entities.add({
        show: visibleFor(sid),
        polyline: {
          positions: new Cesium.CallbackProperty((time) => {
            const head = pool.head;
            const trail = trailDataRef.current[sid];
            if (!head || !trail || trailHead(trail) !== head.point || pausedRef.current) return none;
            const store = storeRef.current;
            if (store.isTelemetryFresh && !store.isTelemetryFresh(sid)) return none;
            const model = entitiesRef.current.uavMap?.[sid]?.position;
            // The model's callback returns its own scratch value: copy it.
            const pos = model?.getValue(time, modelPos);
            if (!pos) return none;
            Cesium.Cartesian3.clone(pos, modelPos);
            positions[0] = head.pos;
            return positions;
          }, false),
          width: 2,
          material: pool.base,
          clampToGround: false,
        },
      });
    }
    refreshRef.current = (sid) => {
      draw(sid);
      const trail = trailDataRef.current[sid];
      if (trail) sampleGround(sid, trail);
    };

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
          const sid = v.sys_id;
          // No position is a gap: the next fresh position starts a new run.
          if (v.lat == null || v.lon == null) { stale.add(sid); return; }
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
            const points = trailUpdatePoints(v, usedPathRef.current[sid]);
            usedPathRef.current[sid] = v.path;
            if (!pausedRef.current) {
              for (const pt of points) addTrailPoint(trail, pt);
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
            // Solid, as the trail's head piece.
            if (pool.live) pool.live.polyline.material = pool.base;
          }
          if (!pool.live) addLiveEntity(pool, sid);

          ents.trailMap = trailMap;
          draw(sid);
          sampleGround(sid, trail);
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
          try { viewer.entities.remove(trailMap[sid].live); } catch {}
          delete trailMap[sid];
          delete trailData[sid];
          delete usedPathRef.current[sid];
          stale.delete(Number(sid));
        }
      }

      ents.trailMap = trailMap;
    }

    update();
    const unsubscribe = storeRef.current.subscribe(update);
    return () => {
      unsubscribe();
      refreshRef.current = null;
      terrainGenRef.current += 1;
    };
  }, [viewerReady]);
}
