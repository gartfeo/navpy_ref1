import { useEffect, useRef } from 'react';
import { addTrailPoint, newTrail, startRun, trailGrounds, trailPieceFlat, trailPieces, TRAIL_SEGMENTS } from '../utils/trailAccum';
import { ingestVehicle, playClock, playoutsOf, prunePlayouts } from '../utils/uavPlayout';
import { backendNowMs } from '../../../utils/backendClock';
import { zoneColorsSolid } from '../../../styles';

/**
 * Render the flight-path trace behind each UAV.
 *
 * Records lon/lat/alt history for every UAV in the fleet, shown or hidden
 * (older points are thinned, see trailAccum): every fix the UAV's playout
 * (uavPlayout, shared with the model) takes from an update, that is every
 * fix of its `path`, or the current position of an update without one, each
 * with its playout time. A fix the playout marks as a jump (a SITL restart)
 * opens a new run. Only fresh telemetry records
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
 * The model (useUavMarkers) is drawn a playout delay behind the newest fix,
 * so the trail is drawn only up to it: an update draws the current run's
 * fixes the playout clock has passed (the drawn head), and a live segment
 * joins the drawn head through the fixes the clock passes after it to the
 * model's drawn position, every frame. Neither end ever runs ahead of the
 * model or back from it (todo 267). The live segment is a polyline whose
 * positions are a CallbackProperty, which Cesium draws through a
 * PolylineCollection and re-reads every frame, as the model's position is;
 * nothing is stored. Its fixes take the trail's grounds, its end the model's
 * own position property. It is not drawn while the current run has no drawn
 * fix, the UAV's telemetry is not fresh, recording is paused (the trail does
 * not grow then, so it would bridge a gap) or the model still holds its pose
 * from before a gap (useUavMarkers); then, as with no model, the whole run is
 * drawn. When the UAV stops, the model comes to rest on its newest fix and
 * the segment ends there.
 *
 * The recorded pieces are polylines of one PolylineCollection per UAV, not
 * entities: Cesium builds a changed static entity polyline in the
 * background and shows the old one for some frames meanwhile, so the live
 * segment, which moves on at once, left a blinking gap at the trail's head
 * on every update (todo 293). A collection's polyline shows its new
 * positions in the next frame; each change requests that frame.
 */
/** Set a trail piece's positions; one with fewer than two is not drawn. */
function setPiece(line, positions, visible) {
  line.positions = positions;
  line.show = visible && positions.length >= 2;
}

export default function useUavTrails(
  cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady,
  show = true, resetKey = 0, hiddenSysIds = null, paused = false,
  terrainReady = false,
) {
  const trailDataRef = useRef({});
  const staleRef = useRef(new Set());
  // sys_id -> id of the newest playout fix taken, so a fix is recorded once.
  const takenRef = useRef({});
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
      for (const segLines of pool.segs) for (const line of segLines) setPiece(line, [], false);
      pool.head = null;
    }
    viewerRef.current?.scene?.requestRender?.();
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
      for (const segLines of pool.segs) for (const line of segLines) setPiece(line, line.positions, vis);
      if (pool.live) pool.live.show = vis;
    }
    viewerRef.current?.scene?.requestRender?.();
  }, [show, hiddenSysIds, viewerReady]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const pieceMaterial = (pool, seg) => Cesium.Material.fromType('Color', {
      color: pool.base.withAlpha((seg + 1) / TRAIL_SEGMENTS),
    });

    function addPiece(pool, seg) {
      const line = pool.lines.add({ positions: [], show: false, width: 2, material: pieceMaterial(pool, seg) });
      pool.segs[seg].push(line);
      return line;
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

    /**
     * Whether the live segment joins sid's trail to its model now: a model
     * out of any gap, fresh telemetry, recording on. Else the whole run is
     * drawn and the segment is not.
     */
    function joinsModel(sid) {
      const ents = entitiesRef.current;
      if (!ents.uavMap?.[sid] || ents.uavGap?.has(sid) || pausedRef.current) return false;
      const store = storeRef.current;
      return !store.isTelemetryFresh || store.isTelemetryFresh(sid);
    }

    function draw(sid) {
      const pool = entitiesRef.current.trailMap?.[sid];
      const trail = trailDataRef.current[sid];
      if (!pool || !trail) return;
      const lastGround = terrainReadyRef.current ? trail.lastGround : 0;
      const grounds = trailGrounds(trail.runs, groundOf, lastGround);
      // Draw the current run to the fixes the model has passed; the live
      // segment draws the rest as it passes them.
      const run = trail.runs[trail.runs.length - 1];
      const playout = playoutsOf(entitiesRef.current)[sid];
      const clock = joinsModel(sid) && playout ? playClock(playout, Date.now()) : null;
      let cut = run.length;
      if (clock !== null) while (cut > 0 && run[cut - 1].t > clock) cut--;
      const runs = cut === run.length ? trail.runs : [...trail.runs.slice(0, -1), run.slice(0, cut)];
      const pieces = trailPieces(runs);
      const vis = visibleFor(sid);
      for (let seg = 0; seg < TRAIL_SEGMENTS; seg++) {
        const segPieces = pieces[seg];
        const segLines = pool.segs[seg];
        while (segLines.length < segPieces.length) addPiece(pool, seg);
        for (let k = 0; k < segLines.length; k++) {
          const flat = segPieces[k] ? trailPieceFlat(segPieces[k], grounds) : [];
          setPiece(segLines[k], flat.length ? Cesium.Cartesian3.fromDegreesArrayHeights(flat) : [], vis);
        }
      }
      viewer.scene.requestRender();
      // The live segment starts where the trail is drawn to, and goes on
      // through the fixes after it.
      const at = (p) => {
        const ground = grounds.get(p);
        return ground === undefined ? null : Cesium.Cartesian3.fromDegrees(p.lon, p.lat, ground + p.alt);
      };
      const head = cut > 0 ? run[cut - 1] : null;
      const headPos = head && at(head);
      pool.head = headPos ? { point: head, run, pos: headPos } : null;
      pool.pending = [];
      for (let k = cut; k < run.length && pool.head; k++) {
        const pos = at(run[k]);
        if (pos) pool.pending.push({ t: run[k].t, pos });
      }
    }

    /**
     * The live segment of a UAV, read every frame: its trail's drawn head,
     * the fixes after it the model has passed, and the model's drawn
     * position. Empty (not drawn) unless the head is in the current run and
     * the segment joins the model (joinsModel).
     */
    function addLiveEntity(pool, sid) {
      const modelPos = new Cesium.Cartesian3();
      const positions = [];
      const none = [];
      pool.live = viewer.entities.add({
        show: visibleFor(sid),
        polyline: {
          positions: new Cesium.CallbackProperty((time) => {
            const head = pool.head;
            const trail = trailDataRef.current[sid];
            if (!head || !trail || trail.runs[trail.runs.length - 1] !== head.run || !joinsModel(sid)) return none;
            const model = entitiesRef.current.uavMap[sid].position;
            // The model's callback returns its own scratch value: copy it.
            const pos = model.getValue(time, modelPos);
            if (!pos) return none;
            Cesium.Cartesian3.clone(pos, modelPos);
            // The clock the model was just drawn at (one clock per page ms).
            const playout = playoutsOf(entitiesRef.current)[sid];
            const clock = (playout && playClock(playout, Date.now())) ?? -Infinity;
            positions.length = 0;
            positions.push(head.pos);
            for (const p of pool.pending) {
              if (p.t > clock) break;
              positions.push(p.pos);
            }
            positions.push(modelPos);
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
          const playout = ingestVehicle(ents, v, fresh, backendNowMs());
          if (!fresh) {
            stale.add(sid);
          } else {
            if (stale.has(sid)) {
              startRun(trail);
              stale.delete(sid);
            }
            const taken = takenRef.current[sid] ?? 0;
            for (const f of playout?.fixes || []) {
              if (f.id <= taken) continue;
              takenRef.current[sid] = f.id;
              if (!f.trail || pausedRef.current) continue;
              if (f.jump) startRun(trail);
              addTrailPoint(trail, { lon: f.lon, lat: f.lat, alt: f.alt, t: f.t });
            }
          }

          // Colour follows the UAV's current list index (same as its zone);
          // re-apply when the index changes, e.g. a lower sys_id connects.
          const colorIdx = vIdx % zoneColorsSolid.length;
          if (!trailMap[sid]) {
            trailMap[sid] = {
              colorIdx: -1,
              base: null,
              lines: viewer.scene.primitives.add(new Cesium.PolylineCollection()),
              segs: Array.from({ length: TRAIL_SEGMENTS }, () => []),
            };
          }
          const pool = trailMap[sid];
          if (pool.colorIdx !== colorIdx) {
            pool.base = Cesium.Color.fromCssColorString(zoneColorsSolid[colorIdx]);
            pool.colorIdx = colorIdx;
            pool.segs.forEach((segLines, seg) => {
              for (const line of segLines) line.material = pieceMaterial(pool, seg);
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
          // Removing the collection destroys it with its pieces.
          viewer.scene.primitives.remove(trailMap[sid].lines);
          try { viewer.entities.remove(trailMap[sid].live); } catch {}
          delete trailMap[sid];
          delete trailData[sid];
          delete takenRef.current[sid];
          stale.delete(Number(sid));
        }
      }

      prunePlayouts(ents, seen);
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
