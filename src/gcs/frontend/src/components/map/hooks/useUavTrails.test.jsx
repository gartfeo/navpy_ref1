import { act, renderHook } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import useUavTrails from './useUavTrails';

/** Minimal telemetry store: vehicles by sys_id, a fresh set, subscribers. */
function fakeStore() {
  const listeners = new Set();
  const store = {
    vehicles: new Map(),
    fresh: new Set(),
    subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    getVehicleList() { return [...store.vehicles.values()]; },
    isTelemetryFresh(sid) { return store.fresh.has(sid); },
    notify() { act(() => { for (const fn of listeners) fn(); }); },
    /** A telemetry message for one UAV (marks it fresh). */
    send(sid, lon) {
      store.vehicles.set(sid, { sys_id: sid, lat: 40, lon, alt_rel: 100 });
      store.fresh.add(sid);
      store.notify();
    },
    /** A message carrying `path` (every fix since the last one); its newest fix is lon. */
    sendPath(sid, pathLons) {
      const path = pathLons.map((l) => ({ lat: 40, lon: l, alt_rel: 100 }));
      store.vehicles.set(sid, { sys_id: sid, lat: 40, lon: pathLons.at(-1) ?? 44, alt_rel: 100, path });
      store.fresh.add(sid);
      store.notify();
    },
    disconnect() { store.fresh.clear(); store.notify(); },
    leave(sid) { store.vehicles.delete(sid); store.fresh.delete(sid); store.notify(); },
  };
  return store;
}

/** Terrain under a point: 1300 m plus 10 m per 0.001 deg of longitude east of 44. */
const terrainAt = (lon) => 1300 + Math.round((lon - 44) * 10000);

class CallbackProperty { constructor(fn) { this.fn = fn; } getValue(time, result) { return this.fn(time, result); } }

function setup(props = {}) {
  const store = fakeStore();
  const entities = { add: vi.fn((e) => e), remove: vi.fn() };
  // Terrain samples resolve only when the test calls settle(); `failing`
  // makes them reject, a lon in `holes` comes back with no height.
  const pending = [];
  const terrain = { failing: false, holes: new Set() };
  const sampleTerrainMostDetailed = vi.fn((provider, cartos) => new Promise((resolve, reject) => {
    pending.push(() => (terrain.failing ? reject(new Error('no tile'))
      : resolve(cartos.map((c) => ({ ...c, height: terrain.holes.has(c.lon) ? undefined : terrainAt(c.lon) })))));
  }));
  const cesium = { current: {
    Color: { fromCssColorString: () => ({ withAlpha: (a) => ({ a }) }) },
    Cartesian3: Object.assign(function Cartesian3() {}, {
      fromDegreesArrayHeights: (flat) => [...flat],
      fromDegrees: (lon, lat, h) => ({ lon, lat, h }),
      clone: (src, dst) => Object.assign(dst, src),
    }),
    CallbackProperty,
    // Polylines drawn as they are: a frame shows their current positions.
    PolylineCollection: class { constructor() { this.lines = []; } add(o) { const l = { ...o }; this.lines.push(l); return l; } },
    Material: { fromType: (type, uniforms) => ({ type, ...uniforms }) },
    Cartographic: { fromDegrees: (lon, lat) => ({ lon, lat }) },
    sampleTerrainMostDetailed,
  } };
  // The rendered globe answers with a coarse tile height, never a trail's ground.
  const primitives = { add: vi.fn((p) => p), remove: vi.fn() };
  const viewer = { current: { entities, terrainProvider: {}, scene: {
    globe: { getHeight: () => 999 }, primitives, requestRender: vi.fn(),
  } } };
  const entitiesRef = { current: { trailMap: {} } };
  const storeRef = { current: store };
  const hook = renderHook(
    ({ show, resetKey, hidden, paused, terrainReady }) => useUavTrails(
      cesium, viewer, entitiesRef, storeRef, true, show, resetKey, hidden, paused, terrainReady,
    ),
    { initialProps: { show: true, resetKey: 0, hidden: new Set(), paused: false, ...props } },
  );
  /** Drawn pieces for sid as lists of lon values, plus their visibility. */
  const drawn = (sid) => {
    const pool = entitiesRef.current.trailMap[sid];
    if (!pool) return { pieces: [], shown: [] };
    const ents = pool.segs.flat().filter((e) => e.positions.length > 0);
    return {
      pieces: ents.map((e) => e.positions.filter((_, i) => i % 3 === 0)),
      shown: ents.map((e) => e.show),
    };
  };
  const lons = (sid) => [...new Set(drawn(sid).pieces.flat())].sort((a, b) => a - b);
  const joins = (sid, a, b) => drawn(sid).pieces.some((p) => {
    for (let i = 1; i < p.length; i++) if (p[i - 1] === a && p[i] === b) return true;
    return false;
  });
  /**
   * Drawn [lon, height] pairs for sid, by lon. A point two pieces share must
   * be drawn at one height.
   */
  const heights = (sid) => {
    const out = new Map();
    for (const e of entitiesRef.current.trailMap[sid]?.segs.flat() || []) {
      const p = e.positions;
      for (let i = 0; i < p.length; i += 3) {
        if (out.has(p[i])) expect(p[i + 2]).toBe(out.get(p[i]));
        out.set(p[i], p[i + 2]);
      }
    }
    return [...out].sort((a, b) => a[0] - b[0]);
  };
  /** Let started terrain requests reach the mock (they start on a microtask). */
  const flush = () => act(async () => { await Promise.resolve(); });
  /** Resolve every terrain sample asked for so far. */
  const settle = async () => {
    await flush();
    await act(async () => { while (pending.length) pending.shift()(); });
    await flush();
  };
  /** Place sid's model, as useUavMarkers' position property returns it (its own scratch value). */
  const placeModel = (sid, lon, h) => {
    const scratch = {};
    entitiesRef.current.uavMap = { ...entitiesRef.current.uavMap, [sid]: { position: { getValue: () => Object.assign(scratch, { lon, lat: 40, h }) } } };
  };
  /** The live segment of sid as drawn this frame: [] or [head, model], each { lon, lat, h }. */
  const live = (sid) => {
    const e = entitiesRef.current.trailMap[sid]?.live;
    return e ? e.polyline.positions.getValue(undefined, {}).map((p) => ({ ...p })) : [];
  };
  /**
   * The recorded pieces a frame shows, as lists of lon values. Cesium builds
   * a changed static entity polyline in the background and shows the old one
   * meanwhile: an entity shows its positions of the frame before. A
   * PolylineCollection's polylines show their current positions.
   */
  const built = new Map();
  const frame = () => {
    const shown = [];
    for (const { value: e } of entities.add.mock.results) {
      if (!e.polyline || e.polyline.positions instanceof CallbackProperty) continue;
      const p = built.get(e) ?? [];
      if (e.show !== false && p.length) shown.push(p);
      built.set(e, e.polyline.positions);
    }
    for (const { value: c } of primitives.add.mock.results) {
      for (const l of c.lines) if (l.show && l.positions.length) shown.push(l.positions);
    }
    return shown.map((p) => p.filter((_, i) => i % 3 === 0));
  };
  return { store, hook, drawn, lons, joins, entities, primitives, entitiesRef, heights, flush, settle, sampleTerrainMostDetailed, terrain, placeModel, live, frame };
}

describe('useUavTrails', () => {
  it('keeps recording a UAV that is hidden by its checkbox or by the layer', () => {
    const t = setup();
    t.store.send(1, 44.000);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set([1]), paused: false });
    t.store.send(1, 44.001);
    t.hook.rerender({ show: false, resetKey: 0, hidden: new Set(), paused: false });
    t.store.send(1, 44.002);
    expect(t.drawn(1).shown.every((s) => s === false)).toBe(true);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: false });
    expect(t.lons(1)).toEqual([44.000, 44.001, 44.002]);
    expect(t.drawn(1).shown.every((s) => s === true)).toBe(true);
  });

  it('pause stops every UAV, also one that joins while paused; resume opens a new run', () => {
    const t = setup();
    t.store.send(1, 44.000);
    t.store.send(1, 44.001);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: true });
    t.store.send(1, 44.002);
    t.store.send(2, 45.000);
    t.store.send(2, 45.001);
    expect(t.lons(1)).toEqual([44.000, 44.001]);
    expect(t.lons(2)).toEqual([]);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: false });
    t.store.send(1, 44.003);
    t.store.send(1, 44.004);
    expect(t.lons(1)).toEqual([44.000, 44.001, 44.003, 44.004]);
    expect(t.joins(1, 44.001, 44.003)).toBe(false);
  });

  it('Clear all empties shown and hidden UAVs', () => {
    const t = setup({ hidden: new Set([2]) });
    t.store.send(1, 44.000); t.store.send(1, 44.001);
    t.store.send(2, 45.000); t.store.send(2, 45.001);
    t.hook.rerender({ show: true, resetKey: 1, hidden: new Set(), paused: false });
    expect(t.lons(1)).toEqual([]);
    expect(t.lons(2)).toEqual([]);
    t.store.send(1, 44.002); t.store.send(1, 44.003);
    expect(t.lons(1)).toEqual([44.002, 44.003]);
  });

  it('never joins across a link gap: the new run starts at the first fresh position', () => {
    const t = setup();
    t.store.send(1, 44.000);
    t.store.send(1, 44.001); // position A
    t.store.disconnect(); // cached A notified while stale
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: true });
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: false }); // Resume while disconnected
    t.store.notify();
    t.store.send(2, 45.000); // another UAV turns fresh; UAV 1 still cached
    expect(t.lons(1)).toEqual([44.000, 44.001]);
    t.store.send(1, 44.005); // position B
    t.store.send(1, 44.006);
    expect(t.joins(1, 44.001, 44.005)).toBe(false);
    expect(t.drawn(1).pieces.some((p) => p[0] === 44.005)).toBe(true);
  });

  it('a UAV that leaves, while hidden and paused, is removed and returns with fresh history', () => {
    const t = setup({ hidden: new Set([1]) });
    t.store.send(1, 44.000); t.store.send(1, 44.001);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set([1]), paused: true });
    t.store.leave(1);
    expect(t.entitiesRef.current.trailMap[1]).toBeUndefined();
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set([1]), paused: false });
    t.store.send(1, 44.010); t.store.send(1, 44.011);
    expect(t.lons(1)).toEqual([44.010, 44.011]);
    expect(t.drawn(1).shown.every((s) => s === false)).toBe(true);
  });

  it('a reconnect alone (no pause) starts a new run at the first fresh position', () => {
    const t = setup();
    t.store.send(1, 44.000);
    t.store.send(1, 44.001); // position A
    t.store.disconnect();
    t.store.notify(); // socket reopened: cached snapshots notified, none fresh
    t.store.send(2, 45.000); // another UAV turns fresh first
    t.store.send(1, 44.005); // position B
    t.store.send(1, 44.006);
    expect(t.joins(1, 44.001, 44.005)).toBe(false);
    expect(t.drawn(1).pieces.some((p) => p[0] === 44.005 && p.includes(44.006))).toBe(true);
  });

  it('a fleet UAV without a position keeps its history', () => {
    const t = setup();
    t.store.send(1, 44.000);
    t.store.send(1, 44.001);
    t.store.vehicles.set(1, { sys_id: 1, lat: null, lon: null });
    t.store.notify();
    expect(t.lons(1)).toEqual([44.000, 44.001]);
  });

  it('a no-position snapshot during a link gap does not let the trail bridge it', () => {
    const t = setup();
    t.store.send(1, 44.000);
    t.store.send(1, 44.001); // position A
    t.store.fresh.clear();
    t.store.vehicles.set(1, { sys_id: 1, lat: null, lon: null });
    t.store.notify();
    t.store.send(1, 44.005); // position B
    t.store.send(1, 44.006);
    expect(t.joins(1, 44.001, 44.005)).toBe(false);
    expect(t.lons(1)).toEqual([44.000, 44.001, 44.005, 44.006]);
  });
  it('records every fix of a path, not only the newest position', () => {
    const t = setup();
    t.store.sendPath(1, [44.000, 44.001, 44.002]);
    t.store.sendPath(1, [44.003, 44.004]);
    expect(t.lons(1)).toEqual([44.000, 44.001, 44.002, 44.003, 44.004]);
    expect(t.joins(1, 44.002, 44.003)).toBe(true);
  });

  it('a kept path is not re-added when the store notifies for another UAV', () => {
    const t = setup();
    t.store.sendPath(1, [44.000, 44.003]);
    t.store.sendPath(2, [45.000]);
    t.store.notify();
    const pool = t.entitiesRef.current.trailMap[1];
    const positions = pool.segs.flat().map((e) => e.positions).filter((p) => p.length);
    expect(positions.flat().filter((_, i) => i % 3 === 0)).toEqual([44.000, 44.003]);
  });

  it('with a path the snapshot position is never drawn, so no step goes back', () => {
    const t = setup();
    // Snapshot taken before the newest fixes arrived: lon 44.001 < 44.003.
    t.store.vehicles.set(1, { sys_id: 1, lat: 40, lon: 44.001, alt_rel: 100,
      path: [44.000, 44.002, 44.003].map((l) => ({ lat: 40, lon: l, alt_rel: 100 })) });
    t.store.fresh.add(1);
    t.store.notify();
    t.store.vehicles.set(1, { sys_id: 1, lat: 40, lon: 44.002, alt_rel: 100, path: [] });
    t.store.notify();
    expect(t.lons(1)).toEqual([44.000, 44.002, 44.003]);
    expect(t.joins(1, 44.000, 44.002) && t.joins(1, 44.002, 44.003)).toBe(true);
  });

  it('the first-connect snapshot (empty path) draws nothing; the next path starts the trail', () => {
    const t = setup();
    // Snapshot newer than fixes still buffered for the next broadcast.
    t.store.vehicles.set(1, { sys_id: 1, lat: 40, lon: 44.005, alt_rel: 100, path: [] });
    t.store.fresh.add(1);
    t.store.notify();
    expect(t.lons(1)).toEqual([]);
    t.store.sendPath(1, [44.003, 44.004, 44.005]);
    expect(t.lons(1)).toEqual([44.003, 44.004, 44.005]);
  });

  it('a path received while paused is not recorded on resume', () => {
    const t = setup();
    t.store.sendPath(1, [44.000, 44.001]);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: true });
    t.store.sendPath(1, [44.002, 44.003]);
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: false });
    t.store.notify();
    expect(t.lons(1)).toEqual([44.000, 44.001]);
  });

  it('with terrain, each point is drawn over the ground under it, as the vehicle model is', async () => {
    // alt_rel is over home: drawn alone as an ellipsoid height it sat ~1.3 km under the terrain.
    const t = setup({ terrainReady: true });
    t.store.sendPath(1, [44.000, 44.001, 44.002]);
    expect(t.heights(1)).toEqual([]); // no ground known yet: nothing under the terrain
    await t.settle();
    expect(t.heights(1)).toEqual([[44.000, 1400], [44.001, 1410], [44.002, 1420]]);
  });

  it('a new point takes the last known ground until its own sample comes', async () => {
    const t = setup({ terrainReady: true });
    t.store.sendPath(1, [44.000, 44.001]);
    await t.settle();
    t.store.sendPath(1, [44.002]);
    expect(t.heights(1).at(-1)).toEqual([44.002, 1410]);
    await t.settle();
    expect(t.heights(1).at(-1)).toEqual([44.002, 1420]);
  });

  it('one sample at a time per trail; points added meanwhile go in the next, with no new telemetry', async () => {
    const t = setup({ terrainReady: true });
    t.store.sendPath(1, [44.000]);
    await t.flush();
    t.store.sendPath(1, [44.001]);
    await t.flush();
    expect(t.sampleTerrainMostDetailed).toHaveBeenCalledTimes(1);
    await t.settle();
    expect(t.sampleTerrainMostDetailed).toHaveBeenCalledTimes(2);
    expect(t.sampleTerrainMostDetailed.mock.calls[1][1]).toEqual([{ lon: 44.001, lat: 40 }]);
    await t.settle();
    expect(t.heights(1)).toEqual([[44.000, 1400], [44.001, 1410]]);
  });

  it('a failed sample keeps the points unsampled; the next update samples them again', async () => {
    const t = setup({ terrainReady: true });
    t.store.sendPath(1, [44.000]);
    await t.settle();
    t.terrain.failing = true;
    t.store.sendPath(1, [44.001]);
    await t.settle();
    expect(t.heights(1)).toEqual([[44.000, 1400], [44.001, 1400]]); // last known ground
    expect(t.sampleTerrainMostDetailed).toHaveBeenCalledTimes(2); // no retry loop
    t.terrain.failing = false;
    t.store.notify();
    await t.settle();
    expect(t.sampleTerrainMostDetailed.mock.calls[2][1]).toEqual([{ lon: 44.001, lat: 40 }]);
    expect(t.heights(1)).toEqual([[44.000, 1400], [44.001, 1410]]);
  });

  it('a point two fade pieces share has one height while its ground is unknown', async () => {
    const t = setup({ terrainReady: true });
    // 10 points in 5 fade pieces: 44.002 ends piece 1 and starts piece 2.
    t.terrain.holes.add(44.002);
    t.store.sendPath(1, [44.000, 44.001, 44.002, 44.003, 44.004, 44.005, 44.006, 44.007, 44.008, 44.009]);
    await t.settle();
    // heights() fails if the two pieces draw it at different heights.
    expect(t.heights(1)[2]).toEqual([44.002, 1410]);
    expect(t.sampleTerrainMostDetailed).toHaveBeenCalledTimes(1); // an incomplete batch is not chained
  });

  it('thinning keeps the point that holds the line over a ridge', async () => {
    const t = setup({ terrainReady: true });
    const lons = Array.from({ length: 2001 }, (_, i) => 44 + i * 0.0001);
    // Early in the old part, which thinning eats from its start when errors tie.
    const ridge = lons[300];
    // Level flight over home: only the ground under one point rises (+500 m).
    const at = (lon) => (lon === ridge ? 1800 : 1300);
    t.sampleTerrainMostDetailed.mockImplementation((provider, cartos) =>
      Promise.resolve(cartos.map((c) => ({ ...c, height: at(c.lon) }))));
    t.store.sendPath(1, lons.slice(0, 1500));
    await t.settle();
    t.store.sendPath(1, lons.slice(1500));
    await t.settle();
    expect(t.heights(1)).toContainEqual([ridge, 1900]);
  });

  it('without terrain heights are over the ellipsoid; terrain arriving later lifts the whole trail', async () => {
    const t = setup();
    t.store.sendPath(1, [44.000, 44.001]);
    expect(t.heights(1)).toEqual([[44.000, 100], [44.001, 100]]);
    expect(t.sampleTerrainMostDetailed).not.toHaveBeenCalled();
    t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: false, terrainReady: true });
    await t.settle(); // no telemetry needed
    expect(t.heights(1)).toEqual([[44.000, 1400], [44.001, 1410]]);
  });

  it('a sample that resolves after Clear all is dropped', async () => {
    const t = setup({ terrainReady: true });
    t.store.sendPath(1, [44.000, 44.001]);
    t.hook.rerender({ show: true, resetKey: 1, hidden: new Set(), paused: false, terrainReady: true });
    t.store.sendPath(1, [44.005, 44.006]);
    await t.settle();
    t.store.notify();
    await t.settle();
    expect(t.heights(1)).toEqual([[44.005, 1450], [44.006, 1460]]);
  });

  describe('live segment to the model', () => {
    // The playout clock (uavPlayout) runs on the page clock: tick() moves it.
    beforeEach(() => { vi.useFakeTimers(); vi.setSystemTime(10000); });
    afterEach(() => { vi.useRealTimers(); });
    const tick = (ms) => vi.advanceTimersByTime(ms);

    it('joins the drawn head, at its ground + alt, to the model and follows the model every frame', async () => {
      const t = setup({ terrainReady: true });
      t.store.sendPath(1, [44.000, 44.001]);
      t.placeModel(1, 44.0013, 1513);
      expect(t.live(1)).toEqual([]); // head ground unknown: the trail is not drawn either
      tick(1000); // the model has passed both fixes
      await t.settle();
      expect(t.live(1)).toEqual([{ lon: 44.001, lat: 40, h: 1410 }, { lon: 44.0013, lat: 40, h: 1513 }]);
      t.placeModel(1, 44.0016, 1516); // no telemetry: only the model moved
      expect(t.live(1)[1]).toEqual({ lon: 44.0016, lat: 40, h: 1516 });
      expect(t.lons(1)).toEqual([44.000, 44.001]); // nothing is stored
    });

    it('draws the trail only to the fixes the model has passed; the segment runs on through them', () => {
      const t = setup();
      t.placeModel(1, 44.0025, 100);
      t.store.sendPath(1, [44.000, 44.001]);
      tick(200);
      t.store.sendPath(1, [44.002]);
      // 44.002 is ahead of the playout clock: not drawn yet, by the trail or the segment.
      expect(t.lons(1)).not.toContain(44.002);
      const before = t.live(1);
      expect(before.slice(0, -1).map((p) => p.lon)).not.toContain(44.002);
      expect(before[0].lon).toBe(Math.max(...t.lons(1))); // starts at the drawn head
      // Without an update, the clock passes 44.002: the segment draws it, then the model.
      tick(1000);
      const after = t.live(1);
      expect(after.at(-2).lon).toBe(44.002);
      expect(after.at(-1)).toEqual({ lon: 44.0025, lat: 40, h: 100 });
      // The next update draws it into the trail and starts the segment there.
      t.store.notify();
      expect(t.lons(1)).toContain(44.002);
      expect(t.live(1)[0].lon).toBe(44.002);
      const pool = t.entitiesRef.current.trailMap[1];
      expect(pool.live.polyline.material).toBe(pool.base); // the UAV's solid colour
    });

    it('starts the segment where the trail shown in the same frame ends, after every update (todo 293)', () => {
      // Static entity pieces showed their old positions for some frames while
      // the segment started at the new head at once: the head blinked.
      const t = setup();
      t.placeModel(1, 44.0100, 100);
      let lon = 44;
      let joined = 0;
      for (let k = 0; k < 12; k++) {
        lon = +(lon + 0.0005).toFixed(4);
        t.store.send(1, lon);
        for (let f = 0; f < 3; f++) {
          tick(70);
          const shown = t.frame();
          const seg = t.live(1);
          // One drawn fix is no piece yet: the segment alone starts there.
          if (!seg.length || !shown.length) continue;
          joined += 1;
          expect(seg[0].lon).toBe(Math.max(...shown.flat()));
        }
      }
      expect(joined).toBeGreaterThan(20);
    });

    it('never draws the trail end backwards over jittery, bursty updates', () => {
      const t = setup();
      // Model and trail at one place: the model comes in through the hook's
      // own playout, as useUavMarkers draws it.
      const scratch = {};
      t.entitiesRef.current.uavMap = { 1: { position: { getValue: () => scratch } } };
      let lon = 44;
      let lastEnd = -Infinity;
      const holds = [0, 0, 120, 0, 450, 0, 0, 30, 0, 0];
      for (let k = 0; k < 60; k++) {
        const path = [1, 2, 3, 4, 5].map(() => { lon += 0.0001; return lon; });
        t.store.sendPath(1, path);
        for (let ms = 0; ms < 200 + holds[k % holds.length]; ms += 16) {
          tick(16);
          const pb = t.entitiesRef.current.uavPlayout[1];
          const clock = pb.clock;
          const pts = pb.fixes;
          // Where useUavMarkers draws the model at this clock (lon only, lat 40).
          let i = pts.length - 1;
          while (i > 0 && pts[i].t > clock) i--;
          const b = pts[i + 1];
          scratch.lon = b && clock > pts[i].t ? pts[i].lon + (b.lon - pts[i].lon) * (clock - pts[i].t) / (b.t - pts[i].t) : pts[i].lon;
          scratch.lat = 40; scratch.h = 100;
          const seg = t.live(1);
          // The drawn trail never reaches past the model, and its end never goes back.
          const end = seg.length ? seg.at(-1).lon : Math.max(...t.lons(1));
          expect(Math.max(...t.lons(1))).toBeLessThanOrEqual(scratch.lon + 1e-9);
          expect(end).toBeGreaterThanOrEqual(lastEnd - 1e-9);
          lastEnd = end;
        }
      }
    });

    it('is not drawn while telemetry is not fresh, and a reconnect does not bridge the gap', () => {
      const t = setup();
      t.placeModel(1, 44.0015, 100);
      t.store.send(1, 44.000); tick(200); t.store.send(1, 44.001);
      tick(1000); t.store.notify();
      expect(t.live(1)).toHaveLength(2);
      t.store.disconnect();
      expect(t.live(1)).toEqual([]);
      t.store.send(1, 44.005); // new run, one point: the segment starts there, not at 44.001
      expect(t.live(1)[0].lon).toBe(44.005);
    });

    it('is not drawn while paused, nor after resume until the new run has a point', () => {
      const t = setup();
      t.placeModel(1, 44.0015, 100);
      t.store.send(1, 44.000); t.store.send(1, 44.001);
      t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: true });
      expect(t.live(1)).toEqual([]);
      t.hook.rerender({ show: true, resetKey: 0, hidden: new Set(), paused: false });
      expect(t.live(1)).toEqual([]); // the new run is empty
      t.store.send(1, 44.003);
      tick(1000); t.store.notify(); // the model has passed it
      expect(t.live(1)[0].lon).toBe(44.003);
    });

    it('is not drawn while the model is still in a gap', () => {
      const t = setup();
      t.placeModel(1, 44.0015, 100);
      t.store.send(1, 44.000); tick(200); t.store.send(1, 44.001);
      tick(1000); t.store.notify();
      t.entitiesRef.current.uavGap = new Set([1]);
      expect(t.live(1)).toEqual([]);
      t.entitiesRef.current.uavGap = new Set();
      expect(t.live(1)).toHaveLength(2);
    });

    it('is not drawn after Clear all or without a model', () => {
      const t = setup();
      t.store.send(1, 44.000); t.store.send(1, 44.001);
      expect(t.live(1)).toEqual([]); // no model yet
      t.placeModel(1, 44.0015, 100);
      t.hook.rerender({ show: true, resetKey: 1, hidden: new Set(), paused: false });
      expect(t.live(1)).toEqual([]);
    });

    it('hides with its trail and is removed when the UAV leaves', () => {
      const t = setup();
      t.store.send(1, 44.000); t.store.send(1, 44.001);
      const e = t.entitiesRef.current.trailMap[1].live;
      t.hook.rerender({ show: true, resetKey: 0, hidden: new Set([1]), paused: false });
      expect(e.show).toBe(false);
      const lines = t.entitiesRef.current.trailMap[1].lines;
      t.store.leave(1);
      expect(t.entities.remove).toHaveBeenCalledWith(e);
      expect(t.primitives.remove).toHaveBeenCalledWith(lines);
    });
  });
});
