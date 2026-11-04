import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
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
    Cartesian3: { fromDegreesArrayHeights: (flat) => [...flat] },
    Cartographic: { fromDegrees: (lon, lat) => ({ lon, lat }) },
    sampleTerrainMostDetailed,
  } };
  // The rendered globe answers with a coarse tile height, never a trail's ground.
  const viewer = { current: { entities, terrainProvider: {}, scene: { globe: { getHeight: () => 999 } } } };
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
    const ents = pool.segs.flat().filter((e) => e.polyline.positions.length > 0);
    return {
      pieces: ents.map((e) => e.polyline.positions.filter((_, i) => i % 3 === 0)),
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
      const p = e.polyline.positions;
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
  return { store, hook, drawn, lons, joins, entities, entitiesRef, heights, flush, settle, sampleTerrainMostDetailed, terrain };
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
    const positions = pool.segs.flat().map((e) => e.polyline.positions).filter((p) => p.length);
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
});
