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
    disconnect() { store.fresh.clear(); store.notify(); },
    leave(sid) { store.vehicles.delete(sid); store.fresh.delete(sid); store.notify(); },
  };
  return store;
}

function setup(props = {}) {
  const store = fakeStore();
  const entities = { add: vi.fn((e) => e), remove: vi.fn() };
  const cesium = { current: {
    Color: { fromCssColorString: () => ({ withAlpha: (a) => ({ a }) }) },
    Cartesian3: { fromDegreesArrayHeights: (flat) => [...flat] },
  } };
  const viewer = { current: { entities } };
  const entitiesRef = { current: { trailMap: {} } };
  const storeRef = { current: store };
  const hook = renderHook(
    ({ show, resetKey, hidden, paused }) => useUavTrails(
      cesium, viewer, entitiesRef, storeRef, true, show, resetKey, hidden, paused,
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
  return { store, hook, drawn, lons, joins, entities, entitiesRef };
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
});
