import { act, renderHook } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import * as Cesium from 'cesium';
import useUavMarkers from './useUavMarkers';

// No terrain requests: the ground stays 0.
vi.mock('../utils/terrainCache', () => ({ needsTerrainSample: () => false, sampleTerrain: () => {} }));

const { Cartesian3, JulianDate } = Cesium;

function setup() {
  const listeners = new Set();
  const store = {
    vehicles: new Map(),
    fresh: new Set(),
    subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    getVehicleList() { return [...store.vehicles.values()]; },
    isTelemetryFresh(sid) { return store.fresh.has(sid); },
    notify() { act(() => { for (const fn of listeners) fn(); }); },
    /** A telemetry message for one UAV flying east (marks it fresh). */
    send(sid, lon) {
      store.vehicles.set(sid, { sys_id: sid, lat: 0, lon, alt_rel: 100, heading: 90, roll: 0, pitch: 0 });
      store.fresh.add(sid);
      store.notify();
    },
    disconnect() { store.fresh.clear(); store.notify(); },
  };
  const entities = { add: vi.fn((e) => e), remove: vi.fn() };
  // preUpdate runs every frame, render or not; frame() is one such frame.
  const preUpdate = new Set();
  const scene = {
    requestRender: vi.fn(),
    preUpdate: { addEventListener: (fn) => { preUpdate.add(fn); return () => preUpdate.delete(fn); } },
  };
  const viewer = { current: { entities, scene } };
  const entitiesRef = { current: {} };
  const hook = renderHook(() => useUavMarkers({ current: Cesium }, viewer, entitiesRef, { current: store }, true));
  /** Run one frame; whether it requested a render. */
  const frame = () => {
    scene.requestRender.mockClear();
    for (const fn of preUpdate) fn();
    return scene.requestRender.mock.calls.length > 0;
  };
  /** Metres between sid's drawn model and lon (lat 0, alt 100). */
  const offFrom = (sid, lon) => Cartesian3.distance(
    entitiesRef.current.uavMap[sid].position.getValue(JulianDate.now(), new Cartesian3()),
    Cartesian3.fromDegrees(lon, 0, 100),
  );
  const inGap = (sid) => entitiesRef.current.uavGap.has(sid);
  return { store, offFrom, inGap, frame, hook, preUpdate };
}

describe('useUavMarkers', () => {
  beforeEach(() => { vi.useFakeTimers(); vi.setSystemTime(0); });
  afterEach(() => { vi.useRealTimers(); });

  it('after a link gap the model starts at the first fresh sample, not gliding across the gap', () => {
    const t = setup();
    for (let i = 0; i < 5; i++) { t.store.send(1, 0.0005 * i); vi.advanceTimersByTime(200); }
    t.store.disconnect();
    vi.advanceTimersByTime(5000);
    t.store.send(1, 0.01); // ~1.1 km further east
    expect(t.offFrom(1, 0.01)).toBeLessThan(0.01);
  });

  it('a snapshot without a position is a gap too, as for the trail', () => {
    const t = setup();
    for (let i = 0; i < 5; i++) { t.store.send(1, 0.0005 * i); vi.advanceTimersByTime(200); }
    t.store.vehicles.set(1, { sys_id: 1, lat: null, lon: null, alt_rel: 100, heading: 90, roll: 0, pitch: 0 });
    t.store.notify();
    vi.advanceTimersByTime(5000);
    t.store.send(1, 0.01);
    expect(t.offFrom(1, 0.01)).toBeLessThan(0.01);
  });

  it('stays in a gap, for the trail, until a fresh sample with a full pose', () => {
    const t = setup();
    t.store.send(1, 0);
    expect(t.inGap(1)).toBe(false);
    t.store.disconnect();
    expect(t.inGap(1)).toBe(true);
    // Fresh, but no attitude: the model keeps its old pose and stays in the gap.
    t.store.vehicles.set(1, { sys_id: 1, lat: 0, lon: 0.01, alt_rel: 100, heading: null, roll: 0, pitch: 0 });
    t.store.fresh.add(1);
    t.store.notify();
    expect(t.inGap(1)).toBe(true);
    t.store.send(1, 0.0105);
    expect(t.inGap(1)).toBe(false);
    expect(t.offFrom(1, 0.0105)).toBeLessThan(0.01);
  });

  it('requests renders while a model settles after its samples stop, and only then', () => {
    const t = setup();
    for (let i = 0; i < 5; i++) { t.store.send(1, 0.0005 * i); vi.advanceTimersByTime(200); }
    // Samples flowing: each one renders through its update, frames ask nothing.
    t.store.send(1, 0.0025);
    vi.advanceTimersByTime(100);
    expect(t.frame()).toBe(false);
    // Samples stopped: every frame renders until the model is at rest...
    const rendered = [];
    for (let ms = 100; ms < 2000; ms += 50) { vi.advanceTimersByTime(50); rendered.push(t.frame()); }
    expect(rendered.slice(2, 20).every(Boolean)).toBe(true);
    // ...on the last sample, then nothing more.
    expect(rendered.at(-1)).toBe(false);
    expect(rendered.filter(Boolean).length).toBeLessThan(rendered.length - 10);
    expect(t.offFrom(1, 0.0025)).toBeLessThan(0.01);
    t.hook.unmount();
    expect(t.preUpdate.size).toBe(0);
  });

  it('a stall across the settling still draws the model at rest once', () => {
    const t = setup();
    for (let i = 0; i < 5; i++) { t.store.send(1, 0.0005 * i); vi.advanceTimersByTime(200); }
    vi.advanceTimersByTime(5000); // no frame ran meanwhile
    expect(t.frame()).toBe(true);
    expect(t.frame()).toBe(false);
  });

  it('without a gap a far sample is still eased into', () => {
    const t = setup();
    for (let i = 0; i < 5; i++) { t.store.send(1, 0.0005 * i); vi.advanceTimersByTime(200); }
    t.store.send(1, 0.01);
    expect(t.offFrom(1, 0.01)).toBeGreaterThan(500);
  });
});
