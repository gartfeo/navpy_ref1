import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useCoverageLayer from './useCoverageLayer';

vi.mock('../../../utils/planner', () => ({ getDeviceConfigs: () => [{ primary: true }] }));
vi.mock('../utils/cameraFootprint', () => ({
  cameraHitsGround: () => true,
  computeLiveGimbalDetectionEnvelope: (lat, lon) => [
    { lat, lon }, { lat: lat + 0.001, lon }, { lat, lon: lon + 0.001 },
  ],
  COVERAGE_SAMPLE_DIST: 10,
  padFootprintLoop: (e) => e,
}));
vi.mock('../utils/gimbalTelemetry', () => ({
  selectGimbalTelemetry: (v) => v.gimbal ?? { pitch: -90 },
  resolveLiveGimbalCameraConfig: (d) => d,
  gimbalTelemetrySignature: (t) => JSON.stringify(t),
}));

class PolygonHierarchy { constructor(p) { this.positions = p; } }
class CallbackProperty { constructor(fn) { this.fn = fn; } }
class Cartesian3 {
  static fromDegrees(lon, lat) { return { lon, lat }; }
  static lerp(a) { return a; }
}

function fakeStore() {
  const listeners = new Set();
  const store = {
    vehicles: new Map(),
    fresh: new Set(),
    subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    getVehicleList() { return [...store.vehicles.values()].sort((a, b) => a.sys_id - b.sys_id); },
    isTelemetryFresh(sid) { return store.fresh.has(sid); },
    notify() { act(() => { for (const fn of listeners) fn(); }); },
    send(sid, lon, extra = {}) {
      store.vehicles.set(sid, {
        sys_id: sid, lat: 40, lon, alt_rel: 100, heading: 0, pitch: 0, roll: 0, mode: 'AUTO', ...extra,
      });
      store.fresh.add(sid);
      store.notify();
    },
  };
  return store;
}

const BASE = {
  showCoverage: true, showVision: false, resetKey: 0, hidden: new Set(), paused: false,
};

function setup(props = {}) {
  const store = fakeStore();
  const live = new Set();
  const entities = {
    list: [],
    add: vi.fn((e) => { live.add(e); entities.list.push(e); return e; }),
    remove: vi.fn((e) => { live.delete(e); }),
  };
  const cesium = { current: {
    Cartesian3,
    PolygonHierarchy,
    CallbackProperty,
    Color: { fromCssColorString: (css) => ({ css, withAlpha: (a) => ({ css, a }) }) },
    ClassificationType: { TERRAIN: 'terrain' },
  } };
  const viewer = { current: { entities } };
  const plan = { altitude_m: 100, zones: [] };
  const hook = renderHook(
    (p) => useCoverageLayer(
      cesium, viewer, { current: store }, plan, p.showCoverage, p.showVision,
      0, p.resetKey, p.hidden, p.paused, true, true,
    ),
    { initialProps: { ...BASE, ...props } },
  );
  const marks = () => [...live].filter((e) => e.polygon.hierarchy instanceof PolygonHierarchy);
  const liveFootprints = () => [...live].filter((e) => e.polygon.hierarchy instanceof CallbackProperty);
  const marksAt = (lon) => marks().filter((e) => e.polygon.hierarchy.positions[0].lon === lon);
  const rerender = (p) => hook.rerender({ ...BASE, ...props, ...p });
  return { store, marks, marksAt, liveFootprints, rerender };
}

describe('useCoverageLayer camera marks', () => {
  it('records with the layer off and with the UAV unchecked, and shows them again', () => {
    const t = setup({ showCoverage: false, hidden: new Set([1]) });
    t.store.send(1, 44.0);
    t.store.send(1, 44.01);
    expect(t.marks()).toHaveLength(2);
    expect(t.marks().every((e) => e.show === false)).toBe(true);
    t.rerender({ showCoverage: true, hidden: new Set() });
    expect(t.marks().every((e) => e.show === true)).toBe(true);
  });

  it('is keyed by sys_id: a lower sys_id joining does not move marks, only recolours', () => {
    const t = setup();
    t.store.send(5, 44.0);
    const [mark] = t.marksAt(44.0);
    const before = mark.polygon.material.css;
    t.store.send(2, 45.0);
    t.store.send(5, 44.01);
    expect(t.marksAt(44.0)).toEqual([mark]);
    expect(mark.polygon.material.css).not.toBe(before);
    t.store.vehicles.delete(5);
    t.store.notify();
    expect(t.marksAt(44.0)).toEqual([]);
    expect(t.marksAt(44.01)).toEqual([]);
    expect(t.marksAt(45.0)).toHaveLength(1);
  });

  it('pause stops recording; only AUTO records; cached telemetry never records', () => {
    const t = setup({ paused: true });
    t.store.send(1, 44.0);
    expect(t.marks()).toHaveLength(0);
    t.rerender({ paused: false });
    t.store.send(1, 44.01, { mode: 'LOITER' });
    expect(t.marks()).toHaveLength(0);
    t.store.send(1, 44.02);
    expect(t.marks()).toHaveLength(1);
    t.store.fresh.clear();
    t.store.vehicles.set(1, { ...t.store.vehicles.get(1), lon: 44.03 });
    t.store.notify();
    expect(t.marks()).toHaveLength(1);
  });

  it('a stationary UAV whose gimbal state changes still gets a mark', () => {
    const t = setup();
    t.store.send(1, 44.0, { gimbal: { pitch: -90 } });
    t.store.send(1, 44.0, { gimbal: { pitch: -45 } });
    expect(t.marks()).toHaveLength(2);
  });

  it('Clear all removes marks and resets the sampling anchor', () => {
    const t = setup();
    t.store.send(1, 44.0);
    t.rerender({ resetKey: 1 });
    expect(t.marks()).toHaveLength(0);
    t.store.send(1, 44.0);
    expect(t.marks()).toHaveLength(1);
  });

  it('live footprint follows Vision, or the marks layer for a checked UAV, not pause', () => {
    const t = setup({ showVision: false, hidden: new Set([1]) });
    t.store.send(1, 44.0);
    t.store.send(2, 45.0);
    const byUav = () => t.liveFootprints().map((e) => e.show);
    expect(byUav().sort()).toEqual([false, true]);
    t.rerender({ showVision: true, hidden: new Set([1]), paused: true });
    expect(byUav()).toEqual([true, true]);
  });
});
