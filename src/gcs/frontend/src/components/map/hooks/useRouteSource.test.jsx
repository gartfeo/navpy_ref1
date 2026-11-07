// The first-person views' route source and RC's route labels on the main map.
import { describe, it, expect, vi } from 'vitest';
import { renderHook } from '@testing-library/react';
import { useRouteSource, useFpvRouteLabels } from './useRouteSource';

const { overlays } = vi.hoisted(() => ({ overlays: [] }));
vi.mock('../utils/routeOverlay', () => ({
  createRouteOverlay: (_Cesium, scene) => {
    const o = { scene, updates: [], destroyed: false };
    overlays.push(o);
    return {
      update: (route, opts) => { o.updates.push({ route, opts }); return true; },
      destroy: () => { o.destroyed = true; },
    };
  },
}));

function store(list) {
  const listeners = new Set();
  return {
    listeners,
    getVehicleList: () => list,
    subscribe: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
  };
}

describe('useRouteSource', () => {
  it("publishes route(sysId) from the main map's plan, positions and progress; cleared on unmount", () => {
    const routeSourceRef = { current: null };
    const track = [{ lat: 0, lon: 0 }, { lat: 1, lon: 0 }];
    const storeRef = { current: store([{ sys_id: 4 }]) };
    const planRef = { current: { zones: [{ track }] } };
    const trackPosRef = { current: [['a', 'b']] };
    const vehiclePosRef = { current: { 0: { trackIdx: 1 } } };
    const { unmount } = renderHook(() => useRouteSource(routeSourceRef, storeRef, planRef, trackPosRef, vehiclePosRef));
    expect(routeSourceRef.current.route(4)).toEqual({ zone: 0, positions: ['a', 'b'], track, next: 1 });
    // Read live: a terrain sample replaces the positions.
    trackPosRef.current = [['c', 'd']];
    expect(routeSourceRef.current.route(4).positions).toEqual(['c', 'd']);
    unmount();
    expect(routeSourceRef.current).toBeNull();
  });
});

describe('useFpvRouteLabels', () => {
  it("while RC owns the camera: the RC UAV's numbered waypoints, no second line, per telemetry update", () => {
    overlays.length = 0;
    const scene = { requestRender: vi.fn() };
    const viewer = { scene, isDestroyed: () => false };
    const s = store([]);
    const route = { zone: 0 };
    const routeSourceRef = { current: { route: vi.fn(() => route) } };
    const { rerender, unmount } = renderHook(
      ({ fpvSysId, showTracks = true }) => useFpvRouteLabels(
        { current: {} }, { current: viewer }, routeSourceRef, fpvSysId, true, { current: s }, showTracks,
      ),
      { initialProps: { fpvSysId: null } },
    );
    expect(overlays).toHaveLength(0);
    rerender({ fpvSysId: 2 });
    expect(overlays).toHaveLength(1);
    expect(overlays[0].updates.at(-1)).toEqual({ route, opts: { line: false, labels: 'all' } });
    expect(routeSourceRef.current.route).toHaveBeenLastCalledWith(2);
    s.listeners.forEach((fn) => fn());
    expect(overlays[0].updates).toHaveLength(2);
    expect(scene.requestRender).toHaveBeenCalled();
    rerender({ fpvSysId: null });
    expect(overlays[0].destroyed).toBe(true);
    expect(s.listeners.size).toBe(0);
    unmount();
  });

  it('tracks hidden on the main map: the overlay draws the line itself, so RC keeps the path (Codex r1)', () => {
    overlays.length = 0;
    const scene = { requestRender: vi.fn() };
    const s = store([]);
    const route = { zone: 0 };
    const { unmount } = renderHook(() => useFpvRouteLabels(
      { current: {} }, { current: { scene, isDestroyed: () => false } }, { current: { route: () => route } }, 2, true,
      { current: s }, false,
    ));
    expect(overlays[0].updates.at(-1)).toEqual({ route, opts: { line: true, labels: 'all' } });
    unmount();
    expect(s.listeners.size).toBe(0);
  });
});

describe('useRouteSource with useTrackLayer progress', () => {
  it('outside AUTO useTrackLayer marks the track done: no next waypoint (the UAV flies to none)', () => {
    const routeSourceRef = { current: null };
    const track = [{ lat: 0, lon: 0 }, { lat: 1, lon: 0 }];
    // useTrackLayer's value for a UAV that left AUTO: trackIdx = the track's length.
    const vehiclePosRef = { current: { 0: { trackIdx: track.length, lat: 0, lon: 0 } } };
    const { unmount } = renderHook(() => useRouteSource(
      routeSourceRef, { current: store([{ sys_id: 4 }]) }, { current: { zones: [{ track }] } },
      { current: [['a', 'b']] }, vehiclePosRef,
    ));
    expect(routeSourceRef.current.route(4)).toMatchObject({ positions: ['a', 'b'], next: null });
    unmount();
  });
});
