// RC FPV camera hold and render requests in useFollowUav, driven with a fake
// Cesium: the math is stubbed, so these check only which camera calls happen.
import { describe, it, expect, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useFollowUav from './useFollowUav';
import { captureCamera, persistCamera, restoreCamera } from './useCesiumViewer';

const obj = () => ({});
const FakeCesium = {
  HeadingPitchRoll: class { constructor() { this.heading = 0; this.pitch = 0; this.roll = 0; } static fromQuaternion(_q, out) { return out; } },
  HeadingPitchRange: class {},
  Quaternion: class { static fromHeadingPitchRoll = obj; static inverse = obj; static multiply = obj; static fromRotationMatrix = obj; },
  Matrix4: class { static IDENTITY = { identity: true }; static getMatrix3 = obj; },
  Matrix3: class {},
  Transforms: { eastNorthUpToFixedFrame: obj },
  Math: { PI: Math.PI, PI_OVER_TWO: Math.PI / 2, toRadians: (d) => d, toDegrees: (r) => r },
  Cartesian3: { clone: (p) => ({ ...p }), magnitude: vi.fn(() => 6.4e6) },
  Ellipsoid: { WGS84: { cartesianToCartographic: () => ({ latitude: 1, longitude: 2, height: 3 }) } },
};

function makeStore(vehicles) {
  const listeners = new Set();
  return {
    vehicles,
    subscribe: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
    fresh: true,
    getVehicles() { return this.vehicles; },
    isTelemetryFresh() { return this.fresh; },
    emit() { listeners.forEach((fn) => fn()); },
  };
}

function setup(vehicle, fpvSysId = 1) {
  let preRender = null;
  const viewer = {
    clock: { currentTime: 0 },
    camera: { setView: vi.fn(), lookAtTransform: vi.fn(), pitch: 0, position: {} },
    scene: {
      requestRender: vi.fn(),
      preRender: { addEventListener: (fn) => { preRender = fn; }, removeEventListener: () => { preRender = null; } },
    },
  };
  const uav = (x) => ({ position: { getValue: () => ({ x }) }, orientation: { getValue: () => ({ w: 1 }) } });
  const entitiesRef = { current: { uavMap: { 1: uav(1), 2: uav(2) } } };
  const store = makeStore({ 1: vehicle, 2: vehicle });
  const hook = renderHook(
    ({ sysId }) => useFollowUav({ current: FakeCesium }, { current: viewer }, entitiesRef, sysId, true, { current: store }),
    { initialProps: { sysId: fpvSysId } },
  );
  return { viewer, store, hook, frame: () => preRender?.(), hasHandler: () => preRender != null };
}

const linked = { link_ok: true, lat: 32, lon: 34, alt_rel: 100, roll: 1, pitch: 2, heading: 90 };

describe('useFollowUav: RC FPV only', () => {
  it('follows a valid pose of the RC target through the nose, never a chase transform', () => {
    const { viewer, frame } = setup(linked);
    frame();
    expect(viewer.camera.setView).toHaveBeenCalledTimes(1);
    expect(viewer.camera.setView.mock.calls[0][0].destination).toEqual({ x: 1 });
    expect(viewer.camera.lookAtTransform).not.toHaveBeenCalled();
  });

  it('a new RC target moves the first-person camera to that UAV', () => {
    const { viewer, frame, hook } = setup(linked);
    hook.rerender({ sysId: 2 });
    frame();
    expect(viewer.camera.setView).toHaveBeenLastCalledWith(expect.objectContaining({ destination: { x: 2 } }));
  });

  it('no RC target: no camera handler, no render requests, no camera moves', () => {
    const { viewer, store, frame, hasHandler } = setup(linked, null);
    expect(hasHandler()).toBe(false);
    act(() => store.emit());
    frame();
    expect(viewer.scene.requestRender).not.toHaveBeenCalled();
    expect(viewer.camera.setView).not.toHaveBeenCalled();
    expect(viewer.camera.lookAtTransform).not.toHaveBeenCalled();
  });

  it('holds the last pose while link_ok is false and resumes when it is back', () => {
    const { viewer, store, frame } = setup(linked);
    frame();
    store.vehicles = { 1: { ...linked, link_ok: false } };
    frame();
    frame();
    expect(viewer.camera.setView).toHaveBeenCalledTimes(1);
    store.vehicles = { 1: linked };
    frame();
    expect(viewer.camera.setView).toHaveBeenCalledTimes(2);
  });

  it('holds while linked with a non-finite position, then attitude, and resumes', () => {
    const { viewer, store, frame } = setup(linked);
    store.vehicles = { 1: { ...linked, lat: null } };
    frame();
    store.vehicles = { 1: { ...linked, pitch: NaN } };
    frame();
    expect(viewer.camera.setView).not.toHaveBeenCalled();
    store.vehicles = { 1: linked };
    frame();
    expect(viewer.camera.setView).toHaveBeenCalledTimes(1);
  });

  it('holds while the telemetry is not current (socket closed, or reopened with no new snapshot)', () => {
    const { viewer, store, frame } = setup(linked);
    store.fresh = false;
    frame();
    expect(viewer.camera.setView).not.toHaveBeenCalled();
    store.fresh = true;
    frame();
    expect(viewer.camera.setView).toHaveBeenCalledTimes(1);
  });

  it('requests a render on entry and per pose update in FPV, and stops when RC ends', () => {
    const { viewer, store, hook, hasHandler } = setup(linked);
    expect(viewer.scene.requestRender).toHaveBeenCalledTimes(1);
    act(() => store.emit());
    act(() => store.emit());
    expect(viewer.scene.requestRender).toHaveBeenCalledTimes(3);
    hook.rerender({ sysId: null });
    act(() => store.emit());
    expect(viewer.scene.requestRender).toHaveBeenCalledTimes(3);
    expect(hasHandler()).toBe(false);
  });
});

describe('map camera persistence and restore', () => {
  const camera = { positionWC: { x: 1 }, heading: 0.1, pitch: -1, roll: 0 };

  it('never stores the camera as the startup view while FPV or RC owns it', () => {
    const storage = { setItem: vi.fn() };
    persistCamera(FakeCesium, camera, true, storage);
    expect(storage.setItem).not.toHaveBeenCalled();
    persistCamera(FakeCesium, camera, false, storage);
    expect(storage.setItem).toHaveBeenCalledTimes(1);
  });

  it('restores a captured camera, unlocked from any follow transform', () => {
    const saved = captureCamera(FakeCesium, camera);
    const viewer = { camera: { lookAtTransform: vi.fn(), setView: vi.fn() }, scene: { requestRender: vi.fn() } };
    restoreCamera(FakeCesium, viewer, saved);
    expect(viewer.camera.lookAtTransform).toHaveBeenCalledWith(FakeCesium.Matrix4.IDENTITY);
    expect(viewer.camera.setView).toHaveBeenCalledWith({
      destination: { x: 1 },
      orientation: { heading: 0.1, pitch: -1, roll: 0 },
    });
    expect(viewer.scene.requestRender).toHaveBeenCalled();
  });
});
