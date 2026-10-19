// Synthetic view widget lifecycle with a fake Cesium
// (docs/design/gcs_video_streams.md section 5, "Synthetic view").
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, waitFor } from '@testing-library/react';
import SyntheticView from './SyntheticView';

vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key, opts) => (opts ? `${key} ${Object.values(opts).join(' ')}` : key),
    i18n: { language: 'en' },
  }),
}));

const { fake } = vi.hoisted(() => {
  const event = () => {
    const fns = new Set();
    return {
      fns,
      addEventListener: (fn) => fns.add(fn),
      removeEventListener: (fn) => fns.delete(fn),
      raise: (...a) => [...fns].forEach((fn) => fn(...a)),
    };
  };
  class Vec { constructor(x = 0, y = 0, z = 0) { Object.assign(this, { x, y, z }); } }
  const fake = { widgets: [], terrainCalls: [], createThrows: false, event };
  class CesiumWidget {
    constructor(container, options) {
      if (fake.createThrows) throw new Error('no WebGL');
      this.container = container;
      this.options = options;
      this.destroyed = false;
      this.canvas = document.createElement('canvas');
      this.creditContainer = document.createElement('div');
      this.camera = { setView: vi.fn() };
      this.scene = {
        globe: {},
        screenSpaceCameraController: { enableInputs: true },
        renderError: event(),
        preRender: event(),
        requestRender: vi.fn(),
      };
      fake.widgets.push(this);
    }

    destroy() { this.destroyed = true; }

    isDestroyed() { return this.destroyed; }
  }
  fake.Cesium = {
    CesiumWidget,
    Cartesian3: Object.assign(Vec, { normalize: (v) => v }),
    Color: { fromCssColorString: (s) => ({ css: s }) },
    DirectionalLight: class { constructor(o) { this.direction = o.direction; } },
    Matrix4: { multiplyByPointAsVector: (_m, v, out) => Object.assign(out, v) },
    CesiumTerrainProvider: {
      fromIonAssetId: (id, opts) => {
        const terrain = { id, opts };
        fake.terrainCalls.push(terrain);
        return Promise.resolve(terrain);
      },
    },
  };
  return { fake };
});
vi.mock('cesium', () => fake.Cesium);
vi.mock('./map/utils/fpvPose', () => ({
  createPoseMath: () => ({ transform: {}, vehicleHpr: () => ({ heading: 1, pitch: 2, roll: 3 }) }),
  noseOrientation: (hpr) => ({ heading: hpr.heading, pitch: hpr.roll, roll: -hpr.pitch }),
}));

const linked = { link_ok: true, lat: 32, lon: 34, alt_rel: 100, roll: 1, pitch: 2, heading: 90 };

function makeStore(vehicles = { 1: linked, 2: linked }) {
  const listeners = new Set();
  return {
    vehicles,
    fresh: true,
    listeners,
    subscribe: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
    getVehicles() { return this.vehicles; },
    isTelemetryFresh() { return this.fresh; },
  };
}

const pose = { position: { x: 1, y: 2, z: 3 }, orientation: { w: 1 } };

function setup({ sysId = 1, ionTerrain = false, poseFn } = {}) {
  const store = makeStore();
  const storeRef = { current: store };
  const poseSourceRef = { current: { pose: poseFn ?? (() => pose) } };
  const onFail = vi.fn();
  const onCreditsHeight = vi.fn();
  const view = (props) => (
    <SyntheticView
      sysId={sysId}
      label={`UAV ${sysId}`}
      poseSourceRef={poseSourceRef}
      storeRef={storeRef}
      ionTerrain={ionTerrain}
      onFail={onFail}
      onCreditsHeight={onCreditsHeight}
      {...props}
    />
  );
  const utils = render(view());
  return {
    ...utils, store, onFail, onCreditsHeight,
    rerenderWith: (props) => utils.rerender(view(props)),
  };
}

const widgetReady = () => waitFor(() => expect(fake.widgets.length).toBeGreaterThan(0), { timeout: 5000 });
const frame = (widget) => act(() => widget.scene.preRender.raise());

beforeEach(() => {
  fake.widgets.length = 0;
  fake.terrainCalls.length = 0;
  fake.createThrows = false;
  vi.spyOn(console, 'warn').mockImplementation(() => {});
});

describe('SyntheticView', () => {
  it('one widget: no imagery, no sky box, render on request, inputs off; destroyed on unmount', async () => {
    const { unmount } = setup();
    await widgetReady();
    await act(async () => {});
    expect(fake.widgets).toHaveLength(1);
    const [w] = fake.widgets;
    expect(w.options).toMatchObject({ baseLayer: false, skyBox: false, requestRenderMode: true, msaaSamples: 1, showRenderLoopErrors: false });
    expect(w.scene.screenSpaceCameraController.enableInputs).toBe(false);
    expect(w.scene.globe.enableLighting).toBe(true);
    unmount();
    expect(w.destroyed).toBe(true);
  });

  // StrictMode's mount-unmount-mount relies on this guard. (Rendering under
  // StrictMode here is not reliable: Vitest 4 hands the second import of the
  // mocked module the real Cesium.)
  it('unmounted before Cesium loads: no widget is created', async () => {
    const { unmount, onFail } = setup();
    unmount();
    await act(async () => { await import('cesium'); });
    await act(async () => {});
    expect(fake.widgets).toHaveLength(0);
    expect(onFail).not.toHaveBeenCalled();
  });

  it('"Waiting for pose" until the window UAV has a valid pose; a held UAV keeps it', async () => {
    const { getByText, queryByText, store } = setup();
    await widgetReady();
    const [w] = fake.widgets;
    expect(getByText('vehicleWindow.waitingForPose UAV 1')).toBeInTheDocument();
    store.fresh = false;
    frame(w);
    expect(w.camera.setView).not.toHaveBeenCalled();
    expect(getByText('vehicleWindow.waitingForPose UAV 1')).toBeInTheDocument();
    store.fresh = true;
    frame(w);
    expect(w.camera.setView).toHaveBeenCalledWith({ destination: pose.position, orientation: { heading: 1, pitch: 3, roll: -2 } });
    expect(queryByText(/waitingForPose/)).toBeNull();
    expect(getByText('vehicleWindow.syntheticTag')).toBeInTheDocument();
  });

  it('a badge tap to another UAV covers the old view until the new UAV has a pose', async () => {
    const { getByText, queryByText, rerenderWith, store } = setup();
    await widgetReady();
    const [w] = fake.widgets;
    frame(w);
    store.vehicles = { 1: linked, 2: { ...linked, link_ok: false } };
    rerenderWith({ sysId: 2, label: 'UAV 2' });
    expect(fake.widgets).toHaveLength(1);
    frame(w);
    expect(getByText('vehicleWindow.waitingForPose UAV 2')).toBeInTheDocument();
    store.vehicles = { 1: linked, 2: linked };
    frame(w);
    expect(queryByText(/waitingForPose/)).toBeNull();
  });

  it('a non-finite copied pose is not applied', async () => {
    setup({ poseFn: () => ({ position: { x: NaN, y: 0, z: 0 }, orientation: { w: 1 } }) });
    await widgetReady();
    frame(fake.widgets[0]);
    expect(fake.widgets[0].camera.setView).not.toHaveBeenCalled();
  });

  it('one render request per telemetry update', async () => {
    const { store } = setup();
    await widgetReady();
    await act(async () => {});
    const w = fake.widgets[0];
    const before = w.scene.requestRender.mock.calls.length;
    store.listeners.forEach((fn) => fn());
    expect(w.scene.requestRender).toHaveBeenCalledTimes(before + 1);
  });

  it('its own Ion terrain instance with vertex normals, once the main map has Ion terrain', async () => {
    const { rerenderWith } = setup();
    await widgetReady();
    await act(async () => {});
    expect(fake.terrainCalls).toHaveLength(0);
    rerenderWith({ ionTerrain: true });
    await waitFor(() => expect(fake.widgets[0].terrainProvider).toBe(fake.terrainCalls[0]));
    expect(fake.terrainCalls[0]).toEqual({ id: 1, opts: { requestVertexNormals: true } });
  });

  it.each([
    ['create throws', () => { fake.createThrows = true; }, async () => {}],
    ['render error', () => {}, async (w) => act(() => w.scene.renderError.raise(w.scene, new Error('shader')))],
    ['WebGL context lost', () => {}, async (w) => act(() => { w.canvas.dispatchEvent(new Event('webglcontextlost')); })],
    ['pose callback throws', () => {}, async (w, store) => { store.getVehicles = () => { throw new Error('bad'); }; frame(w); }],
  ])('failure (%s) calls onFail once', async (_name, before, trigger) => {
    before();
    const { onFail, store } = setup();
    if (fake.createThrows) {
      await waitFor(() => expect(onFail).toHaveBeenCalledTimes(1));
      return;
    }
    await widgetReady();
    await act(async () => {});
    await trigger(fake.widgets[0], store);
    await trigger(fake.widgets[0], store);
    expect(onFail).toHaveBeenCalledTimes(1);
  });

  it('unmount destroys the widget, removes listeners and clears the credits offset', async () => {
    const { unmount, store, onCreditsHeight } = setup();
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    unmount();
    expect(w.destroyed).toBe(true);
    expect(w.scene.preRender.fns.size).toBe(0);
    expect(store.listeners.size).toBe(0);
    expect(onCreditsHeight).toHaveBeenLastCalledWith(0);
  });
});
