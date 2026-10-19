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
      // The context Cesium created (WebGL 2); losing it raises webglcontextlost.
      this.contextsReleased = 0;
      const lose = { loseContext: () => { this.contextsReleased += 1; this.canvas.dispatchEvent(new Event('webglcontextlost')); } };
      const gl = { getExtension: (name) => (name === 'WEBGL_lose_context' ? lose : null) };
      this.canvas.getContext = (type) => (type === 'webgl2' ? gl : null);
      // As in Cesium: without a creditContainer option the credit container is
      // the whole widget; the credit line is the CreditDisplay's container.
      this.creditContainer = container;
      this.creditDisplay = { container: document.createElement('div') };
      this.camera = { setView: vi.fn() };
      this.scene = {
        globe: {},
        screenSpaceCameraController: { enableInputs: true },
        renderError: event(),
        preRender: event(),
        postRender: event(),
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
vi.mock('./map/utils/observeHeight', () => ({
  observeHeight: (el, onHeight) => {
    fake.observed = el;
    onHeight(24);
    return () => { fake.observed = null; };
  },
}));
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

function setup({ sysId = 1, ionTerrain = false, poseFn, fresh = true, expanded = false } = {}) {
  const store = makeStore();
  store.fresh = fresh;
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
      expanded={expanded}
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
// One drawn frame: preRender, then postRender.
const frame = (widget) => act(() => { widget.scene.preRender.raise(); widget.scene.postRender.raise(); });

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

  const emit = (store) => act(() => store.listeners.forEach((fn) => fn()));
  const pictureBox = (getByTestId) => getByTestId('synthetic-widget').parentElement;

  it('"Waiting for pose": one text row with the picture collapsed until the UAV has a valid pose', async () => {
    const { getByText, getByTestId, queryByText, store } = setup({ fresh: false });
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    expect(getByText('vehicleWindow.waitingForPose UAV 1')).toBeInTheDocument();
    // Collapsed, not a 16:9 box over the map; the widget stays mounted.
    expect(pictureBox(getByTestId).style.height).toBe('0px');
    frame(w);
    emit(store);
    expect(w.camera.setView).not.toHaveBeenCalled();
    expect(getByText('vehicleWindow.waitingForPose UAV 1')).toBeInTheDocument();
    // Readiness comes from the telemetry update: a collapsed canvas never renders.
    store.fresh = true;
    emit(store);
    expect(pictureBox(getByTestId).style.height).not.toBe('0px');
    // Expanded, still covered until a frame is drawn with this UAV's pose.
    expect(getByText('vehicleWindow.waitingForPose UAV 1')).toBeInTheDocument();
    frame(w);
    expect(queryByText(/waitingForPose/)).toBeNull();
    expect(w.camera.setView).toHaveBeenCalledWith({ destination: pose.position, orientation: { heading: 1, pitch: 3, roll: -2 } });
    expect(getByText('vehicleWindow.syntheticTag')).toBeInTheDocument();
  });

  it('maximized, waiting keeps the large area and covers it with the text', async () => {
    const { getByText, getByTestId } = setup({ fresh: false, expanded: true });
    await widgetReady();
    expect(getByText('vehicleWindow.waitingForPose UAV 1')).toBeInTheDocument();
    expect(pictureBox(getByTestId).style.height).not.toBe('0px');
  });

  it('a badge tap to another UAV covers the old view until the new UAV has a pose', async () => {
    const { getByText, queryByText, rerenderWith, store } = setup();
    await widgetReady();
    const [w] = fake.widgets;
    await act(async () => {});
    frame(w);
    expect(queryByText(/waitingForPose/)).toBeNull();
    w.camera.setView.mockClear();
    store.vehicles = { 1: linked, 2: { ...linked, link_ok: false } };
    rerenderWith({ sysId: 2, label: 'UAV 2' });
    expect(fake.widgets).toHaveLength(1);
    frame(w);
    expect(w.camera.setView).toHaveBeenCalledTimes(0);
    expect(getByText('vehicleWindow.waitingForPose UAV 2')).toBeInTheDocument();
    store.vehicles = { 1: linked, 2: linked };
    emit(store);
    frame(w);
    expect(queryByText(/waitingForPose/)).toBeNull();
  });

  it('valid pose for the new UAV, then telemetry lost before a frame: the cover stays', async () => {
    const { getByText, rerenderWith, store } = setup({ expanded: true });
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    frame(w);
    rerenderWith({ sysId: 2, label: 'UAV 2' });
    // UAV 2 is valid on the telemetry update...
    emit(store);
    // ...but telemetry drops before the next frame: UAV 1's view must stay covered.
    store.fresh = false;
    frame(w);
    frame(w);
    expect(getByText('vehicleWindow.waitingForPose UAV 2')).toBeInTheDocument();
    store.fresh = true;
    frame(w);
    expect(() => getByText('vehicleWindow.waitingForPose UAV 2')).toThrow();
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

  it('the SYNTHETIC tag wraps at the width it is given (maximized: left of the small map), none by default', async () => {
    const { getByText, rerenderWith } = setup();
    await widgetReady();
    await act(async () => {});
    expect(getByText('vehicleWindow.syntheticTag').style.maxWidth).toBe('');
    rerenderWith({ tagMaxWidth: 140 });
    expect(getByText('vehicleWindow.syntheticTag').style.maxWidth).toBe('140px');
  });

  it('unmount destroys the widget, releases its WebGL context, removes listeners and clears the credits offset', async () => {
    const { unmount, store, onCreditsHeight, onFail } = setup();
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    unmount();
    expect(w.destroyed).toBe(true);
    // Released at once, not left for the garbage collector (the browser's context
    // limit would drop the main map's); its context-lost event is no failure.
    expect(w.contextsReleased).toBe(1);
    expect(onFail).not.toHaveBeenCalled();
    expect(w.scene.preRender.fns.size).toBe(0);
    expect(w.scene.postRender.fns.size).toBe(0);
    expect(store.listeners.size).toBe(0);
    expect(onCreditsHeight).toHaveBeenLastCalledWith(0);
  });

  it('measures the credit line, not the whole widget', async () => {
    const { onCreditsHeight } = setup();
    await widgetReady();
    await act(async () => {});
    expect(fake.observed).toBe(fake.widgets[0].creditDisplay.container);
    expect(fake.observed).not.toBe(fake.widgets[0].creditContainer);
    expect(onCreditsHeight).toHaveBeenCalledWith(24);
  });
});
