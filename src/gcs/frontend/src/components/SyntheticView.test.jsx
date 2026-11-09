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
  const fake = { widgets: [], terrainCalls: [], baseLayers: [], hudProps: [], routes: [], marks: [], createThrows: false, event };
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
      this.camera = { setView: vi.fn(), frustum: { fov: 1.0472 } };
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
    Cartesian3: Vec,
    Color: { fromCssColorString: (s) => ({ css: s }) },
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
vi.mock('./map/utils/baseLayers', () => ({
  applyBaseLayer: (_Cesium, widget, id, onFailed) => {
    fake.baseLayers.push({ widget, id, onFailed });
  },
}));
// The scene marks, recorded: their drawing is sceneMarks.test.js's.
vi.mock('./map/utils/sceneMarks', () => ({
  createSceneMarks: (_Cesium, scene) => {
    const marks = { scene, updates: [], destroyed: false };
    fake.marks.push(marks);
    return {
      update: (m) => { marks.updates.push(m); return true; },
      destroy: () => { marks.destroyed = true; },
    };
  },
}));
// The flight path marker's projection, driven per test: its math is flightPathMarker.test.js's.
vi.mock('./map/utils/flightPathMarker', () => ({
  flightPathMarkerPoint: (_Cesium, _scene, position, enu, size) => fake.fpmPoint?.({ position, enu, size }) ?? null,
}));
// The route overlay, recorded: its drawing is routeOverlay.test.js's.
vi.mock('./map/utils/routeOverlay', () => ({
  createRouteOverlay: (_Cesium, scene) => {
    const overlay = { scene, updates: [], destroyed: false };
    fake.routes.push(overlay);
    return {
      update: (route, opts) => { overlay.updates.push({ route, opts }); return true; },
      destroy: () => { overlay.destroyed = true; },
    };
  },
}));
// The shared HUD, recorded: its own rendering is TelemetryHud.test.jsx's.
vi.mock('./TelemetryHud', () => ({
  default: (props) => {
    fake.hudProps.push(props);
    return <div data-testid="hud" data-sysid={props.sysId} />;
  },
}));
vi.mock('./map/utils/fpvPose', () => ({
  createPoseMath: () => ({ transform: {}, vehicleHpr: () => ({ heading: 1, pitch: 2, roll: 3 }) }),
  noseOrientation: (hpr) => ({ heading: hpr.heading, pitch: hpr.roll, roll: -hpr.pitch }),
}));

// The gimbal view, driven per test: its math is gimbalView.test.js's.
vi.mock('./map/utils/gimbalView', async (importOriginal) => ({
  ...(await importOriginal()),
  GIMBAL_ON_AXIS_RAD: 0.1,
  liveGimbalView: (vehicle) => fake.gimbal?.(vehicle) ?? null,
  gimbalOrientation: () => fake.gimbalOrientation,
}));
vi.mock('../utils/plannerConfig', () => ({ getDeviceConfigs: () => [] }));

const linked = { link_ok: true, lat: 32, lon: 34, alt_rel: 100, roll: 1, pitch: 2, heading: 90 };

function makeStore(vehicles = { 1: linked, 2: linked }) {
  const listeners = new Set();
  return {
    vehicles,
    fresh: true,
    listeners,
    subscribe: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
    getVehicles() { return this.vehicles; },
    getVehicleList() { return Object.values(this.vehicles); },
    isTelemetryFresh() { return this.fresh; },
  };
}

const pose = { position: { x: 1, y: 2, z: 3 }, orientation: { w: 1 } };

function setup({ sysId = 1, ionTerrain = false, poseFn, fresh = true, expanded = false, ...extra } = {}) {
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
      baseLayer="terrain"
      {...extra}
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
  fake.baseLayers.length = 0;
  fake.hudProps.length = 0;
  fake.routes.length = 0;
  fake.marks.length = 0;
  fake.fpmPoint = null;
  fake.createThrows = false;
  fake.gimbal = null;
  fake.gimbalOrientation = null;
  vi.spyOn(console, 'warn').mockImplementation(() => {});
});

describe('SyntheticView', () => {
  it('one widget: no default imagery, no sky box, render on request, inputs off, unlit as the map; destroyed on unmount', async () => {
    const { unmount } = setup();
    await widgetReady();
    await act(async () => {});
    expect(fake.widgets).toHaveLength(1);
    const [w] = fake.widgets;
    expect(w.options).toMatchObject({ baseLayer: false, skyBox: false, requestRenderMode: true, msaaSamples: 1, showRenderLoopErrors: false });
    expect(w.scene.screenSpaceCameraController.enableInputs).toBe(false);
    expect(w.scene.globe.enableLighting).toBe(false);
    expect(w.scene.globe.showGroundAtmosphere).toBe(false);
    unmount();
    expect(w.destroyed).toBe(true);
  });

  it("the main map's base layer, its own instance, following the operator's pick; a failure is reported for that layer", async () => {
    const onBaseLayerFailed = vi.fn();
    const { rerenderWith } = setup({ baseLayer: 'satellite', onBaseLayerFailed });
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    expect(fake.baseLayers.map((l) => [l.widget, l.id])).toEqual([[w, 'satellite']]);
    rerenderWith({ baseLayer: 'street' });
    expect(fake.baseLayers.map((l) => l.id)).toEqual(['satellite', 'street']);
    const err = new Error('refused');
    fake.baseLayers[1].onFailed(err);
    expect(onBaseLayerFailed).toHaveBeenCalledWith('street', err);
  });

  it("RC control's HUD for the window's UAV once a frame is drawn, never while waiting; no controls", async () => {
    const { container, queryByTestId, getByTestId, rerenderWith, store } = setup();
    await widgetReady();
    await act(async () => {});
    expect(queryByTestId('hud')).toBeNull();
    frame(fake.widgets[0]);
    expect(getByTestId('hud').dataset.sysid).toBe('1');
    expect(getByTestId('synthetic-hud').style.pointerEvents).toBe('none');
    const props = fake.hudProps.at(-1);
    expect(props.layout).toEqual(expect.objectContaining({ heading: expect.any(Object), corner: expect.any(Object) }));
    // Display only: no sticks, mode buttons, arming or UAV selector in Synthetic.
    // The one button opens the map credits (owner, 2026-11-08); it flies nothing.
    const controls = [...container.querySelectorAll('button, input, select, [role="slider"]')];
    expect(controls.map((el) => el.getAttribute('aria-label'))).toEqual(['vehicleWindow.mapCredits']);
    // Another UAV: the HUD goes while its pose is awaited.
    store.vehicles = { 1: linked, 2: { ...linked, link_ok: false } };
    rerenderWith({ sysId: 2, label: 'UAV 2' });
    expect(queryByTestId('hud')).toBeNull();
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

  it('its own Ion terrain instance, as the main map requests it, once the main map has Ion terrain', async () => {
    const { rerenderWith } = setup();
    await widgetReady();
    await act(async () => {});
    expect(fake.terrainCalls).toHaveLength(0);
    rerenderWith({ ionTerrain: true });
    await waitFor(() => expect(fake.widgets[0].terrainProvider).toBe(fake.terrainCalls[0]));
    expect(fake.terrainCalls[0]).toEqual({ id: 1, opts: undefined });
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

  // Its width beside the HUD and the small map: utils/hudLayout.test.js.
  it('the SYNTHETIC tag has no wrap width before the picture is measured', async () => {
    const { getByText } = setup({ smallMap: { width: 200, height: 113 } });
    await widgetReady();
    await act(async () => {});
    expect(getByText('vehicleWindow.syntheticTag').style.maxWidth).toBe('');
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

  // Owner (2026-11-08): on the handheld the credit line took a fifth of the
  // small picture. At normal size a © button opens it; maximized it shows.
  it('at normal size the credit line hides behind a © button; maximized it shows', async () => {
    const { getByRole, queryByRole, rerenderWith } = setup();
    await widgetReady();
    await act(async () => {});
    const credits = fake.widgets[0].creditDisplay.container;
    expect(credits.style.display).toBe('none');
    const button = getByRole('button', { name: 'vehicleWindow.mapCredits' });
    expect(button).toHaveAttribute('aria-expanded', 'false');
    act(() => button.click());
    expect(credits.style.display).toBe('');
    expect(button).toHaveAttribute('aria-expanded', 'true');
    act(() => button.click());
    expect(credits.style.display).toBe('none');
    rerenderWith({ expanded: true });
    expect(credits.style.display).toBe('');
    expect(queryByRole('button', { name: 'vehicleWindow.mapCredits' })).toBeNull();
  });

  it('the © button keeps its own row, above the credit line once it is open', async () => {
    const { getByRole } = setup();
    await widgetReady();
    await act(async () => {});
    // jsdom lays nothing out: the © button sits on its own row at the bottom,
    // and above the credit line (the fake measures 24 px) once that is open.
    const button = getByRole('button', { name: 'vehicleWindow.mapCredits' });
    expect(button.style.bottom).toBe('4px');
    act(() => button.click());
    expect(button.style.bottom).toBe('28px');
  });

  it("draws the UAV's own route from the route source: the next number at normal size, all maximized", async () => {
    const route = { zone: 0, positions: [], track: [], next: 1 };
    const routeSourceRef = { current: { route: vi.fn((id) => (id === 1 ? route : null)) } };
    const { store, rerenderWith, unmount } = setup({ routeSourceRef });
    await widgetReady();
    await act(async () => {});
    const [overlay] = fake.routes;
    expect(overlay.scene).toBe(fake.widgets[0].scene);
    act(() => store.listeners.forEach((fn) => fn()));
    expect(overlay.updates.at(-1)).toEqual({ route, opts: { line: true, labels: 'next' } });
    rerenderWith({ expanded: true });
    await act(async () => {});
    expect(overlay.updates.at(-1).opts).toEqual({ line: true, labels: 'all' });
    rerenderWith({ sysId: 2, expanded: true });
    await act(async () => {});
    expect(overlay.updates.at(-1).route).toBeNull();
    unmount();
    expect(overlay.destroyed).toBe(true);
  });

  it('passes the route source to the HUD and leaves to the right panel what it shows', async () => {
    const routeSourceRef = { current: { route: () => null } };
    const { rerenderWith } = setup({ routeSourceRef, panelShown: true, expanded: true, smallMap: { width: 200, height: 113 } });
    await widgetReady();
    await act(async () => {});
    frame(fake.widgets[0]);
    const last = () => fake.hudProps.at(-1);
    expect(last().routeSourceRef).toBe(routeSourceRef);
    expect(last().layout.corner).toMatchObject({ status: false, mode: false, waypoint: 'full' });
    expect(last().layout.strip).toBeNull();
    rerenderWith({ panelShown: false });
    expect(last().layout.corner).toMatchObject({ status: true, mode: true });
    expect(last().layout.strip).not.toBeNull();
  });

  it("with live gimbal telemetry: looks where the gimbal camera looks, with its field of view; the tag says so (owner 2026-11-07)", async () => {
    fake.gimbal = () => ({ cameraToVehicle: [[0, 0, 1], [1, 0, 0], [0, 1, 0]], fovH: 0.5, fovV: 0.3 });
    fake.gimbalOrientation = { heading: 0.4, pitch: -0.7, roll: 0.05, offNose: 0.7 };
    const { getByText, store } = setup();
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    emit(store);
    frame(w);
    expect(w.camera.setView).toHaveBeenLastCalledWith({
      destination: pose.position, orientation: { heading: 0.4, pitch: -0.7, roll: 0.05 },
    });
    expect(w.camera.frustum.fov).toBe(0.5);
    expect(getByText('vehicleWindow.syntheticTagGimbal')).toBeInTheDocument();
    // Off the nose axis: the aircraft's horizon would not match the picture.
    expect(fake.hudProps.at(-1).layout.attitude).toBeNull();
    expect(fake.hudProps.at(-1).layout.corner).toBeTruthy();
    // Near the nose axis: the horizon is back.
    fake.gimbalOrientation = { heading: 0.4, pitch: -0.05, roll: 0, offNose: 0.05 };
    frame(w);
    expect(fake.hudProps.at(-1).layout.attitude).not.toBeNull();
    // A canvas taller than wide: Cesium reads fov as vertical, so it gets the
    // vertical view that keeps the camera's horizontal 0.5 (Codex f1).
    w.camera.frustum.aspectRatio = 0.6;
    frame(w);
    expect(2 * Math.atan(0.6 * Math.tan(w.camera.frustum.fov / 2))).toBeCloseTo(0.5, 9);
    // Telemetry gone: the nose view and its own field of view again.
    fake.gimbal = null;
    frame(w);
    expect(w.camera.setView).toHaveBeenLastCalledWith({ destination: pose.position, orientation: { heading: 1, pitch: 3, roll: -2 } });
    expect(w.camera.frustum.fov).toBe(1.0472);
    expect(getByText('vehicleWindow.syntheticTag')).toBeInTheDocument();
  });
});

// What cockpit synthetic vision and UAV studies show (docs/design/synthetic_view_marks.md):
// a held picture greys out, the dock and delivery hub are marked, the fleet shows,
// and a flight path marker shows where the UAV is going.
describe('Synthetic view marks (owner 2026-11-09)', () => {
  const emitAll = (store) => act(() => store.listeners.forEach((fn) => fn()));
  const drawn = async (opts) => {
    const view = setup(opts);
    await widgetReady();
    await act(async () => {});
    const [w] = fake.widgets;
    frame(w);
    return { ...view, w };
  };

  it('a held picture greys out and shows no flight path marker', async () => {
    fake.fpmPoint = () => ({ x: 50, y: 40 });
    const { getByTestId, queryByTestId, rerenderWith, store, w } = await drawn();
    store.vehicles = { ...store.vehicles, 1: { ...linked, velocity_ned: [20, 0, 0] } };
    emitAll(store);
    frame(w);
    expect(getByTestId('synthetic-widget').style.filter).toBe('');
    expect(queryByTestId('synthetic-fpm')).not.toBeNull();
    rerenderWith({ held: true });
    frame(w);
    expect(getByTestId('synthetic-widget').style.filter).toBe('grayscale(1) brightness(0.7)');
    expect(queryByTestId('synthetic-fpm')).toBeNull();
  });

  it('marks the dock with its distance, the delivery hub, and the other fleet UAVs', async () => {
    const dock = { lat: 32.001, lon: 34 };
    const hub = { lat: 31.99, lon: 34, type: 'building' };
    const { store } = await drawn({ marks: { dock, hub } });
    store.vehicles = {
      1: { ...linked, sys_id: 1, name: 's1-u1' },
      2: { ...linked, sys_id: 2, name: 's1-u2' },
      3: { ...linked, sys_id: 3, name: 's1-u3', link_ok: false },
    };
    emitAll(store);
    const last = fake.marks[0].updates.at(-1);
    expect(last.dock).toEqual(dock);
    expect(last.dockText).toBe('vehicleWindow.sceneDock 111 m'); // 0.001 deg of latitude
    expect(last.hub).toEqual(hub);
    expect(last.hubText).toBe('vehicleWindow.sceneHub');
    // Not the window's own UAV, not one whose link is lost; zone = its place in the fleet list.
    expect(last.fleet).toEqual([{ position: pose.position, label: 's1-u2', zone: 1 }]);
  });

  it('without marks, nothing but the fleet', async () => {
    const { store } = await drawn();
    emitAll(store);
    const last = fake.marks[0].updates.at(-1);
    expect(last.dock).toBeNull();
    expect(last.hub).toBeNull();
  });

  it('the flight path marker follows the measured ground velocity, smoothed, and hides when parked', async () => {
    const calls = [];
    fake.fpmPoint = (c) => { calls.push(c); return { x: 60, y: 30 }; };
    const { store, w, getByTestId, queryByTestId } = await drawn();
    // Parked: no direction of travel, so no marker.
    store.vehicles = { ...store.vehicles, 1: { ...linked, velocity_ned: [0.2, 0.1, 0] } };
    emitAll(store);
    frame(w);
    expect(queryByTestId('synthetic-fpm')).toBeNull();
    // Flying north at 20 m/s: the projection gets east/north/up.
    store.vehicles = { ...store.vehicles, 1: { ...linked, velocity_ned: [20, 0, -1] } };
    emitAll(store);
    frame(w);
    expect(calls.at(-1).enu).toEqual([0, 20, 1]);
    const marker = getByTestId('synthetic-fpm');
    expect(marker.style.left).not.toBe('');
    // A sudden change is smoothed, not followed in one step.
    store.vehicles = { ...store.vehicles, 1: { ...linked, velocity_ned: [0, 20, 0] } };
    emitAll(store);
    frame(w);
    const [e, n] = calls.at(-1).enu;
    expect(e).toBeLessThan(20);
    expect(n).toBeGreaterThan(0);
    // Codex P2: the backend drops a stale velocity (velocity_ned null); the marker goes.
    store.vehicles = { ...store.vehicles, 1: { ...linked, velocity_ned: null } };
    emitAll(store);
    frame(w);
    expect(queryByTestId('synthetic-fpm')).toBeNull();
  });

  it('no marker where the direction of travel is off the picture', async () => {
    fake.fpmPoint = () => null;
    const { store, w, queryByTestId } = await drawn();
    store.vehicles = { ...store.vehicles, 1: { ...linked, velocity_ned: [20, 0, 0] } };
    emitAll(store);
    frame(w);
    expect(queryByTestId('synthetic-fpm')).toBeNull();
  });
});
