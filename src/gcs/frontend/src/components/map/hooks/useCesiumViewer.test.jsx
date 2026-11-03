// Main map base layer and stop detection with a fake Cesium
// (docs/design/gcs_map_stopped_notice.md, todo 87(a)).
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, waitFor } from '@testing-library/react';
import useCesiumViewer from './useCesiumViewer';

const { fake } = vi.hoisted(() => {
  const event = () => ({ addEventListener() {}, removeEventListener() {} });
  const fake = { viewers: [], ionToken: null, ionImagery: null };
  class Event {
    constructor() { this.listeners = []; }
    addEventListener(fn) { this.listeners.push(fn); }
    raise(arg) { this.listeners.forEach((fn) => fn(arg)); }
  }
  class ImageryLayer {
    constructor(imageryProvider) {
      this.imageryProvider = imageryProvider;
      this.errorEvent = new Event();
      this.destroyed = false;
    }

    // Cesium ImageryLayer.fromProviderAsync: the layer exists at once; a
    // provider that fails raises errorEvent.
    static fromProviderAsync(promise) {
      const layer = new ImageryLayer(undefined);
      promise.then((p) => { layer.imageryProvider = p; }, (e) => layer.errorEvent.raise(e));
      return layer;
    }

    isDestroyed() { return this.destroyed; }
  }
  class Viewer {
    constructor(container, options) {
      this.options = options;
      // Cesium 1.138 Viewer.js: a defined imageryProvider only turns the
      // widget's default base layer off; only baseLayer adds imagery.
      const layers = options.baseLayer ? [options.baseLayer] : [];
      this.canvas = document.createElement('canvas');
      container.appendChild(this.canvas);
      // Cesium's own panel: the render loop calls the widget's showErrorPanel
      // when it stops on an error (CesiumWidget.js, loop catch and
      // _onRenderError).
      this.cesiumPanel = vi.fn();
      this.cesiumWidget = {
        showErrorPanel: this.cesiumPanel,
        screenSpaceEventHandler: { removeInputAction() {} },
      };
      this.clock = {};
      this.camera = { setView() {}, moveEnd: event(), flyTo() {} };
      this.imageryLayers = {
        layers,
        get length() { return layers.length; },
        get(i) { return layers[i]; },
        remove(layer, destroy) {
          layers.splice(layers.indexOf(layer), 1);
          if (destroy) layer.destroyed = true;
        },
        add(layer, i) { layers.splice(i, 0, layer); },
      };
      this.scene = {
        globe: { tileLoadProgressEvent: event() },
        skyAtmosphere: {},
        fog: {},
        sun: {},
        moon: {},
        skyBox: {},
        screenSpaceCameraController: {},
        requestRender() {},
      };
      fake.viewers.push(this);
    }

    isDestroyed() { return false; }
  }
  fake.Cesium = {
    Viewer,
    ImageryLayer,
    Credit: class { constructor(html) { this.html = html; } },
    OpenStreetMapImageryProvider: class { constructor(options) { this.url = options.url; } },
    UrlTemplateImageryProvider: class { constructor(options) { this.url = options.url; } },
    IonImageryProvider: { fromAssetId: (id) => fake.ionImagery ?? Promise.resolve({ ionAssetId: id }) },
    CesiumTerrainProvider: {
      fromIonAssetId: async () => ({ requestTileGeometry: async () => {} }),
    },
    ScreenSpaceEventType: { LEFT_DOUBLE_CLICK: 'LEFT_DOUBLE_CLICK' },
    Ion: {},
  };
  return { fake };
});
vi.mock('cesium', () => fake.Cesium);
vi.mock('../../../utils/cesiumConfig', () => ({ loadCesiumIonToken: async () => fake.ionToken }));

function Harness({ onState, options = {} }) {
  const state = useCesiumViewer(null, null, undefined, options);
  onState(state);
  return <div ref={state.containerRef} />;
}

async function start(options) {
  let state = null;
  const onState = (s) => { state = s; };
  const { rerender } = render(<Harness onState={onState} options={options} />);
  await waitFor(() => expect(state.viewerReady).toBe(true));
  return {
    viewer: fake.viewers.at(-1),
    current: () => state,
    pick: (next) => rerender(<Harness onState={onState} options={{ ...options, ...next }} />),
  };
}

const providers = (viewer) => viewer.imageryLayers.layers.map((l) => l.imageryProvider);

describe('useCesiumViewer base layer', () => {
  beforeEach(() => {
    fake.viewers = [];
    fake.ionToken = null;
    fake.ionImagery = null;
  });

  it('starts on the OpenTopoMap terrain layer by default', async () => {
    const { viewer, current } = await start();
    const [topo, ...rest] = providers(viewer);
    expect(topo).toBeInstanceOf(fake.Cesium.UrlTemplateImageryProvider);
    expect(topo.url).toBe('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png');
    expect(rest).toEqual([]);
    expect(current().terrainReady).toBe(false);
  });

  it('starts on the default when satellite is chosen without an Ion token', async () => {
    const { viewer } = await start({ baseLayerId: 'satellite' });
    expect(providers(viewer)[0]).toBeInstanceOf(fake.Cesium.UrlTemplateImageryProvider);
  });

  it('with an Ion token starts on satellite and the terrain upgrade keeps it', async () => {
    fake.ionToken = 'token';
    const { viewer, current } = await start({ baseLayerId: 'satellite' });
    await waitFor(() => expect(current().terrainReady).toBe(true));
    expect(providers(viewer)).toEqual([{ ionAssetId: 3 }]);
  });

  it('swaps the base imagery when another layer is picked', async () => {
    const { viewer, pick } = await start({ baseLayerId: 'terrain' });
    const first = viewer.imageryLayers.layers[0];
    pick({ baseLayerId: 'street' });
    await waitFor(() => expect(providers(viewer)[0]).toBeInstanceOf(fake.Cesium.OpenStreetMapImageryProvider));
    expect(providers(viewer)).toHaveLength(1);
    expect(first.isDestroyed()).toBe(true);
  });

  it('reports a base layer whose provider fails to load', async () => {
    fake.ionToken = 'token';
    const error = new Error('Ion refused');
    fake.ionImagery = Promise.reject(error);
    const onBaseLayerFailed = vi.fn();
    await start({ baseLayerId: 'satellite', onBaseLayerFailed });
    await waitFor(() => expect(onBaseLayerFailed).toHaveBeenCalledWith('satellite', error));
  });
});

describe('useCesiumViewer map stop', () => {
  beforeEach(() => {
    fake.viewers = [];
    fake.ionToken = null;
    vi.spyOn(console, 'error').mockImplementation(() => {});
  });

  it('a normal start leaves the map running, with Cesium reporting loop errors', async () => {
    const { viewer, current } = await start();
    expect(current().mapStopped).toBe(false);
    expect(viewer.options.showRenderLoopErrors).toBe(true);
  });

  it('a lost WebGL context stops the map', async () => {
    const { viewer, current } = await start();
    act(() => { viewer.canvas.dispatchEvent(new Event('webglcontextlost')); });
    expect(current().mapStopped).toBe(true);
    expect(console.error).toHaveBeenCalled();
  });

  it('a render-loop error stops the map and logs it instead of Cesium\'s panel', async () => {
    const { viewer, current } = await start();
    const error = new Error('Expected width to be greater than 0');
    const title = 'An error occurred while rendering.  Rendering has stopped.';
    act(() => { viewer.cesiumWidget.showErrorPanel(title, undefined, error); });
    expect(current().mapStopped).toBe(true);
    expect(viewer.cesiumPanel).not.toHaveBeenCalled();
    expect(console.error).toHaveBeenCalledWith(title, error);
  });

  it('both signals together leave the map stopped', async () => {
    const { viewer, current } = await start();
    act(() => {
      viewer.canvas.dispatchEvent(new Event('webglcontextlost'));
      viewer.cesiumWidget.showErrorPanel('stopped', undefined, new Error('lost'));
    });
    expect(current().mapStopped).toBe(true);
    expect(viewer.cesiumPanel).not.toHaveBeenCalled();
  });
});
