// Main map stop detection with a fake Cesium
// (docs/design/gcs_map_stopped_notice.md, todo 87(a)).
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, waitFor } from '@testing-library/react';
import useCesiumViewer from './useCesiumViewer';

const { fake } = vi.hoisted(() => {
  const event = () => ({ addEventListener() {}, removeEventListener() {} });
  const fake = { viewers: [] };
  class Viewer {
    constructor(container, options) {
      this.options = options;
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
      this.imageryLayers = { removeAll() {}, addImageryProvider() {} };
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
    OpenStreetMapImageryProvider: class {},
    ScreenSpaceEventType: { LEFT_DOUBLE_CLICK: 'LEFT_DOUBLE_CLICK' },
    Ion: {},
  };
  return { fake };
});
vi.mock('cesium', () => fake.Cesium);
vi.mock('../../../utils/cesiumConfig', () => ({ cesiumIonToken: () => '' }));

function Harness({ onState }) {
  const state = useCesiumViewer(null, null, undefined, {});
  onState(state);
  return <div ref={state.containerRef} />;
}

async function start() {
  let state = null;
  render(<Harness onState={(s) => { state = s; }} />);
  await waitFor(() => expect(state.viewerReady).toBe(true));
  return { viewer: fake.viewers.at(-1), current: () => state };
}

describe('useCesiumViewer map stop', () => {
  beforeEach(() => {
    fake.viewers = [];
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
