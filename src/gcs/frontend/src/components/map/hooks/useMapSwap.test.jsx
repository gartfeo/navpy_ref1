// The main map's camera while the vehicle window is maximized
// (docs/design/gcs_video_streams.md section 5, "Vehicle window", Maximize).
import { describe, it, expect, vi } from 'vitest';
import { renderHook } from '@testing-library/react';
import useMapSwap from './useMapSwap';
import { createFitFlight } from './useCesiumViewer';

function setup({ flying = false } = {}) {
  const viewer = {
    scene: { screenSpaceCameraController: { enableInputs: true }, requestRender: vi.fn() },
    camera: { cancelFlight: vi.fn() },
    resize: vi.fn(),
  };
  const fit = vi.fn();
  const fly = vi.fn();
  const refs = {
    viewerRef: { current: viewer },
    fitRef: { current: fit },
    flyRef: { current: fly },
    fitFlyingRef: { current: flying },
    resetViewRef: { current: null },
    flyToRef: { current: null },
  };
  const hook = renderHook((props) => useMapSwap({ ...refs, viewerReady: true, ...props }), {
    initialProps: { swapped: false, cameraTaken: false },
  });
  const set = (props) => hook.rerender(props);
  return {
    viewer, fit, fly, refs, set,
    resetView: (...a) => refs.resetViewRef.current(...a),
    flyTo: (...a) => refs.flyToRef.current(...a),
  };
}

describe('useMapSwap', () => {
  it('not swapped: the reset view fits and the fly-to flies at once', () => {
    const { fit, fly, resetView, flyTo } = setup();
    resetView('all');
    expect(fit).toHaveBeenCalledWith('all');
    flyTo(40.1, 44.5);
    expect(fly).toHaveBeenCalledWith(40.1, 44.5);
  });

  it('swapped: camera inputs off; a fit asked for meanwhile runs once after the swap back, after resize', () => {
    const { viewer, fit, set, resetView } = setup();
    set({ swapped: true, cameraTaken: false });
    expect(viewer.scene.screenSpaceCameraController.enableInputs).toBe(false);
    resetView('a');
    resetView('b');
    expect(fit).not.toHaveBeenCalled();
    set({ swapped: false, cameraTaken: false });
    expect(viewer.scene.screenSpaceCameraController.enableInputs).toBe(true);
    expect(viewer.resize).toHaveBeenCalledTimes(1);
    expect(fit).toHaveBeenCalledTimes(1);
    expect(fit).toHaveBeenCalledWith('b');
    expect(viewer.resize.mock.invocationCallOrder[0]).toBeLessThan(fit.mock.invocationCallOrder[0]);
  });

  it('a sidebar fly-to while swapped waits for the swap back and flies after resize', () => {
    // App swaps back first; the fly-to call still sees the small map and defers.
    const { viewer, fly, set, flyTo } = setup();
    set({ swapped: true, cameraTaken: false });
    flyTo(40.1, 44.5);
    expect(fly).not.toHaveBeenCalled();
    set({ swapped: false, cameraTaken: false });
    expect(fly).toHaveBeenCalledTimes(1);
    expect(fly).toHaveBeenCalledWith(40.1, 44.5);
    expect(viewer.resize.mock.invocationCallOrder[0]).toBeLessThan(fly.mock.invocationCallOrder[0]);
  });

  it('the newest deferred request wins across fit and fly-to', () => {
    const { fit, fly, set, resetView, flyTo } = setup();
    set({ swapped: true, cameraTaken: false });
    resetView('a');
    flyTo(1, 2);
    set({ swapped: false, cameraTaken: false });
    expect(fit).not.toHaveBeenCalled();
    expect(fly).toHaveBeenCalledWith(1, 2);
  });

  it('a fit in flight at the swap is cancelled and re-run after the swap back', () => {
    const { viewer, fit, refs, set, resetView } = setup();
    resetView('x');
    refs.fitFlyingRef.current = true;
    set({ swapped: true, cameraTaken: false });
    expect(viewer.camera.cancelFlight).toHaveBeenCalledTimes(1);
    refs.fitFlyingRef.current = false;
    set({ swapped: false, cameraTaken: false });
    expect(fit).toHaveBeenLastCalledWith('x');
    expect(fit).toHaveBeenCalledTimes(2);
  });

  it('any other flight at the swap (a coordinate fly-to) is cancelled without a rerun', () => {
    const { viewer, fit, fly, set, flyTo } = setup();
    flyTo(1, 2);
    set({ swapped: true, cameraTaken: false });
    expect(viewer.camera.cancelFlight).toHaveBeenCalledTimes(1);
    set({ swapped: false, cameraTaken: false });
    expect(fly).toHaveBeenCalledTimes(1);
    expect(fit).not.toHaveBeenCalled();
  });

  it('a pending request, then RC takes the camera and swaps back: dropped', () => {
    const { fit, fly, set, resetView, flyTo } = setup();
    set({ swapped: true, cameraTaken: false });
    resetView('centre');
    set({ swapped: false, cameraTaken: true });
    expect(fit).not.toHaveBeenCalled();
    set({ swapped: true, cameraTaken: false });
    flyTo(1, 2);
    set({ swapped: false, cameraTaken: true });
    expect(fly).not.toHaveBeenCalled();
  });
});

describe('createFitFlight', () => {
  // Like Camera.flyTo: a new flight cancels the one in progress synchronously.
  function fakeCamera() {
    const camera = { flight: null };
    camera.flyTo = vi.fn((options) => {
      camera.flight?.cancel();
      camera.flight = options;
    });
    return camera;
  }

  it('a replacement fit keeps the flag; it clears when the newest fit ends', () => {
    const camera = fakeCamera();
    const flying = { current: false };
    const fly = createFitFlight(camera, flying);
    fly({ destination: 'a' });
    expect(flying.current).toBe(true);
    fly({ destination: 'b' });
    expect(flying.current).toBe(true);
    expect(camera.flyTo).toHaveBeenLastCalledWith(expect.objectContaining({ destination: 'b', duration: 1 }));
    camera.flight.complete();
    expect(flying.current).toBe(false);
  });

  it('another camera flight cancelling the fit clears the flag', () => {
    const camera = fakeCamera();
    const flying = { current: false };
    createFitFlight(camera, flying)({ destination: 'a' });
    camera.flight.cancel();
    expect(flying.current).toBe(false);
  });
});
