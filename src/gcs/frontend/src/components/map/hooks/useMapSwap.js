import { useEffect, useRef } from 'react';

/**
 * The main map while the vehicle window is maximized and the map is small
 * (docs/design/gcs_video_streams.md, section 5 "Vehicle window", Maximize).
 * The map is only resized by CSS; this hook keeps its camera right:
 *
 * - Swapped: camera inputs off (which also stops the inertia of a gesture made
 *   just before), and a reset-view fit in flight is cancelled and deferred.
 * - Any fit asked for while swapped (reset view, the Free auto-centre) is
 *   deferred: a fit in the small box would use its aspect ratio.
 * - A deferred fit is dropped when follow or RC takes the camera.
 * - Swap back: inputs on, `viewer.resize()`, then the deferred fit once, only
 *   if the map is still Free.
 *
 * Wraps the viewer's fit: `fitRef` is the viewer's own reset view, and this
 * hook publishes the deferring one on `resetViewRef`.
 *
 * @param {object} p
 * @param {{ current: object|null }} p.viewerRef
 * @param {boolean} p.viewerReady
 * @param {{ current: Function|null }} p.fitRef - useCesiumViewer's reset view.
 * @param {{ current: boolean }} p.fitFlyingRef
 * @param {{ current: Function|null }} [p.resetViewRef] - the parent's reset view.
 * @param {boolean} p.swapped
 * @param {boolean} p.cameraTaken - follow or RC owns the map camera.
 */
export default function useMapSwap({
  viewerRef, viewerReady, fitRef, fitFlyingRef, resetViewRef, swapped, cameraTaken,
}) {
  const swappedRef = useRef(swapped);
  swappedRef.current = swapped;
  const cameraTakenRef = useRef(cameraTaken);
  cameraTakenRef.current = cameraTaken;
  const pendingRef = useRef(null);
  const lastFitRef = useRef(null);
  const appliedRef = useRef(false);

  useEffect(() => {
    if (!resetViewRef || !viewerReady) return;
    resetViewRef.current = (...args) => {
      if (swappedRef.current) {
        pendingRef.current = args;
        return;
      }
      lastFitRef.current = args;
      fitRef.current?.(...args);
    };
  }, [resetViewRef, viewerReady, fitRef]);

  // Before the swap effect: an RC entry that also swaps back drops the fit first.
  useEffect(() => {
    if (cameraTaken) pendingRef.current = null;
  }, [cameraTaken]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewerReady || !viewer || appliedRef.current === swapped) return;
    appliedRef.current = swapped;
    viewer.scene.screenSpaceCameraController.enableInputs = !swapped;
    if (swapped) {
      if (fitFlyingRef.current) {
        viewer.camera.cancelFlight();
        pendingRef.current = lastFitRef.current;
      }
    } else {
      viewer.resize();
      const pending = pendingRef.current;
      pendingRef.current = null;
      if (pending && !cameraTakenRef.current) {
        lastFitRef.current = pending;
        fitRef.current?.(...pending);
      }
    }
    viewer.scene.requestRender();
  }, [swapped, viewerReady, viewerRef, fitRef, fitFlyingRef]);
}
