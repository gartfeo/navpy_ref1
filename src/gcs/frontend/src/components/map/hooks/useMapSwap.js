import { useEffect, useRef } from 'react';

/**
 * The main map while the vehicle window is maximized and the map is small
 * (docs/design/gcs_video_streams.md, section 5 "Vehicle window", Maximize).
 * The map is only resized by CSS; this hook keeps its camera right:
 *
 * - Swapped: camera inputs off (which also stops the inertia of a gesture made
 *   just before), and any camera flight is cancelled. A reset-view fit that
 *   was flying is deferred; a coordinate flight is not re-run (the operator
 *   just asked for the window).
 * - A reset-view fit or a coordinate fly-to asked for while swapped waits for
 *   the large map: in the small box it would use the small aspect ratio. The
 *   newest request wins. The caller swaps back first for a coordinate click.
 * - A deferred request is dropped when RC takes the camera.
 * - Swap back: inputs on, `viewer.resize()`, then the deferred request once.
 *
 * Wraps the viewer's own camera moves: `fitRef` and `flyRef` are useCesiumViewer's
 * reset view and fly-to, and this hook publishes the deferring ones on
 * `resetViewRef` and `flyToRef`.
 *
 * @param {object} p
 * @param {{ current: object|null }} p.viewerRef
 * @param {boolean} p.viewerReady
 * @param {{ current: Function|null }} p.fitRef - useCesiumViewer's reset view.
 * @param {{ current: Function|null }} [p.flyRef] - useCesiumViewer's fly-to.
 * @param {{ current: boolean }} p.fitFlyingRef
 * @param {{ current: Function|null }} [p.resetViewRef] - the parent's reset view.
 * @param {{ current: Function|null }} [p.flyToRef] - the parent's fly-to.
 * @param {boolean} p.swapped
 * @param {boolean} p.cameraTaken - RC owns the map camera.
 */
export default function useMapSwap({
  viewerRef, viewerReady, fitRef, flyRef, fitFlyingRef, resetViewRef, flyToRef, swapped, cameraTaken,
}) {
  const swappedRef = useRef(swapped);
  swappedRef.current = swapped;
  const cameraTakenRef = useRef(cameraTaken);
  cameraTakenRef.current = cameraTaken;
  // The deferred request: { move: fitRef | flyRef, args }.
  const pendingRef = useRef(null);
  const lastFitRef = useRef(null);
  const appliedRef = useRef(false);

  useEffect(() => {
    if (!viewerReady) return;
    const deferring = (moveRef, onRun) => (...args) => {
      if (swappedRef.current) {
        pendingRef.current = { move: moveRef, args };
        return;
      }
      onRun?.(args);
      moveRef.current?.(...args);
    };
    if (resetViewRef) resetViewRef.current = deferring(fitRef, (args) => { lastFitRef.current = args; });
    if (flyToRef && flyRef) flyToRef.current = deferring(flyRef);
  }, [resetViewRef, flyToRef, viewerReady, fitRef, flyRef]);

  // Before the swap effect: an RC entry that also swaps back drops the request first.
  useEffect(() => {
    if (cameraTaken) pendingRef.current = null;
  }, [cameraTaken]);

  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewerReady || !viewer || appliedRef.current === swapped) return;
    appliedRef.current = swapped;
    viewer.scene.screenSpaceCameraController.enableInputs = !swapped;
    if (swapped) {
      // Read before cancelling: the fit's cancel callback clears the flag.
      const fitFlying = fitFlyingRef.current;
      viewer.camera.cancelFlight();
      if (fitFlying && lastFitRef.current) pendingRef.current = { move: fitRef, args: lastFitRef.current };
    } else {
      viewer.resize();
      const pending = pendingRef.current;
      pendingRef.current = null;
      if (pending && !cameraTakenRef.current) {
        if (pending.move === fitRef) lastFitRef.current = pending.args;
        pending.move.current?.(...pending.args);
      }
    }
    viewer.scene.requestRender();
  }, [swapped, viewerReady, viewerRef, fitRef, fitFlyingRef]);
}
