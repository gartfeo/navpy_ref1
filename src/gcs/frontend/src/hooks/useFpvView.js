import { useEffect, useRef } from 'react';
import { PHASES } from './useMissionState';

/**
 * RC FPV and map-camera ownership (docs/design/gcs_video_streams.md, section 5
 * "Synthetic view", RC). Only RC takes the map camera: RC shows a body FPV of
 * the RC target on the main map. The vehicle window's synthetic view is a
 * separate Cesium view and never touches the map camera.
 *
 * Map camera: the free-map camera is saved whenever the map leaves the free
 * map (a chase may become RC), and restored when it comes back to the free
 * map only if RC took the camera meanwhile. A plain chase back to Free
 * centres the map, as before (deferred by the map while it is small).
 *
 * | Event                 | Result                                  |
 * |-----------------------|-----------------------------------------|
 * | RC entry              | RC FPV of the RC target                 |
 * | RC exit               | Free map with the saved map camera      |
 * | Chase set to Free     | map centred                             |
 *
 * @param {object} p
 * @param {string} p.phase
 * @param {number|null} p.followSysId
 * @param {boolean} p.rcEnabled
 * @param {number|null} p.rcTarget
 * @param {{ current: { capture: () => object, restore: (cam: object) => void } | null }} p.mapCameraRef
 * @param {() => void} p.centerView
 */
export default function useFpvView({ phase, followSysId, rcEnabled, rcTarget, mapCameraRef, centerView }) {
  // Runs after CesiumMap's follow hook (child effects run first), so a restore
  // or fly-to never fights a still-locked chase camera.
  const savedCameraRef = useRef(null);
  const cameraTakenRef = useRef(false);
  const prevRef = useRef({ follow: followSysId, rc: rcEnabled });
  useEffect(() => {
    const prev = prevRef.current;
    prevRef.current = { follow: followSysId, rc: rcEnabled };
    const wasFreeMap = prev.follow == null && !prev.rc;
    const isFreeMap = followSysId == null && !rcEnabled;

    if (wasFreeMap && !isFreeMap) {
      savedCameraRef.current = mapCameraRef.current?.capture() ?? null;
      cameraTakenRef.current = false;
    }
    if (rcEnabled) cameraTakenRef.current = true;
    if (isFreeMap && !wasFreeMap) {
      const saved = cameraTakenRef.current ? savedCameraRef.current : null;
      savedCameraRef.current = null;
      cameraTakenRef.current = false;
      if (saved) mapCameraRef.current?.restore(saved);
      else if (phase === PHASES.MONITOR && prev.follow != null) centerView();
    }
  }, [followSysId, rcEnabled, phase, mapCameraRef, centerView]);

  const fpvSysId = rcEnabled && rcTarget != null ? rcTarget : null;
  return {
    /** The map camera is first-person (RC FPV). */
    cameraFpv: fpvSysId != null,
    /** The RC target the first-person map camera shows. */
    fpvSysId,
    /** RC owns the camera: camera moves must not be persisted as the startup view. */
    cameraOwned: rcEnabled,
  };
}
