import { useEffect, useRef } from 'react';

/**
 * RC FPV and map-camera ownership (docs/design/gcs_video_streams.md, section 5
 * "Synthetic view", RC and "Map camera ownership"). Only RC takes the map
 * camera: RC shows a body FPV of the RC target on the main map. The map never
 * follows a UAV otherwise, and the vehicle window's synthetic view is a
 * separate Cesium view that never touches the map camera.
 *
 * | Event     | Result                                   |
 * |-----------|------------------------------------------|
 * | RC entry  | map camera saved; RC FPV of the RC target |
 * | RC exit   | the saved map camera restored            |
 *
 * @param {object} p
 * @param {boolean} p.rcEnabled
 * @param {number|null} p.rcTarget
 * @param {{ current: { capture: () => object, restore: (cam: object) => void } | null }} p.mapCameraRef
 */
export default function useFpvView({ rcEnabled, rcTarget, mapCameraRef }) {
  // Runs after CesiumMap's RC FPV hook (child effects run first), so a
  // restore never fights a still-locked first-person camera.
  const savedCameraRef = useRef(null);
  const wasRcRef = useRef(rcEnabled);
  useEffect(() => {
    const wasRc = wasRcRef.current;
    wasRcRef.current = rcEnabled;
    if (rcEnabled && !wasRc) {
      savedCameraRef.current = mapCameraRef.current?.capture() ?? null;
    } else if (!rcEnabled && wasRc) {
      const saved = savedCameraRef.current;
      savedCameraRef.current = null;
      if (saved) mapCameraRef.current?.restore(saved);
    }
  }, [rcEnabled, mapCameraRef]);

  return {
    /** The RC target the first-person map camera shows, or null. */
    fpvSysId: rcEnabled && rcTarget != null ? rcTarget : null,
    /** RC owns the camera: camera moves must not be persisted as the startup view. */
    cameraOwned: rcEnabled,
  };
}
