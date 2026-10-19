import { useEffect } from 'react';

/**
 * Publishes a narrow pose source for the vehicle window's synthetic view
 * (docs/design/gcs_video_streams.md, section 5 "Synthetic view"): copies of
 * the same interpolated position and orientation the map's UAV model uses,
 * wired like the map camera ref. A UAV without a model (not in the fleet, or
 * no valid pose yet) is "no pose" (null), never a throw.
 */
export default function useUavPoseSource(poseSourceRef, cesiumRef, viewerRef, entitiesRef, viewerReady) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!poseSourceRef || !viewerReady || !Cesium || !viewer) return undefined;
    poseSourceRef.current = {
      pose(sysId) {
        if (viewer.isDestroyed()) return null;
        const entity = entitiesRef.current.uavMap?.[sysId];
        if (!entity) return null;
        const time = viewer.clock.currentTime;
        const position = entity.position?.getValue(time);
        const orientation = entity.orientation?.getValue(time);
        if (!position || !orientation) return null;
        return {
          position: Cesium.Cartesian3.clone(position),
          orientation: Cesium.Quaternion.clone(orientation),
        };
      },
    };
    return () => { poseSourceRef.current = null; };
  }, [poseSourceRef, viewerReady]);
}
