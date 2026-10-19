import { useEffect } from 'react';
import { fpvHoldReason } from '../../../utils/fpvView';
import { createPoseMath, noseOrientation } from '../utils/fpvPose';

/**
 * Pure helper: returns sysId if the vehicle exists in vehicleList, else null.
 * Exported for testability.
 */
export function resolveFollowTarget(sysId, vehicleList) {
  if (sysId == null) return null;
  if (!vehicleList || vehicleList.length === 0) return null;
  const found = vehicleList.some(v => v.sys_id === sysId);
  return found ? sysId : null;
}

/**
 * RC FPV: the main map's first-person camera on the RC target
 * (docs/design/gcs_video_streams.md, section 5 "Synthetic view", RC). The map
 * never chases a UAV otherwise.
 *
 * The camera looks through the vehicle's nose, using the model's slerped
 * orientation (utils/fpvPose.js), so it moves with the model. It holds its last
 * valid pose while the telemetry is not current, the vehicle is unlinked or
 * its snapshot position/attitude is not finite (fpvHoldReason), and resumes on
 * the next valid one. The viewer runs in request-render mode, so FPV requests
 * a render per telemetry (pose) update.
 *
 * @param {number|null} fpvSysId - the RC target while RC owns the camera, else null.
 */
export default function useFollowUav(cesiumRef, viewerRef, entitiesRef, fpvSysId, viewerReady, storeRef) {
  // One render per pose update while first-person, plus one on entry.
  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !viewerReady || fpvSysId == null) return undefined;
    viewer.scene.requestRender();
    return storeRef?.current?.subscribe?.(() => viewer.scene.requestRender());
  }, [fpvSysId, viewerReady]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer || !viewerReady || fpvSysId == null) return undefined;

    // Scratch-backed pose math — no allocations at 60 fps
    const poseMath = createPoseMath(Cesium);

    const onPreRender = () => {
      const entity = entitiesRef.current.uavMap?.[fpvSysId];
      if (!entity) return;

      const time = viewer.clock.currentTime;
      const position = entity.position?.getValue(time);
      const orientation = entity.orientation?.getValue(time);
      if (!position || !orientation) return;

      // Hold the last valid pose: telemetry not current, no link, or no finite position/attitude.
      const store = storeRef.current;
      if (fpvHoldReason(store.getVehicles()[fpvSysId], store.isTelemetryFresh(fpvSysId))) return;
      viewer.camera.setView({ destination: position, orientation: noseOrientation(poseMath.vehicleHpr(position, orientation)) });
    };

    viewer.scene.preRender.addEventListener(onPreRender);
    return () => viewer.scene.preRender.removeEventListener(onPreRender);
  }, [fpvSysId, viewerReady]);
}
