import { useEffect, useRef } from 'react';
import { fpvHoldReason } from '../../../utils/fpvView';
import { createPoseMath, noseOrientation } from '../utils/fpvPose';

/**
 * Pure helper: returns followSysId if the vehicle exists in vehicleList, else null.
 * Exported for testability.
 */
export function resolveFollowTarget(followSysId, vehicleList) {
  if (followSysId == null) return null;
  if (!vehicleList || vehicleList.length === 0) return null;
  const found = vehicleList.some(v => v.sys_id === followSysId);
  return found ? followSysId : null;
}

/**
 * Chase-cam / FPV-cam that follows a UAV.
 *
 * When fpvMode is false (default): chase-cam that orbits with smooth heading.
 * When fpvMode is true (RC FPV): first-person view using the vehicle's
 * orientation. In FPV the camera holds its last valid pose while the telemetry
 * is not current, the vehicle is unlinked or its snapshot position/attitude is
 * not finite (fpvHoldReason), and resumes on the next valid one. The viewer
 * runs in request-render mode, so FPV requests a render per telemetry (pose)
 * update.
 *
 * Heading, pitch and roll come from the model's slerped orientation
 * (utils/fpvPose.js), so the camera moves with the model.
 */
export default function useFollowUav(cesiumRef, viewerRef, entitiesRef, followSysId, viewerReady, fpvMode, storeRef) {
  const followSysIdRef = useRef(followSysId);
  followSysIdRef.current = followSysId;

  const fpvModeRef = useRef(fpvMode);
  fpvModeRef.current = fpvMode;

  const initializedRef = useRef(false);
  // The last frame was first-person: the chase view must re-initialize its
  // range instead of reading it off a camera that is no longer orbiting.
  const wasFpvRef = useRef(false);

  // One render per pose update while first-person, plus one on entry.
  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || !viewerReady || followSysId == null || !fpvMode) return undefined;
    viewer.scene.requestRender();
    return storeRef?.current?.subscribe?.(() => viewer.scene.requestRender());
  }, [followSysId, viewerReady, fpvMode]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer || !viewerReady) return;

    if (followSysId == null) {
      if (initializedRef.current) {
        viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
        initializedRef.current = false;
      }
      return;
    }

    initializedRef.current = false;

    // Scratch-backed pose math — no allocations at 60 fps
    const poseMath = createPoseMath(Cesium);
    const scratchTransform = poseMath.transform;

    const onPreRender = () => {
      const sid = followSysIdRef.current;
      if (sid == null) return;

      const entity = entitiesRef.current.uavMap?.[sid];
      if (!entity) return;

      const time = viewer.clock.currentTime;
      const position = entity.position?.getValue(time);
      const orientation = entity.orientation?.getValue(time);
      if (!position || !orientation) return;

      // Smooth heading/pitch/roll; also builds the ENU frame used for lookAtTransform
      const scratchHpr = poseMath.vehicleHpr(position, orientation);

      if (fpvModeRef.current) {
        // Hold the last valid pose: telemetry not current, no link, or no finite position/attitude.
        const store = storeRef.current;
        if (fpvHoldReason(store.getVehicles()[sid], store.isTelemetryFresh(sid))) return;
        // FPV mode: camera looks through vehicle's eyes
        // Unlock from lookAtTransform first if needed
        if (initializedRef.current) {
          viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
        }
        viewer.camera.setView({ destination: position, orientation: noseOrientation(scratchHpr) });
        initializedRef.current = true;
        wasFpvRef.current = true;
      } else {
        // Chase-cam mode
        const smoothHeading = scratchHpr.heading;

        if (!initializedRef.current || wasFpvRef.current) {
          wasFpvRef.current = false;
          viewer.camera.lookAtTransform(
            scratchTransform,
            new Cesium.HeadingPitchRange(smoothHeading, Cesium.Math.toRadians(-25), 200),
          );
          initializedRef.current = true;
        } else {
          const p = viewer.camera.pitch;
          const r = Cesium.Cartesian3.magnitude(viewer.camera.position);
          viewer.camera.lookAtTransform(
            scratchTransform,
            new Cesium.HeadingPitchRange(smoothHeading, p, r),
          );
        }
      }
    };

    viewer.scene.preRender.addEventListener(onPreRender);

    return () => {
      viewer.scene.preRender.removeEventListener(onPreRender);
      if (initializedRef.current) {
        viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
        initializedRef.current = false;
      }
    };
  }, [followSysId, viewerReady]);
}
