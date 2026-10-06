import { useEffect, useRef } from 'react';

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
 * When fpvMode is true: first-person view using the vehicle's orientation.
 *
 * Extracts the heading from the entity's slerped orientation quaternion
 * (60 fps via CallbackProperty) instead of from vehicleList (5 Hz React
 * state), giving buttery-smooth camera rotation that matches the model.
 *
 * The entity orientation in useUavMarkers is built as:
 *   entityOri = enuRotation(pos) * hprQuat(heading,pitch,roll) * modelFixQuat
 *
 * To recover heading we reverse this:
 *   hprQuat = inverse(enuRotation) * entityOri * inverse(modelFix)
 *   heading = HeadingPitchRoll.fromQuaternion(hprQuat).heading
 */
export default function useFollowUav(cesiumRef, viewerRef, entitiesRef, followSysId, viewerReady, fpvMode) {
  const followSysIdRef = useRef(followSysId);
  followSysIdRef.current = followSysId;

  const fpvModeRef = useRef(fpvMode);
  fpvModeRef.current = fpvMode;

  const initializedRef = useRef(false);

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

    // Inverse of the model-fix rotation applied in useUavMarkers
    const modelFixHpr = new Cesium.HeadingPitchRoll(
      -Cesium.Math.PI, Cesium.Math.PI_OVER_TWO, 0,
    );
    const invModelFix = Cesium.Quaternion.inverse(
      Cesium.Quaternion.fromHeadingPitchRoll(modelFixHpr),
      new Cesium.Quaternion(),
    );

    // Scratch variables — reused every frame to avoid allocations at 60 fps
    const scratchTransform = new Cesium.Matrix4();
    const scratchQuat1 = new Cesium.Quaternion();
    const scratchQuat2 = new Cesium.Quaternion();
    const scratchMat3 = new Cesium.Matrix3();
    const scratchHpr = new Cesium.HeadingPitchRoll();

    const onPreRender = () => {
      const sid = followSysIdRef.current;
      if (sid == null) return;

      const entity = entitiesRef.current.uavMap?.[sid];
      if (!entity) return;

      const time = viewer.clock.currentTime;
      const position = entity.position?.getValue(time);
      const orientation = entity.orientation?.getValue(time);
      if (!position || !orientation) return;

      // Build ENU frame at entity position (also used for lookAtTransform)
      Cesium.Transforms.eastNorthUpToFixedFrame(position, undefined, scratchTransform);

      // Extract smooth heading/pitch/roll from the slerped orientation quaternion
      Cesium.Quaternion.multiply(orientation, invModelFix, scratchQuat1);
      Cesium.Matrix4.getMatrix3(scratchTransform, scratchMat3);
      Cesium.Quaternion.fromRotationMatrix(scratchMat3, scratchQuat2);
      Cesium.Quaternion.inverse(scratchQuat2, scratchQuat2);
      Cesium.Quaternion.multiply(scratchQuat2, scratchQuat1, scratchQuat1);
      Cesium.HeadingPitchRoll.fromQuaternion(scratchQuat1, scratchHpr);

      if (fpvModeRef.current) {
        // FPV mode: camera looks through vehicle's eyes
        // Unlock from lookAtTransform first if needed
        if (initializedRef.current) {
          viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
        }
        // Un-swap pitch/roll: useUavMarkers swaps them for the 3D model
        // (modelPitch = -vehicle_roll, modelRoll = vehicle_pitch)
        viewer.camera.setView({
          destination: position,
          orientation: {
            heading: scratchHpr.heading,
            pitch: scratchHpr.roll,
            roll: -scratchHpr.pitch,
          },
        });
        initializedRef.current = true;
      } else {
        // Chase-cam mode
        const smoothHeading = scratchHpr.heading;

        if (!initializedRef.current) {
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
