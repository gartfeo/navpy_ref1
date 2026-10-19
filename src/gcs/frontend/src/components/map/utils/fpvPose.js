/**
 * First-person pose math shared by the RC FPV map camera and the vehicle
 * window's synthetic view (docs/design/gcs_video_streams.md, section 5
 * "Synthetic view").
 *
 * The UAV model's orientation in useUavMarkers is built as
 *   entityOri = enuRotation(pos) * hprQuat(heading, pitch, roll) * modelFixQuat
 * so the vehicle's smooth heading/pitch/roll is recovered as
 *   hprQuat = inverse(enuRotation) * entityOri * inverse(modelFix).
 * Reading it off the slerped model orientation (60 fps) instead of the 5 Hz
 * snapshot gives camera motion that matches the model.
 *
 * Scratch-backed: one instance per consumer, no allocation per frame.
 */
export function createPoseMath(Cesium) {
  const invModelFix = Cesium.Quaternion.inverse(
    Cesium.Quaternion.fromHeadingPitchRoll(new Cesium.HeadingPitchRoll(-Cesium.Math.PI, Cesium.Math.PI_OVER_TWO, 0)),
    new Cesium.Quaternion(),
  );
  const transform = new Cesium.Matrix4();
  const q1 = new Cesium.Quaternion();
  const q2 = new Cesium.Quaternion();
  const m3 = new Cesium.Matrix3();
  const hpr = new Cesium.HeadingPitchRoll();

  return {
    /** East-north-up frame at the last `vehicleHpr` position. */
    transform,
    /** The vehicle's heading/pitch/roll (model axes) from the model's world pose; fills `transform`. */
    vehicleHpr(position, orientation) {
      Cesium.Transforms.eastNorthUpToFixedFrame(position, undefined, transform);
      Cesium.Quaternion.multiply(orientation, invModelFix, q1);
      Cesium.Matrix4.getMatrix3(transform, m3);
      Cesium.Quaternion.fromRotationMatrix(m3, q2);
      Cesium.Quaternion.inverse(q2, q2);
      Cesium.Quaternion.multiply(q2, q1, q1);
      return Cesium.HeadingPitchRoll.fromQuaternion(q1, hpr);
    },
  };
}

/**
 * Camera orientation looking through the vehicle's nose. useUavMarkers swaps
 * pitch and roll for the 3D model (model pitch = -vehicle roll, model roll =
 * vehicle pitch); this un-swaps them.
 */
export function noseOrientation(hpr) {
  return { heading: hpr.heading, pitch: hpr.roll, roll: -hpr.pitch };
}
