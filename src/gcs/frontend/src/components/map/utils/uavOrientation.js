/**
 * Orientation of the Peron15 model at `position` for a UAV attitude in
 * degrees. The model's axes differ from the aircraft's, so roll and pitch
 * swap and a fixed rotation follows (shared by the live model and the
 * closest-approach ghost).
 */
export function uavModelOrientation(Cesium, position, headingDeg, rollDeg, pitchDeg) {
  const vehicleHpr = new Cesium.HeadingPitchRoll(
    Cesium.Math.toRadians(headingDeg),
    -Cesium.Math.toRadians(rollDeg),
    Cesium.Math.toRadians(pitchDeg),
  );
  const vehicleQuat = Cesium.Transforms.headingPitchRollQuaternion(position, vehicleHpr);
  const modelFixHpr = new Cesium.HeadingPitchRoll(-Cesium.Math.PI, Cesium.Math.PI_OVER_TWO, 0);
  const modelFixQuat = Cesium.Quaternion.fromHeadingPitchRoll(modelFixHpr);
  return Cesium.Quaternion.multiply(vehicleQuat, modelFixQuat, new Cesium.Quaternion());
}
