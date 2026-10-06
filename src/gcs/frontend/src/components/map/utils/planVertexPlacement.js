/**
 * Keep the height returned by Cesium's terrain pick so a freshly clicked
 * vertex can render at once instead of waiting for another terrain clamp.
 */
export function cartographicToMapPoint(Cesium, cartographic) {
  return {
    lat: Cesium.Math.toDegrees(cartographic.latitude),
    lon: Cesium.Math.toDegrees(cartographic.longitude),
    height: cartographic.height,
  };
}

/**
 * Freshly picked points already have a terrain-surface height. Loaded plans do
 * not, so they retain Cesium's normal terrain-clamping behavior.
 */
export function planVertexBillboardPlacement(Cesium, point) {
  if (Number.isFinite(point?.height)) {
    return {
      position: Cesium.Cartesian3.fromDegrees(point.lon, point.lat, point.height),
      heightReference: Cesium.HeightReference.NONE,
    };
  }

  return {
    position: Cesium.Cartesian3.fromDegrees(point.lon, point.lat),
    heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
  };
}
