/**
 * Camera fly-to helpers for the Cesium map.
 *
 * Pure and import-free (the Cesium namespace is passed in) so the coordinate
 * ordering can be unit-tested via the Node-subprocess convention without pulling
 * in the WebGL viewer. Note the axis swap: callers pass (lat, lon) but
 * Cartesian3.fromDegrees expects (lon, lat, height).
 */

/** Build camera.flyTo options for an oblique view of a lat/lon target. */
export function flyToOptions(Cesium, lat, lon, height = 2000) {
  return {
    destination: Cesium.Cartesian3.fromDegrees(lon, lat, height),
    orientation: { heading: 0, pitch: Cesium.Math.toRadians(-60), roll: 0 },
    duration: 1.0,
  };
}
