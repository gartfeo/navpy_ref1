/**
 * Distance along the ground velocity at which the flight path marker's point
 * is projected. Far enough that the marker shows the direction of travel, not
 * a point near the aircraft (at 10 km a 25 m/s UAV's own offset is lost in a
 * pixel), so it reads like a cockpit flight path vector.
 */
export const FPM_PROJECTION_M = 10000;

/**
 * Where the flight path marker draws in the picture, in CSS px from its
 * top-left, or null when the direction of travel is outside the picture or
 * behind the camera.
 *
 * @param {object} Cesium
 * @param {object} scene
 * @param {object} position - the camera (UAV) position, Cartesian3.
 * @param {number[]} enu - the smoothed ground velocity, east/north/up.
 * @param {{width: number, height: number}} size - the picture.
 * @returns {{x: number, y: number}|null}
 */
export function flightPathMarkerPoint(Cesium, scene, position, enu, size) {
  const { Cartesian3, Matrix4, Transforms, SceneTransforms } = Cesium;
  const dir = new Cartesian3(enu[0], enu[1], enu[2]);
  if (Cartesian3.magnitude(dir) === 0) return null;
  Cartesian3.normalize(dir, dir);
  const toFixed = Transforms.eastNorthUpToFixedFrame(position);
  Matrix4.multiplyByPointAsVector(toFixed, dir, dir);
  const ahead = Cartesian3.add(position, Cartesian3.multiplyByScalar(dir, FPM_PROJECTION_M, new Cartesian3()), new Cartesian3());
  // Behind the camera: the projection would mirror into the picture.
  const toAhead = Cartesian3.subtract(ahead, scene.camera.positionWC, new Cartesian3());
  if (Cartesian3.dot(toAhead, scene.camera.directionWC) <= 0) return null;
  const win = SceneTransforms.worldToWindowCoordinates(scene, ahead);
  if (!win || !Number.isFinite(win.x) || !Number.isFinite(win.y)) return null;
  if (win.x < 0 || win.y < 0 || win.x > size.width || win.y > size.height) return null;
  return { x: Math.round(win.x), y: Math.round(win.y) };
}
