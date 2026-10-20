/**
 * Camera close to a closest-approach mark (gcs_sim_closest_approach.md).
 *
 * An explicit destination: automatic framing (flyTo an entity) clamps to the
 * viewer's 50 m minimum zoom, too far to see a miss of a metre. The camera
 * controller also lifts the camera to that minimum over the ground every
 * frame, so the minimum is lowered for the close view and restored once the
 * camera is back above it.
 */

// Behind the closest point along the recorded heading, and above it.
export const CLOSEST_APPROACH_VIEW_BACK_M = 15;
export const CLOSEST_APPROACH_VIEW_UP_M = 6;
const CLOSEST_APPROACH_FLY_S = 1.5;
/** Camera minimum over the ground while near a mark. */
export const CLOSEST_APPROACH_MIN_ZOOM_M = 2;

// viewer -> { normalMin, stop, flight }: the minimum to restore, the
// active watch, the latest flight.
const closeViews = new WeakMap();

/** Destination and orientation for `view` = {lon, lat, alt, headingDeg|null}. */
export function closestApproachCamera(Cesium, view) {
  // Without a recorded heading, look from the south (heading north).
  const heading = Cesium.Math.toRadians(Number.isFinite(view.headingDeg) ? view.headingDeg : 0);
  const target = Cesium.Cartesian3.fromDegrees(view.lon, view.lat, view.alt);
  const enu = Cesium.Transforms.eastNorthUpToFixedFrame(target);
  const offset = new Cesium.Cartesian3(
    -CLOSEST_APPROACH_VIEW_BACK_M * Math.sin(heading),
    -CLOSEST_APPROACH_VIEW_BACK_M * Math.cos(heading),
    CLOSEST_APPROACH_VIEW_UP_M,
  );
  return {
    destination: Cesium.Matrix4.multiplyByPoint(enu, offset, new Cesium.Cartesian3()),
    orientation: {
      heading,
      pitch: -Math.atan2(CLOSEST_APPROACH_VIEW_UP_M, CLOSEST_APPROACH_VIEW_BACK_M),
      roll: 0,
    },
  };
}

function heightOverGround(viewer) {
  const camera = viewer.camera.positionCartographic;
  return camera.height - (viewer.scene.globe.getHeight(camera) ?? 0);
}

/** Restore the normal minimum once the camera is above it again. */
function watchUntilAbove(viewer, state) {
  const controller = viewer.scene.screenSpaceCameraController;
  const check = () => {
    if (heightOverGround(viewer) <= state.normalMin) return;
    state.stop();
    controller.minimumZoomDistance = state.normalMin;
    closeViews.delete(viewer);
  };
  state.stop = viewer.camera.changed.addEventListener(check);
  check();
}

export function flyToClosestApproach(Cesium, viewer, view) {
  const controller = viewer.scene.screenSpaceCameraController;
  let state = closeViews.get(viewer);
  if (state) {
    state.stop();
  } else {
    state = { normalMin: controller.minimumZoomDistance };
    closeViews.set(viewer, state);
  }
  state.stop = () => {};
  controller.minimumZoomDistance = Math.min(state.normalMin, CLOSEST_APPROACH_MIN_ZOOM_M);
  // Starting this flight cancels an earlier one: only this one may watch.
  const flight = {};
  state.flight = flight;
  const arrived = () => {
    if (closeViews.get(viewer) === state && state.flight === flight) watchUntilAbove(viewer, state);
  };
  viewer.camera.flyTo({
    ...closestApproachCamera(Cesium, view),
    duration: CLOSEST_APPROACH_FLY_S,
    complete: arrived,
    cancel: arrived,
  });
}
