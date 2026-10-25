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

/**
 * `position` raised to `CLOSEST_APPROACH_VIEW_UP_M` over the rendered ground
 * under it (the height the collision step uses); null while that ground is
 * not rendered.
 */
function withClearance(Cesium, viewer, position) {
  const carto = Cesium.Cartographic.fromCartesian(position);
  const ground = viewer.scene.globe.getHeight(carto);
  if (!Number.isFinite(ground)) return null;
  const lowest = ground + CLOSEST_APPROACH_VIEW_UP_M;
  if (carto.height >= lowest) return position;
  return Cesium.Cartesian3.fromRadians(carto.longitude, carto.latitude, lowest);
}

/** Look from `from` straight at `target`, the horizon level. */
function aimAt(Cesium, from, target) {
  const { Cartesian3 } = Cesium;
  const direction = Cartesian3.normalize(
    Cartesian3.subtract(target, from, new Cartesian3()), new Cartesian3(),
  );
  const surfaceUp = Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(from, new Cartesian3());
  const right = Cartesian3.normalize(
    Cartesian3.cross(direction, surfaceUp, new Cartesian3()), new Cartesian3(),
  );
  return { direction, up: Cartesian3.cross(right, direction, new Cartesian3()) };
}

/**
 * Destination and orientation for `view` = {lon, lat, alt, headingDeg|null},
 * `alt` the drawn point's height; null while the ground there is not rendered.
 */
export function closestApproachCamera(Cesium, viewer, view) {
  // Without a recorded heading, look from the south (heading north).
  const heading = Cesium.Math.toRadians(Number.isFinite(view.headingDeg) ? view.headingDeg : 0);
  const target = Cesium.Cartesian3.fromDegrees(view.lon, view.lat, view.alt);
  const enu = Cesium.Transforms.eastNorthUpToFixedFrame(target);
  const offset = new Cesium.Cartesian3(
    -CLOSEST_APPROACH_VIEW_BACK_M * Math.sin(heading),
    -CLOSEST_APPROACH_VIEW_BACK_M * Math.cos(heading),
    CLOSEST_APPROACH_VIEW_UP_M,
  );
  const destination = withClearance(
    Cesium, viewer, Cesium.Matrix4.multiplyByPoint(enu, offset, new Cesium.Cartesian3()),
  );
  if (!destination) return null;
  return { destination, orientation: aimAt(Cesium, destination, target) };
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

/**
 * Fly to the mark `entity` shows. On arrival the clearance and the aim are
 * applied once more over the ground rendered by then; after that the camera
 * is the operator's.
 */
export function flyToClosestApproach(Cesium, viewer, view, entity) {
  const camera = closestApproachCamera(Cesium, viewer, view);
  if (!camera) return; // no rendered ground there yet: a later click flies
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
  const latest = () => (
    !viewer.isDestroyed() && closeViews.get(viewer) === state && state.flight === flight
  );
  const target = Cesium.Cartesian3.fromDegrees(view.lon, view.lat, view.alt);
  const arrived = () => {
    if (!latest()) return;
    if (viewer.entities.contains(entity) && entity.isShowing) {
      const here = Cesium.Cartesian3.clone(viewer.camera.positionWC);
      const from = withClearance(Cesium, viewer, here) ?? here;
      viewer.camera.setView({ destination: from, orientation: aimAt(Cesium, from, target) });
    }
    watchUntilAbove(viewer, state);
  };
  const cancelled = () => {
    if (latest()) watchUntilAbove(viewer, state);
  };
  viewer.camera.flyTo({
    ...camera,
    duration: CLOSEST_APPROACH_FLY_S,
    complete: arrived,
    cancel: cancelled,
  });
}
