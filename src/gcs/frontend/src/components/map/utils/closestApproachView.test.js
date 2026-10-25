import * as Cesium from 'cesium';
import { expect, it, vi } from 'vitest';
import {
  CLOSEST_APPROACH_MIN_ZOOM_M, CLOSEST_APPROACH_VIEW_BACK_M, CLOSEST_APPROACH_VIEW_UP_M,
  closestApproachCamera, flyToClosestApproach,
} from './closestApproachView';
import { uavModelOrientation } from './uavOrientation';

const VIEW = { lon: 44.5, lat: 40.2, alt: 41 };

const targetOf = (view) => Cesium.Cartesian3.fromDegrees(view.lon, view.lat, view.alt);
const heightOf = (position) => Cesium.Cartographic.fromCartesian(position).height;

function enuOffset(destination, view) {
  const toEnu = Cesium.Matrix4.inverseTransformation(
    Cesium.Transforms.eastNorthUpToFixedFrame(targetOf(view)), new Cesium.Matrix4(),
  );
  return Cesium.Matrix4.multiplyByPoint(toEnu, destination, new Cesium.Cartesian3());
}

/** The camera at `from` looks straight at `target`, its up away from the ground. */
function expectAimed(orientation, from, target) {
  const { Cartesian3 } = Cesium;
  const want = Cartesian3.normalize(Cartesian3.subtract(target, from, new Cartesian3()), new Cartesian3());
  expect(Cartesian3.equalsEpsilon(orientation.direction, want, 1e-9)).toBe(true);
  expect(Cartesian3.magnitude(orientation.up)).toBeCloseTo(1, 9);
  expect(Cartesian3.dot(orientation.up, orientation.direction)).toBeCloseTo(0, 9);
  const surfaceUp = Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(from, new Cartesian3());
  expect(Cartesian3.dot(orientation.up, surfaceUp)).toBeGreaterThan(0);
}

/** Ground `behind` south of the point (where a north-bound view's camera is), `at` elsewhere. */
const southGround = (view, behind, at) => (carto) => (
  carto.latitude < Cesium.Math.toRadians(view.lat - 5e-5) ? behind : at
);

function fakeViewer(ground = () => 0) {
  const listeners = new Set();
  const viewer = {
    height: 500,
    ground,
    destroyed: false,
    shown: new Set(),
    isDestroyed: () => viewer.destroyed,
    entities: { contains: (entity) => viewer.shown.has(entity) },
    scene: {
      screenSpaceCameraController: { minimumZoomDistance: 50 },
      globe: { getHeight: (carto) => viewer.ground(carto) },
    },
    camera: {
      // The flight ends at its destination unless a test moves the camera.
      flyTo: vi.fn((options) => { viewer.camera.positionWC = Cesium.Cartesian3.clone(options.destination); }),
      setView: vi.fn(),
      positionWC: Cesium.Cartesian3.fromDegrees(VIEW.lon, VIEW.lat - 0.001, 500),
      get positionCartographic() { return { height: viewer.height }; },
      changed: {
        addEventListener: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
      },
    },
    moveTo: (height) => { viewer.height = height; [...listeners].forEach(fn => fn()); },
    listeners,
  };
  return viewer;
}

function markIn(viewer) {
  const entity = { isShowing: true };
  viewer.shown.add(entity);
  return entity;
}

it('puts the camera behind the closest point along the heading and above it, aimed at it', () => {
  const view = { ...VIEW, headingDeg: 90 };
  const { destination, orientation } = closestApproachCamera(Cesium, fakeViewer(), view);
  const offset = enuOffset(destination, view);
  expect(offset.x).toBeCloseTo(-CLOSEST_APPROACH_VIEW_BACK_M, 3); // west of an east-bound UAV
  expect(offset.y).toBeCloseTo(0, 3);
  expect(offset.z).toBeCloseTo(CLOSEST_APPROACH_VIEW_UP_M, 3);
  expectAimed(orientation, destination, targetOf(view));
});

it('looks from the south without a recorded heading', () => {
  const view = { ...VIEW, headingDeg: null };
  const { destination, orientation } = closestApproachCamera(Cesium, fakeViewer(), view);
  const offset = enuOffset(destination, view);
  expect(offset.x).toBeCloseTo(0, 3);
  expect(offset.y).toBeCloseTo(-CLOSEST_APPROACH_VIEW_BACK_M, 3);
  expectAimed(orientation, destination, targetOf(view));
});

it.each([
  ['flat ground, the point on it', 0, () => 0, CLOSEST_APPROACH_VIEW_UP_M],
  ['a point below the ground', -4, () => 0, CLOSEST_APPROACH_VIEW_UP_M],
  ['ground rising behind the mark', 1311, southGround({ ...VIEW, alt: 1311 }, 1330, 1310), 1336],
  ['ground falling behind the mark', 1311, southGround({ ...VIEW, alt: 1311 }, 1290, 1310), 1317],
])('the destination clears the ground under it: %s', (_name, alt, ground, height) => {
  const view = { ...VIEW, alt, headingDeg: 0 };
  const { destination, orientation } = closestApproachCamera(Cesium, fakeViewer(ground), view);
  expect(heightOf(destination)).toBeCloseTo(height, 3);
  const offset = enuOffset(destination, view);
  expect(offset.x).toBeCloseTo(0, 2);
  expect(offset.y).toBeCloseTo(-CLOSEST_APPROACH_VIEW_BACK_M, 2); // raised, not moved
  expectAimed(orientation, destination, targetOf(view));
});

it('does not fly while the ground at the destination is not rendered', () => {
  const viewer = fakeViewer(() => undefined);
  const view = { ...VIEW, headingDeg: 0 };
  expect(closestApproachCamera(Cesium, viewer, view)).toBeNull();
  flyToClosestApproach(Cesium, viewer, view, markIn(viewer));
  expect(viewer.camera.flyTo).not.toHaveBeenCalled();
  expect(viewer.scene.screenSpaceCameraController.minimumZoomDistance).toBe(50);
  viewer.ground = () => 0; // rendered now: a later click flies
  flyToClosestApproach(Cesium, viewer, view, markIn(viewer));
  expect(viewer.camera.flyTo).toHaveBeenCalledOnce();
});

it('flies the camera to that explicit destination below the normal minimum zoom', () => {
  const viewer = fakeViewer();
  const view = { ...VIEW, headingDeg: 0 };
  flyToClosestApproach(Cesium, viewer, view, markIn(viewer));
  const [options] = viewer.camera.flyTo.mock.calls[0];
  expect(options).toEqual(expect.objectContaining(closestApproachCamera(Cesium, viewer, view)));
  const controller = viewer.scene.screenSpaceCameraController;
  expect(controller.minimumZoomDistance).toBe(CLOSEST_APPROACH_MIN_ZOOM_M);
  viewer.moveTo(47); // arrived close: the low minimum stays
  options.complete();
  expect(controller.minimumZoomDistance).toBe(CLOSEST_APPROACH_MIN_ZOOM_M);
  viewer.moveTo(30); // orbiting close
  expect(controller.minimumZoomDistance).toBe(CLOSEST_APPROACH_MIN_ZOOM_M);
  viewer.moveTo(80); // zoomed back out: the normal minimum returns
  expect(controller.minimumZoomDistance).toBe(50);
  expect(viewer.listeners.size).toBe(0);
});

it('a second mark keeps the original minimum to restore', () => {
  const viewer = fakeViewer();
  flyToClosestApproach(Cesium, viewer, VIEW, markIn(viewer));
  viewer.moveTo(20);
  viewer.camera.flyTo.mock.calls[0][0].complete();
  flyToClosestApproach(Cesium, viewer, VIEW, markIn(viewer));
  expect(viewer.listeners.size).toBe(0); // the first watch stopped
  viewer.moveTo(300); // mid-flight, high
  viewer.camera.flyTo.mock.calls[0][0].cancel(); // Cesium cancels the first flight
  expect(viewer.scene.screenSpaceCameraController.minimumZoomDistance).toBe(CLOSEST_APPROACH_MIN_ZOOM_M);
  viewer.moveTo(20);
  viewer.camera.flyTo.mock.calls[1][0].cancel();
  viewer.moveTo(400);
  expect(viewer.scene.screenSpaceCameraController.minimumZoomDistance).toBe(50);
});

it('on arrival clears the ground rendered by then and aims at the drawn point again', () => {
  const viewer = fakeViewer(() => 1310);
  const view = { ...VIEW, alt: 1311, headingDeg: 0 };
  flyToClosestApproach(Cesium, viewer, view, markIn(viewer));
  const [options] = viewer.camera.flyTo.mock.calls[0];
  expect(heightOf(options.destination)).toBeCloseTo(1317, 3);
  viewer.ground = () => 1340; // finer tiles came in during the flight
  options.complete();
  expect(viewer.camera.setView).toHaveBeenCalledOnce();
  const [{ destination, orientation }] = viewer.camera.setView.mock.calls[0];
  expect(heightOf(destination)).toBeCloseTo(1346, 3);
  expectAimed(orientation, destination, targetOf(view));
});

it('on arrival over ground no longer rendered keeps the position and still aims', () => {
  const viewer = fakeViewer(() => 1310);
  const view = { ...VIEW, alt: 1311, headingDeg: 0 };
  flyToClosestApproach(Cesium, viewer, view, markIn(viewer));
  const [options] = viewer.camera.flyTo.mock.calls[0];
  viewer.ground = () => undefined;
  options.complete();
  const [{ destination, orientation }] = viewer.camera.setView.mock.calls[0];
  expect(Cesium.Cartesian3.equals(destination, options.destination)).toBe(true);
  expectAimed(orientation, destination, targetOf(view));
});

it.each([
  ['the mark was removed', (viewer, entity) => viewer.shown.delete(entity), 'complete'],
  ['the mark is hidden', (_viewer, entity) => { entity.isShowing = false; }, 'complete'],
  ['a newer flight started', (viewer) => flyToClosestApproach(Cesium, viewer, VIEW, markIn(viewer)), 'complete'],
  ['the viewer is destroyed', (viewer) => { viewer.destroyed = true; }, 'complete'],
  ['the operator interrupted the flight', () => {}, 'cancel'],
])('does not re-aim when %s', (_name, change, end) => {
  const viewer = fakeViewer();
  const entity = markIn(viewer);
  flyToClosestApproach(Cesium, viewer, { ...VIEW, headingDeg: 0 }, entity);
  const [options] = viewer.camera.flyTo.mock.calls[0];
  change(viewer, entity);
  options[end]();
  expect(viewer.camera.setView).not.toHaveBeenCalled();
});

it('keeps the live model orientation it was moved from', () => {
  const position = Cesium.Cartesian3.fromDegrees(VIEW.lon, VIEW.lat, VIEW.alt);
  const [heading, roll, pitch] = [91, -3, 2];
  const vehicleQuat = Cesium.Transforms.headingPitchRollQuaternion(
    position,
    new Cesium.HeadingPitchRoll(
      Cesium.Math.toRadians(heading), -Cesium.Math.toRadians(roll), Cesium.Math.toRadians(pitch),
    ),
  );
  const fix = Cesium.Quaternion.fromHeadingPitchRoll(
    new Cesium.HeadingPitchRoll(-Cesium.Math.PI, Cesium.Math.PI_OVER_TWO, 0),
  );
  const expected = Cesium.Quaternion.multiply(vehicleQuat, fix, new Cesium.Quaternion());
  expect(Cesium.Quaternion.equalsEpsilon(
    uavModelOrientation(Cesium, position, heading, roll, pitch), expected, 1e-12,
  )).toBe(true);
});
