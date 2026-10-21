import * as Cesium from 'cesium';
import { expect, it, vi } from 'vitest';
import {
  CLOSEST_APPROACH_MIN_ZOOM_M, CLOSEST_APPROACH_VIEW_BACK_M, CLOSEST_APPROACH_VIEW_UP_M,
  closestApproachCamera, flyToClosestApproach,
} from './closestApproachView';
import { uavModelOrientation } from './uavOrientation';

const VIEW = { lon: 44.5, lat: 40.2, alt: 41 };

function enuOffset(destination, view) {
  const target = Cesium.Cartesian3.fromDegrees(view.lon, view.lat, view.alt);
  const toEnu = Cesium.Matrix4.inverseTransformation(
    Cesium.Transforms.eastNorthUpToFixedFrame(target), new Cesium.Matrix4(),
  );
  return Cesium.Matrix4.multiplyByPoint(toEnu, destination, new Cesium.Cartesian3());
}

it('puts the camera behind the closest point along the heading and above it', () => {
  const { destination, orientation } = closestApproachCamera(Cesium, { ...VIEW, headingDeg: 90 });
  const offset = enuOffset(destination, VIEW);
  expect(offset.x).toBeCloseTo(-CLOSEST_APPROACH_VIEW_BACK_M, 3); // west of an east-bound UAV
  expect(offset.y).toBeCloseTo(0, 3);
  expect(offset.z).toBeCloseTo(CLOSEST_APPROACH_VIEW_UP_M, 3);
  expect(orientation.heading).toBeCloseTo(Math.PI / 2, 6);
  expect(orientation.pitch).toBeCloseTo(-Math.atan2(CLOSEST_APPROACH_VIEW_UP_M, CLOSEST_APPROACH_VIEW_BACK_M), 6);
});

it('looks from the south without a recorded heading', () => {
  const { destination, orientation } = closestApproachCamera(Cesium, { ...VIEW, headingDeg: null });
  const offset = enuOffset(destination, VIEW);
  expect(offset.x).toBeCloseTo(0, 3);
  expect(offset.y).toBeCloseTo(-CLOSEST_APPROACH_VIEW_BACK_M, 3);
  expect(orientation.heading).toBeCloseTo(0, 6);
});

function fakeViewer() {
  const listeners = new Set();
  const viewer = {
    height: 500,
    scene: {
      screenSpaceCameraController: { minimumZoomDistance: 50 },
      globe: { getHeight: () => 0 },
    },
    camera: {
      flyTo: vi.fn(),
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

it('flies the camera to that explicit destination below the normal minimum zoom', () => {
  const viewer = fakeViewer();
  flyToClosestApproach(Cesium, viewer, { ...VIEW, headingDeg: 0 });
  const [options] = viewer.camera.flyTo.mock.calls[0];
  expect(options.destination).toBeInstanceOf(Cesium.Cartesian3);
  expect(options.orientation.roll).toBe(0);
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
  flyToClosestApproach(Cesium, viewer, VIEW);
  viewer.moveTo(20);
  viewer.camera.flyTo.mock.calls[0][0].complete();
  flyToClosestApproach(Cesium, viewer, VIEW);
  expect(viewer.listeners.size).toBe(0); // the first watch stopped
  viewer.moveTo(300); // mid-flight, high
  viewer.camera.flyTo.mock.calls[0][0].cancel(); // Cesium cancels the first flight
  expect(viewer.scene.screenSpaceCameraController.minimumZoomDistance).toBe(CLOSEST_APPROACH_MIN_ZOOM_M);
  viewer.moveTo(20);
  viewer.camera.flyTo.mock.calls[1][0].cancel();
  viewer.moveTo(400);
  expect(viewer.scene.screenSpaceCameraController.minimumZoomDistance).toBe(50);
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
