// Where the flight path marker draws (flightPathMarker.js): along the ground
// velocity, far ahead, projected by the scene; never behind the camera or off
// the picture.
import * as Cesium from 'cesium';
import { describe, expect, it } from 'vitest';
import { FPM_PROJECTION_M, flightPathMarkerPoint } from './flightPathMarker';

const position = Cesium.Cartesian3.fromDegrees(44.45, 40.31, 1350);
const enuToFixed = Cesium.Transforms.eastNorthUpToFixedFrame(position);
const fixedDir = (e, n, u) => Cesium.Cartesian3.normalize(
  Cesium.Matrix4.multiplyByPointAsVector(enuToFixed, new Cesium.Cartesian3(e, n, u), new Cesium.Cartesian3()),
  new Cesium.Cartesian3(),
);
const SIZE = { width: 320, height: 180 };
const sceneLooking = (direction) => ({ camera: { positionWC: position, directionWC: direction } });
const withProjection = (fn) => ({ ...Cesium, SceneTransforms: { worldToWindowCoordinates: fn } });

describe('flightPathMarkerPoint', () => {
  it('projects the point FPM_PROJECTION_M along the ground velocity', () => {
    let projected = null;
    const C = withProjection((_s, p) => { projected = p; return { x: 160.4, y: 90.6 }; });
    const point = flightPathMarkerPoint(C, sceneLooking(fixedDir(0, 1, 0)), position, [0, 25, 0], SIZE);
    expect(point).toEqual({ x: 160, y: 91 });
    const offset = Cesium.Cartesian3.subtract(projected, position, new Cesium.Cartesian3());
    expect(Cesium.Cartesian3.magnitude(offset)).toBeCloseTo(FPM_PROJECTION_M, 3);
    const along = Cesium.Cartesian3.normalize(offset, new Cesium.Cartesian3());
    expect(Cesium.Cartesian3.equalsEpsilon(along, fixedDir(0, 1, 0), 1e-9)).toBe(true);
  });

  it('none when the UAV flies away from where the camera looks', () => {
    const C = withProjection(() => ({ x: 100, y: 100 }));
    expect(flightPathMarkerPoint(C, sceneLooking(fixedDir(0, 1, 0)), position, [0, 25, 0], SIZE)).toEqual({ x: 100, y: 100 });
    expect(flightPathMarkerPoint(C, sceneLooking(fixedDir(0, -1, 0)), position, [0, 25, 0], SIZE)).toBeNull();
  });

  it('none off the picture, without a projection, or without a velocity', () => {
    const north = sceneLooking(fixedDir(0, 1, 0));
    expect(flightPathMarkerPoint(withProjection(() => ({ x: 320, y: 180 })), north, position, [0, 25, 0], SIZE)).toEqual({ x: 320, y: 180 });
    expect(flightPathMarkerPoint(withProjection(() => ({ x: 400, y: 10 })), north, position, [0, 25, 0], SIZE)).toBeNull();
    expect(flightPathMarkerPoint(withProjection(() => undefined), north, position, [0, 25, 0], SIZE)).toBeNull();
    expect(flightPathMarkerPoint(withProjection(() => ({ x: 1, y: 1 })), north, position, [0, 0, 0], SIZE)).toBeNull();
  });
});
