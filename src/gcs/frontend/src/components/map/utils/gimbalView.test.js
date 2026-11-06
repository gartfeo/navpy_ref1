// The Synthetic view through the gimbal camera (gimbalView.js).
import { describe, it, expect } from 'vitest';
import { GIMBAL_ON_AXIS_RAD, cesiumFov, gimbalOrientation, liveGimbalView } from './gimbalView';
import { cameraToNedMatrix } from './cameraFootprint';

const D = Math.PI / 180;
// Camera frame (x right, y down, z optical axis) to body FRD.
const FORWARD = [[0, 0, 1], [1, 0, 0], [0, 1, 0]];
const tilted = (pitchDeg) => {
  const c = Math.cos(pitchDeg * D), s = Math.sin(pitchDeg * D);
  // The static mount at a camera pitch, up positive (cameraFootprint _staticCameraToBodyMatrix).
  return [[0, s, c], [1, 0, 0], [0, c, -s]];
};

describe('gimbalOrientation', () => {
  it('a camera along the nose gives the nose view, on axis', () => {
    const nose = { heading: 30 * D, pitch: 10 * D, roll: 20 * D };
    const o = gimbalOrientation(nose, FORWARD);
    expect(o.heading).toBeCloseTo(nose.heading, 9);
    expect(o.pitch).toBeCloseTo(nose.pitch, 9);
    expect(o.roll).toBeCloseTo(nose.roll, 9);
    expect(o.offNose).toBeCloseTo(0, 6);
  });

  it('a camera tilted down 45 degrees on a level UAV looks 45 down along the heading, unrolled, off axis', () => {
    const o = gimbalOrientation({ heading: -60 * D, pitch: 0, roll: 0 }, tilted(-45));
    expect(o.heading).toBeCloseTo(-60 * D, 9);
    expect(o.pitch).toBeCloseTo(-45 * D, 9);
    expect(o.roll).toBeCloseTo(0, 9);
    expect(o.offNose).toBeCloseTo(45 * D, 9);
    expect(o.offNose).toBeGreaterThan(GIMBAL_ON_AXIS_RAD);
  });

  it('a banked UAV with a down-looking camera: the view keeps the bank as roll about its axis only when along the nose', () => {
    // 30 degrees right bank, camera along the nose: the picture rolls 30 degrees.
    expect(gimbalOrientation({ heading: 0, pitch: 0, roll: 30 * D }, FORWARD).roll).toBeCloseTo(30 * D, 9);
  });

  it('a camera along the nose but rolled 90 degrees is off the nose view (Codex f1)', () => {
    // Camera x (right) along body down, y (down) along body left.
    const rolled = [[0, 0, 1], [0, -1, 0], [1, 0, 0]];
    const o = gimbalOrientation({ heading: 0, pitch: 0, roll: 0 }, rolled);
    expect(o.pitch).toBeCloseTo(0, 9);
    expect(Math.abs(o.roll)).toBeCloseTo(90 * D, 9);
    expect(o.offNose).toBeCloseTo(90 * D, 9);
    expect(o.offNose).toBeGreaterThan(GIMBAL_ON_AXIS_RAD);
  });

  it("any attitude and mount: the orientation rebuilds the camera's own axis and right side", () => {
    const nose = { heading: 120 * D, pitch: -8 * D, roll: 25 * D };
    const mount = tilted(-35);
    const o = gimbalOrientation(nose, mount);
    const m = cameraToNedMatrix(120, -8, 25, mount);
    const forward = [Math.cos(o.pitch) * Math.cos(o.heading), Math.cos(o.pitch) * Math.sin(o.heading), -Math.sin(o.pitch)];
    const levelRight = [-Math.sin(o.heading), Math.cos(o.heading), 0];
    const down0 = [
      forward[1] * levelRight[2] - forward[2] * levelRight[1],
      forward[2] * levelRight[0] - forward[0] * levelRight[2],
      forward[0] * levelRight[1] - forward[1] * levelRight[0],
    ];
    const right = levelRight.map((v, i) => Math.cos(o.roll) * v + Math.sin(o.roll) * down0[i]);
    for (let i = 0; i < 3; i++) {
      expect(forward[i]).toBeCloseTo(m[i][2], 9);
      expect(right[i]).toBeCloseTo(m[i][0], 9);
    }
  });
});

describe('liveGimbalView', () => {
  const device = { gimbal_device_id: 154, setup_att: [0, 0, 0], setup_seq: 'XYZ', imageHeight: 1080 };
  const vehicle = (over = {}) => ({
    gimbals: {
      154: {
        stale: false, device_id: 154, flags: 32, failure_flags: 0, q: [1, 0, 0, 0],
        optics_stale: false, fov_h_rad: 1.2, fov_v_rad: 0.7, ...over,
      },
    },
  });

  it("the first device with live telemetry: its camera-to-vehicle matrix and field of view", () => {
    const view = liveGimbalView(vehicle(), [device]);
    expect(view.fovH).toBe(1.2);
    expect(view.fovV).toBe(0.7);
    expect(view.cameraToVehicle).toHaveLength(3);
  });

  it('none without live telemetry (stale, failed, or no device): the nose view', () => {
    expect(liveGimbalView(vehicle({ stale: true }), [device])).toBeNull();
    expect(liveGimbalView(vehicle({ failure_flags: 1 }), [device])).toBeNull();
    expect(liveGimbalView(vehicle(), [])).toBeNull();
    expect(liveGimbalView(null, [device])).toBeNull();
  });
});

describe('cesiumFov', () => {
  it('a wide canvas takes the horizontal view as is', () => {
    expect(cesiumFov(1.2, 16 / 9)).toBe(1.2);
    expect(cesiumFov(1.2, 1)).toBe(1.2);
  });

  it('a tall canvas gets the vertical view that keeps the horizontal one (Codex f1)', () => {
    const fov = cesiumFov(1.2, 0.6);
    // Cesium's own horizontal view for a vertical fov: 2 atan(aspect tan(fov / 2)).
    expect(2 * Math.atan(0.6 * Math.tan(fov / 2))).toBeCloseTo(1.2, 9);
  });
});
