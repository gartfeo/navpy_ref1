// What the Synthetic view marks (syntheticScene.js): the UAV's dock as its card
// shows it, its zone's delivery hub, and the flight path marker's velocity.
import { describe, expect, it } from 'vitest';
import { FPM_SMOOTHING_S, smoothVector, syntheticDock, syntheticHub, velocityEnu } from './syntheticScene';

const entry = (over) => ({ receiverId: 4, status: 'assigned', lat: 40.3, lon: 44.4, ...over });

describe('syntheticDock', () => {
  it('the dock the UAV is assigned to', () => {
    expect(syntheticDock({ a: entry() }, 4)).toEqual({ lat: 40.3, lon: 44.4 });
  });
  it('none for another UAV, a handled dock, or no position', () => {
    expect(syntheticDock({ a: entry() }, 5)).toBeNull();
    expect(syntheticDock({ a: entry({ status: 'handled' }) }, 4)).toBeNull();
    expect(syntheticDock({ a: entry({ lat: null }) }, 4)).toBeNull();
    expect(syntheticDock(null, 4)).toBeNull();
  });
});

describe('syntheticHub', () => {
  const hubs = [{ lat: 1, lon: 2, type: 'building' }, { lat: 3, lon: 4 }];
  const fleet = [{ sys_id: 4 }, { sys_id: 5 }];
  it('the hub assigned to the zone of the UAV (its place in the fleet list)', () => {
    expect(syntheticHub(fleet, hubs, [1, 0], 4)).toEqual({ lat: 3, lon: 4 });
    expect(syntheticHub(fleet, hubs, [1, 0], 5)).toEqual({ lat: 1, lon: 2, type: 'building' });
  });
  it('none without an assignment, a hub or the UAV', () => {
    expect(syntheticHub(fleet, hubs, [null], 4)).toBeNull();
    expect(syntheticHub(fleet, hubs, [1, 0], 9)).toBeNull();
    expect(syntheticHub(fleet, null, [0], 4)).toBeNull();
  });
});

describe('velocityEnu', () => {
  it('turns north/east/down into east/north/up', () => {
    expect(velocityEnu([20, 5, -2])).toEqual([5, 20, 2]);
  });
  it('none for a parked UAV, a missing or bad reading', () => {
    expect(velocityEnu([0.5, 0.5, 0])).toBeNull();
    expect(velocityEnu(null)).toBeNull();
    expect(velocityEnu([1, Number.NaN, 0])).toBeNull();
  });
});

describe('smoothVector', () => {
  it('starts at the first reading and moves 1 - e^-dt/tau of the way after that', () => {
    expect(smoothVector(null, [0, 20, 0], 0.2)).toEqual([0, 20, 0]);
    const [e, n] = smoothVector([0, 20, 0], [20, 0, 0], FPM_SMOOTHING_S);
    expect(e).toBeCloseTo(20 * (1 - Math.exp(-1)), 9);
    expect(n).toBeCloseTo(20 * Math.exp(-1), 9);
  });
  it('none when the UAV stops', () => {
    expect(smoothVector([0, 20, 0], null, 0.2)).toBeNull();
  });
});
