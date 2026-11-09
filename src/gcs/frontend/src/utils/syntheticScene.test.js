// What the Synthetic view marks (syntheticScene.js): the dock the UAV works on
// (given to it, or its own while it asks to confirm), its zone's delivery hub, and the flight path marker's velocity.
import { describe, expect, it } from 'vitest';
import { FPM_SMOOTHING_S, smoothVector, syntheticDock, syntheticHub, velocityEnu } from './syntheticScene';
import { INITIAL_ASSIGNMENT_STATE, reduceTaskAssignment } from './taskAssignmentState';

const entry = (over) => ({ receiverId: 4, status: 'assigned', lat: 40.3, lon: 44.4, ...over });

describe('syntheticDock', () => {
  it('the dock the UAV is assigned to', () => {
    expect(syntheticDock({ a: entry() }, 4)).toEqual({ lat: 40.3, lon: 44.4 });
  });
  // Owner 2026-11-09: "in synthetic view I don't see the dock mark when loitering for
  // confirmation". The UAV's own request (withOwnRound) has askedBy, receiverId null.
  it('the dock the UAV loiters over while it asks the operator to confirm', () => {
    const asking = { a: entry({ receiverId: null, askedBy: 4, status: 'confirming', lat: 40.31, lon: 44.41 }) };
    expect(syntheticDock(asking, 4)).toEqual({ lat: 40.31, lon: 44.41 });
    expect(syntheticDock(asking, 5)).toBeNull();
  });
  it('through the reducer: a UAV asking to confirm its own detected dock has that dock', () => {
    const state = reduceTaskAssignment(INITIAL_ASSIGNMENT_STATE, {
      type: 'task_confirming',
      data: { sys_id: 2, task_id: 7, task_type: 'dock', round_uid: '1:5', lat: 40.315, lon: 44.47, alt: 0 },
    });
    expect(syntheticDock(state.assignments, 2)).toEqual({ lat: 40.315, lon: 44.47 });
  });
  it('the furthest round of the UAV wins; an offer it has not taken is not its dock', () => {
    const both = {
      a: entry({ status: 'assigned', lat: 1, lon: 1 }),
      b: entry({ receiverId: null, askedBy: 4, status: 'confirming', lat: 2, lon: 2 }),
    };
    expect(syntheticDock(both, 4)).toEqual({ lat: 2, lon: 2 });
    expect(syntheticDock({ a: entry({ status: 'waiting' }) }, 4)).toBeNull();
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
    expect(syntheticHub(fleet, hubs, [0], 4)).toEqual({ lat: 1, lon: 2, type: 'building' });
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
    expect(velocityEnu([1.5, 0, 0])).toEqual([0, 1.5, -0]);
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
    expect(smoothVector([0, 20, 0], [0, 10, 0], 0.2)).not.toBeNull();
    expect(smoothVector([0, 20, 0], null, 0.2)).toBeNull();
  });
});
