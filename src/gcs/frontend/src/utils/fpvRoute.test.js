// The first-person views' route and next waypoint (utils/fpvRoute.js).
import { describe, it, expect } from 'vitest';
import { MIN_ETE_SPEED_MPS, formatDistance, formatEte, nextWaypointInfo, routeFor } from './fpvRoute';

const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0 }, { lat: 0.002, lon: 0 }];
const positions = ['p0', 'p1', 'p2'];
const base = {
  vehicleList: [{ sys_id: 7 }, { sys_id: 3 }],
  trackPositions: [[], positions],
  plan: { zones: [{ track: [] }, { track }] },
  progress: { 1: { trackIdx: 1, lat: 0, lon: 0 } },
};

describe('routeFor', () => {
  it("is the UAV's own zone (its index in the vehicle list) with the main map's positions and next waypoint", () => {
    expect(routeFor(3, base)).toEqual({ zone: 1, positions, track, next: 1 });
  });

  it('null for a UAV not in the list, or a zone without a track', () => {
    expect(routeFor(9, base)).toBeNull();
    expect(routeFor(7, base)).toBeNull();
    expect(routeFor(3, { ...base, plan: null })).toBeNull();
  });

  it('positions out of step with the plan (a new plan before its positions): no route', () => {
    expect(routeFor(3, { ...base, trackPositions: [[], ['p0']] })).toBeNull();
  });

  it('no next waypoint before the track, after its end, or without progress', () => {
    expect(routeFor(3, { ...base, progress: { 1: null } }).next).toBeNull();
    expect(routeFor(3, { ...base, progress: { 1: { trackIdx: 3 } } }).next).toBeNull();
    expect(routeFor(3, { ...base, progress: {} }).next).toBeNull();
  });
});

describe('nextWaypointInfo', () => {
  const route = { zone: 1, positions, track, next: 2 };

  it('the 1-based number, the horizontal distance and the time at ground speed', () => {
    const info = nextWaypointInfo(route, { lat: 0, lon: 0, ground_speed: 20 });
    expect(info.number).toBe(3);
    expect(info.distanceM).toBeCloseTo(222.64, 1);
    expect(info.eteS).toBeCloseTo(222.64 / 20, 2);
  });

  it('no time below the minimum ground speed or without one', () => {
    expect(nextWaypointInfo(route, { lat: 0, lon: 0, ground_speed: MIN_ETE_SPEED_MPS - 0.01 }).eteS).toBeNull();
    expect(nextWaypointInfo(route, { lat: 0, lon: 0 }).eteS).toBeNull();
  });

  it('nothing without a next waypoint or a position', () => {
    expect(nextWaypointInfo(null, { lat: 0, lon: 0 })).toBeNull();
    expect(nextWaypointInfo({ ...route, next: null }, { lat: 0, lon: 0 })).toBeNull();
    expect(nextWaypointInfo(route, { lat: null, lon: 0 })).toBeNull();
  });
});

describe('formats', () => {
  it('distance in metres, then kilometres', () => {
    expect(formatDistance(420.4)).toBe('420 m');
    expect(formatDistance(1234)).toBe('1.2 km');
  });

  it('time as mm:ss, h:mm:ss from an hour', () => {
    expect(formatEte(21)).toBe('00:21');
    expect(formatEte(725)).toBe('12:05');
    expect(formatEte(3723)).toBe('1:02:03');
  });
});
