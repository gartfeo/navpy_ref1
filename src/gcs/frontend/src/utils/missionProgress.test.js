// Which track waypoint a vehicle flies to, from its mission progress (the
// current mission item). Seen on the UniRC (2026-11-08): a 5-item mission
// made outside the GCS (home, takeoff, three waypoints, no GCS metadata
// items), downloaded, UAV in AUTO at item 3. The planner's offset counted
// metadata items that mission does not have (the polygon is a hull of the
// track), so the HUD had no next waypoint, the card read 0.0 km and the
// bottom bar 0% complete.
import { describe, expect, it } from 'vitest';
import { missionWpOffset } from './geo';
import { computeZoneDistances, interpolatedDistance, missionTrackIndex, withoutSeqs, zoneWpOffset } from './missionProgress';
import { assembleMissionsFromResults, mergeDownloadedMissionZones } from './missionAssembly';

// The backend's parse of that mission: seq 0 home and seq 1 takeoff are not
// track points.
const DOWNLOADED = {
  sys_id: 1,
  waypoints: [
    { lat: 40.300, lon: 44.450, alt: 60, mission_sequence: 2 },
    { lat: 40.310, lon: 44.450, alt: 60, mission_sequence: 3 },
    { lat: 40.310, lon: 44.460, alt: 60, mission_sequence: 4 },
  ],
  altitude_m: 60,
};
const TRACK = mergeDownloadedMissionZones([], DOWNLOADED, 1)[0].track;
// The offset the GCS computed for it: a 3-vertex hull polygon read as metadata.
const PLAN_OFFSET = missionWpOffset('distributed', 3, 0, false);

describe('missionTrackIndex', () => {
  it('maps the downloaded mission by the vehicle\'s own item numbers', () => {
    expect(PLAN_OFFSET).toBe(6);
    expect(3 - PLAN_OFFSET).toBeLessThan(0); // the old rule: no next waypoint
    expect(missionTrackIndex(3, TRACK, PLAN_OFFSET)).toBe(1);
    expect(missionTrackIndex(2, TRACK, PLAN_OFFSET)).toBe(0);
    expect(missionTrackIndex(4, TRACK, PLAN_OFFSET)).toBe(2);
  });

  it('is negative before the track and past its end after it', () => {
    expect(missionTrackIndex(1, TRACK, PLAN_OFFSET)).toBeLessThan(0);
    expect(missionTrackIndex(0, TRACK, PLAN_OFFSET)).toBeLessThan(0);
    expect(missionTrackIndex(5, TRACK, PLAN_OFFSET)).toBe(TRACK.length);
  });

  it('a gap in the numbering (an item without a location) maps to the next track point', () => {
    const track = [{ lat: 0, lon: 0, seq: 2 }, { lat: 0, lon: 1, seq: 5 }];
    expect(missionTrackIndex(3, track, 0)).toBe(1);
  });

  it('a track the GCS planned, or one edited since, keeps the planner\'s offset', () => {
    const planned = [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }];
    expect(missionTrackIndex(7, planned, 6)).toBe(1);
    const edited = [TRACK[0], { lat: 40.305, lon: 44.455 }, ...TRACK.slice(1)];
    expect(missionTrackIndex(7, edited, 6)).toBe(1);
  });

  it('the first track point\'s number is the zone\'s offset (en route before it)', () => {
    expect(zoneWpOffset(TRACK, PLAN_OFFSET)).toBe(2);
    expect(zoneWpOffset([{ lat: 0, lon: 0 }], PLAN_OFFSET)).toBe(PLAN_OFFSET);
  });
});

describe('downloaded tracks', () => {
  it('carry each waypoint\'s mission item number', () => {
    expect(TRACK.map((p) => p.seq)).toEqual([2, 3, 4]);
    const { zones } = assembleMissionsFromResults([DOWNLOADED]);
    expect(zones[0].track.map((p) => p.seq)).toEqual([2, 3, 4]);
  });

  it('lose them once the GCS uploads its own layout', () => {
    expect(withoutSeqs(TRACK).some((p) => 'seq' in p)).toBe(false);
    expect(withoutSeqs(TRACK)[1]).toEqual({ lat: 40.310, lon: 44.450, alt: 60 });
  });
});

describe('interpolatedDistance', () => {
  it('counts the flown part of the downloaded mission', () => {
    const [zd] = computeZoneDistances({ zones: [{ track: TRACK }] });
    const v = { mission_progress: 3, lat: 40.305, lon: 44.450 };
    const flown = interpolatedDistance(v, zd, PLAN_OFFSET);
    expect(flown).toBeGreaterThan(0.4 * zd.cumulative[1]);
    expect(flown).toBeLessThan(0.6 * zd.cumulative[1]);
  });
});
