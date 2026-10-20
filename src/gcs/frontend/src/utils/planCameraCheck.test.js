import { describe, expect, it, vi } from 'vitest';
import {
  cameraCheckConfirmText,
  cameraCheckZone,
  cameraCheckZonesForPlan,
  confirmCameraCheck,
  describeCameraCheck,
  zoneUploadAltitude,
} from './planCameraCheck';

const t = (key) => key;
const wp = (i) => ({ lat: i, lon: 10 + i });

describe('zoneUploadAltitude', () => {
  it('stacks zones down from the top by the separation', () => {
    const plan = { altitude_m: 100, altitude_separation_m: 30, zones: [{}, {}, {}] };
    expect(plan.zones.map((z, i) => zoneUploadAltitude(plan, z, i))).toEqual([160, 130, 100]);
  });

  it('keeps a zone altitude and defaults the separation to 20', () => {
    const plan = { altitude_m: 100, zones: [{ altitude_m: 55 }, {}] };
    expect(zoneUploadAltitude(plan, plan.zones[0], 0)).toBe(55);
    expect(zoneUploadAltitude(plan, { }, 0)).toBe(120);
  });
});

describe('cameraCheckZone', () => {
  const waypoints = [wp(0), wp(1), wp(2), wp(3), wp(4)];

  it('maps targ_wps bit n-1 to waypoint n and starts the route at nav_last_wp-1', () => {
    const z = cameraCheckZone({
      zoneIndex: 2, waypoints, altitude: 260, targWps: 0b10100, navLastWp: 2, dockAlt: 150, useTerrain: false,
    });
    expect(z.docks).toEqual([{ lat: 2, lon: 12, wp: 3 }, { lat: 4, lon: 14, wp: 5 }]);
    expect(z.route).toEqual(waypoints.slice(1));
    expect(z).toMatchObject({ zone_index: 2, altitude_m: 260, dock_alt_m: 150, use_terrain: false });
  });

  it('starts at the first waypoint for nav_last_wp 0 and at the last one for the last', () => {
    expect(cameraCheckZone({ waypoints, targWps: 0, navLastWp: 0 }).route).toHaveLength(5);
    expect(cameraCheckZone({ waypoints, targWps: 0, navLastWp: 5 }).route).toEqual([wp(4)]);
  });

  it('drops bits past the mission and sends unknown params as null', () => {
    const z = cameraCheckZone({ waypoints, targWps: 1 << 7, navLastWp: 0, dockAlt: undefined, useTerrain: 1 });
    expect(z.docks).toEqual([]);
    expect(z.dock_alt_m).toBeNull();
    expect(z.use_terrain).toBeNull();
  });

  it('accepts Leaflet-style lng points', () => {
    const z = cameraCheckZone({ waypoints: [{ lat: 1, lng: 2 }], targWps: 1, navLastWp: 0 });
    expect(z.docks).toEqual([{ lat: 1, lon: 2, wp: 1 }]);
    expect(z.route).toEqual([{ lat: 1, lon: 2 }]);
  });
});

describe('cameraCheckZonesForPlan', () => {
  const corridor = [wp(90), wp(91)];
  const track = [wp(0), wp(1)];
  const getAas = (_sysId, name) => ({ targ_alt: 150, use_trn: false })[name];

  it('puts the set corridor before the track for distributed search', () => {
    const plan = { altitude_m: 200, zones: [{ track, set_index: 0 }] };
    const [z] = cameraCheckZonesForPlan({
      plan, corridorPointsArr: [corridor], searchPattern: 'distributed',
      vehicleList: [{ sys_id: 7 }], targWpsBySys: { 7: 0b100 }, navLastWpBySys: { 7: 0 }, getAas,
    });
    expect(z.docks).toEqual([{ ...wp(0), wp: 3 }]);
    expect(z.route).toEqual([...corridor, ...track]);
    expect(z.dock_alt_m).toBe(150);
  });

  it('uses the track alone for corridor search and for a raw downloaded track', () => {
    for (const [pattern, zone] of [['corridor', { track }], ['distributed', { track, corridor_end_index: 2 }]]) {
      const [z] = cameraCheckZonesForPlan({
        plan: { altitude_m: 200, zones: [zone] }, corridorPointsArr: [corridor], searchPattern: pattern,
        vehicleList: [{ sys_id: 7 }], targWpsBySys: { 7: 0b1 }, navLastWpBySys: {}, getAas,
      });
      expect(z.docks).toEqual([{ ...wp(0), wp: 1 }]);
      expect(z.route).toEqual(track);
    }
  });

  it('prefers the zone sys_id over the vehicle order', () => {
    const plan = { altitude_m: 200, zones: [{ track, sys_id: 8 }] };
    const [z] = cameraCheckZonesForPlan({
      plan, searchPattern: 'corridor', vehicleList: [{ sys_id: 7 }],
      targWpsBySys: { 7: 0b1, 8: 0b10 }, navLastWpBySys: {}, getAas,
    });
    expect(z.docks.map((d) => d.wp)).toEqual([2]);
  });
});

const noWindow = {
  status: 'problems',
  zones: [{
    zone_index: 0, checked: true, estimate: false, height_m: 403, max_height_m: 389.58, windows: [],
    problems: [{ kind: 'no_search_window', severity: 'error', near_m: 757.09, range_m: 829.1, max_height_m: 389.58 }],
  }],
};

describe('describeCameraCheck', () => {
  it('gives the cause and the fixes for each problem', () => {
    const { problems, notes } = describeCameraCheck(noWindow, t);
    expect(problems).toEqual([{
      severity: 'error',
      cause: 'cameraCheck.noWindow',
      fixes: ['cameraCheck.fixLowerAltitude', 'cameraCheck.fixPitch', 'cameraCheck.fixProfile'],
    }]);
    expect(notes).toEqual([]);
  });

  it('marks an estimate and lists dock fixes for a dock never in view', () => {
    const result = { zones: [{
      zone_index: 0, checked: true, estimate: true, height_m: 260, max_height_m: 389.58,
      windows: [{ near_m: 488, range_edge_m: 787 }],
      problems: [{ kind: 'dock_never_in_view', severity: 'warning', wp: 3 }],
    }] };
    const [p] = describeCameraCheck(result, t).problems;
    expect(p.cause).toBe('cameraCheck.dockNeverInView cameraCheck.estimate');
    expect(p.fixes).toContain('cameraCheck.fixMoveDock');
    expect(p.fixes).toContain('cameraCheck.fixDetectAfter');
  });

  it('rounds the altitude limit down so following it passes', () => {
    const tp = (k, o) => (o ? `${k}:${JSON.stringify(o)}` : k);
    const { problems } = describeCameraCheck(noWindow, tp);
    expect(problems[0].fixes[0]).toBe('cameraCheck.fixLowerAltitude:{"max":389}');
    // The cause names the same limit as the fix.
    expect(problems[0].cause).toContain('"max":389}');
    expect(389).toBeLessThanOrEqual(noWindow.zones[0].problems[0].max_height_m);
  });

  it('says nothing for a compatible plan and notes an unchecked zone', () => {
    expect(describeCameraCheck({ status: 'ok', zones: [{ checked: true, problems: [] }] }, t))
      .toEqual({ problems: [], notes: [] });
    const unchecked = { zones: [{ zone_index: 0, checked: false, reason: 'dock_alt_unknown' }] };
    expect(describeCameraCheck(unchecked, t)).toEqual({ problems: [], notes: ['cameraCheck.dockAltUnknown'] });
  });
});

describe('confirmCameraCheck', () => {
  const zones = [{ docks: [{ lat: 0, lon: 0, wp: 1 }] }];

  it('asks with the cause and fixes when the camera cannot detect the docks', async () => {
    const confirm = vi.fn(() => false);
    const ok = await confirmCameraCheck({ zones, check: async () => noWindow, t, confirm });
    expect(ok).toBe(false);
    expect(confirm).toHaveBeenCalledWith(cameraCheckConfirmText(noWindow, t));
    expect(confirm.mock.calls[0][0]).toContain('cameraCheck.noWindow');
    expect(confirm.mock.calls[0][0]).toContain('cameraCheck.fixLowerAltitude');
  });

  it('goes ahead without asking for a compatible plan', async () => {
    const confirm = vi.fn();
    const ok = await confirmCameraCheck({ zones, check: async () => ({ status: 'ok', zones: [] }), t, confirm });
    expect(ok).toBe(true);
    expect(confirm).not.toHaveBeenCalled();
  });

  it('asks when the check fails instead of passing silently', async () => {
    const confirm = vi.fn(() => true);
    const ok = await confirmCameraCheck({ zones, check: async () => { throw new Error('down'); }, t, confirm });
    expect(ok).toBe(true);
    expect(confirm).toHaveBeenCalledWith('cameraCheck.couldNotRun');
  });

  it.each([
    ['an empty body', null],
    ['a body without zones', { status: 'ok' }],
    ['no profile', { status: 'skipped', reason: 'no_profile', zones: [] }],
    ['no camera', { status: 'skipped', reason: 'no_camera', zones: [] }],
    ['no status', { zones: [] }],
    ['an error status', { status: 'error', zones: [] }],
    ['problems without a known problem', { status: 'problems', zones: [{ checked: true, problems: [{ kind: 'new' }] }] }],
    ['a non-object zone', { status: 'ok', zones: [null] }],
    ['an array zone', { status: 'ok', zones: [[]] }],
    ['a null problem', { status: 'problems', zones: [{ checked: true, problems: [null] }] }],
    ['problems not a list', { status: 'problems', zones: [{ checked: true, problems: {} }] }],
    ['an ideal_360 skip with a null zone', { status: 'skipped', reason: 'ideal_360', zones: [null] }],
  ])('asks when the answer is %s', async (_name, answer) => {
    const confirm = vi.fn(() => false);
    expect(await confirmCameraCheck({ zones, check: async () => answer, t, confirm })).toBe(false);
    expect(confirm).toHaveBeenCalledWith('cameraCheck.couldNotRun');
  });

  it('passes the intentional ideal_360 skip without asking', async () => {
    const confirm = vi.fn();
    const answer = { status: 'skipped', reason: 'ideal_360', zones: [] };
    expect(await confirmCameraCheck({ zones, check: async () => answer, t, confirm })).toBe(true);
    expect(confirm).not.toHaveBeenCalled();
  });

  it('does not call the backend without sim docks', async () => {
    const check = vi.fn();
    expect(await confirmCameraCheck({ zones: [{ docks: [] }], check, t, confirm: vi.fn() })).toBe(true);
    expect(check).not.toHaveBeenCalled();
  });
});
