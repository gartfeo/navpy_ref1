import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useMissionUpload from './useMissionUpload';
import { confirmStub } from '../test/confirmStub';
import { activateUploadedSimPlan } from '../utils/missionUploadActivation';

vi.mock('../utils/missionUploadActivation', () => ({
  activateUploadedSimPlan: vi.fn(async () => ({ targWpsUpdates: {}, navLastWpUpdates: {} })),
}));

it.each(['distributed', 'corridor'])('sends current upload fields for %s', async (searchPattern) => {
  const dock = { name: 'Fixture', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }];
  const mission = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
    searchPattern,
    launchPoint: null, corridorPoints: [],
    plan: { zones: [{ zone_index: 0, track }], altitude_m: 120 },
    setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
    goToMonitor: vi.fn(), deliveryHubAssignments: [0], manualDeliveryHubEdit: true,
    simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(),
  };
  const api = { uploadMissions: vi.fn().mockResolvedValue({ status: 'complete' }),
    writeAasParams: vi.fn(), restartCompanionsReady: vi.fn() };
  const { result } = renderHook(() => useMissionUpload({ mission, effectiveUavCount: 1,
    localGenerate: vi.fn().mockResolvedValue(null), vehicleList: [{ sys_id: 999 }], api,
    settings: { default_delivery_hubs: [dock], simulation: { sim_mode: false } }, fence: null }));
  await act(() => result.current.handleUpload());
  expect(api.uploadMissions).toHaveBeenCalledOnce();
  const [assignments, fence] = api.uploadMissions.mock.calls[0];
  expect(assignments[0]).toMatchObject({ sys_id: 999, zone_index: 0, altitude_m: 120,
    waypoints: track,
    default_delivery_hub: { lat: dock.lat, lon: dock.lon, type: dock.type } });
  // Every zone targets the single dock class; the upload model forbids the field.
  expect(Object.hasOwn(assignments[0], 'dock_classes')).toBe(false);
  expect(Object.hasOwn(assignments[0], 'poi_classes')).toBe(false);
  expect(Object.hasOwn(assignments[0], 'default_poi')).toBe(false);
  expect(assignments[0].search_pattern).toBe(searchPattern);
  expect(Object.hasOwn(assignments[0], 'tactic')).toBe(false);
  expect(Object.hasOwn(assignments[0], 'searchPattern')).toBe(false);
  expect(fence).toBeNull();
  expect(api.writeAasParams).not.toHaveBeenCalled();
  expect(api.restartCompanionsReady).not.toHaveBeenCalled();
  expect(mission.goToMonitor).toHaveBeenCalledOnce();
});

it('drops the downloaded item numbers of each vehicle that took the upload', async () => {
  // A downloaded track carries its mission item numbers (utils/missionProgress.js);
  // after the upload the vehicle flies the GCS's layout, so they no longer apply.
  const dock = { name: 'Fixture', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0, seq: 2 }, { lat: 0.001, lon: 0.001, seq: 3 }];
  let plan = { zones: [{ zone_index: 0, track, sys_id: 5 }, { zone_index: 1, track, sys_id: 6 }], altitude_m: 120 };
  const setPlan = vi.fn((next) => { plan = typeof next === 'function' ? next(plan) : next; });
  const mission = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }], searchPattern: 'distributed',
    launchPoint: null, corridorPoints: [], plan,
    setPlan, setLaunchPoints: [null], setCorridorPointsArr: [[]],
    goToMonitor: vi.fn(), deliveryHubAssignments: [0, 0], manualDeliveryHubEdit: true,
    simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(),
  };
  const api = { uploadMissions: vi.fn().mockResolvedValue({ status: 'partial', results: [
    { sys_id: 5, progress: 1, done: true }, { sys_id: 6, progress: 0, done: true, error: 'timeout' },
  ] }), writeAasParams: vi.fn(), restartCompanionsReady: vi.fn() };
  const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
  const { result } = renderHook(() => useMissionUpload({ mission, effectiveUavCount: 2,
    localGenerate: vi.fn().mockResolvedValue(null), vehicleList: [{ sys_id: 5 }, { sys_id: 6 }], api,
    settings: { default_delivery_hubs: [dock], simulation: { sim_mode: false } }, fence: null }));
  await act(() => result.current.handleUpload());
  expect(plan.zones[0].track.some((p) => 'seq' in p)).toBe(false);
  expect(plan.zones[1].track.map((p) => p.seq)).toEqual([2, 3]);
  alert.mockRestore();
});

// Codex review of the plan-mode change: upload A, edit to B, upload B with one
// UAV failing. The old baseline A must not survive: Don't save would restore
// A and offer Start while UAV 5 carries B.
it('drops the plan-on-the-UAVs baseline as it starts writing, and takes a new one only on success', async () => {
  const dock = { name: 'Fixture', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }];
  const mission = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }], searchPattern: 'distributed',
    launchPoint: null, corridorPoints: [],
    plan: { zones: [{ zone_index: 0, track, sys_id: 5 }, { zone_index: 1, track, sys_id: 6 }], altitude_m: 120 },
    setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
    goToMonitor: vi.fn(), deliveryHubAssignments: [0, 0], manualDeliveryHubEdit: true,
    simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(),
  };
  const order = [];
  const onUploadStart = vi.fn(() => { order.push('unsynced'); });
  const editor = { fenceEnabled: false, simDockWps: {} };
  const capturePlan = vi.fn(() => ({ ...editor }));
  const onPlanSynced = vi.fn(() => order.push('synced'));
  const partial = { status: 'partial', results: [
    { sys_id: 5, progress: 1, done: true }, { sys_id: 6, progress: 0, done: true, error: 'timeout' },
  ] };
  const api = { uploadMissions: vi.fn(async () => { order.push('write'); return partial; }),
    writeAasParams: vi.fn(), restartCompanionsReady: vi.fn() };
  const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
  const { result } = renderHook(() => useMissionUpload({ mission, effectiveUavCount: 2,
    localGenerate: vi.fn().mockResolvedValue(null), vehicleList: [{ sys_id: 5 }, { sys_id: 6 }], api,
    onPlanSynced, onUploadStart, capturePlan,
    settings: { default_delivery_hubs: [dock], simulation: { sim_mode: false } }, fence: null }));
  await act(() => result.current.handleUpload());
  expect(order).toEqual(['unsynced', 'write']);

  api.uploadMissions.mockImplementation(async () => { order.push('write'); return { status: 'complete' }; });
  order.length = 0;
  await act(() => result.current.handleUpload());
  expect(order).toEqual(['unsynced', 'write', 'synced']);
  // The baseline is what was sent: the editor the upload read, with the plan it sent.
  expect(onPlanSynced).toHaveBeenCalledWith({ upload: {
    ...editor,
    plan: { altitude_m: 120, zones: [
      { zone_index: 0, track, sys_id: 5, altitude_m: 140 }, { zone_index: 1, track, sys_id: 6, altitude_m: 120 },
    ] },
    deliveryHubAssignments: undefined,
    fenceIntent: undefined,
  } });
  alert.mockRestore();
});

describe('camera check before upload (sim)', () => {
  const dock = { name: 'Hub', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }, { lat: 0.002, lon: 0.002 }];
  const impossible = { status: 'problems', zones: [{
    zone_index: 0, checked: true, estimate: false, height_m: 403, max_height_m: 389.58, windows: [],
    problems: [{ kind: 'no_search_window', severity: 'error', near_m: 757, range_m: 829, max_height_m: 389.58 }],
  }] };

  // `answer`: the operator's reply to the in-page confirm, if one is asked.
  function run(checkPlanCamera, simDockWps = { 0: [1] }, answer = false) {
    const { confirm, wrapper } = confirmStub(answer);
    const mission = {
      polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
      searchPattern: 'corridor', launchPoint: null, corridorPoints: [],
      plan: { zones: [{ zone_index: 0, track }], altitude_m: 403 },
      setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
      goToMonitor: vi.fn(), deliveryHubAssignments: [0], manualDeliveryHubEdit: true,
      simDockWps, detectAfterWps: {}, setUploadProgress: vi.fn(),
      setVehicleTargWps: vi.fn(), setVehicleNavLastWp: vi.fn(),
    };
    const api = {
      uploadMissions: vi.fn().mockResolvedValue({ status: 'complete' }),
      writeAasParams: vi.fn().mockResolvedValue({ results: [] }),
      restartCompanionsReady: vi.fn().mockResolvedValue({ ok: true }),
      checkPlanCamera,
    };
    const getAas = (_sysId, name) => ({ targ_alt: 0, use_trn: false })[name];
    const { result } = renderHook(() => useMissionUpload({ mission, effectiveUavCount: 1,
      localGenerate: vi.fn().mockResolvedValue(null), vehicleList: [{ sys_id: 5 }], api, getAas,
      settings: { default_delivery_hubs: [dock], simulation: { sim_mode: true } }, fence: null }), { wrapper });
    return { result, api, mission, confirm };
  }

  it('asks what needs to be done and a cancel sends nothing', async () => {
    const { result, api, confirm } = run(vi.fn().mockResolvedValue(impossible));
    await act(() => result.current.handleUpload());
    // Request: track waypoint 2 is the dock (targ_wps bit 1), flown at the upload altitude.
    expect(api.checkPlanCamera).toHaveBeenCalledWith([expect.objectContaining({
      altitude_m: 403, dock_alt_m: 0, use_terrain: false, docks: [{ ...track[1], wp: 2 }],
    })]);
    expect(confirm.mock.calls[0][0]).toContain('cameraCheck.noWindow');
    expect(confirm.mock.calls[0][0]).toContain('cameraCheck.fixLowerAltitude');
    expect(api.uploadMissions).not.toHaveBeenCalled();
  });

  it('uploads after the operator confirms', async () => {
    const { result, api, mission } = run(vi.fn().mockResolvedValue(impossible), undefined, true);
    await act(() => result.current.handleUpload());
    expect(api.uploadMissions).toHaveBeenCalledOnce();
    // Activation writes the targ_wps the check was given.
    expect(activateUploadedSimPlan).toHaveBeenCalledWith(expect.objectContaining({
      targWpsCalls: [{ sys_id: 5, targ_wps: 0b10 }],
    }));
    expect(mission.goToMonitor).toHaveBeenCalledOnce();
  });

  it('uploads a compatible plan without asking', async () => {
    const { result, api, confirm } = run(vi.fn().mockResolvedValue({ status: 'ok', zones: [] }));
    await act(() => result.current.handleUpload());
    expect(confirm).not.toHaveBeenCalled();
    expect(api.uploadMissions).toHaveBeenCalledOnce();
  });

  it('asks when the check fails', async () => {
    const { result, api, confirm } = run(vi.fn().mockRejectedValue(new Error('down')));
    await act(() => result.current.handleUpload());
    expect(confirm).toHaveBeenCalledWith('cameraCheck.couldNotRun');
    expect(api.uploadMissions).not.toHaveBeenCalled();
  });

  it('skips the check without sim docks', async () => {
    const check = vi.fn();
    const { result, api } = run(check, {});
    await act(() => result.current.handleUpload());
    expect(check).not.toHaveBeenCalled();
    expect(api.uploadMissions).toHaveBeenCalledOnce();
  });
});

describe('return-to-hub rally point', () => {
  function run(uploadResult) {
    const mission = {
      polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
      searchPattern: 'distributed', launchPoint: null, corridorPoints: [],
      plan: { zones: [{ zone_index: 0, track: [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }] }], altitude_m: 120 },
      setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
      goToMonitor: vi.fn(), deliveryHubAssignments: [0], manualDeliveryHubEdit: true,
      simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(),
    };
    const api = { uploadMissions: vi.fn().mockResolvedValue(uploadResult),
      writeAasParams: vi.fn(), restartCompanionsReady: vi.fn() };
    const { result } = renderHook(() => useMissionUpload({ mission, effectiveUavCount: 1,
      localGenerate: vi.fn().mockResolvedValue(null), vehicleList: [{ sys_id: 3 }], api,
      settings: { default_delivery_hubs: [{ name: 'Hub', lat: 0.01, lon: 0.02 }],
        simulation: { sim_mode: false } }, fence: null }));
    return { result, mission };
  }

  it('stays in planning and names the vehicle whose RTL would not reach the hub', async () => {
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    const { result, mission } = run({
      status: 'complete', fence_ok: true, rally_ok: false,
      results: [{ sys_id: 3, error: null,
        rally: { applied: false, action: 'set', error: 'rally point not acknowledged after 3 attempts', failed_params: [] } }],
    });
    await act(() => result.current.handleUpload());
    expect(mission.goToMonitor).not.toHaveBeenCalled();
    expect(alert).toHaveBeenCalledOnce();
    expect(alert.mock.calls[0][0]).toContain('Return-to-hub point not set');
    expect(alert.mock.calls[0][0]).toContain('Vehicle 3: rally point not acknowledged');
    alert.mockRestore();
  });

  it('moves to monitoring once every rally point is set', async () => {
    const { result, mission } = run({
      status: 'complete', fence_ok: true, rally_ok: true,
      results: [{ sys_id: 3, error: null, rally: { applied: true, action: 'set', total: 1 } }],
    });
    await act(() => result.current.handleUpload());
    expect(mission.goToMonitor).toHaveBeenCalledOnce();
  });
});
