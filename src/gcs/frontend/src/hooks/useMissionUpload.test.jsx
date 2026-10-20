import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useMissionUpload from './useMissionUpload';
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

describe('camera check before upload (sim)', () => {
  const dock = { name: 'Hub', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }, { lat: 0.002, lon: 0.002 }];
  const impossible = { status: 'problems', zones: [{
    zone_index: 0, checked: true, estimate: false, height_m: 403, max_height_m: 389.58, windows: [],
    problems: [{ kind: 'no_search_window', severity: 'error', near_m: 757, range_m: 829, max_height_m: 389.58 }],
  }] };

  function run(checkPlanCamera, simDockWps = { 0: [1] }) {
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
      settings: { default_delivery_hubs: [dock], simulation: { sim_mode: true } }, fence: null }));
    return { result, api, mission };
  }

  it('asks what needs to be done and a cancel sends nothing', async () => {
    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(false);
    const { result, api } = run(vi.fn().mockResolvedValue(impossible));
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
    vi.spyOn(window, 'confirm').mockReturnValue(true);
    const { result, api, mission } = run(vi.fn().mockResolvedValue(impossible));
    await act(() => result.current.handleUpload());
    expect(api.uploadMissions).toHaveBeenCalledOnce();
    // Activation writes the targ_wps the check was given.
    expect(activateUploadedSimPlan).toHaveBeenCalledWith(expect.objectContaining({
      targWpsCalls: [{ sys_id: 5, targ_wps: 0b10 }],
    }));
    expect(mission.goToMonitor).toHaveBeenCalledOnce();
  });

  it('uploads a compatible plan without asking', async () => {
    const confirm = vi.spyOn(window, 'confirm');
    const { result, api } = run(vi.fn().mockResolvedValue({ status: 'ok', zones: [] }));
    await act(() => result.current.handleUpload());
    expect(confirm).not.toHaveBeenCalled();
    expect(api.uploadMissions).toHaveBeenCalledOnce();
  });

  it('asks when the check fails', async () => {
    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(false);
    const { result, api } = run(vi.fn().mockRejectedValue(new Error('down')));
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
