import { act, renderHook } from '@testing-library/react';
import { expect, it, vi } from 'vitest';
import useMissionUpload from './useMissionUpload';

it.each(['distributed', 'corridor'])('sends current upload fields for %s', async (searchPattern) => {
  const dock = { name: 'Fixture', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }];
  const mission = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
    searchPattern,
    launchPoint: null, corridorPoints: [],
    plan: { zones: [{ zone_index: 0, track }], altitude_m: 120 },
    setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
    goToMonitor: vi.fn(), fallbackLocationAssignments: [0], manualFallbackLocationEdit: true,
    simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(),
  };
  const api = { uploadMissions: vi.fn().mockResolvedValue({ status: 'complete' }),
    writeAasParams: vi.fn(), restartCompanionsReady: vi.fn() };
  const { result } = renderHook(() => useMissionUpload({ mission, effectiveUavCount: 1,
    localGenerate: vi.fn().mockResolvedValue(null), vehicleList: [{ sys_id: 999 }], api,
    settings: { fallback_delivery_locations: [dock], simulation: { sim_mode: false } }, fence: null }));
  await act(() => result.current.handleUpload());
  expect(api.uploadMissions).toHaveBeenCalledOnce();
  const [assignments, fence] = api.uploadMissions.mock.calls[0];
  expect(assignments[0]).toMatchObject({ sys_id: 999, zone_index: 0, altitude_m: 120,
    waypoints: track,
    fallback_delivery_location: { lat: dock.lat, lon: dock.lon, type: dock.type } });
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
