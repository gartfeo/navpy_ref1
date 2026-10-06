import { act, renderHook } from '@testing-library/react';
import { expect, it, vi } from 'vitest';
import useMissionUpload from './useMissionUpload';

it.each([
  [{}, 'distributed'], [{ 0: ['override-fixture'] }, 'distributed'],
  [{}, 'corridor'], [{ 0: ['override-fixture'] }, 'corridor'],
])('sends current upload fields and preserves selection %j for %s', async (overrides, searchPattern) => {
  const dock = { name: 'Fixture', lat: 0.01, lon: 0.02, type: 'other' };
  const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }];
  const mission = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
    dockClasses: ['fixture-class'], perUavDockClasses: overrides, searchPattern,
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
    waypoints: track, dock_classes: overrides[0] || mission.dockClasses,
    fallback_delivery_location: { lat: dock.lat, lon: dock.lon, type: dock.type } });
  expect(Object.hasOwn(assignments[0], 'target_classes')).toBe(false);
  expect(Object.hasOwn(assignments[0], 'default_target')).toBe(false);
  expect(assignments[0].search_pattern).toBe(searchPattern);
  expect(Object.hasOwn(assignments[0], 'tactic')).toBe(false);
  expect(Object.hasOwn(assignments[0], 'searchPattern')).toBe(false);
  expect(fence).toBeNull();
  expect(api.writeAasParams).not.toHaveBeenCalled();
  expect(api.restartCompanionsReady).not.toHaveBeenCalled();
  expect(mission.goToMonitor).toHaveBeenCalledOnce();
});
