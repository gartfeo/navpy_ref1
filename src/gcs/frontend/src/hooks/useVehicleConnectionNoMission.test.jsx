/**
 * A connect that finds no mission on any connected UAV says so to App
 * (onFleetWithoutMission), which opens the plan editor. Only a pass that asked
 * every connected UAV, and heard "empty" from each, counts: a failed answer is
 * not "no mission", and a UAV not asked in this pass is not known to be empty.
 */
import { act, renderHook } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import useVehicleConnection, { fleetHasNoMission } from './useVehicleConnection';
import { planFromDownload } from '../utils/planDownloadRun';

const EMPTY = (sysId) => ({
  sys_id: sysId, mission_count: 0, waypoints: [], items: [], _diagnostic_request_id: `r${sysId}`,
});
const FAILED = (sysId) => ({ error: 'Mission download failed', _diagnostic_request_id: `r${sysId}` });
const MISSION = (sysId) => ({
  sys_id: sysId,
  mission_count: 2,
  altitude_m: 100,
  waypoints: [{ lat: 40.1, lon: 44.1, alt: 100 }, { lat: 40.2, lon: 44.2, alt: 100 }],
  items: [],
  _diagnostic_request_id: `r${sysId}`,
});

const settle = () => act(() => new Promise((resolve) => { setTimeout(resolve, 0); }));

function harness(vehicleList, answerFor, { discovered = [] } = {}) {
  const onFleetWithoutMission = vi.fn();
  const ref = {};
  const hook = renderHook(({ vehicles }) => {
    const mission = useMissionState();
    const connection = useVehicleConnection({
      mission,
      vehicleList: vehicles,
      removeVehicle: vi.fn(),
      api: {
        downloadMission: vi.fn((sysId) => Promise.resolve(answerFor(sysId))),
        downloadFence: vi.fn().mockResolvedValue(null),
        disconnectVehicle: vi.fn(),
        connectVehicle: vi.fn().mockResolvedValue({ status: 'connected' }),
        discoverVehicles: vi.fn().mockResolvedValue({ found: discovered.length, vehicles: discovered }),
      },
      telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
      derivePlanPolygon: vi.fn(),
      onFleetWithoutMission,
      settings: {},
      setNotification: vi.fn(),
    });
    ref.current = { mission, connection };
  }, { initialProps: { vehicles: vehicleList } });
  return { ref, onFleetWithoutMission, hook };
}

beforeEach(() => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue({ ok: false, json: async () => null }));
});
afterEach(() => { vi.unstubAllGlobals(); });

describe('fleetHasNoMission', () => {
  const kinds = (entries) => new Map(entries);
  it('every connected UAV answered empty', () => {
    expect(fleetHasNoMission(kinds([[1, 'empty'], [2, 'empty']]), [1, 2])).toBe(true);
  });
  it('a UAV with a mission, a failed answer or no answer yet is not "no mission"', () => {
    expect(fleetHasNoMission(kinds([[1, 'empty'], [2, 'mission']]), [1, 2])).toBe(false);
    expect(fleetHasNoMission(kinds([[1, 'empty'], [2, 'failed']]), [1, 2])).toBe(false);
    expect(fleetHasNoMission(kinds([[1, 'empty']]), [1, 2])).toBe(false);
  });
  it('the asked UAVs count even before telemetry lists them; no UAV is no answer', () => {
    expect(fleetHasNoMission(kinds([[3, 'empty']]), [])).toBe(true);
    expect(fleetHasNoMission(kinds([]), [])).toBe(false);
  });
});

describe('a connect that finds a mission', () => {
  it('a single UAV\'s download publishes the UAVs\' own plan', async () => {
    const { ref, onFleetWithoutMission } = harness([], MISSION);
    await settle();
    await act(async () => { await ref.current.connection.handleConnect('udp:x', 3, 'UAV3'); });
    await settle();
    expect(ref.current.mission.plan?.zones?.length).toBe(1);
    expect(planFromDownload(ref.current.mission.plan)).toBe(true);
    expect(onFleetWithoutMission).not.toHaveBeenCalled();
  });
});

describe('a connect that finds no mission', () => {
  it('page-load restore: every UAV empty opens the plan editor once', async () => {
    const { onFleetWithoutMission } = harness([{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }], EMPTY);
    await settle();
    expect(onFleetWithoutMission).toHaveBeenCalledTimes(1);
  });

  it('page-load restore: a failed answer keeps the flight view', async () => {
    const { onFleetWithoutMission } = harness(
      [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }],
      (sysId) => (sysId === 1 ? EMPTY(sysId) : FAILED(sysId)),
    );
    await settle();
    expect(onFleetWithoutMission).not.toHaveBeenCalled();
  });

  it('page-load restore: a UAV with a mission keeps the flight view', async () => {
    const { onFleetWithoutMission, ref } = harness(
      [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }],
      (sysId) => (sysId === 1 ? EMPTY(sysId) : MISSION(sysId)),
    );
    await settle();
    expect(ref.current.mission.plan?.zones?.length).toBe(1);
    expect(onFleetWithoutMission).not.toHaveBeenCalled();
  });

  it('Connect (scan): every discovered UAV empty opens the plan editor', async () => {
    const discovered = [{ sys_id: 5, name: 'UAV5' }, { sys_id: 6, name: 'UAV6' }];
    const { ref, onFleetWithoutMission } = harness([], EMPTY, { discovered });
    await settle();
    await act(async () => { await ref.current.connection.handleScan('udp:x'); });
    await settle();
    expect(onFleetWithoutMission).toHaveBeenCalledTimes(1);
  });

  it('a single connect counts only when it is the whole fleet', async () => {
    const alone = harness([], EMPTY);
    await settle();
    await act(async () => { await alone.ref.current.connection.handleConnect('udp:x', 3, 'UAV3'); });
    await settle();
    expect(alone.onFleetWithoutMission).toHaveBeenCalledTimes(1);

    // UAV 1 is connected and was not asked in this pass: it may carry a mission.
    const joined = harness([{ sys_id: 1, name: 'UAV1' }], (sysId) => (sysId === 1 ? MISSION(1) : EMPTY(sysId)));
    await settle();
    // The restored plan is the UAVs' own (the monitor's start gate reads this).
    expect(planFromDownload(joined.ref.current.mission.plan)).toBe(true);
    joined.onFleetWithoutMission.mockClear();
    await act(async () => { await joined.ref.current.connection.handleConnect('udp:x', 3, 'UAV3'); });
    await settle();
    expect(joined.onFleetWithoutMission).not.toHaveBeenCalled();
  });
});
