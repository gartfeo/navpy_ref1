/**
 * A vehicle with no mission is not a failed download. The backend answers it
 * with 200 and an empty waypoint list: the startup restore shows nothing, and
 * a download the operator started says "No mission on <UAV>". A real failure
 * still says it failed.
 */
import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import useVehicleConnection from './useVehicleConnection';

const EMPTY = (sysId) => ({
  sys_id: sysId, mission_count: 0, waypoints: [], items: [], _diagnostic_request_id: `r${sysId}`,
});
const FAILED = (sysId) => ({ error: 'Mission download failed', _diagnostic_request_id: `r${sysId}` });

// Let every queued promise continuation run (the mocks answer at once).
const settle = () => act(() => new Promise((resolve) => { setTimeout(resolve, 0); }));

function harness(vehicleList, answerFor) {
  const setNotification = vi.fn();
  const downloadMission = vi.fn((sysId) => Promise.resolve(answerFor(sysId)));
  const ref = {};
  renderHook(() => {
    const mission = useMissionState();
    const connection = useVehicleConnection({
      mission,
      vehicleList,
      removeVehicle: vi.fn(),
      api: {
        downloadMission,
        downloadFence: vi.fn().mockResolvedValue(null),
        disconnectVehicle: vi.fn(),
        connectVehicle: vi.fn().mockResolvedValue({ status: 'connected' }),
      },
      telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
      derivePlanPolygon: vi.fn(),
      settings: {},
      setNotification,
    });
    ref.current = { mission, connection };
  });
  return { ref, setNotification, downloadMission };
}

describe('an empty vehicle', () => {
  it('shows no error on the startup restore', async () => {
    const { ref, setNotification, downloadMission } = harness(
      [{ sys_id: 1, name: 'UAV1' }], EMPTY,
    );
    await settle();

    expect(downloadMission).toHaveBeenCalledWith(1);
    expect(setNotification).not.toHaveBeenCalled();
    expect(ref.current.mission.plan).toBeNull();
  });

  it('still reports a failed startup restore', async () => {
    const { setNotification } = harness([{ sys_id: 1, name: 'UAV1' }], FAILED);
    await settle();

    expect(setNotification).toHaveBeenCalledWith('Mission download failed for UAV1');
  });

  it('says "No mission on <UAV>" on an operator fleet download', async () => {
    const { ref, setNotification } = harness(
      [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }],
      (sysId) => (sysId === 1 ? EMPTY(sysId) : FAILED(sysId)),
    );
    await settle();
    setNotification.mockClear();

    await act(async () => { await ref.current.connection.handleDownloadPlan(); });

    expect(setNotification).toHaveBeenCalledWith('No mission on UAV1');
    expect(setNotification).toHaveBeenCalledWith('Mission download failed for UAV2');
    expect(setNotification).toHaveBeenCalledTimes(2);
    expect(ref.current.mission.plan).toBeNull();
  });

  it('says "No mission on <UAV>" when the operator connects it', async () => {
    const { ref, setNotification } = harness([], EMPTY);
    await settle();

    await act(async () => { await ref.current.connection.handleConnect('udp:x', 3, 'UAV3'); });
    await settle();

    expect(setNotification).toHaveBeenCalledWith('No mission on UAV3');
    expect(setNotification).toHaveBeenCalledTimes(1);
  });

  it('names an unnamed vehicle by its id', async () => {
    const { ref, setNotification } = harness([], EMPTY);
    await settle();

    await act(async () => { await ref.current.connection.handleConnect('udp:x', 4, undefined); });
    await settle();

    expect(setNotification).toHaveBeenCalledWith('No mission on UAV 4');
  });
});
