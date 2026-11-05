/**
 * App.jsx hands this hook a new vehicleList array on every telemetry update
 * (~5 Hz). The hook's targ_wps / nav_last_wp fetch (/params) must go out once
 * per vehicle per connect, even when updates arrive while it is pending.
 */
import { act, renderHook } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import useVehicleConnection from './useVehicleConnection';

const settle = () => act(() => new Promise((resolve) => { setTimeout(resolve, 0); }));

const uav = (sysId) => ({ sys_id: sysId, name: `UAV${sysId}` });
const answer = (targWps) => ({ ok: true, json: () => Promise.resolve({ params: { targ_wps: targWps } }) });

// Each /params call waits until the test answers it.
function stubFetch() {
  const calls = [];
  const fetchMock = vi.fn((url) => new Promise((resolve) => { calls.push({ url: String(url), resolve }); }));
  vi.stubGlobal('fetch', fetchMock);
  const params = () => calls.filter((c) => c.url.endsWith('/params'));
  return { params, urls: () => params().map((c) => c.url).sort() };
}

function render(initialList) {
  const setVehicleTargWps = vi.fn();
  const props = {
    removeVehicle: vi.fn(),
    api: {
      downloadMission: vi.fn().mockResolvedValue({ sys_id: 0, mission_count: 0, waypoints: [], items: [] }),
      downloadFence: vi.fn().mockResolvedValue(null),
      disconnectVehicle: vi.fn(),
      connectVehicle: vi.fn().mockResolvedValue({ status: 'connected' }),
    },
    telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
    derivePlanPolygon: vi.fn(),
    settings: {},
    setNotification: vi.fn(),
    setVehicleTargWps,
    setVehicleNavLastWp: vi.fn(),
  };
  const hook = renderHook(({ vehicleList }) => {
    const mission = useMissionState();
    return useVehicleConnection({ ...props, mission, vehicleList });
  }, { initialProps: { vehicleList: initialList } });
  const show = async (list) => { hook.rerender({ vehicleList: list }); await settle(); };
  return { show, setVehicleTargWps };
}

describe('per-vehicle /params fetch', () => {
  beforeEach(() => { vi.stubGlobal('fetch', vi.fn()); });
  afterEach(() => { vi.unstubAllGlobals(); });

  it('sends one fetch per vehicle while new vehicleList arrays arrive during the pending fetch', async () => {
    const f = stubFetch();
    const { show } = render([uav(1), uav(2)]);
    await settle();
    // Telemetry updates: equal content, a new array each time, fetch still pending.
    for (let i = 0; i < 10; i += 1) await show([uav(1), uav(2)]);
    expect(f.urls()).toEqual(['/api/vehicles/1/params', '/api/vehicles/2/params']);

    // A vehicle that appears later gets its own single fetch.
    await show([uav(1), uav(2), uav(3)]);
    await show([uav(1), uav(2), uav(3)]);
    expect(f.urls().filter((u) => u === '/api/vehicles/3/params')).toHaveLength(1);
  });

  it('fetches again after a vehicle reconnects', async () => {
    const f = stubFetch();
    const { show } = render([uav(1)]);
    await settle();
    f.params()[0].resolve(answer(4));
    await settle();

    await show([]); // disconnect
    await show([uav(1)]); // reconnect
    await show([uav(1)]);
    expect(f.urls()).toEqual(['/api/vehicles/1/params', '/api/vehicles/1/params']);
  });

  it('drops an answer that comes back after its vehicle disconnected', async () => {
    const f = stubFetch();
    const { show, setVehicleTargWps } = render([uav(1)]);
    await settle();
    await show([]);
    await show([uav(1)]);
    const [stale, fresh] = f.params();

    stale.resolve(answer(4));
    await settle();
    expect(setVehicleTargWps).not.toHaveBeenCalled();

    fresh.resolve(answer(6));
    await settle();
    expect(setVehicleTargWps).toHaveBeenCalledTimes(1);
    expect(setVehicleTargWps.mock.calls[0][0]({})).toEqual({ 1: 6 });
  });
});
