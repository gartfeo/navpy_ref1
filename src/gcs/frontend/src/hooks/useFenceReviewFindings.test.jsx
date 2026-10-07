/**
 * Regressions for the five review findings on the geofence round-trip fix.
 *
 * Each block reproduces one finding through the real code path, not through a
 * stand-in for it.
 */
import { act, renderHook, waitFor } from '@testing-library/react';
import { useEffect } from 'react';
import { describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import useVehicleConnection from './useVehicleConnection';
import useMissionUpload from './useMissionUpload';
import {
  FENCE_MODE,
  fenceSignature,
  fenceUploadPayload,
  shouldAutoEnableDemoFence,
  summarizeFenceObservations,
} from '../utils/fenceIntent';

const ring = (d = 0) => [
  { lat: 32.0 + d, lon: 34.0 },
  { lat: 32.01 + d, lon: 34.0 },
  { lat: 32.0 + d, lon: 34.01 },
];

const fenceResponse = (over = {}) => ({
  vertices: ring(), exclusions: [], total_items: 3,
  params: { enable: 1, autoenable: 1, type: 4, action: 1 },
  params_readback: 'ok',
  ...over,
});

const downloadedMission = {
  waypoints: [{ lat: 32, lon: 34, alt: 120 }, { lat: 32.001, lon: 34.001, alt: 120 }],
  search_pattern: 'distributed', altitude_m: 120,
};

// ---------------------------------------------------------------- finding 1

describe('finding 1: demo auto-enable must not win the race with the fence read', () => {
  /**
   * Harness that reproduces App's real ordering: the plan download publishes
   * the polygon (that is what derivePlanPolygon does), React then runs the demo
   * auto-enable effect, and only later does the fence response arrive. The
   * effect uses the same decision function App uses, so this exercises the real
   * gate rather than a copy of it.
   */
  function demoHarness(downloadFence, mission1 = downloadedMission) {
    const missionRef = {};
    const { result } = renderHook(() => {
      const mission = useMissionState();
      missionRef.current = mission;
      const connection = useVehicleConnection({
        mission,
        vehicleList: [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }],
        removeVehicle: vi.fn(),
        api: {
          downloadFence,
          downloadMission: vi.fn().mockResolvedValue(mission1),
          disconnectVehicle: vi.fn(),
        },
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        // The real derivePlanPolygon publishes the polygon; that publication is
        // exactly what arms the demo effect.
        derivePlanPolygon: () => mission.setPolygon(ring()),
        settings: {},
      });
      useEffect(() => {
        if (shouldAutoEnableDemoFence({
          simMode: true,
          polygonLength: mission.polygon.length,
          fenceTouched: mission.fenceTouched,
          fenceEnabled: mission.fenceEnabled,
          observations: summarizeFenceObservations(mission.fenceObservations, [1, 2]),
        })) mission.setFenceEnabled(true);
      });
      return { mission, connection };
    });
    return { result, missionRef };
  }

  it('does not enable the fence while the fleet fence read is still pending', async () => {
    let release;
    const gate = new Promise((r) => { release = r; });
    const downloadFence = vi.fn().mockImplementation(() => gate.then(() => fenceResponse()));
    const { result, missionRef } = demoHarness(downloadFence);

    let pending;
    await act(async () => { pending = result.current.connection.handleDownloadPlan(); });
    // The polygon is published and React has run the effect, but no fence
    // answer has arrived yet. The demo default must stay out of it.
    await waitFor(() => expect(missionRef.current.polygon.length).toBe(3));
    expect(missionRef.current.fenceEnabled).toBe(false);

    await act(async () => { release(); await pending; });
    expect(missionRef.current.fenceEnabled).toBe(false);
  });

  it('still applies the demo default to a genuinely new plan', async () => {
    // The ordinary demo start: the UAVs are connected and carry no mission, so
    // the startup download publishes nothing and claims nothing. The zone the
    // operator then draws is a new plan, and the default belongs to it.
    const downloadFence = vi.fn().mockResolvedValue(fenceResponse());
    const { result, missionRef } = demoHarness(downloadFence, { waypoints: [] });
    await act(async () => { await Promise.resolve(); });
    act(() => { missionRef.current.setPolygon(ring()); });
    expect(downloadFence).not.toHaveBeenCalled();
    expect(result.current.mission.fenceEnabled).toBe(true);
  });

  it('keeps the default inhibited when every fence read failed', async () => {
    const { result, missionRef } = demoHarness(vi.fn().mockResolvedValue(null));
    await act(() => result.current.connection.handleDownloadPlan());
    expect(missionRef.current.fenceEnabled).toBe(false);
  });
});

// ---------------------------------------------------------------- finding 2

describe('finding 2: an acknowledged enable must not be re-sent', () => {
  const enabled = { enabled: true, vertices: ring(), exclusions: [] };

  it('omits the fence when the same enabled geometry was already acknowledged', () => {
    const acknowledged = { signature: fenceSignature(enabled) };
    expect(fenceUploadPayload({ ...enabled, intent: null, acknowledged }).payload).toBeNull();
  });

  it('sends the first enable, before anything was acknowledged', () => {
    expect(fenceUploadPayload({ ...enabled, intent: null, acknowledged: null }).payload)
      .toMatchObject({ enabled: true });
  });

  it('sends again after the operator edits the ring', () => {
    const acknowledged = { signature: fenceSignature(enabled) };
    const edited = { enabled: true, vertices: ring(0.4), exclusions: [] };
    expect(fenceUploadPayload({ ...edited, intent: null, acknowledged }).payload)
      .toMatchObject({ enabled: true });
  });

  it('sends again after the operator edits a keep-out', () => {
    const acknowledged = { signature: fenceSignature(enabled) };
    const edited = { ...enabled, exclusions: [ring(0.6)] };
    expect(fenceUploadPayload({ ...edited, intent: null, acknowledged }).payload)
      .toMatchObject({ enabled: true });
  });

  it('sends while an explicit enable request is still pending', () => {
    const acknowledged = { signature: fenceSignature(enabled) };
    expect(fenceUploadPayload({
      ...enabled, intent: { generation: 3, enabled: true }, acknowledged,
    }).payload).toMatchObject({ enabled: true });
  });

  it('records the acknowledged signature so an unchanged re-upload is a no-op', () => {
    const { result } = renderHook(() => useMissionState());
    act(() => result.current.authorFenceIntent(true));
    const gen = result.current.fenceIntent.generation;
    act(() => result.current.resolveFenceIntent(gen, true, fenceSignature(enabled)));
    expect(result.current.fenceIntent).toBeNull();
    expect(fenceUploadPayload({
      ...enabled, intent: result.current.fenceIntent,
      acknowledged: result.current.fenceAcknowledged,
    }).payload).toBeNull();
  });
});

// ---------------------------------------------------------------- finding 3

describe('finding 3: an incomplete roster is never a fleet answer', () => {
  it('does not report a fleet mode or ring while a vehicle is unaccounted for', () => {
    const { result } = renderHook(() => useMissionState());
    act(() => result.current.recordFenceObservation(
      1, { status: 'known', mode: FENCE_MODE.ON, ring: ring(), exclusions: [], params: { enable: 1, autoenable: 1, type: 4, action: 1 } },
      result.current.beginFenceObservation(),
    ));
    const s = summarizeFenceObservations(result.current.fenceObservations, [1, 2]);
    expect(s.count).toBe(2);
    expect(s.mode).not.toBe(FENCE_MODE.ON);
    expect(s.ring).toBeNull();
    expect(s.modes[2]).toBe(FENCE_MODE.UNKNOWN);
  });

  it('reserves the whole roster as pending before any answer arrives', () => {
    const { result } = renderHook(() => useMissionState());
    act(() => { result.current.reserveFenceObservations([1, 2, 3]); });
    const s = summarizeFenceObservations(result.current.fenceObservations, [1, 2, 3]);
    expect(s.hasObservations).toBe(true);
    expect(s.mode).toBe(FENCE_MODE.UNKNOWN);
    expect(s.ring).toBeNull();
  });
});

// ---------------------------------------------------------------- finding 4

describe('finding 4: mission and fence failures are reported independently', () => {
  it('reports a fence failure on one vehicle and a mission failure on another', async () => {
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    const mission = {
      polygon: ring(), searchPattern: 'distributed',
      launchPoint: null, corridorPoints: [],
      plan: { zones: [{ zone_index: 0, track: ring() }, { zone_index: 1, track: ring(0.1) }], altitude_m: 120 },
      setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
      goToMonitor: vi.fn(), deliveryHubAssignments: [0, 0], manualDeliveryHubEdit: true,
      simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(),
    };
    const api = {
      uploadMissions: vi.fn().mockResolvedValue({
        status: 'partial_failure', fence_ok: false,
        results: [
          { sys_id: 1, error: null,
            fence: { applied: false, action: 'enable', error: 'no ACK', failed_params: ['FENCE_ENABLE'] } },
          { sys_id: 2, error: 'mission upload failed', fence: null },
        ],
      }),
      writeAasParams: vi.fn(), restartCompanionsReady: vi.fn(),
    };
    const { result } = renderHook(() => useMissionUpload({
      mission, effectiveUavCount: 2,
      localGenerate: vi.fn().mockResolvedValue(null),
      vehicleList: [{ sys_id: 1 }, { sys_id: 2 }], api,
      settings: { default_delivery_hubs: [{ name: 'F', lat: 0.01, lon: 0.02 }],
        simulation: { sim_mode: false } },
      fence: { enabled: true, vertices: ring() },
      fenceIntentGeneration: 5, onFenceIntentResolved: vi.fn(),
    }));
    await act(() => result.current.handleUpload());

    const message = alert.mock.calls.at(-1)[0];
    // Vehicle 2's mission failed.
    expect(message).toMatch(/Vehicle 2/);
    expect(message).toMatch(/mission upload failed/);
    // Vehicle 1's MISSION succeeded but its FENCE did not — it must be named,
    // and must not be presented as a mission failure.
    expect(message).toMatch(/Vehicle 1/);
    expect(message).toMatch(/FENCE_ENABLE|no ACK/);
    expect(mission.goToMonitor).not.toHaveBeenCalled();
    alert.mockRestore();
  });
});

// ---------------------------------------------------------------- finding 5

describe('finding 5: only the newest request for a vehicle is accepted', () => {
  const observation = (mode) => ({
    status: 'known', mode, ring: [], exclusions: [],
    params: { enable: mode === FENCE_MODE.ON ? 1 : 0, autoenable: 0, type: 4, action: 1 },
  });

  it('rejects an older same-vehicle read that resolves after a newer one', () => {
    const { result } = renderHook(() => useMissionState());
    let older; let newer;
    act(() => { older = result.current.beginFenceObservation(); });
    act(() => { newer = result.current.beginFenceObservation(); });
    act(() => result.current.recordFenceObservation(1, observation(FENCE_MODE.ON), newer));
    act(() => result.current.recordFenceObservation(1, observation(FENCE_MODE.OFF), older));
    expect(result.current.fenceObservations.bySysId[1].mode).toBe(FENCE_MODE.ON);
  });

  it('rejects a read that was in flight when the operator authored a change', () => {
    const { result } = renderHook(() => useMissionState());
    let token;
    act(() => { token = result.current.beginFenceObservation(); });
    act(() => result.current.authorFenceIntent(false));
    act(() => result.current.recordFenceObservation(1, observation(FENCE_MODE.ON), token));
    expect(result.current.fenceObservations.bySysId[1]?.status).not.toBe('known');
  });
});
