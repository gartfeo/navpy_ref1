/**
 * Round-trip behavior of the geofence across download -> upload.
 *
 * The defect these cover: a plan download turned ONE vehicle's observed fence
 * into authored planner state, and the "operator touched the fence" flag was
 * sticky. Together, an untouched download/upload round trip re-sent an
 * explicit fence request — disabling a fence the operator never switched off,
 * or re-uploading a downloaded ring as if the operator had drawn it.
 */
import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import useVehicleConnection from './useVehicleConnection';
import useMissionUpload from './useMissionUpload';
import {
  FENCE_MODE, fenceSignature, fenceUploadPayload, summarizeFenceObservations,
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

function renderConnection(downloadFence) {
  const missionRef = {};
  const { result } = renderHook(() => {
    const mission = useMissionState();
    missionRef.current = mission;
    const connection = useVehicleConnection({
      mission,
      vehicleList: [],
      removeVehicle: vi.fn(),
      api: { downloadFence, downloadMission: vi.fn(), disconnectVehicle: vi.fn() },
      telemetryStoreRef: { current: {} },
      derivePlanPolygon: vi.fn(),
      settings: {},
    });
    return { mission, connection };
  });
  return result;
}

describe('observing a vehicle fence', () => {
  it('never turns an observed ring into authored upload geometry', async () => {
    const result = renderConnection(vi.fn().mockResolvedValue(fenceResponse()));
    await act(() => result.current.connection.observeFenceFromVehicle(1));

    const m = result.current.mission;
    expect(m.fenceCustomVertices).toBeNull();
    expect(m.fenceEnabled).toBe(false);
    expect(m.fenceTouched).toBe(false);
    // The ring is visible as an observation only.
    const summary = summarizeFenceObservations(m.fenceObservations, [1]);
    expect(summary.ring).toHaveLength(3);
    expect(fenceUploadPayload({
      enabled: m.fenceEnabled, intent: m.fenceIntent, vertices: summary.ring,
    }).payload).toBeNull();
  });

  it('observes every rostered vehicle separately, keeping raw modes distinct', async () => {
    const byId = {
      1: fenceResponse(),
      2: fenceResponse({ params: { enable: 0, autoenable: 2, type: 4, action: 1 } }),
      3: null, // readback unreachable
    };
    const result = renderConnection(vi.fn().mockImplementation((id) => Promise.resolve(byId[id])));
    await act(() => result.current.connection.observeFleetFence([
      { sys_id: 1 }, { sys_id: 2 }, { sys_id: 3 },
    ]));

    const obs = result.current.mission.fenceObservations.bySysId;
    expect(obs[1].mode).toBe(FENCE_MODE.ON);
    expect(obs[2].mode).toBe(FENCE_MODE.AUTO);
    expect(obs[2].params.autoenable).toBe(2);
    expect(obs[3].status).toBe('failed');
    expect(summarizeFenceObservations(result.current.mission.fenceObservations, [1, 2, 3]).mode)
      .toBe(FENCE_MODE.MIXED);
  });

  it('discards a result whose roster generation is stale', async () => {
    let release;
    const gate = new Promise((r) => { release = r; });
    const downloadFence = vi.fn().mockImplementation(() => gate.then(() => fenceResponse()));
    const result = renderConnection(downloadFence);

    let pending;
    await act(async () => { pending = result.current.connection.observeFenceFromVehicle(1); });
    // A plan reset advances the observation epoch while the read is in flight.
    act(() => { result.current.mission.resetFenceObservations(); });
    await act(async () => { release(); await pending; });

    expect(result.current.mission.fenceObservations.bySysId[1]).toBeUndefined();
  });
});

describe('upload payload across a round trip', () => {
  const uploadHarness = (mission, fence, extra = {}) => {
    const api = {
      uploadMissions: vi.fn().mockResolvedValue({ status: 'complete', fence_ok: true, results: [] }),
      writeAasParams: vi.fn(), restartCompanionsReady: vi.fn(),
    };
    const { result } = renderHook(() => useMissionUpload({
      mission, effectiveUavCount: 1,
      localGenerate: vi.fn().mockResolvedValue(null),
      vehicleList: [{ sys_id: 1 }], api,
      settings: { fallback_delivery_locations: [{ name: 'F', lat: 0.01, lon: 0.02 }],
        simulation: { sim_mode: false } },
      fence, ...extra,
    }));
    return { result, api };
  };

  const baseMission = (over = {}) => ({
    polygon: ring(), searchPattern: 'distributed',
    launchPoint: null, corridorPoints: [],
    plan: { zones: [{ zone_index: 0, track: ring() }], altitude_m: 120 },
    setPlan: vi.fn(), setLaunchPoints: [null], setCorridorPointsArr: [[]],
    goToMonitor: vi.fn(), fallbackLocationAssignments: [0], manualFallbackLocationEdit: true,
    simDockWps: {}, detectAfterWps: {}, setUploadProgress: vi.fn(), ...over,
  });

  it('sends no fence block for an untouched download -> re-upload round trip', async () => {
    const mission = baseMission();
    const { result, api } = uploadHarness(mission, null);
    await act(() => result.current.handleUpload());
    expect(api.uploadMissions.mock.calls[0][1]).toBeNull();
  });

  it('blocks the upload when an enabled fence has invalid geometry', async () => {
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    const mission = baseMission();
    const { result, api } = uploadHarness(mission, null, { fenceInvalid: true });
    await act(() => result.current.handleUpload());
    expect(api.uploadMissions).not.toHaveBeenCalled();
    expect(alert).toHaveBeenCalled();
    alert.mockRestore();
  });

  it('consumes acknowledged intent for its own generation', async () => {
    const onFenceIntentResolved = vi.fn();
    const mission = baseMission();
    const sent = { enabled: false, vertices: [] };
    const { result } = uploadHarness(mission, sent, {
      fenceIntentGeneration: 4, onFenceIntentResolved,
    });
    await act(() => result.current.handleUpload());
    // The signature of the request the vehicles confirmed travels with the
    // acknowledgement, so an identical re-upload is not re-sent.
    expect(onFenceIntentResolved).toHaveBeenCalledWith(4, true, fenceSignature(sent));
    expect(mission.goToMonitor).toHaveBeenCalledOnce();
  });

  it('retains intent and stays in planning when the fence work failed', async () => {
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    const onFenceIntentResolved = vi.fn();
    const mission = baseMission();
    const api = {
      uploadMissions: vi.fn().mockResolvedValue({
        status: 'complete', fence_ok: false,
        results: [{ sys_id: 1, error: null,
          fence: { applied: false, action: 'enable', error: 'no ACK', failed_params: ['FENCE_ENABLE'] } }],
      }),
      writeAasParams: vi.fn(), restartCompanionsReady: vi.fn(),
    };
    const { result } = renderHook(() => useMissionUpload({
      mission, effectiveUavCount: 1,
      localGenerate: vi.fn().mockResolvedValue(null),
      vehicleList: [{ sys_id: 1 }], api,
      settings: { fallback_delivery_locations: [{ name: 'F', lat: 0.01, lon: 0.02 }],
        simulation: { sim_mode: false } },
      fence: { enabled: true, vertices: ring() },
      fenceIntentGeneration: 4, onFenceIntentResolved,
    }));
    await act(() => result.current.handleUpload());
    expect(onFenceIntentResolved).toHaveBeenCalledWith(
      4, false, fenceSignature({ enabled: true, vertices: ring() }),
    );
    expect(mission.goToMonitor).not.toHaveBeenCalled();
    expect(alert).toHaveBeenCalled();
    alert.mockRestore();
  });
});

describe('download plan -> re-upload, through the real operator path', () => {
  const downloadedMission = {
    waypoints: [{ lat: 32, lon: 34, alt: 120 }, { lat: 32.001, lon: 34.001, alt: 120 }],
    search_pattern: 'distributed', altitude_m: 120,
  };

  const downloadHarness = (fenceResponse) => {
    const missionRef = {};
    const { result } = renderHook(() => {
      const m = useMissionState();
      missionRef.current = m;
      return useVehicleConnection({
        mission: m, vehicleList: [{ sys_id: 1, name: 'UAV1' }], removeVehicle: vi.fn(),
        api: {
          downloadFence: vi.fn().mockResolvedValue(fenceResponse),
          downloadMission: vi.fn().mockResolvedValue(downloadedMission),
          disconnectVehicle: vi.fn(),
        },
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        derivePlanPolygon: vi.fn(), settings: {},
      });
    });
    return { result, missionRef };
  };

  it('leaves an enabled vehicle fence untouched on re-upload', async () => {
    const { result, missionRef } = downloadHarness(fenceResponse());
    await act(() => result.current.handleDownloadPlan());
    const m = missionRef.current;
    expect(m.fenceCustomVertices).toBeNull();
    expect(m.fenceEnabled).toBe(false);
    expect(m.fenceTouched).toBe(false);
    expect(fenceUploadPayload({
      enabled: m.fenceEnabled, intent: m.fenceIntent, vertices: m.fenceCustomVertices,
    }).payload).toBeNull();
  });

  it('preserves an autoenable-only vehicle by omitting the fence block', async () => {
    const { result, missionRef } = downloadHarness(fenceResponse({
      params: { enable: 0, autoenable: 1, type: 4, action: 1 },
    }));
    await act(() => result.current.handleDownloadPlan());
    const m = missionRef.current;
    // The numeric mode survives because nothing is written back.
    expect(summarizeFenceObservations(m.fenceObservations, [1]).mode).toBe(FENCE_MODE.AUTO);
    expect(fenceUploadPayload({
      enabled: m.fenceEnabled, intent: m.fenceIntent, vertices: [],
    }).payload).toBeNull();
  });

  it('does not turn an empty vehicle fence table into an explicit disable', async () => {
    const { result, missionRef } = downloadHarness(fenceResponse({
      vertices: [], total_items: 0, params: { enable: 0, autoenable: 0, type: 4, action: 1 },
    }));
    await act(() => result.current.handleDownloadPlan());
    const m = missionRef.current;
    expect(m.fenceTouched).toBe(false);
    expect(m.fenceIntent).toBeNull();
    expect(fenceUploadPayload({
      enabled: m.fenceEnabled, intent: m.fenceIntent, vertices: [],
    }).payload).toBeNull();
  });

  it('records an unreadable fence as unknown, inhibiting implicit writes', async () => {
    const { result, missionRef } = downloadHarness(null);
    await act(() => result.current.handleDownloadPlan());
    const m = missionRef.current;
    const summary = summarizeFenceObservations(m.fenceObservations, [1]);
    expect(summary.mode).toBe(FENCE_MODE.UNKNOWN);
    expect(summary.allFailed).toBe(true);
    expect(fenceUploadPayload({
      enabled: m.fenceEnabled, intent: m.fenceIntent, vertices: [],
    }).payload).toBeNull();
  });

  it('still allows an explicit disable while the observed state is unknown', async () => {
    const { result, missionRef } = downloadHarness(null);
    await act(() => result.current.handleDownloadPlan());
    act(() => missionRef.current.authorFenceIntent(false));
    const m = missionRef.current;
    expect(fenceUploadPayload({
      enabled: m.fenceEnabled, intent: m.fenceIntent, vertices: [],
    }).payload).toMatchObject({ enabled: false });
  });
});

describe('mission fence state', () => {
  it('bumps the intent generation on every operator toggle', () => {
    const { result } = renderHook(() => useMissionState());
    act(() => result.current.toggleFence());
    const first = result.current.fenceIntent;
    expect(first).toMatchObject({ enabled: true });
    expect(result.current.fenceTouched).toBe(true);
    act(() => result.current.toggleFence());
    expect(result.current.fenceIntent.enabled).toBe(false);
    expect(result.current.fenceIntent.generation).not.toBe(first.generation);
  });

  it('keeps the operator toggle authored after its intent is consumed', () => {
    const { result } = renderHook(() => useMissionState());
    act(() => result.current.toggleFence());
    act(() => result.current.toggleFence());
    const gen = result.current.fenceIntent.generation;
    act(() => result.current.resolveFenceIntent(gen, true));
    expect(result.current.fenceIntent).toBeNull();
    // fenceTouched must survive so demo auto-enable cannot revive the fence.
    expect(result.current.fenceTouched).toBe(true);
    expect(result.current.fenceEnabled).toBe(false);
  });

  it('does not consume intent edited after the upload started', () => {
    const { result } = renderHook(() => useMissionState());
    act(() => result.current.toggleFence());
    const gen = result.current.fenceIntent.generation;
    act(() => result.current.toggleFence());
    act(() => result.current.resolveFenceIntent(gen, true));
    expect(result.current.fenceIntent).not.toBeNull();
  });
});
