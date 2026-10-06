/**
 * Two lifecycle defects found on the geofence round-trip diff.
 *
 * 1. A reservation dropped the per-vehicle ordering watermark, so a reply from
 *    an older read was accepted after a newer one had already landed.
 * 2. A plan download replaces the planner geometry, which changes the derived
 *    fence ring — and the content signature then read that replacement as an
 *    operator edit, so the next untouched upload re-sent the fence.
 *
 * Both are driven through the real hooks: the real observation lifecycle, the
 * real usePlanPersistence.derivePlanPolygon publication, and the real
 * useMissionUpload acknowledgement path. Nothing here uses timers.
 */
import { act, renderHook } from '@testing-library/react';
import { useEffect, useRef } from 'react';
import { describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import usePlanPersistence from './usePlanPersistence';
import useVehicleConnection from './useVehicleConnection';
import useMissionUpload from './useMissionUpload';
import {
  FENCE_MODE, fenceUploadPayload, summarizeFenceObservations,
} from '../utils/fenceIntent';
import { fenceInclusion, DEFAULT_ORBIT_RADIUS_M } from '../utils/geo';

const ringA = () => [
  { lat: 32.000, lon: 34.000 },
  { lat: 32.010, lon: 34.000 },
  { lat: 32.010, lon: 34.010 },
  { lat: 32.000, lon: 34.010 },
];

// A clearly different search area, so the derived fence ring must differ too.
const ringB = () => [
  { lat: 32.200, lon: 34.200 },
  { lat: 32.215, lon: 34.200 },
  { lat: 32.215, lon: 34.215 },
  { lat: 32.200, lon: 34.215 },
];

const fenceResponse = (over = {}) => ({
  vertices: ringA(), exclusions: [], total_items: 4,
  params: { enable: 1, autoenable: 1, type: 4, action: 1 },
  params_readback: 'ok',
  ...over,
});

const offResponse = () => fenceResponse({
  params: { enable: 0, autoenable: 0, type: 4, action: 1 },
});

// ------------------------------------------------------- finding 1: ordering

describe('finding 1: the newest STARTED read wins, across reservations', () => {
  function observationHarness(vehicleList = [{ sys_id: 1, name: 'UAV1' }]) {
    const gates = [];
    const downloadFence = vi.fn().mockImplementation(
      () => new Promise((resolve) => { gates.push(resolve); }),
    );
    const ref = {};
    renderHook(() => {
      const mission = useMissionState();
      const connection = useVehicleConnection({
        mission,
        vehicleList,
        removeVehicle: vi.fn(),
        api: {
          downloadFence,
          // No mission on any vehicle: the startup reconciliation publishes
          // nothing, so this test sees only the reads it issues itself.
          downloadMission: vi.fn().mockResolvedValue({ waypoints: [] }),
          disconnectVehicle: vi.fn(),
        },
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        derivePlanPolygon: vi.fn(),
        settings: {},
      });
      ref.current = { mission, connection };
    });
    const modeOf = (sysId) => summarizeFenceObservations(
      ref.current.mission.fenceObservations, vehicleList.map((v) => v.sys_id),
    ).modes[sysId];
    const statusOf = (sysId) => ref.current.mission.fenceObservations.bySysId[sysId]?.status;
    return { ref, gates, downloadFence, modeOf, statusOf };
  }

  it('rejects the first read\'s late reply after a second landed and a third started', async () => {
    const { ref, gates, modeOf, statusOf } = observationHarness();
    const reads = [];

    // Read 1 starts and stays outstanding.
    await act(async () => { reads[0] = ref.current.connection.observeFenceFromVehicle(1); });
    // Read 2 starts and lands: the vehicle reports its fence enabled.
    await act(async () => { reads[1] = ref.current.connection.observeFenceFromVehicle(1); });
    await act(async () => { gates[1](fenceResponse()); await reads[1]; });
    expect(modeOf(1)).toBe(FENCE_MODE.ON);

    // Read 3 starts. Its reservation puts the vehicle back to "asked, unknown"
    // — and that must NOT forget that reads older than read 3 are stale.
    await act(async () => { reads[2] = ref.current.connection.observeFenceFromVehicle(1); });
    expect(statusOf(1)).toBe('pending');

    // Read 1 finally answers, with the opposite state. It is the oldest reply
    // in flight and must be discarded, not adopted.
    await act(async () => { gates[0](offResponse()); await reads[0]; });
    expect(statusOf(1)).toBe('pending');
    expect(modeOf(1)).not.toBe(FENCE_MODE.OFF);

    // Read 3 answers and is accepted, because it is the newest started read.
    await act(async () => { gates[2](fenceResponse()); await reads[2]; });
    expect(modeOf(1)).toBe(FENCE_MODE.ON);
  });

  it('keeps each vehicle\'s own watermark when reads interleave across the fleet', async () => {
    const roster = [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }];
    const { ref, gates, modeOf } = observationHarness(roster);
    const reads = [];

    // 1: read A then read B. 2: a single read. Order of replies is shuffled.
    await act(async () => { reads[0] = ref.current.connection.observeFenceFromVehicle(1); });
    await act(async () => { reads[1] = ref.current.connection.observeFenceFromVehicle(2); });
    await act(async () => { reads[2] = ref.current.connection.observeFenceFromVehicle(1); });

    // Vehicle 1's newest read lands first, then its oldest: the oldest loses.
    await act(async () => { gates[2](fenceResponse()); await reads[2]; });
    await act(async () => { gates[0](offResponse()); await reads[0]; });
    expect(modeOf(1)).toBe(FENCE_MODE.ON);

    // Vehicle 2's own read is untouched by vehicle 1's ordering.
    await act(async () => { gates[1](offResponse()); await reads[1]; });
    expect(modeOf(2)).toBe(FENCE_MODE.OFF);
    expect(modeOf(1)).toBe(FENCE_MODE.ON);
  });
});

// ------------------------------------------- finding 2: download vs authored

describe('finding 2: a downloaded plan replacement is not an operator edit', () => {
  /**
   * Real App wiring for the fence: the real derivation (fenceInclusion), the
   * real upload payload decision, the real acknowledgement path through
   * useMissionUpload, and the real plan publication through
   * usePlanPersistence.derivePlanPolygon driven by useVehicleConnection.
   */
  function planHarness(downloadedMission) {
    const api = {
      uploadMissions: vi.fn().mockResolvedValue({
        status: 'complete', fence_ok: true,
        results: [{ sys_id: 1, error: null, fence: { applied: true, action: 'enable',
          enabled: true, total: 4, error: null, failed_params: [] } }],
      }),
      writeAasParams: vi.fn(), restartCompanionsReady: vi.fn(),
      downloadMission: vi.fn().mockResolvedValue(downloadedMission),
      downloadFence: vi.fn().mockResolvedValue(fenceResponse()),
      disconnectVehicle: vi.fn(),
    };
    const ref = {};
    renderHook(() => {
      const mission = useMissionState();
      const drawing = { loadVertices: vi.fn() };
      const undoRef = useRef([]);
      const suppressRegenRef = useRef(false);
      const persistence = usePlanPersistence({
        ...mission,
        drawing, undoRef, suppressRegenRef,
        localAnalyze: vi.fn(),
        handleSaveSettings: vi.fn(),
        settings: { fallback_delivery_locations: [] },
        plannerReady: false,
        authorFenceIntent: mission.authorFenceIntent,
        clearFenceIntent: mission.clearFenceIntent,
        resetFenceObservations: mission.resetFenceObservations,
      });
      const connection = useVehicleConnection({
        mission,
        vehicleList: [{ sys_id: 1, name: 'UAV1' }],
        removeVehicle: vi.fn(),
        api,
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        derivePlanPolygon: persistence.derivePlanPolygon,
        settings: {},
      });
      // App's own derivation and payload decision, unchanged.
      const fenceAuto = mission.fenceEnabled && mission.polygon.length >= 3
        ? fenceInclusion(mission.polygon, [], [], {
            marginM: mission.fenceOffsetM,
            takeoffRadiusM: mission.takeoffRoundM,
            orbitRadiusM: DEFAULT_ORBIT_RADIUS_M,
          })
        : null;
      const fencePolygon = mission.fenceEnabled
        ? (mission.fenceCustomVertices ?? fenceAuto) : null;
      // App's sanitized keep-out list, which is upload state.
      const exclusions = (mission.exclusionPolygons || [])
        .filter((e) => Array.isArray(e) && e.length >= 3);
      const upload = fenceUploadPayload({
        enabled: mission.fenceEnabled, intent: mission.fenceIntent,
        acknowledged: mission.fenceAcknowledged,
        vertices: fencePolygon, exclusions,
      });
      // App's effect: geometry replaced by a download is adopted, not authored.
      useEffect(() => {
        mission.adoptDownloadedFenceGeometry(upload.signature);
      }, [mission.fenceGeometryRevision, mission.adoptDownloadedFenceGeometry, upload.signature]);
      const { handleUpload } = useMissionUpload({
        mission, effectiveUavCount: 1,
        localGenerate: vi.fn().mockResolvedValue(null),
        vehicleList: [{ sys_id: 1 }], api,
        settings: {
          fallback_delivery_locations: [{ name: 'F', type: 'other', lat: 32.0, lon: 34.0 }],
          simulation: { sim_mode: false },
        },
        fence: upload.payload,
        fenceInvalid: upload.invalid,
        fenceIntentGeneration: mission.fenceIntent?.generation ?? null,
        onFenceIntentResolved: mission.resolveFenceIntent,
      });
      ref.current = { mission, connection, handleUpload, fencePolygon, upload };
    });
    return { ref, api };
  }

  const uploadedPlan = (polygon) => ({
    waypoints: polygon.map((p) => ({ ...p, alt: 120 })),
    search_pattern: 'distributed', altitude_m: 120,
    polygon, launch_point: null, corridor_backbone: [],
  });

  async function enableAndUpload(ref) {
    act(() => {
      ref.current.mission.setPolygon(ringA());
      ref.current.mission.setPlan({
        zones: [{ zone_index: 0, track: ringA() }], altitude_m: 120,
      });
    });
    act(() => { ref.current.mission.authorFenceIntent(true); });
    await act(() => ref.current.handleUpload());
  }

  it('omits the fence on an unedited upload after a download replaced the plan', async () => {
    const { ref, api } = planHarness(uploadedPlan(ringB()));
    await enableAndUpload(ref);
    // The authored enable went out and was acknowledged.
    expect(api.uploadMissions.mock.calls[0][1]).toMatchObject({ enabled: true });
    const sentRing = api.uploadMissions.mock.calls[0][1].vertices;
    expect(ref.current.mission.fenceIntent).toBeNull();

    // A real download of a DIFFERENT mission republishes the plan geometry, so
    // the derived ring changes on its own.
    await act(() => ref.current.connection.handleDownloadPlan());
    expect(ref.current.mission.polygon).toEqual(ringB());
    expect(ref.current.fencePolygon).not.toEqual(sentRing);

    // Nobody touched the fence, so this upload must leave every vehicle's
    // fence exactly as it is.
    await act(() => ref.current.handleUpload());
    expect(api.uploadMissions.mock.calls[1][1]).toBeNull();
  });

  it('still sends a fresh request when the operator edits the ring after a download', async () => {
    const { ref, api } = planHarness(uploadedPlan(ringB()));
    await enableAndUpload(ref);
    await act(() => ref.current.connection.handleDownloadPlan());

    // An operator drag of the derived ring IS an authored change.
    act(() => {
      ref.current.mission.setFenceCustomVertices(
        ref.current.fencePolygon.map((p, i) => (i === 0 ? { lat: p.lat + 0.002, lon: p.lon } : p)),
      );
    });
    await act(() => ref.current.handleUpload());
    expect(api.uploadMissions.mock.calls[1][1]).toMatchObject({ enabled: true });
  });

  it('still sends a fresh request when the operator changes the fence offset after a download', async () => {
    const { ref, api } = planHarness(uploadedPlan(ringB()));
    await enableAndUpload(ref);
    await act(() => ref.current.connection.handleDownloadPlan());

    act(() => { ref.current.mission.setFenceOffsetM(ref.current.mission.fenceOffsetM + 25); });
    await act(() => ref.current.handleUpload());
    expect(api.uploadMissions.mock.calls[1][1]).toMatchObject({ enabled: true });
  });

  it('still sends a fresh request when the operator adds a keep-out after a download', async () => {
    const { ref, api } = planHarness(uploadedPlan(ringB()));
    await enableAndUpload(ref);
    await act(() => ref.current.connection.handleDownloadPlan());

    act(() => {
      ref.current.mission.setExclusionPolygons([[
        { lat: 32.205, lon: 34.205 }, { lat: 32.206, lon: 34.205 }, { lat: 32.206, lon: 34.206 },
      ]]);
    });
    await act(() => ref.current.handleUpload());
    const sent = api.uploadMissions.mock.calls[1][1];
    expect(sent).toMatchObject({ enabled: true });
    expect(sent.exclusions).toHaveLength(1);
  });

  // A download that arrives while a request is pending cannot be adopted at the
  // time (the pending request still has to go out). That replacement must not
  // stay outstanding: once the request is acknowledged, the next authored edit
  // would be adopted instead — silently dropping the operator's own geometry.
  it.each([
    ['ring', (ref) => {
      ref.current.mission.setFenceCustomVertices(
        ref.current.fencePolygon.map((p, i) => (i === 0 ? { lat: p.lat + 0.002, lon: p.lon } : p)),
      );
    }],
    ['offset', (ref) => {
      ref.current.mission.setFenceOffsetM(ref.current.mission.fenceOffsetM + 25);
    }],
  ])('sends a %s edit made after a download that landed while the request was pending', async (_label, edit) => {
    const { ref, api } = planHarness(uploadedPlan(ringB()));
    act(() => {
      ref.current.mission.setPolygon(ringA());
      ref.current.mission.setPlan({ zones: [{ zone_index: 0, track: ringA() }], altitude_m: 120 });
    });
    // Authored, not yet uploaded: the request is pending when the plan lands.
    act(() => { ref.current.mission.authorFenceIntent(true); });
    await act(() => ref.current.connection.handleDownloadPlan());
    expect(ref.current.mission.fenceIntent).not.toBeNull();

    // The pending request goes out and is acknowledged.
    await act(() => ref.current.handleUpload());
    expect(api.uploadMissions.mock.calls[0][1]).toMatchObject({ enabled: true });
    expect(ref.current.mission.fenceIntent).toBeNull();

    // Now the operator edits the fence. That IS authored geometry and must be
    // uploaded, not swallowed by the earlier download replacement.
    act(() => { edit(ref); });
    const editedRing = ref.current.fencePolygon;
    await act(() => ref.current.handleUpload());
    const sent = api.uploadMissions.mock.calls[1][1];
    expect(sent).toMatchObject({ enabled: true });
    expect(sent.vertices).toEqual(editedRing.map((p) => ({ lat: p.lat, lon: p.lon })));
  });

  it('keeps a still-pending operator request against a late download', async () => {
    const { ref, api } = planHarness(uploadedPlan(ringB()));
    act(() => {
      ref.current.mission.setPolygon(ringA());
      ref.current.mission.setPlan({ zones: [{ zone_index: 0, track: ringA() }], altitude_m: 120 });
    });
    // Authored but never uploaded: the request is still pending.
    act(() => { ref.current.mission.authorFenceIntent(true); });
    await act(() => ref.current.connection.handleDownloadPlan());
    expect(ref.current.mission.fenceIntent).not.toBeNull();

    await act(() => ref.current.handleUpload());
    expect(api.uploadMissions.mock.calls[0][1]).toMatchObject({ enabled: true });
  });
});
