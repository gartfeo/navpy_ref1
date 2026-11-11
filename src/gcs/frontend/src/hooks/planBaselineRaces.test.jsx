/**
 * The start control shows only for the plan the UAVs carry. The baseline is
 * taken from what an upload sent or a download read, never from the editor
 * when the request completes: an edit made while the request was out is not
 * on the UAVs. The real usePlanSnapshot, wired to the real upload and download
 * hooks as usePlanningOrchestrator and App wire them.
 */
import { useEffect, useRef, useState } from 'react';
import { act, renderHook } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import usePlanSnapshot from './usePlanSnapshot';
import useMissionUpload from './useMissionUpload';
import useMissionState from './useMissionState';
import useVehicleConnection from './useVehicleConnection';
import usePlanPersistence from './usePlanPersistence';
import { confirmStub } from '../test/confirmStub';
import { buildDeliveryHubsFromDownload } from '../utils/deliveryHubAssignment';

vi.mock('../utils/missionUploadActivation', () => ({
  activateUploadedSimPlan: vi.fn(async () => ({ targWpsUpdates: {}, navLastWpUpdates: {} })),
}));

function deferred() {
  let resolve;
  const promise = new Promise((r) => { resolve = r; });
  return { promise, resolve };
}

const settle = () => act(() => new Promise((resolve) => { setTimeout(resolve, 0); }));

const SNAPSHOT_DEPS = () => ({
  setPolygon: vi.fn(), setSearchPattern: vi.fn(), setAnalysis: vi.fn(),
  setUavCount: vi.fn(), setPartitionAngleDeg: vi.fn(), setRouteOffsetM: vi.fn(),
  setSetLaunchPoints: vi.fn(), setSetCorridorPoints: vi.fn(), setDeliveryHubAssignments: vi.fn(),
  setManualDeliveryHubEdit: vi.fn(), restoreFenceState: vi.fn(),
  setSimDockWps: vi.fn(), setDetectAfterWps: vi.fn(), setFenceCustomVertices: vi.fn(),
  setExclusionPolygons: vi.fn(),
  drawingRef: { current: { loadVertices: vi.fn() } },
  suppressRegenRef: { current: false },
  undoRef: { current: [] },
  goToMonitor: vi.fn(),
  savePlanRef: { current: vi.fn() },
});

describe('upload', () => {
  const HUBS = [{ name: 'Hub 1', lat: 0.01, lon: 0.02, type: 'other' }];
  const TRACK = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }, { lat: 0.002, lon: 0.002 }];
  // Two zones stacked by the altitude separation (planner: 20 m).
  const PLAN = {
    altitude_m: 120, altitude_separation_m: 20,
    zones: [{ zone_index: 0, track: TRACK, altitude_m: 140 }, { zone_index: 1, track: TRACK, altitude_m: 120 }],
  };
  const EDITOR = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
    searchPattern: 'distributed', analysis: null, uavCount: 2, effectiveUavCount: 2,
    partitionAngleDeg: null, routeOffsetM: null,
    setLaunchPoints: [null], setCorridorPointsArr: [[]],
    deliveryHubAssignments: [0, 0], manualDeliveryHubEdit: false,
    simDockWps: { 0: [1] }, detectAfterWps: {},
    fenceTouched: true, fenceEnabled: true, fenceOffsetM: 50,
    fenceIntent: null, fenceAcknowledged: null, fenceObservations: null,
    fenceCustomVertices: null, exclusionPolygons: [],
    deliveryHubs: HUBS,
  };

  // `regenerate`: the plan localGenerate produces at upload (as usePlanGeneration
  // does, it publishes it), or nothing (the plan on the map is uploaded).
  function harness({ uploadMissions, checkPlanCamera, regenerate = null, editor = EDITOR }) {
    const { wrapper } = confirmStub(true);
    const deps = SNAPSHOT_DEPS();
    const api = {
      uploadMissions,
      checkPlanCamera,
      writeAasParams: vi.fn(), restartCompanionsReady: vi.fn(),
    };
    const hook = renderHook((editor) => {
      const [plan, setPlan] = useState(PLAN);
      const [simDockWps, setSimDockWps] = useState(editor.simDockWps);
      const [detectAfterWps, setDetectAfterWps] = useState(editor.detectAfterWps);
      const [deliveryHubAssignments, setDeliveryHubAssignments] = useState(editor.deliveryHubAssignments);
      // usePlanningOrchestrator: a new zone count clears the picks in planning.
      const zoneCount = useRef(plan?.zones?.length || 0);
      useEffect(() => {
        const n = plan?.zones?.length || 0;
        if (zoneCount.current !== 0 && n !== zoneCount.current) {
          setSimDockWps({});
          setDetectAfterWps({});
        }
        zoneCount.current = n;
      }, [plan]);
      const localGenerate = () => {
        if (!regenerate) return null;
        setPlan(regenerate);
        return regenerate;
      };
      const picks = { simDockWps, detectAfterWps, deliveryHubAssignments };
      const snap = usePlanSnapshot({ ...deps, ...editor, ...picks, plan, setPlan });
      const upload = useMissionUpload({
        mission: {
          ...editor, ...picks, plan, setPlan, setSimDockWps, setDetectAfterWps, setDeliveryHubAssignments,
          launchPoint: null, corridorPoints: [],
          goToMonitor: vi.fn(), setUploadProgress: vi.fn(),
        },
        effectiveUavCount: 2,
        localGenerate,
        vehicleList: [{ sys_id: 5 }, { sys_id: 6 }],
        api,
        getAas: (_sysId, name) => ({ targ_alt: 0, use_trn: false })[name],
        onPlanSynced: snap.onPlanSynced,
        onUploadStart: snap.beginPlanUpload,
        capturePlan: snap.capturePlanState,
        settings: { default_delivery_hubs: editor.deliveryHubs, simulation: { sim_mode: true } },
        fence: null,
        setVehicleTargWps: vi.fn(), setVehicleNavLastWp: vi.fn(),
      });
      return { snap, upload, setPlan, picks };
    }, { wrapper, initialProps: editor });
    return { hook, api };
  }

  const OK = { status: 'ok', zones: [] };
  const COMPLETE = { status: 'complete', results: [{ sys_id: 5 }, { sys_id: 6 }] };

  it('an unedited upload is the plan on the UAVs', async () => {
    const { hook } = harness({
      uploadMissions: vi.fn().mockResolvedValue(COMPLETE),
      checkPlanCamera: vi.fn().mockResolvedValue(OK),
    });
    await act(() => hook.result.current.upload.handleUpload());
    expect(hook.result.current.snap.planSynced).toBe(true);
  });

  // Case 1: the operator changed the fence margin while the camera check that
  // runs before anything is sent was still open. The request was built from
  // the old margin.
  it('a fence margin changed while the camera check is open is not on the UAVs', async () => {
    const check = deferred();
    const { hook, api } = harness({
      uploadMissions: vi.fn().mockResolvedValue(COMPLETE),
      checkPlanCamera: vi.fn(() => check.promise),
    });
    let done;
    act(() => { done = hook.result.current.upload.handleUpload(); });
    await settle();
    expect(api.checkPlanCamera).toHaveBeenCalledOnce();
    hook.rerender({ ...EDITOR, fenceOffsetM: 100 });
    await act(async () => { check.resolve(OK); await done; });
    expect(api.uploadMissions).toHaveBeenCalledOnce();
    expect(hook.result.current.snap.planSynced).toBe(false);
    // Back to the margin that was sent: the plan is the UAVs' again.
    hook.rerender(EDITOR);
    expect(hook.result.current.snap.planSynced).toBe(true);
  });

  // Case 2: the altitude separation setting changed while the upload was in
  // flight; the planner regenerated the zones at the new altitudes.
  it('zones regenerated at a new altitude separation during the upload are not on the UAVs', async () => {
    const write = deferred();
    const { hook, api } = harness({
      uploadMissions: vi.fn(() => write.promise),
      checkPlanCamera: vi.fn().mockResolvedValue(OK),
    });
    let done;
    act(() => { done = hook.result.current.upload.handleUpload(); });
    await settle();
    expect(api.uploadMissions).toHaveBeenCalledOnce();
    act(() => {
      hook.result.current.setPlan({
        ...PLAN, altitude_separation_m: 40,
        zones: [{ ...PLAN.zones[0], altitude_m: 160 }, PLAN.zones[1]],
      });
    });
    await act(async () => { write.resolve(COMPLETE); await done; });
    expect(hook.result.current.snap.planSynced).toBe(false);
  });

  // Codex review, round 3 (and the live check): regenerated at upload with
  // another zone count, the plan loses its dock picks in the editor. The
  // request must not carry them either, or Start never shows after a clean
  // upload (and the UAVs dock at picks made on other zones).
  it('an upload that regenerates the zones sends and keeps no stale dock picks', async () => {
    const { hook, api } = harness({
      uploadMissions: vi.fn().mockResolvedValue({ status: 'complete', results: [{ sys_id: 5 }] }),
      checkPlanCamera: vi.fn().mockResolvedValue(OK),
      regenerate: { altitude_m: 120, zones: [{ zone_index: 0, track: TRACK }] },
    });
    await act(() => hook.result.current.upload.handleUpload());
    expect(api.uploadMissions).toHaveBeenCalledOnce();
    // No dock to check: the picks did not reach the request.
    expect(api.checkPlanCamera).not.toHaveBeenCalled();
    expect(hook.result.current.picks.simDockWps).toEqual({});
    expect(hook.result.current.snap.planSynced).toBe(true);
  });

  // Codex review, round 4: a plan with no hub assigned (a downloaded legacy
  // mission) gets the hub the upload resolves; the editor must show it.
  it('an upload that assigns the hubs itself shows them and is on the UAVs', async () => {
    const { hook, api } = harness({
      uploadMissions: vi.fn().mockResolvedValue(COMPLETE),
      checkPlanCamera: vi.fn().mockResolvedValue(OK),
      editor: { ...EDITOR, deliveryHubAssignments: [] },
    });
    await act(() => hook.result.current.upload.handleUpload());
    expect(api.uploadMissions.mock.calls[0][0][0].default_delivery_hub).toMatchObject({ lat: HUBS[0].lat });
    expect(hook.result.current.picks.deliveryHubAssignments).toEqual([0, 0]);
    expect(hook.result.current.snap.planSynced).toBe(true);
  });

  // Case 4: an assigned home hub moved in Settings after the upload. The UAVs
  // return to where it was.
  it('moving an assigned home hub takes the plan off the UAVs', async () => {
    const { hook } = harness({
      uploadMissions: vi.fn().mockResolvedValue(COMPLETE),
      checkPlanCamera: vi.fn().mockResolvedValue(OK),
    });
    await act(() => hook.result.current.upload.handleUpload());
    expect(hook.result.current.snap.planSynced).toBe(true);
    hook.rerender({ ...EDITOR, deliveryHubs: [{ ...HUBS[0], lat: 0.015 }] });
    expect(hook.result.current.snap.planSynced).toBe(false);
    // An unassigned hub is not where any UAV returns.
    hook.rerender({ ...EDITOR, deliveryHubs: [...HUBS, { name: 'Hub 2', lat: 0.5, lon: 0.5, type: 'other' }] });
    expect(hook.result.current.snap.planSynced).toBe(true);
  });
});

// Case 3: the operator switched the fence on while a mission download's
// post-processing (the fence read, the parameter read) was still running.
describe('download', () => {
  const MISSION = (sysId) => ({
    sys_id: sysId, mission_count: 2, altitude_m: 100,
    waypoints: [{ lat: 40.1, lon: 44.1, alt: 100 }, { lat: 40.2, lon: 44.2, alt: 100 }],
    items: [], _diagnostic_request_id: `r${sysId}`,
  });

  beforeEach(() => {
    vi.stubGlobal('fetch', vi.fn().mockResolvedValue({ ok: false, json: async () => null }));
  });
  afterEach(() => { vi.unstubAllGlobals(); });

  function harness(vehicles, {
    fenceRead = deferred(),
    downloadMission = (sysId) => Promise.resolve(MISSION(sysId)),
  } = {}) {
    const downloadFence = vi.fn(() => fenceRead.promise);
    const { wrapper } = confirmStub(false);
    const ref = {};
    renderHook(() => {
      const mission = useMissionState();
      const snap = usePlanSnapshot({
        ...SNAPSHOT_DEPS(), ...mission, deliveryHubs: [],
      });
      const connection = useVehicleConnection({
        mission,
        vehicleList: vehicles,
        removeVehicle: vi.fn(),
        api: {
          downloadMission: vi.fn(downloadMission),
          downloadFence,
          disconnectVehicle: vi.fn(),
          connectVehicle: vi.fn().mockResolvedValue({ status: 'connected' }),
          discoverVehicles: vi.fn(),
        },
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        derivePlanPolygon: vi.fn(),
        onPlanSynced: snap.onPlanSynced,
        isPlanSynced: snap.isPlanSynced,
        forgetPlanSync: snap.forgetPlanSync,
        beginPlanDownload: snap.beginPlanDownload,
        notePlanPublished: snap.notePlanPublished,
        settings: {},
        setNotification: vi.fn(),
      });
      ref.current = { mission, snap, connection };
    }, { wrapper });
    return { ref, fenceRead, downloadFence };
  }

  const PATHS = {
    'Download plan': {
      vehicles: [{ sys_id: 1, name: 'UAV1' }],
      start: (connection) => connection.handleDownloadPlan(),
    },
    'a UAV connecting': {
      vehicles: [],
      start: (connection) => connection.handleConnect('udp:x', 1, 'UAV1'),
    },
  };

  async function download(path, editDuringPostProcessing) {
    const { vehicles, start } = PATHS[path];
    const h = harness(vehicles);
    await settle();
    let done;
    act(() => { done = start(h.ref.current.connection); });
    await settle();
    expect(h.downloadFence).toHaveBeenCalled();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(1);
    if (editDuringPostProcessing) act(() => { h.ref.current.mission.authorFenceIntent(true); });
    await act(async () => { h.fenceRead.resolve(null); await done; });
    await settle();
    return h.ref.current;
  }

  it.each(Object.keys(PATHS))('%s, unedited, is the plan on the UAVs', async (path) => {
    const { snap } = await download(path, false);
    expect(snap.planSynced).toBe(true);
  });

  it.each(Object.keys(PATHS))('%s: the fence switched on during post-processing is not on the UAVs', async (path) => {
    const { snap, mission } = await download(path, true);
    expect(mission.fenceEnabled).toBe(true);
    expect(snap.planSynced).toBe(false);
  });
});

// Codex review: a download publishes one UAV's mission at a time. An edit made
// after the first UAV's baseline landed was taken in when the next UAV's
// mission published.
describe('a download that publishes UAV by UAV', () => {
  const MISSION = (sysId) => ({
    sys_id: sysId, mission_count: 2, altitude_m: 100,
    waypoints: [{ lat: 40 + sysId / 10, lon: 44.1, alt: 100 }, { lat: 40.2, lon: 44 + sysId / 10, alt: 100 }],
    items: [], _diagnostic_request_id: `r${sysId}`,
  });
  const FLEET = [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }];

  beforeEach(() => {
    vi.stubGlobal('fetch', vi.fn().mockResolvedValue({ ok: false, json: async () => null }));
  });
  afterEach(() => { vi.unstubAllGlobals(); });

  const PATHS = {
    'Download plan': {
      vehicles: [],
      // The fleet connects, then the operator downloads (the restore stops).
      start: (h) => h.ref.current.connection.handleDownloadPlan(),
      connect: (h) => h.hook.rerender({ vehicles: FLEET }),
    },
    'the page-load restore': { vehicles: FLEET, start: () => Promise.resolve() },
  };

  async function download(path, editBetweenUavs) {
    const second = deferred();
    const { wrapper } = confirmStub(false);
    const ref = {};
    const hook = renderHook(({ vehicles }) => {
      const mission = useMissionState();
      const snap = usePlanSnapshot({ ...SNAPSHOT_DEPS(), ...mission, deliveryHubs: [] });
      const connection = useVehicleConnection({
        mission,
        vehicleList: vehicles,
        removeVehicle: vi.fn(),
        api: {
          downloadMission: vi.fn((sysId) => (sysId === 2 ? second.promise : Promise.resolve(MISSION(sysId)))),
          downloadFence: vi.fn().mockResolvedValue(null),
          disconnectVehicle: vi.fn(),
          connectVehicle: vi.fn(),
          discoverVehicles: vi.fn(),
        },
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        derivePlanPolygon: vi.fn(),
        onPlanSynced: snap.onPlanSynced,
        isPlanSynced: snap.isPlanSynced,
        forgetPlanSync: snap.forgetPlanSync,
        beginPlanDownload: snap.beginPlanDownload,
        notePlanPublished: snap.notePlanPublished,
        settings: {},
        setNotification: vi.fn(),
      });
      ref.current = { mission, snap, connection };
    }, { wrapper, initialProps: { vehicles: PATHS[path].vehicles } });
    await settle();
    PATHS[path].connect?.({ hook, ref });
    let done;
    act(() => { done = PATHS[path].start({ hook, ref }); });
    await settle();
    await settle();
    expect(ref.current.mission.plan?.zones?.length).toBe(1);
    expect(ref.current.snap.planSynced).toBe(true);
    if (editBetweenUavs) {
      act(() => { ref.current.mission.authorFenceIntent(true); });
      expect(ref.current.snap.planSynced).toBe(false);
    }
    await act(async () => { second.resolve(MISSION(2)); await done; });
    await settle();
    await settle();
    expect(ref.current.mission.plan?.zones?.length).toBe(2);
    return ref.current;
  }

  it.each(Object.keys(PATHS))('%s, unedited, is the plan on the UAVs', async (path) => {
    const { snap } = await download(path, false);
    expect(snap.planSynced).toBe(true);
  });

  it.each(Object.keys(PATHS))('%s: the fence switched on between two UAVs is not on the UAVs', async (path) => {
    const { snap, mission } = await download(path, true);
    expect(mission.fenceEnabled).toBe(true);
    expect(snap.planSynced).toBe(false);
  });
});

// Codex review, round 2: edits before a download's first mission, a merge
// into an edited plan, and a UAV leaving a page-load restore.
describe('a download and edits around it', () => {
  const HUB = { name: 'Hub 1', lat: 40.01, lon: 44.01, type: 'other' };
  const MISSION = (sysId) => ({
    sys_id: sysId, mission_count: 2, altitude_m: 100,
    waypoints: [{ lat: 40 + sysId / 10, lon: 44.1, alt: 100 }, { lat: 40.2, lon: 44 + sysId / 10, alt: 100 }],
    items: [], default_delivery_hub: HUB, _diagnostic_request_id: `r${sysId}`,
  });
  const FLEET = [{ sys_id: 1, name: 'UAV1' }, { sys_id: 2, name: 'UAV2' }];

  beforeEach(() => {
    vi.stubGlobal('fetch', vi.fn().mockResolvedValue({ ok: false, json: async () => null }));
  });
  afterEach(() => { vi.unstubAllGlobals(); });

  function harness({ vehicles = [], hubs = [HUB], downloadMission, downloadFence } = {}) {
    const { wrapper } = confirmStub(false);
    const ref = {};
    const hook = renderHook(({ vehicles: list }) => {
      const mission = useMissionState();
      // Settings' home hubs (useSettings saves optimistically).
      const [deliveryHubs, setDeliveryHubs] = useState(hubs);
      // What usePlanPersistence.derivePlanPolygon writes besides the area: the
      // UAV count, and the hubs the missions carry, added to Settings.
      const derivePlanPolygon = (zones, _pattern, _polygon, _backbone, _launch, missionHubs) => {
        mission.setUavCount(zones.length);
        if (!missionHubs) return;
        const restored = buildDeliveryHubsFromDownload(missionHubs, deliveryHubs);
        if (restored.deliveryHubs.length === 0) return;
        setDeliveryHubs(restored.deliveryHubs);
        mission.setDeliveryHubAssignments(restored.assignments);
      };
      const snap = usePlanSnapshot({ ...SNAPSHOT_DEPS(), ...mission, deliveryHubs });
      const connection = useVehicleConnection({
        mission,
        vehicleList: list,
        removeVehicle: vi.fn(),
        api: {
          downloadMission: vi.fn(downloadMission ?? ((sysId) => Promise.resolve(MISSION(sysId)))),
          downloadFence: vi.fn(downloadFence ?? (() => Promise.resolve(null))),
          disconnectVehicle: vi.fn((sysId) => Promise.resolve({ removed: [sysId] })),
          connectVehicle: vi.fn().mockResolvedValue({ status: 'connected' }),
          discoverVehicles: vi.fn(),
        },
        telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
        derivePlanPolygon,
        onPlanSynced: snap.onPlanSynced,
        isPlanSynced: snap.isPlanSynced,
        forgetPlanSync: snap.forgetPlanSync,
        beginPlanDownload: snap.beginPlanDownload,
        notePlanPublished: snap.notePlanPublished,
        settings: { default_delivery_hubs: deliveryHubs },
        setNotification: vi.fn(),
      });
      ref.current = { mission, snap, connection, setDeliveryHubs };
    }, { wrapper, initialProps: { vehicles } });
    return { ref, hook };
  }

  // Finding 1: the fence switched on before any mission answered.
  it.each([
    ['Download plan', []],
    ['the page-load restore', FLEET],
  ])('%s: an edit made before the first mission arrives is not on the UAVs', async (path, vehicles) => {
    const missions = deferred();
    const h = harness({
      vehicles,
      downloadMission: (sysId) => missions.promise.then(() => MISSION(sysId)),
    });
    await settle();
    let done = Promise.resolve();
    if (path === 'Download plan') {
      h.hook.rerender({ vehicles: FLEET });
      act(() => { done = h.ref.current.connection.handleDownloadPlan(); });
    }
    await settle();
    expect(h.ref.current.mission.plan).toBeNull();
    act(() => { h.ref.current.mission.authorFenceIntent(true); });
    await act(async () => { missions.resolve(); await done; });
    await settle();
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    expect(h.ref.current.snap.planSynced).toBe(false);
  });

  // Finding 2: a single connect merges into a plan whose assigned hub the
  // operator moved; the merge rebuilds the other zones' hubs from Settings.
  it('a UAV connecting into an edited plan does not make the edit the UAVs\'', async () => {
    const h = harness();
    await settle();
    await act(async () => { await h.ref.current.connection.handleConnect('udp:x', 1, 'UAV1'); });
    await settle();
    expect(h.ref.current.mission.deliveryHubAssignments).toEqual([0]);
    expect(h.ref.current.snap.planSynced).toBe(true);
    act(() => { h.ref.current.setDeliveryHubs([{ ...HUB, lat: 41.01 }]); });
    expect(h.ref.current.snap.planSynced).toBe(false);
    await act(async () => { await h.ref.current.connection.handleConnect('udp:x', 2, 'UAV2'); });
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    expect(h.ref.current.snap.planSynced).toBe(false);
  });

  // Round 3: two UAVs connect onto an empty map; the second answers after the
  // first one's plan became the UAVs'.
  it('two UAVs connecting at once onto an empty map are the plan on the UAVs', async () => {
    const answers = { 1: deferred(), 2: deferred() };
    const h = harness({ downloadMission: (sysId) => answers[sysId].promise });
    await settle();
    let first;
    let second;
    act(() => {
      first = h.ref.current.connection.handleConnect('udp:x', 1, 'UAV1');
      second = h.ref.current.connection.handleConnect('udp:x', 2, 'UAV2');
    });
    await settle();
    await act(async () => { answers[1].resolve(MISSION(1)); await first; });
    await settle();
    expect(h.ref.current.snap.planSynced).toBe(true);
    await act(async () => { answers[2].resolve(MISSION(2)); await second; });
    await settle();
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    expect(h.ref.current.snap.planSynced).toBe(true);
  });

  // Round 4: the second UAV's download finishes before the first one's.
  it('two UAVs whose downloads finish out of order are the plan on the UAVs', async () => {
    const firstFence = deferred();
    const h = harness({ downloadFence: (sysId) => (sysId === 1 ? firstFence.promise : Promise.resolve(null)) });
    await settle();
    let first;
    act(() => { first = h.ref.current.connection.handleConnect('udp:x', 1, 'UAV1'); });
    await settle();
    await act(async () => { await h.ref.current.connection.handleConnect('udp:x', 2, 'UAV2'); });
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    expect(h.ref.current.snap.planSynced).toBe(true);
    await act(async () => { firstFence.resolve(null); await first; });
    await settle();
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    expect(h.ref.current.snap.planSynced).toBe(true);
  });

  it('a UAV connecting into the UAVs\' plan keeps it the UAVs\'', async () => {
    const h = harness();
    await settle();
    await act(async () => { await h.ref.current.connection.handleConnect('udp:x', 1, 'UAV1'); });
    await settle();
    await act(async () => { await h.ref.current.connection.handleConnect('udp:x', 2, 'UAV2'); });
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    expect(h.ref.current.snap.planSynced).toBe(true);
  });

  // Finding 3: a UAV leaving while the page-load restore still reads the fences.
  it('a UAV leaving during the page-load restore leaves the rest on the UAVs', async () => {
    const fences = deferred();
    const h = harness({ vehicles: FLEET, downloadFence: () => fences.promise });
    await settle();
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(2);
    await act(async () => { await h.ref.current.connection.handleDisconnect(2); });
    h.hook.rerender({ vehicles: [FLEET[0]] });
    await act(async () => { fences.resolve(null); });
    await settle();
    await settle();
    expect(h.ref.current.mission.plan?.zones?.length).toBe(1);
    expect(h.ref.current.mission.uavCount).toBe(1);
    expect(h.ref.current.snap.planSynced).toBe(true);
  });
});

// Codex review, round 3: a mission too short to derive an area from still
// restores its home hub, or the baseline keeps a hub the operator moved.
describe('derivePlanPolygon without an area', () => {
  it('restores the mission\'s hub', () => {
    const HUB = { lat: 40.01, lon: 44.01, type: 'other' };
    const handleSaveSettings = vi.fn();
    const { result } = renderHook(() => {
      const mission = useMissionState();
      const persistence = usePlanPersistence({
        ...mission,
        drawing: { loadVertices: vi.fn() },
        undoRef: { current: [] },
        suppressRegenRef: { current: false },
        localAnalyze: vi.fn(),
        handleSaveSettings,
        settings: { default_delivery_hubs: [{ name: 'Moved', lat: 41.01, lon: 44.01, type: 'other' }] },
        plannerReady: false,
        authorFenceIntent: mission.authorFenceIntent,
        clearFenceIntent: mission.clearFenceIntent,
        resetFenceObservations: mission.resetFenceObservations,
      });
      return { mission, persistence };
    });
    const zones = [{ sys_id: 1, track: [{ lat: 40, lon: 44 }, { lat: 40.1, lon: 44.1 }] }];
    act(() => { result.current.persistence.derivePlanPolygon(zones, 'distributed', null, null, null, [HUB]); });
    expect(result.current.mission.polygon).toEqual([]);
    expect(handleSaveSettings).toHaveBeenCalledWith({ default_delivery_hubs: [
      { name: 'Moved', lat: 41.01, lon: 44.01, type: 'other' },
      expect.objectContaining({ lat: 40.01, lon: 44.01 }),
    ] });
    expect(result.current.mission.deliveryHubAssignments).toEqual([1]);
  });
});
