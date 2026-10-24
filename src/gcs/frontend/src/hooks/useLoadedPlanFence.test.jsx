/**
 * A plan file with no fence block, loaded in demo mode (todo 91).
 *
 * The defect: the load started a new fence observation round with nothing
 * asked, so the demo default authored "fence on" before anyone read the
 * fleet, and the next upload sent a fence nobody touched. Owner ruling
 * (2026-10-23): the demo default waits for the fleet read. With vehicles
 * connected the load re-reads their fence, the switch shows what they hold and
 * nothing is sent; with none connected the default applies as for any new plan.
 *
 * Runs the real mission state, orchestrator, file load, vehicle connection and
 * App's demo-default and upload-payload decisions.
 */
import { act, renderHook } from '@testing-library/react';
import { useEffect, useMemo } from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import usePlanningOrchestrator from './usePlanningOrchestrator';
import useVehicleConnection from './useVehicleConnection';
import useVehicleFence from './useVehicleFence';
import {
  fenceUploadPayload, shouldAutoEnableDemoFence, summarizeFenceObservations,
} from '../utils/fenceIntent';

afterEach(() => {
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

const AREA = [
  { lat: 40.32, lon: 44.43 }, { lat: 40.32, lon: 44.456 },
  { lat: 40.3, lon: 44.456 }, { lat: 40.3, lon: 44.43 },
];
// The demo plan file shape: no fence block.
const PLAN_FILE = {
  polygon: AREA,
  search_pattern: 'distributed',
  set_launch_points: [{ lat: 40.31, lon: 44.47 }],
};
const RING = [
  { lat: 40.33, lon: 44.42 }, { lat: 40.33, lon: 44.47 },
  { lat: 40.29, lon: 44.47 }, { lat: 40.29, lon: 44.42 },
];
const fenceReply = (enable) => ({
  vertices: RING, exclusions: [], total_items: 4,
  params: { enable, autoenable: 0, type: 4, action: 1 },
  params_readback: 'ok',
});

function stubFileIo() {
  const io = { contents: null, input: null };
  const create = document.createElement.bind(document);
  vi.spyOn(document, 'createElement').mockImplementation((tag, ...args) => {
    const element = create(tag, ...args);
    if (tag === 'input') io.input = element;
    if (tag === 'input' || tag === 'a') element.click = () => {};
    return element;
  });
  vi.stubGlobal('FileReader', class {
    readAsText() { this.onload({ target: { result: io.contents } }); }
  });
  return io;
}

function renderApp({ vehicles }) {
  const io = stubFileIo();
  // Each fence read waits until the test answers it.
  const replies = [];
  const api = {
    downloadMission: vi.fn(),
    downloadFence: vi.fn((sysId) => new Promise((resolve) => { replies.push({ sysId, resolve }); })),
    disconnectVehicle: vi.fn(),
  };
  const settings = { default_delivery_hubs: [] };
  const { result, rerender } = renderHook(({ vehicles }) => {
    const mission = useMissionState();
    const planning = usePlanningOrchestrator({
      mission, settings, handleSaveSettings: vi.fn(), settingsVersion: 1, plannerReady: true,
    });
    // App's fleet summary, demo default and upload payload, unchanged and in
    // App's order: the demo default runs before useVehicleConnection's effects.
    const observedFence = useMemo(
      () => summarizeFenceObservations(mission.fenceObservations, vehicles.map((v) => v.sys_id)),
      [mission.fenceObservations, vehicles],
    );
    const vehicleFence = useVehicleFence({
      mission, observedFence, polygonLength: mission.polygon.length,
    });
    useEffect(() => {
      if (shouldAutoEnableDemoFence({
        simMode: true, polygonLength: mission.polygon.length,
        fenceTouched: mission.fenceTouched, fenceEnabled: mission.fenceEnabled,
        observations: observedFence,
      })) {
        mission.authorFenceIntent(true, { touched: false });
      }
    }, [mission.polygon.length, mission.fenceTouched, mission.fenceEnabled,
      mission.authorFenceIntent, observedFence]);
    useVehicleConnection({
      mission,
      vehicleList: vehicles,
      removeVehicle: vi.fn(),
      api,
      telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
      derivePlanPolygon: planning.derivePlanPolygon,
      onPlanSynced: planning.onPlanSynced,
      settings,
    });
    const upload = fenceUploadPayload({
      enabled: mission.fenceEnabled, intent: mission.fenceIntent,
      acknowledged: mission.fenceAcknowledged,
      vertices: mission.fenceEnabled ? AREA : null, exclusions: [],
    });
    return { mission, planning, vehicleFence, upload };
  }, { initialProps: { vehicles } });
  act(() => result.current.mission.goToPlanning());
  return {
    result,
    api,
    connect(next) {
      act(() => rerender({ vehicles: next }));
    },
    load() {
      io.contents = JSON.stringify(PLAN_FILE);
      act(() => {
        result.current.planning.handleLoadPolygon();
        io.input.onchange({ target: { files: [{}] } });
      });
    },
    async answerFences(enable) {
      const pending = replies.splice(0);
      await act(async () => {
        pending.forEach(({ resolve }) => resolve(fenceReply(enable)));
      });
      return pending.map(({ sysId }) => sysId);
    },
  };
}

const VEHICLES = [1, 2, 3].map((sys_id) => ({ sys_id, name: `UAV${sys_id}` }));

describe('loading a plan file with no fence block in demo mode', () => {
  it('re-reads the connected fleet and authors nothing before it answers', async () => {
    const h = renderApp({ vehicles: VEHICLES });
    h.load();
    expect(h.result.current.mission.fenceIntent).toBeNull();
    expect(h.result.current.mission.fenceEnabled).toBe(false);
    expect(h.api.downloadFence.mock.calls.map(([id]) => id).sort()).toEqual([1, 2, 3]);
  });

  it('sends no fence after the fleet answers that it holds none', async () => {
    const h = renderApp({ vehicles: VEHICLES });
    h.load();
    await h.answerFences(0);
    expect(h.result.current.mission.fenceIntent).toBeNull();
    expect(h.result.current.vehicleFence.switchOn).toBe(false);
    expect(h.result.current.upload.payload).toBeNull();
  });

  it('shows the fence the fleet holds, still without sending it', async () => {
    const h = renderApp({ vehicles: VEHICLES });
    h.load();
    await h.answerFences(1);
    expect(h.result.current.vehicleFence.switchOn).toBe(true);
    expect(h.result.current.vehicleFence.vehicleFence.ring).toEqual(RING);
    expect(h.result.current.upload.payload).toBeNull();
  });

  it("reads a vehicle that connects after the load, and shows the fleet's fence", async () => {
    const h = renderApp({ vehicles: VEHICLES.slice(0, 2) });
    h.load();
    expect(await h.answerFences(1)).toEqual([1, 2]);
    h.connect(VEHICLES);
    expect(await h.answerFences(1)).toEqual([3]);
    expect(h.result.current.vehicleFence.switchOn).toBe(true);
    expect(h.result.current.vehicleFence.vehicleFence.ring).toEqual(RING);
    expect(h.result.current.upload.payload).toBeNull();
  });

  it('authors nothing when the roster is replaced by a vehicle not yet read', async () => {
    const h = renderApp({ vehicles: VEHICLES.slice(0, 1) });
    h.load();
    expect(await h.answerFences(0)).toEqual([1]);
    h.connect(VEHICLES.slice(1, 2));
    expect(h.result.current.mission.fenceIntent).toBeNull();
    expect(h.result.current.mission.fenceEnabled).toBe(false);
    expect(await h.answerFences(0)).toEqual([2]);
    expect(h.result.current.upload.payload).toBeNull();
  });

  it('reads a vehicle again when it reconnects after the load', async () => {
    const h = renderApp({ vehicles: VEHICLES.slice(0, 2) });
    h.load();
    expect(await h.answerFences(0)).toEqual([1, 2]);
    h.connect(VEHICLES.slice(1, 2));
    h.connect(VEHICLES.slice(0, 2));
    // UAV 1 came back with its fence enabled.
    expect(await h.answerFences(1)).toEqual([1]);
    expect(h.result.current.mission.fenceObservations.bySysId[1].mode).toBe('on');
  });

  it('applies the demo default when no vehicle is connected', () => {
    const h = renderApp({ vehicles: [] });
    h.load();
    expect(h.api.downloadFence).not.toHaveBeenCalled();
    expect(h.result.current.mission.fenceIntent).toMatchObject({ enabled: true });
    expect(h.result.current.mission.fenceTouched).toBe(false);
  });
});
