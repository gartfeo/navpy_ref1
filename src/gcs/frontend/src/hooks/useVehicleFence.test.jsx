/**
 * Owner report: "if the plan already has fence ... in panel it shows off. also
 * enabling disabling it not cleaning".
 *
 * 1. A fleet that already holds an active fence reads ON on a plan with no
 *    fence decision, and an untouched re-upload still sends nothing.
 * 2. Switching the fence off leaves nothing behind: no ring or keep-outs on
 *    the map (not even the ones the vehicles still store) and no hand-edited
 *    shape that comes back on the next switch-on.
 */
import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import useVehicleConnection from './useVehicleConnection';
import useVehicleFence from './useVehicleFence';
import usePlanningOrchestrator from './usePlanningOrchestrator';
import { fenceVertexDelete } from '../utils/fenceEdit';
import {
  fenceMapDisplay, fenceUploadPayload, summarizeFenceObservations,
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

// App's fence wiring, minus the map. The plan's ring is the custom ring: the
// auto ring needs a drawn search zone, which these tests do not draw.
function renderFence(downloadFence, { sysIds = [1], polygonLength = 3 } = {}) {
  const { result } = renderHook(() => {
    const mission = useMissionState();
    const connection = useVehicleConnection({
      mission,
      vehicleList: sysIds.map((sys_id) => ({ sys_id })),
      removeVehicle: vi.fn(),
      api: { downloadFence, downloadMission: vi.fn(), disconnectVehicle: vi.fn() },
      telemetryStoreRef: { current: {} },
      derivePlanPolygon: vi.fn(),
      settings: {},
    });
    const observedFence = summarizeFenceObservations(mission.fenceObservations, sysIds);
    const fence = useVehicleFence({ mission, observedFence, polygonLength });
    const polygon = mission.fenceEnabled ? mission.fenceCustomVertices : null;
    const request = fenceUploadPayload({
      enabled: mission.fenceEnabled, intent: mission.fenceIntent,
      acknowledged: mission.fenceAcknowledged, vertices: polygon,
      exclusions: mission.exclusionPolygons,
    });
    const map = fenceMapDisplay({
      enabled: mission.fenceEnabled, polygon, vehicleFence: fence.vehicleFence,
    });
    return { mission, connection, fence, request, map };
  });
  return result;
}

const observeAll = (result, sysIds = [1]) => act(
  () => result.current.connection.observeFleetFence(sysIds.map((sys_id) => ({ sys_id }))),
);

describe("bug 1: a fleet that already holds a fence reads ON", () => {
  it("shows the switch on with the fleet\'s ring, sending nothing", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse()));
    await observeAll(result);

    const { mission, fence, request, map } = result.current;
    expect(fence.switchOn).toBe(true);
    expect(map).toEqual({ show: true, polygon: ring(), source: 'vehicles' });
    expect(request.payload).toBeNull();
    // Display only: the plan itself made no fence decision, so a saved plan
    // carries no fence block and loading it requests nothing.
    expect(mission.fenceEnabled).toBe(false);
    expect(mission.fenceTouched).toBe(false);
    expect(mission.fenceIntent).toBeNull();
    expect(mission.fenceCustomVertices).toBeNull();
  });

  it("treats an auto-enable fence as on too", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({
      params: { enable: 0, autoenable: 1, type: 4, action: 1 },
    })));
    await observeAll(result);
    expect(result.current.fence.switchOn).toBe(true);
    expect(result.current.request.payload).toBeNull();
  });

  it("switching it off is an explicit disable", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse()));
    await observeAll(result);
    act(() => result.current.fence.handleToggleFence());
    expect(result.current.fence.switchOn).toBe(false);
    expect(result.current.request.payload).toMatchObject({ enabled: false });
  });

  it("editing the fleet\'s ring makes it the plan\'s fence, keeping its keep-outs", async () => {
    const keepOut = ring(0.001);
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({ exclusions: [keepOut] })));
    await observeAll(result);
    act(() => result.current.fence.editFenceVertices(ring(0.002)));

    const { mission, request } = result.current;
    expect(mission.fenceEnabled).toBe(true);
    expect(mission.fenceTouched).toBe(true);
    expect(request.payload).toMatchObject({ enabled: true, vertices: ring(0.002), exclusions: [keepOut] });
  });

  it("a fleet answer arriving after an uploaded edit never replaces the edit", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse()));
    await observeAll(result);
    act(() => result.current.fence.editFenceVertices(ring(0.003)));
    const generation = result.current.mission.fenceIntent.generation;
    act(() => result.current.mission.resolveFenceIntent(
      generation, true, result.current.request.signature,
    ));
    await observeAll(result); // still reports the old ring

    expect(result.current.map).toMatchObject({ polygon: ring(0.003), source: 'planner' });
    expect(result.current.request.payload).toBeNull();
  });

  it("does not offer editing when the fleet\'s keep-outs disagree", async () => {
    const byId = { 1: fenceResponse({ exclusions: [ring(0.1)] }), 2: fenceResponse() };
    const result = renderFence(
      vi.fn().mockImplementation((id) => Promise.resolve(byId[id])), { sysIds: [1, 2] },
    );
    await observeAll(result, [1, 2]);
    expect(result.current.fence.switchOn).toBe(true);
    expect(result.current.fence.editable).toBe(false);
  });

  it("a vertex delete the edit refuses (ring already at 3) takes nothing over", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse()));
    await observeAll(result);
    act(() => result.current.fence.editFenceVertices(
      (prev) => fenceVertexDelete(prev, result.current.map.polygon, 0),
    ));

    const { mission, fence, map, request } = result.current;
    expect(mission.fenceIntent).toBeNull();
    expect(mission.fenceCustomVertices).toBeNull();
    expect(fence.vehicleFence).not.toBeNull();
    expect(map).toEqual({ show: true, polygon: ring(), source: 'vehicles' });
    expect(request.payload).toBeNull();
  });

  it("does not offer editing without a search zone", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse()), { polygonLength: 0 });
    await observeAll(result);
    expect(result.current.fence.editable).toBe(false);
  });

  it("leaves a fence that is off on the vehicles off, and draws nothing", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({
      params: { enable: 0, autoenable: 0, type: 4, action: 1 },
    })));
    await observeAll(result);
    expect(result.current.fence.switchOn).toBe(false);
    expect(result.current.map).toMatchObject({ show: false, polygon: null });
  });
});

describe("keep-outs are part of the fleet's fence", () => {
  it("a keep-out drawn while the fleet's fence shows takes the fence over and is sent", async () => {
    const theirs = ring(0.1);
    const mine = ring(0.2);
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({ exclusions: [theirs] })));
    await observeAll(result);
    act(() => result.current.fence.editExclusionPolygons((prev) => [...prev, mine]));

    expect(result.current.request.payload).toMatchObject({
      enabled: true, vertices: ring(), exclusions: [theirs, mine],
    });
  });

  it("a ring edit keeps the fleet's keep-outs next to the plan's own", async () => {
    const theirs = ring(0.1);
    const mine = ring(0.2);
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({ exclusions: [theirs] })));
    act(() => result.current.mission.setExclusionPolygons([mine]));
    await observeAll(result);
    expect(result.current.fence.displayExclusionPolygons).toEqual([theirs, mine]);
    act(() => result.current.fence.editFenceVertices(ring(0.002)));

    expect(result.current.request.payload.exclusions).toEqual([theirs, mine]);
  });

  it("refuses ring, reset and keep-out edits when the fleet's keep-outs disagree", async () => {
    const byId = { 1: fenceResponse({ exclusions: [ring(0.1)] }), 2: fenceResponse() };
    const result = renderFence(
      vi.fn().mockImplementation((id) => Promise.resolve(byId[id])), { sysIds: [1, 2] },
    );
    await observeAll(result, [1, 2]);
    act(() => result.current.fence.editFenceVertices(null)); // "Reset to auto"
    act(() => result.current.fence.editExclusionPolygons([ring(0.2)]));

    expect(result.current.mission.fenceIntent).toBeNull();
    expect(result.current.mission.exclusionPolygons).toEqual([]);
    expect(result.current.request.payload).toBeNull();
    expect(result.current.fence.keepOutDrawable).toBe(false);
  });

  it("draws no keep-out once off, even after a take-over copied them into the plan", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({ exclusions: [ring(0.1)] })));
    await observeAll(result);
    act(() => result.current.fence.editFenceVertices(ring(0.002)));
    expect(result.current.mission.exclusionPolygons).toHaveLength(1);
    act(() => result.current.fence.handleToggleFence());

    expect(result.current.fence.displayExclusionPolygons).toEqual([]);
    expect(result.current.fence.keepOutDrawable).toBe(false);
  });
});

describe("keep-out list indexes", () => {
  it("removing the first listed keep-out removes the fleet's, the one listed first", async () => {
    const theirs = ring(0.1);
    const mine = ring(0.2);
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({ exclusions: [theirs] })));
    act(() => result.current.mission.setExclusionPolygons([mine]));
    await observeAll(result);
    expect(result.current.fence.editableExclusions).toEqual([theirs, mine]);
    act(() => result.current.fence.editExclusionPolygons((prev) => prev.filter((_, i) => i !== 0)));

    expect(result.current.request.payload.exclusions).toEqual([mine]);
  });
});

describe("switching off ends keep-out drawing", () => {
  it("leaves placement mode and drops the draft once keep-outs stop being drawable", () => {
    const { result, rerender } = renderHook(({ drawable }) => {
      const mission = useMissionState();
      return usePlanningOrchestrator({
        mission, settings: {}, handleSaveSettings: () => {}, settingsVersion: 0, plannerReady: false,
        exclusionEditing: {
          exclusionPolygons: mission.exclusionPolygons,
          setExclusionPolygons: mission.setExclusionPolygons,
          drawable,
        },
      });
    }, { initialProps: { drawable: true } });
    act(() => result.current.toggleExclusionPlacement());
    act(() => result.current.handlePlaceExclusionVertex({ lat: 32, lon: 34 }));
    expect(result.current.placingExclusion).toBe(true);
    expect(result.current.draftExclusion).toHaveLength(1);

    rerender({ drawable: false });
    expect(result.current.placingExclusion).toBe(false);
    expect(result.current.draftExclusion).toEqual([]);
    rerender({ drawable: true });
    expect(result.current.placingExclusion).toBe(false);
  });
});

describe("bug 2: switching the fence off leaves nothing behind", () => {
  it("draws no ring or keep-outs once off, though the vehicles still store them", async () => {
    const result = renderFence(vi.fn().mockResolvedValue(fenceResponse({ exclusions: [ring(0.1)] })));
    await observeAll(result);
    expect(result.current.fence.displayExclusionPolygons).toHaveLength(1);
    act(() => result.current.fence.handleToggleFence());

    expect(result.current.map).toEqual({ show: false, polygon: null, source: 'none' });
    expect(result.current.fence.displayExclusionPolygons).toEqual([]);
  });

  it("discards a hand-edited shape, so switching on again starts from auto", async () => {
    const result = renderFence(vi.fn());
    act(() => result.current.fence.handleToggleFence());
    act(() => result.current.fence.editFenceVertices(ring(0.004)));
    act(() => result.current.fence.handleToggleFence());
    expect(result.current.mission.fenceCustomVertices).toBeNull();
    act(() => result.current.fence.handleToggleFence());
    expect(result.current.mission.fenceEnabled).toBe(true);
    expect(result.current.mission.fenceCustomVertices).toBeNull();
  });
});
