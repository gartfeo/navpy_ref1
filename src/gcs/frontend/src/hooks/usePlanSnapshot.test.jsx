/**
 * Leaving plan mode: a plan that is neither on the UAVs (uploaded or
 * downloaded) nor in the file last loaded or saved asks whether to save it
 * first; a plan just loaded and unchanged does not. Leaving with a plan that
 * is not the one on the UAVs returns the map to the UAVs' plan. `planSynced`
 * (the monitor's start gate) holds exactly while the editor shows the UAVs' plan.
 */
import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import usePlanSnapshot from './usePlanSnapshot';
import { confirmStub } from '../test/confirmStub';
import { tagPlanForRun } from '../utils/planDownloadRun';

const POLYGON = [{ lat: 40, lon: 44 }, { lat: 40.01, lon: 44 }, { lat: 40.01, lon: 44.01 }];
const PLAN = { altitude_m: 100, zones: [{ sys_id: 1, set_index: 0, track: [{ lat: 40, lon: 44 }] }] };

const STATE = {
  plan: PLAN,
  polygon: POLYGON,
  searchPattern: 'distributed',
  analysis: { required_uavs: 1 },
  uavCount: null,
  effectiveUavCount: 1,
  partitionAngleDeg: null,
  routeOffsetM: null,
  setLaunchPoints: [{ lat: 39.99, lon: 44 }],
  setCorridorPointsArr: [[]],
  deliveryHubAssignments: [0],
  manualDeliveryHubEdit: false,
  simDockWps: {},
  detectAfterWps: {},
  fenceTouched: false,
  fenceEnabled: false,
  fenceOffsetM: 50,
  fenceCustomVertices: null,
  exclusionPolygons: [],
};

function harness(answer, state = STATE) {
  const { confirm, wrapper } = confirmStub(answer);
  const setters = {
    setPlan: vi.fn(), setPolygon: vi.fn(), setSearchPattern: vi.fn(), setAnalysis: vi.fn(),
    setUavCount: vi.fn(), setPartitionAngleDeg: vi.fn(), setRouteOffsetM: vi.fn(),
    setSetLaunchPoints: vi.fn(), setSetCorridorPoints: vi.fn(), setDeliveryHubAssignments: vi.fn(),
    setSimDockWps: vi.fn(), setDetectAfterWps: vi.fn(), setFenceCustomVertices: vi.fn(),
    setExclusionPolygons: vi.fn(),
  };
  const goToMonitor = vi.fn();
  const savePlan = vi.fn();
  const fixed = {
    ...setters,
    drawingRef: { current: { loadVertices: vi.fn() } },
    suppressRegenRef: { current: false },
    undoRef: { current: [] },
    goToMonitor,
    savePlanRef: { current: savePlan },
  };
  const hook = renderHook((props) => usePlanSnapshot({ ...fixed, ...props }), {
    wrapper, initialProps: state,
  });
  const exit = () => act(async () => { await hook.result.current.handleExitPlanning(); });
  return { hook, confirm, setters, goToMonitor, savePlan, exit };
}

describe('leaving plan mode', () => {
  it('a new plan asks to save; Don\'t save leaves and keeps it on the map', async () => {
    const h = harness('alt');
    await h.exit();
    expect(h.confirm).toHaveBeenCalledWith('planning.savePrompt.message');
    expect(h.savePlan).not.toHaveBeenCalled();
    expect(h.setters.setPlan).not.toHaveBeenCalled();
    expect(h.goToMonitor).toHaveBeenCalledTimes(1);
  });

  it('Save writes the file, then leaves', async () => {
    const h = harness(true);
    await h.exit();
    expect(h.savePlan).toHaveBeenCalledTimes(1);
    expect(h.goToMonitor).toHaveBeenCalledTimes(1);
  });

  it('Keep editing stays in plan mode', async () => {
    const h = harness(false);
    await h.exit();
    expect(h.goToMonitor).not.toHaveBeenCalled();
  });

  it('a plan just loaded (or saved) and unchanged leaves without a question', async () => {
    const h = harness(false);
    act(() => { h.hook.result.current.markPlanSaved(); });
    await h.exit();
    expect(h.confirm).not.toHaveBeenCalled();
    expect(h.goToMonitor).toHaveBeenCalledTimes(1);
  });

  it('values the GCS derives by itself are no change to a loaded plan', async () => {
    const h = harness(false);
    act(() => { h.hook.result.current.markPlanSaved(); });
    // The analysis landed, the unset count took the area's value, automatic
    // hub assignment ran, the demo fence default switched on, a set slot padded.
    h.hook.rerender({
      ...STATE,
      analysis: { required_uavs: 1, area_km2: 2 },
      uavCount: 1,
      deliveryHubAssignments: [1],
      fenceEnabled: true,
      setLaunchPoints: [...STATE.setLaunchPoints, null],
    });
    await h.exit();
    expect(h.confirm).not.toHaveBeenCalled();
  });

  it('a loaded plan the operator changed asks again', async () => {
    const h = harness(false);
    act(() => { h.hook.result.current.markPlanSaved(); });
    h.hook.rerender({ ...STATE, polygon: [...POLYGON, { lat: 40, lon: 44.02 }] });
    await h.exit();
    expect(h.confirm).toHaveBeenCalledWith('planning.savePrompt.message');
  });

  it('an empty editor leaves without a question', async () => {
    const h = harness(false, { ...STATE, plan: null, polygon: [], setLaunchPoints: [null] });
    await h.exit();
    expect(h.confirm).not.toHaveBeenCalled();
    expect(h.goToMonitor).toHaveBeenCalledTimes(1);
  });
});

describe('the plan on the UAVs', () => {
  it('is synced after upload or download, until the operator edits it', () => {
    const h = harness(false);
    expect(h.hook.result.current.planSynced).toBe(false);
    act(() => { h.hook.result.current.onPlanSynced(); });
    expect(h.hook.result.current.planSynced).toBe(true);
    // Derived values do not unsync it.
    h.hook.rerender({ ...STATE, analysis: { required_uavs: 2 }, deliveryHubAssignments: [1] });
    expect(h.hook.result.current.planSynced).toBe(true);
    // New routes do (a planner setting can move them without an input change).
    h.hook.rerender({ ...STATE, plan: { ...PLAN, zones: [{ ...PLAN.zones[0], track: [{ lat: 41, lon: 44 }] }] } });
    expect(h.hook.result.current.planSynced).toBe(false);
  });

  it('leaving with an edited, unsaved copy asks, then returns the map to the UAVs\' plan', async () => {
    const h = harness('alt');
    act(() => { h.hook.result.current.onPlanSynced(); });
    h.hook.rerender({ ...STATE, simDockWps: { 0: [0] } });
    await h.exit();
    expect(h.confirm).toHaveBeenCalledWith('planning.savePrompt.messageRestore');
    expect(h.setters.setPlan).toHaveBeenCalledWith(PLAN);
    expect(h.setters.setSimDockWps).toHaveBeenCalledWith({});
    expect(h.goToMonitor).toHaveBeenCalledTimes(1);
  });

  it('leaving with the UAVs\' plan unchanged asks nothing and restores nothing', async () => {
    const h = harness(false);
    act(() => { h.hook.result.current.onPlanSynced(); });
    await h.exit();
    expect(h.confirm).not.toHaveBeenCalled();
    expect(h.setters.setPlan).not.toHaveBeenCalled();
    expect(h.goToMonitor).toHaveBeenCalledTimes(1);
  });

  // Live finding: on page load the notice "not on the UAVs" flashed while the
  // restore was still deriving the area and dock picks of the plan it had
  // just published, before the baseline was taken.
  it('a plan a download just published counts before its baseline lands', () => {
    const h = harness(false, { ...STATE, plan: tagPlanForRun(PLAN, { single: true }) });
    expect(h.hook.result.current.planSynced).toBe(true);
    // A plan the operator generates carries no tag.
    h.hook.rerender({ ...STATE, plan: { ...PLAN } });
    expect(h.hook.result.current.planSynced).toBe(false);
  });

  it('is forgotten when no UAV is left to carry it', () => {
    const h = harness(false);
    act(() => { h.hook.result.current.onPlanSynced(); });
    act(() => { h.hook.result.current.forgetPlanSync(); });
    expect(h.hook.result.current.planSynced).toBe(false);
  });
});
