/**
 * A saved plan file must regenerate the same tracks it was saved from.
 *
 * The defect: the file did not carry the route offset (track spacing), so a
 * plan saved with 9 lines at 90 m loaded back with auto spacing and fewer
 * lines, and its sim-dock waypoint indices pointed past the regenerated
 * tracks. Runs the real mission state, orchestrator, generation and planner.
 */
import { act, renderHook } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import usePlanningOrchestrator from './usePlanningOrchestrator';
import { MAX_UAV_COUNT } from '../utils/planLimits';

// Record the spacing override the planner sees at every track generation, so
// the test proves the loaded spacing is applied BEFORE the load regenerates.
const spacingSeen = vi.hoisted(() => []);
vi.mock('../utils/planner', async (importOriginal) => {
  const actual = await importOriginal();
  return {
    ...actual,
    generatePlan: (...args) => {
      spacingSeen.push(actual.getConfig().TRACK_SPACING_OVERRIDE_M);
      return actual.generatePlan(...args);
    },
  };
});

afterEach(() => {
  vi.unstubAllGlobals();
  spacingSeen.length = 0;
});

// The owner's saved plan (C:/Users/GARTF/Downloads/plan.json), which lost its
// 90 m route offset because the file had no field for it.
const OWNER_FILE = {
  polygon: [
    { lat: 40.31, lon: 44.4332 }, { lat: 40.31, lon: 44.453 },
    { lat: 40.3028, lon: 44.453 }, { lat: 40.3028, lon: 44.4332 },
  ],
  search_pattern: 'distributed',
  set_launch_points: [{ lat: 40.30643869633256, lon: 44.459596190863664 }],
  delivery_hub_assignments: [0, 0, 0],
  sim_dock_wps: { 0: [1, 2, 5] },
  detect_after_wps: { 0: 0, 1: 0, 2: 0 },
  fence: { enabled: false, offset_m: 900 },
  default_delivery_hubs: [{ lat: 40.3099864, lon: 44.4561868, type: 'building' }],
};

function stubFileIo() {
  const io = { contents: null, written: null, input: null };
  const create = document.createElement.bind(document);
  vi.spyOn(document, 'createElement').mockImplementation((tag, ...args) => {
    const element = create(tag, ...args);
    if (tag === 'input') io.input = element;
    if (tag === 'input' || tag === 'a') element.click = () => {};
    return element;
  });
  vi.stubGlobal('Blob', class { constructor(parts) { io.written = parts.join(''); } });
  vi.stubGlobal('URL', { createObjectURL: () => 'blob:fixture', revokeObjectURL: () => {} });
  vi.stubGlobal('FileReader', class {
    readAsText() { this.onload({ target: { result: io.contents } }); }
  });
  return io;
}

// A 2.2 x 2.2 km area: 3 UAVs at 90 m route offset, 6 UAVs (2 sets) at 20 m.
const LARGE_AREA = [
  { lat: 40.32, lon: 44.43 }, { lat: 40.32, lon: 44.456 },
  { lat: 40.3, lon: 44.456 }, { lat: 40.3, lon: 44.43 },
];
const LAUNCH_A = { lat: 40.31, lon: 44.47 };
const LAUNCH_B = { lat: 40.29, lon: 44.44 };

function renderPlanner(io, { unlocked = false, settings = {} } = {}) {
  vi.useFakeTimers();
  const props = { settingsVersion: 0, plannerReady: true };
  const { result, rerender } = renderHook(({ settingsVersion, plannerReady }) => {
    const mission = useMissionState();
    const planning = usePlanningOrchestrator({
      mission, settings, handleSaveSettings: () => {}, settingsVersion, plannerReady,
    });
    return { mission, planning };
  }, { initialProps: props });
  act(() => result.current.mission.goToPlanning());
  if (unlocked) act(() => result.current.mission.setUavCountLocked(false));
  const update = (over) => {
    Object.assign(props, over);
    rerender({ ...props });
    act(() => vi.runAllTimers());
  };
  return {
    result,
    // A backend settings round trip (e.g. the file's delivery hubs saved).
    bumpSettings() { update({ settingsVersion: props.settingsVersion + 1 }); },
    // Planner configuration (settings + vision catalog) available or not.
    setPlannerReady(plannerReady) { update({ plannerReady }); },
    set(fn) {
      act(() => fn(result.current.mission));
      act(() => vi.runAllTimers());
    },
    load(data) {
      io.contents = JSON.stringify(data);
      spacingSeen.length = 0;
      act(() => {
        result.current.planning.handleLoadPolygon();
        io.input.onchange({ target: { files: [{}] } });
      });
      // Drain the throttled approach-point regen as well.
      act(() => vi.runAllTimers());
    },
    save() {
      act(() => result.current.planning.handleSavePolygon());
      return JSON.parse(io.written);
    },
  };
}

const waypointsPerUav = (plan) => plan.zones.map((z) => z.track.length);

describe('plan file round trip', () => {
  it('regenerates the owner\'s 9 lines from a file carrying route_offset_m: 90', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ ...OWNER_FILE, route_offset_m: 90 });

    const { mission } = planner.result.current;
    expect(mission.routeOffsetM).toBe(90);
    expect(mission.plan.strip_count).toBe(9);
    expect(waypointsPerUav(mission.plan)).toEqual([6, 6, 6]);
    // Every track generation the load triggered already used the loaded spacing.
    expect(spacingSeen.length).toBeGreaterThan(0);
    expect(spacingSeen.every((m) => m === 90)).toBe(true);
    // The saved sim-dock indices still name existing waypoints.
    expect(mission.simDockWps).toEqual({ 0: [1, 2, 5] });
    for (const i of mission.simDockWps[0]) expect(i).toBeLessThan(mission.plan.zones[0].track.length);
  });

  it('keeps the file\'s delivery hub assignment through the load\'s regeneration', () => {
    const io = stubFileIo();
    const hubs = [
      { name: 'Near', type: 'building', lat: 40.3099864, lon: 44.4561868 },
      { name: 'Far', type: 'other', lat: 40.35, lon: 44.50 },
    ];
    const planner = renderPlanner(io, { settings: { default_delivery_hubs: hubs } });
    const { default_delivery_hubs: _h, ...file } = OWNER_FILE;
    // Two hubs, three zones: auto-assign uses both, so [1, 1, 1] is not its result.
    planner.load({ ...file, route_offset_m: 90, delivery_hub_assignments: [1, 1, 1] });
    const { mission } = planner.result.current;
    expect(mission.plan.zones).toHaveLength(3);
    expect(mission.deliveryHubAssignments).toEqual([1, 1, 1]);
    expect(mission.manualDeliveryHubEdit).toBe(true);
  });

  it('applies the loaded spacing over an auto-spaced plan already on screen', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load(OWNER_FILE); // older file: no route_offset_m -> auto
    const auto = planner.result.current.mission;
    expect(auto.routeOffsetM).toBeNull();
    expect(spacingSeen.every((m) => m === null)).toBe(true);
    expect(auto.plan.strip_count).not.toBe(9);

    planner.load({ ...OWNER_FILE, route_offset_m: 90 });
    const { mission } = planner.result.current;
    expect(spacingSeen.every((m) => m === 90)).toBe(true);
    expect(mission.plan.strip_count).toBe(9);
    expect(waypointsPerUav(mission.plan)).toEqual([6, 6, 6]);
    expect(mission.simDockWps).toEqual({ 0: [1, 2, 5] });
  });

  it('saves what it loaded and a fresh session regenerates the same tracks', () => {
    const io = stubFileIo();
    // The saved assignment indexes the settings hubs, so a session that saves
    // and reloads it has them (the stubbed settings save does not store any).
    const settings = { default_delivery_hubs: [{ name: 'Hub', type: 'building', lat: 40.3099864, lon: 44.4561868 }] };
    const first = renderPlanner(io, { settings });
    first.load({ ...OWNER_FILE, route_offset_m: 90 });
    const saved = first.save();
    expect(saved).toMatchObject({ route_offset_m: 90, uav_count: 3, partition_angle_deg: null });
    const tracks = first.result.current.mission.plan.zones.map((z) => z.track);

    const second = renderPlanner(io, { settings });
    second.load(saved);
    const { mission } = second.result.current;
    expect(mission.plan.zones.map((z) => z.track)).toEqual(tracks);
    expect(mission.simDockWps).toEqual({ 0: [1, 2, 5] });
  });

  it('keeps the loaded selections when the file\'s UAV count differs from the replaced plan', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ ...OWNER_FILE, route_offset_m: 90, uav_count: 6, sim_dock_wps: {} });
    expect(planner.result.current.mission.plan.zones).toHaveLength(6);

    planner.load({ ...OWNER_FILE, route_offset_m: 90, uav_count: 3 });
    const { mission } = planner.result.current;
    expect(mission.uavCount).toBe(3);
    expect(waypointsPerUav(mission.plan)).toEqual([6, 6, 6]);
    expect(mission.simDockWps).toEqual({ 0: [1, 2, 5] });
  });

  it('honors a saved UAV count below the requirement when the count is unlocked', () => {
    const io = stubFileIo();
    const file = { polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: 3 };
    const locked = renderPlanner(io);
    locked.load(file);
    const reference = locked.result.current.mission.plan.zones.map((z) => z.track);
    expect(reference).toHaveLength(3);
    const lastOfZone1 = reference[1].length - 1;

    const planner = renderPlanner(io, { unlocked: true });
    planner.load({ ...file, sim_dock_wps: { 1: [lastOfZone1] } });
    planner.bumpSettings();
    const { mission } = planner.result.current;
    expect(mission.analysis.required_uavs).toBe(6);
    expect(mission.uavCount).toBe(3);
    expect(mission.plan.zones.map((z) => z.track)).toEqual(reference);
    expect(mission.simDockWps).toEqual({ 1: [lastOfZone1] });
  });

  it('still raises an unlocked count when the operator grows the requirement', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io, { unlocked: true });
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 90, uav_count: 3 });
    expect(planner.result.current.mission.analysis.required_uavs).toBe(3);
    planner.set((m) => m.setRouteOffsetM(20));
    expect(planner.result.current.mission.uavCount).toBe(6);
  });

  it('drops the selections of the replaced plan when the file has none', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: 6,
      sim_dock_wps: { 4: [3] }, detect_after_wps: { 4: 1 } });
    expect(planner.result.current.mission.simDockWps).toEqual({ 4: [3] });

    const { sim_dock_wps: _s, detect_after_wps: _d, ...noSelections } = OWNER_FILE;
    planner.load({ ...noSelections, route_offset_m: 90, uav_count: 3 });
    const { mission } = planner.result.current;
    expect(mission.plan.zones).toHaveLength(3);
    expect(mission.simDockWps).toEqual({});
    expect(mission.detectAfterWps).toEqual({});
  });

  it('sizes the loaded launch points from the auto UAV count of the loaded plan', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    // Replaced plan: its analysis asks for 3 UAVs (1 set), its count is 6 (2 sets).
    planner.load({ ...OWNER_FILE, route_offset_m: 90, uav_count: 6, sim_dock_wps: {} });
    expect(planner.result.current.mission.analysis.required_uavs).toBe(3);

    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20,
      uav_count: null, set_launch_points: [LAUNCH_A, LAUNCH_B] });
    const { mission } = planner.result.current;
    expect(mission.uavCount).toBe(6);
    expect(mission.setLaunchPoints).toEqual([LAUNCH_A, LAUNCH_B]);
    expect(mission.plan.zones).toHaveLength(6);
  });

  it('keeps an unlocked saved count and its selections across planner readiness', () => {
    const io = stubFileIo();
    const file = { polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: 3 };
    const locked = renderPlanner(io);
    locked.load(file);
    const reference = locked.result.current.mission.plan.zones.map((z) => z.track);
    const lastOfZone1 = reference[1].length - 1;

    const planner = renderPlanner(io, { unlocked: true });
    planner.setPlannerReady(false);
    planner.load({ ...file, sim_dock_wps: { 1: [lastOfZone1] } });
    planner.setPlannerReady(true);
    const { mission } = planner.result.current;
    expect(mission.uavCount).toBe(3);
    expect(mission.plan.zones.map((z) => z.track)).toEqual(reference);
    expect(mission.simDockWps).toEqual({ 1: [lastOfZone1] });
  });

  it('keeps a held file\'s selections when readiness returns over a plan with more zones', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: 6,
      sim_dock_wps: { 4: [3] }, detect_after_wps: { 4: 1 } });
    expect(planner.result.current.mission.plan.zones).toHaveLength(6);
    planner.setPlannerReady(false);
    planner.load({ ...OWNER_FILE, route_offset_m: 90, uav_count: 3 });
    expect(planner.result.current.mission.plan.zones).toHaveLength(6); // held, not applied
    planner.setPlannerReady(true);
    const { mission } = planner.result.current;
    expect(waypointsPerUav(mission.plan)).toEqual([6, 6, 6]);
    expect(mission.simDockWps).toEqual(OWNER_FILE.sim_dock_wps);
    expect(mission.detectAfterWps).toEqual(OWNER_FILE.detect_after_wps);
  });

  it('keeps the loaded launch points of an auto-count file across planner readiness', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ ...OWNER_FILE, route_offset_m: 90, uav_count: 6, sim_dock_wps: {} });
    planner.setPlannerReady(false);
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20,
      uav_count: null, set_launch_points: [LAUNCH_A, LAUNCH_B] });
    planner.setPlannerReady(true);
    const { mission } = planner.result.current;
    expect(mission.uavCount).toBe(6);
    expect(mission.setLaunchPoints).toEqual([LAUNCH_A, LAUNCH_B]);
    expect(mission.plan.zones).toHaveLength(6);
  });

  it('keeps a count the operator lowered below the requirement until the requirement grows', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io, { unlocked: true });
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 90, uav_count: 3 });
    planner.set((m) => m.setUavCount(2)); // the sidebar minus button
    planner.bumpSettings();               // re-analysis, same requirement (3)
    expect(planner.result.current.mission.uavCount).toBe(2);
    expect(planner.result.current.mission.plan.zones).toHaveLength(2);
    planner.set((m) => m.setRouteOffsetM(20)); // requirement grows to 6
    expect(planner.result.current.mission.uavCount).toBe(6);
  });

  it('caps the automatic count at the import limit, so the saved file loads', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    // ~11 x 11 km at 20 m: the area alone would ask for more than MAX_UAV_COUNT.
    const huge = [
      { lat: 40.4, lon: 44.43 }, { lat: 40.4, lon: 44.56 },
      { lat: 40.3, lon: 44.56 }, { lat: 40.3, lon: 44.43 },
    ];
    planner.load({ polygon: huge, search_pattern: 'distributed', route_offset_m: 20, uav_count: null });
    const { mission } = planner.result.current;
    expect(mission.analysis.required_uavs).toBeGreaterThan(MAX_UAV_COUNT);
    expect(mission.uavCount).toBe(MAX_UAV_COUNT);
    const saved = planner.save();
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    renderPlanner(io).load(saved);
    expect(alert).not.toHaveBeenCalled();
  });

  it('cancels a held file when the operator clears the plan', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ ...OWNER_FILE, route_offset_m: 90 });
    planner.setPlannerReady(false);
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: 6,
      sim_dock_wps: { 4: [3] } });
    act(() => planner.result.current.planning.handleClearAll());
    planner.setPlannerReady(true);
    const { mission } = planner.result.current;
    expect(mission.polygon).toEqual([]);
    expect(mission.plan).toBeNull();
    expect(mission.simDockWps).toEqual({});
  });

  it('cancels a held file when the operator draws a new area first', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.setPlannerReady(false);
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: 6 });
    planner.set((m) => m.setPolygon(OWNER_FILE.polygon)); // drawing commits the polygon
    planner.setPlannerReady(true);
    const { mission } = planner.result.current;
    expect(mission.polygon).toEqual(OWNER_FILE.polygon);
    expect(mission.routeOffsetM).toBeNull();
    expect(mission.uavCount).not.toBe(6);
  });

  it('gives an auto-count corridor file the corridor default, not the replaced area\'s count', () => {
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load({ polygon: LARGE_AREA, search_pattern: 'distributed', route_offset_m: 20, uav_count: null });
    expect(planner.result.current.mission.analysis.required_uavs).toBe(6);

    const corridor = {
      polygon: [], search_pattern: 'corridor', uav_count: null,
      set_launch_points: [LAUNCH_A],
      set_corridors: [[{ lat: 40.32, lon: 44.47 }, { lat: 40.34, lon: 44.47 }]],
    };
    planner.load(corridor);
    const { mission } = planner.result.current;
    expect(mission.analysis).toBeNull();
    expect(mission.polygon).toEqual([]);
    expect(mission.plan.zones).toHaveLength(3);

    // A later re-analysis has no replaced area left to bring the count back.
    planner.setPlannerReady(false);
    planner.setPlannerReady(true);
    expect(planner.result.current.mission.plan.zones).toHaveLength(3);

    // The same file in a fresh session (no area analysed before) agrees.
    const fresh = renderPlanner(io);
    fresh.load(corridor);
    expect(fresh.result.current.mission.plan.zones).toHaveLength(3);
  });
});

// The hand delivery demo's plan file (scripts/delivery_demo_plan.py): three
// routes of three lines, and two docks 150 m apart on the lowest zone's track,
// so its UAV can see both together.
describe('delivery demo plan file', () => {
  it('regenerates three routes with the two docks 150 m apart on the lowest zone', async () => {
    const { default: file } = await import('../../../../../scripts/delivery_demo_plan.json');
    const io = stubFileIo();
    const planner = renderPlanner(io);
    planner.load(file);

    const { mission } = planner.result.current;
    expect(waypointsPerUav(mission.plan)).toEqual([6, 6, 6]);
    expect(mission.simDockWps).toEqual({ 2: [1, 2] });
    expect(mission.detectAfterWps).toEqual({ 0: 0, 1: 0, 2: 0 });
    expect(mission.deliveryHubAssignments).toEqual([2, 1, 0]);
    const altitudes = mission.plan.zones.map((z) => z.altitude_m);
    expect(altitudes.every(Number.isFinite)).toBe(true);
    expect(altitudes[2]).toBe(Math.min(...altitudes));
    const [a, b] = [1, 2].map((i) => mission.plan.zones[2].track[i]);
    const metres = Math.hypot(
      (a.lat - b.lat) * 111320,
      (a.lon - b.lon) * 111320 * Math.cos((a.lat * Math.PI) / 180),
    );
    expect(metres).toBeGreaterThan(140);
    expect(metres).toBeLessThan(160);
  });
});
