/**
 * A plan downloaded from the vehicles stays on screen until the operator
 * changes what the plan is generated from (todo 90).
 *
 * The defect: the download protected its plan with a one-shot flag that the
 * first approach-point effect consumed. Any later regen in PLANNING then
 * replaced the vehicles' missions with a plan regenerated from the polygon, so
 * an upload would have sent that plan. Three later commits did it: the
 * download's own delivery hub save (a settings round trip), the per-set arrays
 * resizing to the area's auto UAV count, and the planner configuration
 * arriving after the download.
 *
 * Runs the real mission state, orchestrator, generation, planner and the real
 * download path in useVehicleConnection.
 */
import { act, renderHook } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import useMissionState from './useMissionState';
import usePlanningOrchestrator from './usePlanningOrchestrator';
import useVehicleConnection from './useVehicleConnection';
import { configurePlanner, getConfig } from '../utils/planner';

afterEach(() => {
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

const AREA = [
  { lat: 40.32, lon: 44.43 }, { lat: 40.32, lon: 44.456 },
  { lat: 40.3, lon: 44.456 }, { lat: 40.3, lon: 44.43 },
];
// Its auto UAV count needs more than one set, unlike the default count of 3.
const LARGE_AREA = [
  { lat: 40.4, lon: 44.3 }, { lat: 40.4, lon: 44.6 },
  { lat: 40.2, lon: 44.6 }, { lat: 40.2, lon: 44.3 },
];
const LAUNCH = { lat: 40.31, lon: 44.47 };
const HUB = { name: 'Default delivery hub 1', type: 'building', lat: 40.31, lon: 44.46 };
const VEHICLES = [1, 2, 3].map((sys_id) => ({ sys_id, name: `UAV${sys_id}` }));

// What a vehicle returns for the mission the GCS uploaded to it.
function mission(sysId, area) {
  const [a, , c] = area;
  const lat = (k) => c.lat + ((a.lat - c.lat) * (sysId + k / 4)) / 4;
  return {
    sys_id: sysId,
    waypoints: [0, 1, 2, 3].map((k) => ({
      lat: lat(k), lon: a.lon + ((c.lon - a.lon) * (k + 1)) / 5, alt: 120,
    })),
    altitude_m: 120,
    search_pattern: 'distributed',
    polygon: area,
    launch_point: LAUNCH,
    corridor_backbone: [],
    default_delivery_hub: { lat: HUB.lat, lon: HUB.lon, type: HUB.type },
  };
}

// A corridor mission: no area, the track follows the backbone.
const BACKBONE = [{ lat: 40.3, lon: 44.44 }, { lat: 40.29, lon: 44.42 }];
function corridorMission(sysId) {
  return {
    ...mission(sysId, AREA),
    search_pattern: 'corridor',
    polygon: [],
    corridor_backbone: BACKBONE,
  };
}

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

const settle = () => act(() => new Promise((resolve) => { setTimeout(resolve, 150); }));

async function renderPlanner({
  area = AREA, plannerReady = true, missionFor = (sysId) => mission(sysId, area),
  vehicles = VEHICLES,
} = {}) {
  const io = stubFileIo();
  const api = {
    downloadMission: vi.fn((sysId) => Promise.resolve(missionFor(sysId))),
    downloadFence: vi.fn().mockResolvedValue(null),
    disconnectVehicle: vi.fn(),
  };
  const handleSaveSettings = vi.fn();
  const settings = { default_delivery_hubs: [HUB] };
  const props = { settingsVersion: 1, plannerReady };
  const { result, rerender } = renderHook(({ settingsVersion, plannerReady: ready }) => {
    const missionState = useMissionState();
    const planning = usePlanningOrchestrator({
      mission: missionState, settings, handleSaveSettings, settingsVersion, plannerReady: ready,
    });
    const connection = useVehicleConnection({
      mission: missionState,
      vehicleList: vehicles,
      removeVehicle: vi.fn(),
      api,
      telemetryStoreRef: { current: { getVehicles: () => ({}), subscribe: () => () => {} } },
      derivePlanPolygon: planning.derivePlanPolygon,
      onPlanSynced: planning.onPlanSynced,
      settings,
    });
    return { mission: missionState, planning, connection };
  }, { initialProps: props });
  const update = async (over) => {
    Object.assign(props, over);
    rerender({ ...props });
    await settle();
  };
  return {
    result,
    api,
    handleSaveSettings,
    sysIds: () => result.current.mission.plan?.zones?.map((z) => z.sys_id),
    async openPlanner() {
      act(() => result.current.planning.handleGoToPlanning());
      await settle();
    },
    async download() {
      await act(() => result.current.connection.handleDownloadPlan());
      await settle();
    },
    // useSettings bumps the version when a save round trip completes.
    bumpSettings: () => update({ settingsVersion: props.settingsVersion + 1 }),
    setPlannerReady: (ready) => update({ plannerReady: ready }),
    // useSettings on the first load: configure the planner, then ready + bump.
    configureAndReady(settingsPatch) {
      configurePlanner(settingsPatch);
      return update({ plannerReady: true, settingsVersion: props.settingsVersion + 1 });
    },
    async loadFile(contents) {
      io.contents = JSON.stringify(contents);
      act(() => {
        result.current.planning.handleLoadPolygon();
        io.input.onchange({ target: { files: [{}] } });
      });
      await settle();
    },
    settle,
    async edit(fn) {
      act(() => fn(result.current.mission));
      await settle();
    },
  };
}

describe('a plan downloaded while the planner is open', () => {
  it('survives the settings round trip of its own delivery hub save', async () => {
    const h = await renderPlanner();
    await h.openPlanner();
    await h.download();
    expect(h.sysIds()).toEqual([1, 2, 3]);
    // The download saved the vehicles' hub to settings; the save's round trip
    // re-analyzes the area.
    expect(h.handleSaveSettings).toHaveBeenCalled();
    await h.bumpSettings();
    expect(h.sysIds()).toEqual([1, 2, 3]);
  });

  it('survives the per-set arrays resizing to the area\'s UAV count', async () => {
    const h = await renderPlanner({ area: LARGE_AREA });
    await h.openPlanner();
    await h.download();
    expect(h.sysIds()).toEqual([1, 2, 3]);
    expect(h.result.current.planning.effectiveUavCount).toBe(3);
  });

  it('survives the planner configuration arriving after the download', async () => {
    const h = await renderPlanner({ area: LARGE_AREA, plannerReady: false });
    await h.openPlanner();
    await h.download();
    await h.setPlannerReady(true);
    expect(h.sysIds()).toEqual([1, 2, 3]);
  });

  it('survives a configuration that differs from the fallback arriving after the download', async () => {
    const overlap = getConfig().OVERLAP_FRACTION;
    try {
      const h = await renderPlanner({ plannerReady: false });
      await h.openPlanner();
      await h.download();
      await h.configureAndReady({ flight: { overlap_fraction: overlap / 2 } });
      expect(h.sysIds()).toEqual([1, 2, 3]);
    } finally {
      configurePlanner({ flight: { overlap_fraction: overlap } });
    }
  });
});

describe('an operator edit after a download still regenerates', () => {
  it('regenerates when the operator changes the UAV count', async () => {
    const h = await renderPlanner();
    await h.openPlanner();
    await h.download();
    await h.edit((m) => m.setUavCount(2));
    expect(h.result.current.mission.plan.zones).toHaveLength(2);
    expect(h.sysIds()).toEqual([undefined, undefined]);
  });

  it('regenerates when the operator moves the launch point', async () => {
    const h = await renderPlanner();
    await h.openPlanner();
    await h.download();
    await h.edit((m) => m.setLaunchPoint({ lat: 40.29, lon: 44.44 }));
    expect(h.sysIds()).toEqual([undefined, undefined, undefined]);
  });

  it('regenerates when a settings edit changes the planner configuration', async () => {
    const overlap = getConfig().OVERLAP_FRACTION;
    try {
      const h = await renderPlanner();
      await h.openPlanner();
      await h.download();
      // What useSettings does on a save: configure the planner, then bump.
      configurePlanner({ flight: { overlap_fraction: overlap / 2 } });
      await h.bumpSettings();
      expect(h.sysIds()).toEqual([undefined, undefined, undefined]);
    } finally {
      configurePlanner({ flight: { overlap_fraction: overlap } });
    }
  });

  it('regenerates on a UAV count change after a drag that left the geometry unchanged', async () => {
    const h = await renderPlanner();
    await h.openPlanner();
    await h.download();
    // An intermediate corridor point dragged away and back: the drag end asks
    // to skip one regen and commits an equal, new corridor array.
    await h.edit((m) => {
      h.result.current.planning.handleSuppressRegen();
      m.setSetCorridorPoints((prev) => prev.map((cp) => [...cp]));
    });
    await h.edit((m) => m.setUavCount(2));
    expect(h.result.current.mission.plan.zones).toHaveLength(2);
  });

  it('a loaded corridor file replaces the downloaded plan, even with equal inputs', async () => {
    const h = await renderPlanner({ missionFor: corridorMission });
    await h.openPlanner();
    await h.download();
    expect(h.sysIds()).toEqual([1, 2, 3]);
    await h.loadFile({
      polygon: [], search_pattern: 'corridor', uav_count: 3,
      set_launch_points: [LAUNCH], set_corridors: [BACKBONE],
    });
    expect(h.sysIds()).toEqual([undefined, undefined, undefined]);
  });

  it('keeps one per-set slot per set through a repeated multi-set download', async () => {
    const h = await renderPlanner({ vehicles: [...VEHICLES, { sys_id: 4, name: 'UAV4' }] });
    await h.openPlanner();
    await h.download();
    await h.download();
    expect(h.sysIds()).toEqual([1, 2, 3, 4]);
    const sets = h.result.current.planning.effectiveSets;
    expect(sets).toBeGreaterThan(1);
    expect(h.result.current.mission.setLaunchPoints).toHaveLength(sets);
    expect(h.result.current.mission.setCorridorPointsArr).toHaveLength(sets);
    // The second set's slot exists, so an edit regenerates once and settles.
    await h.edit((m) => m.setActiveSetIndex(1));
    await h.edit((m) => m.setPartitionAngleDeg(30));
    const regenerated = h.result.current.mission.plan;
    await h.settle();
    expect(h.result.current.mission.plan).toBe(regenerated);
  });

  it('a later settings change regenerates once the operator has edited', async () => {
    const h = await renderPlanner();
    await h.openPlanner();
    await h.download();
    await h.edit((m) => m.setLaunchPoint({ lat: 40.29, lon: 44.44 }));
    const regenerated = h.result.current.mission.plan;
    await h.bumpSettings();
    expect(h.result.current.mission.plan).not.toBe(regenerated);
  });
});
