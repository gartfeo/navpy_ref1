import { act, renderHook } from '@testing-library/react';
import { beforeEach, expect, it, vi } from 'vitest';
import useClosestApproachMarks, {
  SIM_CLOSEST_APPROACH_DETAIL_RANGE_M, SIM_CLOSEST_APPROACH_MARKS_PER_UAV,
} from './useClosestApproachMarks';

vi.mock('../utils/uavOrientation', () => ({
  uavModelOrientation: (_Cesium, position, heading, roll, pitch) => ({ position, heading, roll, pitch }),
}));

const color = (css) => ({ css, withAlpha: (alpha) => ({ css, alpha }) });
const cesium = { current: {
  Cartesian3: Object.assign(class { constructor(x, y, z) { Object.assign(this, { x, y, z }); } }, {
    fromDegrees: (lon, lat, height) => ({ lon, lat, height }),
    midpoint: (a, b) => ({ mid: [a, b] }),
  }),
  Cartesian2: class { constructor(x, y) { this.x = x; this.y = y; } },
  NearFarScalar: class {},
  DistanceDisplayCondition: class { constructor(near, far) { this.near = near; this.far = far; } },
  PolylineDashMaterialProperty: class { constructor(options) { this.dashed = options; } },
  Color: { fromCssColorString: color, WHITE: color('white'), BLACK: color('black') },
  ColorBlendMode: { MIX: 'mix' },
  LabelStyle: { FILL_AND_OUTLINE: 'outlined' },
  VerticalOrigin: { BOTTOM: 'bottom' },
} };

let entities;
let viewer;
let store;

function makeStore(sysIds) {
  let listeners = [];
  let list = sysIds.map(sys_id => ({ sys_id }));
  return {
    getVehicleList: () => list,
    subscribe: (fn) => { listeners.push(fn); return () => { listeners = listeners.filter(l => l !== fn); }; },
    setFleet: (ids) => { list = ids.map(sys_id => ({ sys_id })); listeners.forEach(fn => fn()); },
  };
}

beforeEach(() => {
  entities = new Set();
  viewer = { current: { entities: {
    add: vi.fn((entity) => { entities.add(entity); return entity; }),
    remove: vi.fn((entity) => entities.delete(entity)),
  } } };
  store = makeStore([2, 3]);
});

function record(overrides = {}) {
  return {
    key: 'b:1:1', sys_id: 2, vehicle: 2, task_id: 7, dock_uid: 3, episode: 1,
    outcome: 'missed', aircraft_time_s: 512.3, lat: 40, lon: 44, alt_amsl_m: 1101,
    alt_rel_m: 41, d3_m: 1.84, h_m: 1.52, v_m: 1.04,
    dock_lat: 40.00001, dock_lon: 44.00001, dock_alt_amsl_m: 1100, dock_alt_rel_m: 40,
    roll_deg: -3, pitch_deg: 2, yaw_deg: 91,
    ...overrides,
  };
}

const DEFAULTS = {
  records: null, simMode: true, show: true, resetKey: '0:0', hiddenSysIds: null, paused: false, ready: true,
};

function render(props = {}) {
  const storeRef = { current: store };
  return renderHook((p) => useClosestApproachMarks(
    cesium, viewer, storeRef, p.ready, p.records, p.simMode, p.show, p.resetKey, p.hiddenSysIds, p.paused,
  ), { initialProps: { ...DEFAULTS, ...props } });
}

/** Load the page (first status consumed unshown), then show `records`. */
function renderWith(records, props = {}) {
  const hook = render({ records: [], ...props });
  let current = { ...DEFAULTS, records: [], ...props };
  const update = (next) => { current = { ...current, ...next }; hook.rerender(current); };
  update({ records });
  return { ...hook, update };
}

const marks = () => [...entities].filter(e => e.properties && e.point);
const labelOf = (entity) => entity.label.text;

it('draws the mark, the dock dot, the triangle and the ghost of a missed approach', () => {
  renderWith([record()]);
  const all = [...entities];
  const [mark] = marks();
  expect(mark.position).toEqual({ lon: 44, lat: 40, height: 41 }); // alt_rel_m, on the trail
  expect(mark.point.color.css).toBe('#ef4444');
  expect(labelOf(mark)).toBe(
    'UAV 2 · 1.84 m mapMarkers.closestApproach.missed\nh 1.52 m · v 1.04 m',
  );
  const dock = all.find(e => e.point && !e.properties);
  expect(dock.position).toEqual({ lon: 44.00001, lat: 40.00001, height: 40 });
  const legs = all.filter(e => e.polyline && e.label);
  expect(legs.map(e => e.polyline.positions)).toEqual([
    [{ lon: 44.00001, lat: 40.00001, height: 40 }, { lon: 44, lat: 40, height: 40 }],
    [{ lon: 44, lat: 40, height: 40 }, { lon: 44, lat: 40, height: 41 }],
  ]);
  expect(legs.map(e => e.label.text)).toEqual(['h 1.52 m', 'v 1.04 m']);
  expect(legs.every(e => e.polyline.material.dashed)).toBe(true);
  const line = all.find(e => e.polyline && !e.label);
  expect(line.polyline.positions).toEqual([dock.position, mark.position]);
  expect(line.polyline.material.css).toBe('#ef4444');
  const ghost = all.find(e => e.model);
  expect(ghost.position).toEqual(mark.position);
  expect(ghost.orientation).toEqual({ position: mark.position, heading: 91, roll: -3, pitch: 2 });
  expect(ghost.model.color.alpha).toBeCloseTo(0.35);
  expect(ghost.model.uri).toBe('/Peron15.glb');
  for (const detail of [ghost.model, ...legs.map(e => e.label)]) {
    expect(detail.distanceDisplayCondition.far).toBe(SIM_CLOSEST_APPROACH_DETAIL_RANGE_M);
  }
  expect(mark.properties.closestApproachView).toEqual({ lon: 44, lat: 40, alt: 41, headingDeg: 91 });
});

it('colours a docked approach green and says so; no attitude, no ghost', () => {
  renderWith([record({ outcome: 'docked', roll_deg: null, pitch_deg: null, yaw_deg: null })]);
  const [mark] = marks();
  expect(mark.point.color.css).toBe('#22c55e');
  expect(labelOf(mark)).toContain('mapMarkers.closestApproach.docked');
  expect([...entities].some(e => e.model)).toBe(false);
  expect(mark.properties.closestApproachView.headingDeg).toBeNull();
});

it('colours an aborted approach amber and names its reason', () => {
  renderWith([record({ outcome: 'aborted', reason: 'source_command_stale' })]);
  const [mark] = marks();
  expect(mark.point.color.css).toBe('#f59e0b');
  expect(labelOf(mark)).toBe(
    'UAV 2 · 1.84 m mapMarkers.closestApproach.aborted (source_command_stale)\nh 1.52 m · v 1.04 m',
  );
});

it('an aborted approach without a known reason says only aborted', () => {
  renderWith([record({ outcome: 'aborted', reason: null })]);
  expect(labelOf(marks()[0])).toContain('mapMarkers.closestApproach.aborted\n');
});

it('consumes the first status after load unshown and never re-adds a key', () => {
  const { update } = renderWith([]);
  const hook = render({ records: [record()] });
  expect(entities.size).toBe(0); // reload: the first status is consumed
  hook.rerender({ ...DEFAULTS, records: [record()] });
  expect(entities.size).toBe(0);
  update({ records: [record({ key: 'b:1:2' })] });
  expect(marks()).toHaveLength(1);
});

it('hides with the layer and the UAV, keeps the marks, and clears on reset', () => {
  const { update } = renderWith([record(), record({ key: 'b:2:1', sys_id: 3, vehicle: 3 })]);
  const count = entities.size;
  update({ show: false });
  expect([...entities].every(e => e.show === false)).toBe(true);
  update({ show: true, hiddenSysIds: new Set([3]) });
  expect(marks().map(e => e.show)).toEqual([true, false]);
  expect(entities.size).toBe(count);
  update({ hiddenSysIds: null, resetKey: '0:1' }); // Clear all
  expect(entities.size).toBe(0);
  update({ records: [record(), record({ key: 'b:2:1', sys_id: 3, vehicle: 3 })] }); // replayed status
  expect(entities.size).toBe(0);
});

it('a record first seen while paused is never shown; after resume new ones are', () => {
  const { update } = renderWith([record()], { paused: true });
  expect(entities.size).toBe(0);
  update({ paused: false, records: [record()] });
  expect(entities.size).toBe(0);
  update({ records: [record(), record({ key: 'b:1:2' })] });
  expect(marks()).toHaveLength(1);
});

it('a UAV leaving the fleet loses its marks; records seen while absent are consumed', () => {
  const { update } = renderWith([record()]);
  expect(marks()).toHaveLength(1);
  act(() => store.setFleet([3]));
  expect(entities.size).toBe(0);
  update({ records: [record(), record({ key: 'b:1:2' })] });
  act(() => store.setFleet([2, 3]));
  update({ records: [record(), record({ key: 'b:1:2' })] });
  expect(entities.size).toBe(0);
  update({ records: [record({ key: 'b:1:3' })] });
  expect(marks()).toHaveLength(1);
});

it('sim mode off removes the marks and adds nothing; on again shows only new records', () => {
  const { update } = renderWith([record()]);
  update({ simMode: false });
  expect(entities.size).toBe(0);
  update({ records: [record(), record({ key: 'b:1:2' })] });
  update({ simMode: true, records: [record(), record({ key: 'b:1:2' })] });
  expect(entities.size).toBe(0);
  update({ records: [record({ key: 'b:1:3' })] });
  expect(marks()).toHaveLength(1);
});

it('keeps marks across a disconnect and adds a record made meanwhile', () => {
  const { update } = renderWith([record()]);
  update({ records: [record(), record({ key: 'b:1:2' })] }); // reconnect status
  expect(marks()).toHaveLength(2);
});

it('waits for the viewer and draws the consumed marks when it is ready', () => {
  const live = viewer.current;
  viewer.current = null;
  const { update } = renderWith([record()], { ready: false });
  expect(entities.size).toBe(0);
  viewer.current = live;
  update({ ready: true });
  expect(marks()).toHaveLength(1);
});

it('keeps at most the per-UAV cap, dropping the oldest', () => {
  const records = Array.from({ length: SIM_CLOSEST_APPROACH_MARKS_PER_UAV + 2 }, (_, i) => record({ key: `b:1:${i}`, episode: i }));
  renderWith(records);
  const shown = marks().map(e => e.properties.closestApproachKey);
  expect(shown).toHaveLength(SIM_CLOSEST_APPROACH_MARKS_PER_UAV);
  expect(shown[0]).toBe('b:1:2');
});

it('skips a record without heights over home', () => {
  renderWith([record({ alt_rel_m: null })]);
  expect(entities.size).toBe(0);
});
