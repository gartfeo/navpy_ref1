import { renderHook } from '@testing-library/react';
import { beforeEach, expect, it, vi } from 'vitest';
import useAssignmentMarkers from './useAssignmentMarkers';
import { dockLabelStyle } from '../utils/dockLabel';
import { makeApprovedDockIcon, makeAssignmentIcon, makeAwaitingDockIcon } from '../constants/icons';
import { zoneColorsSolid } from '../../../styles';

vi.mock('../../../hooks/useTelemetryStore', () => ({ useTelemetryStore: (read) => read({ getSysIdKey: () => '1,2' }) }));
const i18nMock = vi.hoisted(() => ({ calls: [], language: 'en' }));
vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key, args) => {
      i18nMock.calls.push([key, args]);
      return i18nMock.language === 'en' ? key : `${i18nMock.language}:${key}`;
    },
    i18n: { language: i18nMock.language },
  }),
}));

const cesium = { current: {
  Cartesian3: { fromDegrees: (lon, lat, height) => ({ lon, lat, height }) },
  Cartesian2: class { constructor(x, y) { this.x = x; this.y = y; } },
  Cartographic: { fromDegrees: (lon, lat) => ({ lon, lat }) },
  NearFarScalar: class { constructor(...args) { this.args = args; } },
  DistanceDisplayCondition: class { constructor(near, far) { this.near = near; this.far = far; } },
  Color: { fromCssColorString: value => value, BLACK: 'black', WHITE: 'white' },
  HeightReference: { CLAMP_TO_GROUND: 'clamped' },
  LabelStyle: { FILL_AND_OUTLINE: 'outlined' },
  VerticalOrigin: { BOTTOM: 'bottom', TOP: 'top' },
} };
const store = { current: { getVehicleList: () => [{ sys_id: 1 }, { sys_id: 2 }] } };
const UAV_2_COLOR = zoneColorsSolid[1];

const dock = { taskId: 3, taskType: 'DOCK', lat: 40, lon: 44, alt: 0 };
const ENTRIES = {
  waiting: { ...dock, key: 'waiting', status: 'waiting', senderId: 1, receiverId: 2 },
  confirming: { ...dock, key: 'confirming', status: 'confirming', receiverId: 2 },
  own: { ...dock, key: 'own', status: 'confirming', receiverId: null, askedBy: 2 },
  ownApproved: { ...dock, key: 'ownApproved', status: 'confirmed', receiverId: null, askedBy: 2 },
  assigned: { ...dock, key: 'assigned', status: 'assigned', receiverId: 2 },
  confirmed: { ...dock, key: 'confirmed', status: 'confirmed', receiverId: 2 },
  docked: { ...dock, key: 'docked', status: 'handled', receiverId: null, handledBy: 2, outcome: 'docked' },
  failed: { ...dock, key: 'failed', status: 'handled', receiverId: null, handledBy: 2, outcome: 'failed' },
  canceled: { ...dock, key: 'canceled', status: 'handled', receiverId: null, handledBy: 2, outcome: 'canceled' },
  unknown: { ...dock, key: 'unknown', status: 'handled', receiverId: null, handledBy: 2, outcome: 'unknown' },
};

function render(assignments, getHeight = () => undefined) {
  const entities = { add: vi.fn(value => value), remove: vi.fn() };
  const scene = { globe: { getHeight } };
  const view = renderHook(
    ({ entries }) => useAssignmentMarkers(cesium, { current: { entities, scene } }, entries, store, true),
    { initialProps: { entries: assignments } },
  );
  const byKey = (key) => {
    const index = Object.keys(assignments).indexOf(key);
    return entities.add.mock.results[index].value;
  };
  return { entities, byKey, ...view };
}

beforeEach(() => {
  i18nMock.language = 'en';
});

it('draws each state with its own icon and label', () => {
  const { byKey } = render(ENTRIES);
  const labels = Object.fromEntries(Object.keys(ENTRIES).map(key => [key, byKey(key).label.text]));
  expect(labels).toEqual({
    waiting: '#3 DOCK: mapMarkers.dockState.offered',
    confirming: 'UAV 2 > DOCK: mapMarkers.dockState.confirm',
    own: 'UAV 2 > DOCK: mapMarkers.dockState.confirm',
    ownApproved: 'UAV 2 > DOCK: mapMarkers.dockState.approved',
    assigned: 'UAV 2 > DOCK: mapMarkers.dockState.assigned',
    confirmed: 'UAV 2 > DOCK: mapMarkers.dockState.approved',
    docked: 'UAV 2 > DOCK: mapMarkers.dockState.docked',
    failed: 'UAV 2 > DOCK: mapMarkers.dockState.dockFailed',
    canceled: 'UAV 2 > DOCK: mapMarkers.dockState.canceled',
    unknown: 'UAV 2 > DOCK: mapMarkers.dockState.outcomeUnknown',
  });
  const images = ['waiting', 'confirming', 'assigned', 'confirmed', 'docked', 'failed', 'canceled']
    .map(key => byKey(key).billboard.image);
  expect(new Set(images).size).toBe(images.length);
  expect(byKey('own').billboard.image).toBe(byKey('confirming').billboard.image);
});

it('reads outcome unknown for a handled dock with no word it knows', () => {
  const { byKey } = render({ dock: { ...ENTRIES.docked, outcome: undefined } });
  expect(byKey('dock').label.text).toBe('UAV 2 > DOCK: mapMarkers.dockState.outcomeUnknown');
});

it('names the UAV an offer went to', () => {
  i18nMock.calls.length = 0;
  render({ dock: ENTRIES.waiting });
  expect(i18nMock.calls).toContainEqual(['mapMarkers.dockState.offered', { uav: 2 }]);
});

it('colours by state: a handled dock is grey whichever UAV it was', () => {
  const { byKey } = render(ENTRIES);
  expect(byKey('docked').label.fillColor).toBe('#9aa7b4');
  expect(byKey('confirming').label.fillColor).toBe('#ffc107');
  expect(byKey('waiting').label.fillColor).toBe('#ff9800');
  expect(byKey('assigned').label.fillColor).toBe(byKey('confirmed').label.fillColor);
  expect(byKey('assigned').label.fillColor).not.toBe('#9aa7b4');
  expect(byKey('ownApproved').label.fillColor).toBe(UAV_2_COLOR);
});

it('follows a slot through its states under one key', () => {
  const slot = { ...dock, key: 'helper:2', receiverId: 2 };
  const { entities, rerender, byKey } = render({ 'helper:2': { ...slot, status: 'assigned' } });
  const marker = byKey('helper:2');
  const read = () => [marker.label.text, marker.billboard.image, marker.label.fillColor, marker.label.outlineColor];
  expect(read()).toEqual([
    'UAV 2 > DOCK: mapMarkers.dockState.assigned', makeAssignmentIcon(32, UAV_2_COLOR), UAV_2_COLOR, 'white',
  ]);

  rerender({ entries: { 'helper:2': { ...slot, status: 'confirming' } } });
  expect(read()).toEqual([
    'UAV 2 > DOCK: mapMarkers.dockState.confirm', makeAwaitingDockIcon(32), '#ffc107', 'black',
  ]);

  rerender({ entries: { 'helper:2': { ...slot, status: 'confirmed' } } });
  expect(read()).toEqual([
    'UAV 2 > DOCK: mapMarkers.dockState.approved', makeApprovedDockIcon(32, UAV_2_COLOR), UAV_2_COLOR, 'white',
  ]);
  expect(entities.add).toHaveBeenCalledTimes(1);
  expect(entities.remove).not.toHaveBeenCalled();

  rerender({ entries: {} });
  expect(entities.remove).toHaveBeenCalledWith(marker);
});

it('draws a marker at its entry position, at creation and after an update', () => {
  const { rerender, byKey } = render({ dock: ENTRIES.assigned });
  const marker = byKey('dock');
  expect(marker.position).toEqual({ lon: 44, lat: 40, height: 0 });

  rerender({ entries: { dock: { ...ENTRIES.assigned, lon: 45, lat: 41, alt: 12 } } });
  expect(marker.position).toEqual({ lon: 45, lat: 41, height: 12 });
});

// Cesium 1.138 draws a clamped label's new letters at this raw height until a new tile
// renders: at 1316 m they were above a camera at 650 m, and the word was cut.
it('puts a marker on the rendered ground, at creation and after an update', () => {
  const ground = vi.fn(({ lat }) => (lat === 40 ? 0 : 1290));
  const { rerender, byKey } = render({ dock: { ...ENTRIES.assigned, alt: 1316 } }, ground);
  const marker = byKey('dock');
  expect(marker.position).toEqual({ lon: 44, lat: 40, height: 0 });
  expect(ground).toHaveBeenLastCalledWith({ lon: 44, lat: 40 });

  rerender({ entries: { dock: { ...ENTRIES.assigned, lon: 45, lat: 41, alt: 1316 } } });
  expect(marker.position).toEqual({ lon: 45, lat: 41, height: 1290 });
  expect(ground).toHaveBeenLastCalledWith({ lon: 45, lat: 41 });
});

it('uses the entry height until any ground is rendered there', () => {
  expect(render({ dock: { ...ENTRIES.assigned, alt: 1316 } }).byKey('dock').position.height).toBe(1316);
  expect(render({ dock: { ...ENTRIES.assigned, alt: undefined } }).byKey('dock').position.height).toBe(0);
});

it('draws a pad only where the selector left one, at creation and after an update', () => {
  const { rerender, byKey } = render({ a: { ...ENTRIES.canceled, hidePad: true }, b: ENTRIES.failed });
  const [a, b] = [byKey('a'), byKey('b')];
  expect([a.billboard.show, b.billboard.show]).toEqual([false, true]);

  rerender({ entries: { a: ENTRIES.canceled, b: { ...ENTRIES.failed, hidePad: true } } });
  expect([a.billboard.show, b.billboard.show]).toEqual([true, false]);
});

it('keeps a pad large enough to read its badge far out, and the word under it', () => {
  const { billboard, label } = render({ dock: ENTRIES.docked }).byKey('dock');
  expect(billboard.scaleByDistance.args).toEqual([500, 1.0, 20000, 0.6]);
  expect(label.pixelOffsetScaleByDistance.args).toEqual(billboard.scaleByDistance.args);
});

it('labels a dock below its pad, clear of the hub and closest-approach labels above it', () => {
  const { byKey } = render(ENTRIES);
  for (const key of Object.keys(ENTRIES)) {
    expect(byKey(key).label.verticalOrigin).toBe('top');
    expect(byKey(key).label.pixelOffset.y).toBeGreaterThan(16);   // half the 32 px pad
  }
});

it('gives each label at one spot its own row, at creation and after an update', () => {
  const { rerender, byKey } = render({ a: ENTRIES.assigned, b: { ...ENTRIES.confirming, labelRow: 1 } });
  const [a, b] = [byKey('a'), byKey('b')];
  const base = a.label.pixelOffset.y;
  expect(b.label.pixelOffset.y).toBe(base + 14);

  rerender({ entries: { a: { ...ENTRIES.assigned, labelRow: 1 }, b: ENTRIES.confirming } });
  expect(a.label.pixelOffset.y).toBe(base + 14);
  expect(b.label.pixelOffset.y).toBe(base);
});

it('shows the state word only within its range; the pad and badge show at any range', () => {
  const { byKey } = render({ dock: ENTRIES.docked });
  const { near, far } = byKey('dock').label.distanceDisplayCondition;
  expect([near, far]).toEqual([0, 1500]);
  expect(byKey('dock').billboard.distanceDisplayCondition).toBeUndefined();
});

it('gives only a label colour too dark for a black outline a white one', () => {
  const white = zoneColorsSolid.filter(color => dockLabelStyle(cesium.current, color).outlineColor === 'white');
  expect(white).toEqual([UAV_2_COLOR]);
  // Green weighs most in luminance and red far less: this red is too dark, the same green is not.
  expect(dockLabelStyle(cesium.current, '#800000').outlineColor).toBe('white');
  expect(dockLabelStyle(cesium.current, '#008000').outlineColor).toBe('black');
  const { byKey } = render(ENTRIES);
  expect(byKey('assigned').label.outlineColor).toBe('white');
  expect(byKey('docked').label.outlineColor).toBe('black');
  expect(byKey('waiting').label.outlineColor).toBe('black');
});

it('draws nothing for an entry with no position, never a guessed point', () => {
  for (const missing of [{ lat: null, lon: null }, { lat: null }, { lon: null }]) {
    const { entities } = render({ dock: { ...ENTRIES.own, ...missing } });
    expect(entities.add).not.toHaveBeenCalled();
  }
});

// A live state hides a handled dock at its spot, and it returns under the same key.
it('draws a dock again when its entry comes back', () => {
  const { entities, rerender } = render({ dock: ENTRIES.docked });
  rerender({ entries: {} });
  rerender({ entries: { dock: ENTRIES.docked } });
  expect(entities.add).toHaveBeenCalledTimes(2);
});

it('follows a live locale switch', () => {
  const entries = { dock: ENTRIES.docked };
  const { rerender, byKey } = render(entries);
  i18nMock.language = 'hy';
  rerender({ entries });
  expect(byKey('dock').label.text).toBe('UAV 2 > DOCK: hy:mapMarkers.dockState.docked');
});
