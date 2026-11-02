import { renderHook } from '@testing-library/react';
import { beforeEach, expect, it, vi } from 'vitest';
import useAvailableTaskMarkers from './useAvailableTaskMarkers';
import { makeAvailableTaskIcon } from '../constants/icons';

const i18nState = vi.hoisted(() => ({ language: 'en' }));
vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key) => (i18nState.language === 'en' ? key : `${i18nState.language}:${key}`),
    i18n: { language: i18nState.language },
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

const task = { key: '1:3', taskId: 3, taskType: 'DOCK', lat: 40, lon: 44, alt: 0, senderId: 1 };

function render(tasks, getHeight = () => undefined) {
  const entities = { add: vi.fn(value => value), remove: vi.fn() };
  const scene = { globe: { getHeight } };
  const view = renderHook(
    (props) => useAvailableTaskMarkers(cesium, { current: { entities, scene } }, props.tasks, true),
    { initialProps: { tasks } },
  );
  return { entities, marker: entities.add.mock.results[0]?.value, ...view };
}

beforeEach(() => {
  i18nState.language = 'en';
});

it('names the state on a detected dock and removes it when it is gone', () => {
  const { entities, marker, rerender } = render({ '1:3': task });
  expect(marker.label.text).toBe('#3 DOCK: mapMarkers.dockState.detected');
  expect(marker.billboard.image).toBe(makeAvailableTaskIcon(32));
  expect(marker.label.fillColor).toBe('#ff9800');
  expect(marker.label.outlineColor).toBe('black');
  expect(marker.label.verticalOrigin).toBe('top');   // the same side as the later states
  expect(marker.label.pixelOffset.y).toBeGreaterThan(16);

  rerender({ tasks: {} });
  expect(entities.remove).toHaveBeenCalledWith(marker);
});

it('updates a marker in place: position, label and row', () => {
  const { entities, marker, rerender } = render({ '1:3': task });
  const base = marker.label.pixelOffset.y;
  expect(marker.position).toEqual({ lon: 44, lat: 40, height: 0 });

  rerender({ tasks: { '1:3': { ...task, taskId: 4, lon: 45, lat: 41, alt: 12, labelRow: 1 } } });
  expect(entities.add).toHaveBeenCalledTimes(1);
  expect(marker.position).toEqual({ lon: 45, lat: 41, height: 12 });
  expect(marker.label.text).toBe('#4 DOCK: mapMarkers.dockState.detected');
  expect(marker.label.pixelOffset.y).toBe(base + 14);
});

it('puts a marker on the rendered ground, at creation and after an update', () => {
  const ground = vi.fn(({ lat }) => (lat === 40 ? 0 : 1290));
  const { marker, rerender } = render({ '1:3': { ...task, alt: 1316 } }, ground);
  expect(marker.position).toEqual({ lon: 44, lat: 40, height: 0 });
  expect(ground).toHaveBeenLastCalledWith({ lon: 44, lat: 40 });

  rerender({ tasks: { '1:3': { ...task, lon: 45, lat: 41, alt: 1316 } } });
  expect(marker.position).toEqual({ lon: 45, lat: 41, height: 1290 });
  expect(ground).toHaveBeenLastCalledWith({ lon: 45, lat: 41 });
});

it('uses the entry height until any ground is rendered there', () => {
  expect(render({ '1:3': { ...task, alt: 1316 } }).marker.position.height).toBe(1316);
  expect(render({ '1:3': { ...task, alt: undefined } }).marker.position.height).toBe(0);
});

it('draws a ring only where the selector left one, at creation and after an update', () => {
  const { marker, rerender } = render({ '1:3': { ...task, hidePad: true } });
  expect(marker.billboard.show).toBe(false);

  rerender({ tasks: { '1:3': task } });
  expect(marker.billboard.show).toBe(true);

  rerender({ tasks: { '1:3': { ...task, hidePad: true } } });
  expect(marker.billboard.show).toBe(false);
});

it('keeps the ring as large as a dock pad far out, and the word under it', () => {
  const { billboard, label } = render({ '1:3': task }).marker;
  expect(billboard.scaleByDistance.args).toEqual([500, 1.0, 20000, 0.6]);
  expect(label.pixelOffsetScaleByDistance.args).toEqual(billboard.scaleByDistance.args);
});

it('follows a live locale switch', () => {
  const tasks = { '1:3': task };
  const { marker, rerender } = render(tasks);
  i18nState.language = 'hy';
  rerender({ tasks });
  expect(marker.label.text).toBe('#3 DOCK: hy:mapMarkers.dockState.detected');
});

it('shows the state word only within its range; the ring shows at any range', () => {
  const { marker } = render({ '1:3': task });
  const { near, far } = marker.label.distanceDisplayCondition;
  expect([near, far]).toEqual([0, 1500]);
  expect(marker.billboard.distanceDisplayCondition).toBeUndefined();
});

it('draws nothing for a task with no position, never a guessed point', () => {
  for (const missing of [{ lat: null, lon: null }, { lat: null }, { lon: null }]) {
    const { entities } = render({ '1:3': { ...task, ...missing } });
    expect(entities.add).not.toHaveBeenCalled();
  }
});

it('draws a task again when it comes back', () => {
  const { entities, rerender } = render({ '1:3': task });
  rerender({ tasks: {} });
  rerender({ tasks: { '1:3': task } });
  expect(entities.add).toHaveBeenCalledTimes(2);
});
