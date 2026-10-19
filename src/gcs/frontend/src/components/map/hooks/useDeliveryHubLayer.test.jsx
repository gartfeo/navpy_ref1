import { renderHook } from '@testing-library/react';
import { expect, it, vi } from 'vitest';
import useDeliveryHubLayer from './useDeliveryHubLayer';

function fakeCesium() {
  return { current: {
    Cartesian3: { fromDegrees: (lon, lat, height) => ({ lon, lat, height }) },
    Cartesian2: class {},
    NearFarScalar: class {},
    PropertyBag: class { addProperty() {} },
    Color: { WHITE: 'white', BLACK: 'black' },
    HeightReference: { CLAMP_TO_GROUND: 'clamped' },
    LabelStyle: { FILL_AND_OUTLINE: 'outlined' },
    VerticalOrigin: { BOTTOM: 'bottom' },
  } };
}

const HUBS = [
  { lat: 40, lon: 44, name: 'Hub A' },
  { lat: 40.1, lon: 44.1, name: 'Hub B' },
  { lat: 40.2, lon: 44.2, name: 'Hub C' },
];

it('draws the hubs before terrain is ready (no Ion token) and redraws when terrain arrives', () => {
  const entities = { add: vi.fn(value => value), remove: vi.fn() };
  const viewer = { current: { entities, scene: { requestRender: vi.fn() } } };
  const { rerender } = renderHook(
    ({ terrainReady }) => useDeliveryHubLayer(
      fakeCesium(), viewer, HUBS, null, null, true, terrainReady, null, null, false,
    ),
    { initialProps: { terrainReady: false } },
  );
  expect(entities.add).toHaveBeenCalledTimes(3);
  const first = entities.add.mock.calls.map(([e]) => e);
  expect(first.map(e => e.label.text)).toEqual(['Hub A', 'Hub B', 'Hub C']);
  expect(first.every(e => e.billboard.heightReference === 'clamped')).toBe(true);

  rerender({ terrainReady: true });
  for (const e of first) expect(entities.remove).toHaveBeenCalledWith(e);
  expect(entities.add).toHaveBeenCalledTimes(6);
});
