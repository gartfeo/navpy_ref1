import { renderHook } from '@testing-library/react';
import { expect, it, vi } from 'vitest';
import useDockMarkers from './useDockMarkers';
import { DOCK_ICON } from '../constants/deliveryLocationIcons';

it('keeps every dock at its own coordinates without snapping, and replaces or clears markers', () => {
  const previous = { id: 'previous-marker' };
  const entities = { add: vi.fn(value => value), remove: vi.fn() };
  const cesium = { current: {
    Cartesian3: { fromDegrees: (lon, lat, height) => ({ lon, lat, height }) },
    Cartesian2: class { constructor(x, y) { this.x = x; this.y = y; } },
    NearFarScalar: class {},
    Color: { fromCssColorString: value => value, BLACK: 'black' },
    HeightReference: { CLAMP_TO_GROUND: 'clamped' },
    LabelStyle: { FILL_AND_OUTLINE: 'outlined' },
    VerticalOrigin: { BOTTOM: 'bottom' },
  } };
  const viewer = { current: { entities } };
  const state = { current: { simDocks: [previous] } };
  const { rerender } = renderHook(({ fallbackLocations }) => useDockMarkers(cesium, viewer, state, fallbackLocations, true), {
    initialProps: { fallbackLocations: [
      { lat: 40, lon: 44, zoneIndex: 0, wpNumber: 2, sys_id: 1 },
      { lat: 40.0001, lon: 44.0001, zoneIndex: 1, wpNumber: 3, sys_id: 2 },
      { lat: 41, lon: 45, zoneIndex: 2, wpNumber: 4, sys_id: 3 },
    ] },
  });
  expect(entities.remove).toHaveBeenCalledWith(previous);
  expect(state.current.simDocks.map(marker => marker.position)).toEqual([
    { lat: 40, lon: 44, height: 0 },
    { lat: 40.0001, lon: 44.0001, height: 0 },
    { lat: 41, lon: 45, height: 0 },
  ]);
  expect(state.current.simDocks.map(marker => marker.label.text)).toEqual([
    'mapMarkers.dock1', 'mapMarkers.dock2', 'mapMarkers.dock3',
  ]);
  const originalMarkers = [...state.current.simDocks];
  const first = state.current.simDocks[0];
  expect(first.position).toEqual({ lat: 40, lon: 44, height: 0 });
  expect(first.billboard).toMatchObject({ image: DOCK_ICON, width: 40, height: 40, heightReference: 'clamped' });
  expect(first).not.toHaveProperty('model');
  expect(first.label.text).toBe('mapMarkers.dock1');
  rerender({ fallbackLocations: [{ lat: 41, lon: 45, zoneIndex: 1, wpNumber: 3 }] });
  for (const marker of originalMarkers) expect(entities.remove).toHaveBeenCalledWith(marker);
  expect(state.current.simDocks[0].position).toEqual({ lat: 41, lon: 45, height: 0 });
  const second = state.current.simDocks[0];
  rerender({ fallbackLocations: [] });
  expect(entities.remove).toHaveBeenCalledWith(second);
  expect(state.current.simDocks).toEqual([]);
});
