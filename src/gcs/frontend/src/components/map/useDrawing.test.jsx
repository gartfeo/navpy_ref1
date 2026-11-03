/**
 * Draw flow of the search zone: the free polygon (unchanged) and the
 * three-click rectangle, which ends as a plain 4-corner polygon.
 */
import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useDrawing from './useDrawing';
import { rectangleCorners } from '../../utils/rectangleDraw';
import { offsetLatLon } from '../../utils/geo';

const a = { lat: 40.18, lon: 44.51 };
const b = offsetLatLon(a.lat, a.lon, 347, 1211); // ~1260 m at ~74 deg
const c = offsetLatLon(b.lat, b.lon, -740, 212); // ~770 m to the right

function setup() {
  const setPolygon = vi.fn();
  const analyze = vi.fn();
  const hook = renderHook(() => useDrawing({ setPolygon, analyze }));
  act(() => hook.result.current.startDraw());
  return { hook, setPolygon, analyze };
}

function click(hook, p) {
  let accepted;
  act(() => { accepted = hook.result.current.onMapClick(p); });
  return accepted;
}

describe('useDrawing rectangle shape', () => {
  it('makes a 4-corner polygon from three clicks and stops drawing', () => {
    const { hook, setPolygon, analyze } = setup();
    act(() => hook.result.current.setDrawShape('rectangle'));
    expect(click(hook, a)).toBe(true);
    expect(click(hook, b)).toBe(true);
    expect(hook.result.current.vertices).toEqual([a, b]);
    expect(analyze).not.toHaveBeenCalled();
    expect(click(hook, c)).toBe(true);
    const corners = rectangleCorners(a, b, c);
    expect(hook.result.current.vertices).toEqual(corners);
    expect(setPolygon).toHaveBeenLastCalledWith(corners);
    expect(analyze).toHaveBeenCalledExactlyOnceWith(corners);
    expect(hook.result.current.isDrawing).toBe(false);
  });

  it('ignores a repeated click (double-click) without an undo step', () => {
    const { hook } = setup();
    act(() => hook.result.current.setDrawShape('rectangle'));
    click(hook, a);
    expect(click(hook, a)).toBe(false);
    expect(hook.result.current.vertices).toEqual([a]);
    click(hook, b);
    expect(click(hook, b)).toBe(false);
    expect(hook.result.current.vertices).toEqual([a, b]);
    expect(hook.result.current.isDrawing).toBe(true);
    // Two real clicks, two undo steps.
    act(() => hook.result.current.undo());
    expect(hook.result.current.vertices).toEqual([a]);
    act(() => hook.result.current.undo());
    expect(hook.result.current.vertices).toEqual([]);
  });

  it('undo of the width click returns to the two-corner draft, still drawing', () => {
    const { hook, setPolygon } = setup();
    act(() => hook.result.current.setDrawShape('rectangle'));
    for (const p of [a, b, c]) click(hook, p);
    act(() => hook.result.current.undo());
    expect(hook.result.current.vertices).toEqual([a, b]);
    expect(setPolygon).toHaveBeenLastCalledWith([a, b]);
    expect(hook.result.current.isDrawing).toBe(true);
    // A new width click completes it again.
    click(hook, c);
    expect(hook.result.current.vertices).toEqual(rectangleCorners(a, b, c));
  });

  it('edits a finished rectangle like any polygon', () => {
    const { hook } = setup();
    act(() => hook.result.current.setDrawShape('rectangle'));
    for (const p of [a, b, c]) click(hook, p);
    act(() => hook.result.current.startDraw());
    click(hook, offsetLatLon(a.lat, a.lon, 170, 600));
    expect(hook.result.current.vertices).toHaveLength(5);
    expect(hook.result.current.isDrawing).toBe(true);
  });

  it('a shape change drops a started draft', () => {
    const { hook, setPolygon } = setup();
    act(() => hook.result.current.setDrawShape('rectangle'));
    click(hook, a);
    act(() => hook.result.current.setDrawShape('polygon'));
    expect(hook.result.current.drawShape).toBe('polygon');
    expect(hook.result.current.vertices).toEqual([]);
    expect(setPolygon).toHaveBeenLastCalledWith([]);
    expect(hook.result.current.canUndo).toBe(false);
  });

  it('a shape change keeps a finished zone', () => {
    const { hook } = setup();
    for (const p of [a, b, c]) click(hook, p);
    act(() => hook.result.current.setDrawShape('rectangle'));
    expect(hook.result.current.vertices).toEqual([a, b, c]);
  });

  it('ignores an unknown shape', () => {
    const { hook } = setup();
    act(() => hook.result.current.setDrawShape('circle'));
    expect(hook.result.current.drawShape).toBe('polygon');
  });
});

describe('useDrawing polygon shape', () => {
  it('keeps the free polygon: every click adds a vertex and drawing continues', () => {
    const { hook, analyze } = setup();
    for (const p of [a, b, c]) expect(click(hook, p)).toBe(true);
    expect(hook.result.current.vertices).toEqual([a, b, c]);
    expect(analyze).toHaveBeenCalledExactlyOnceWith([a, b, c]);
    expect(hook.result.current.isDrawing).toBe(true);
  });
});
