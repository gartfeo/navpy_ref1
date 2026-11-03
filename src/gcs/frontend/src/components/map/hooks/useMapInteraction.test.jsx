/**
 * Click wiring around the rectangle's finishing click: the double-click that
 * finishes a rectangle must not delete a corner, also when its second click
 * lands on the new corner and starts a vertex drag; a later corner
 * double-click still deletes it.
 */
import { renderHook } from '@testing-library/react';
import { beforeEach, describe, expect, it, vi } from 'vitest';
import useMapInteraction from './useMapInteraction';
import { handleClick } from './handlers/handleClick';
import { handleDoubleClick } from './handlers/handleDoubleClick';
import { handleDragStart } from './handlers/handleDragStart';
import { handleDragEnd } from './handlers/handleDragEnd';

vi.mock('./handlers/handleClick', () => ({ handleClick: vi.fn() }));
vi.mock('./handlers/handleDoubleClick', () => ({ handleDoubleClick: vi.fn() }));
vi.mock('./handlers/handleDragStart', () => ({ handleDragStart: vi.fn() }));
vi.mock('./handlers/handleDragMove', () => ({ handleDragMove: vi.fn() }));
vi.mock('./handlers/handleDragEnd', () => ({ handleDragEnd: vi.fn() }));
vi.mock('./handlers/handleDrawHover', () => ({ handleDrawHover: vi.fn() }));

const a = { lat: 40.18, lon: 44.51 };
const b = { lat: 40.183, lon: 44.524 };
const rect = [a, b, { lat: 40.177, lon: 44.527 }, { lat: 40.174, lon: 44.513 }];
const drafting = { editable: true, isDrawing: true, drawShape: 'rectangle', polygon: [a, b] };
const finished = { editable: true, isDrawing: false, drawShape: 'rectangle', polygon: rect };

let actions;
const Cesium = {
  ScreenSpaceEventType: {
    LEFT_CLICK: 'click', LEFT_DOUBLE_CLICK: 'dbl', LEFT_DOWN: 'down', MOUSE_MOVE: 'move', LEFT_UP: 'up',
  },
  ScreenSpaceEventHandler: class {
    setInputAction(fn, type) { actions[type] = fn; }
    destroy() {}
  },
};

function mount() {
  const callbacksRef = { current: drafting };
  renderHook(() => useMapInteraction(
    { current: Cesium }, { current: { scene: { canvas: {} } } }, { current: true }, callbacksRef, { drawPreviewRef: { current: [] } },
  ));
  return callbacksRef;
}

// One mouse click; onCorner: LEFT_DOWN hits a vertex and starts a drag.
function click(onCorner = false) {
  handleDragStart.mockReturnValueOnce(onCorner ? { dragState: { type: 'vertex' }, skipNextClick: true } : null);
  actions.down({ position: {} });
  handleDragEnd.mockReturnValueOnce(onCorner);
  actions.up({});
  actions.click({ position: {} });
}

beforeEach(() => {
  actions = {};
  vi.clearAllMocks();
});

describe('useMapInteraction finishing double-click', () => {
  it('drops the double-click whose first click finished the rectangle, second click on the new corner', () => {
    const cbs = mount();
    handleClick.mockImplementationOnce(() => { cbs.current = finished; });
    click();
    click(true);
    actions.dbl({ position: {} });
    expect(handleClick).toHaveBeenCalledOnce();
    expect(handleDoubleClick).not.toHaveBeenCalled();
  });

  it('drops it with the second click off the corners too', () => {
    const cbs = mount();
    handleClick.mockImplementationOnce(() => { cbs.current = finished; });
    click();
    click();
    actions.dbl({ position: {} });
    expect(handleDoubleClick).not.toHaveBeenCalled();
  });

  it('keeps a later corner double-click after a blank click', () => {
    const cbs = mount();
    handleClick.mockImplementationOnce(() => { cbs.current = finished; });
    click();
    click();
    click(true);
    click(true);
    actions.dbl({ position: {} });
    expect(handleDoubleClick).toHaveBeenCalledOnce();
  });

  it('keeps a corner double-click right after finishing with a single click', () => {
    const cbs = mount();
    handleClick.mockImplementationOnce(() => { cbs.current = finished; });
    click();
    click(true);
    click(true);
    actions.dbl({ position: {} });
    expect(handleDoubleClick).toHaveBeenCalledOnce();
  });

  it('never drops a polygon double-click', () => {
    const cbs = mount();
    cbs.current = { ...finished, drawShape: 'polygon', isDrawing: true, polygon: [a, b] };
    click(true);
    click(true);
    actions.dbl({ position: {} });
    expect(handleDoubleClick).toHaveBeenCalledOnce();
  });

  it('a drag-ending click on a draft corner is not a finishing click', () => {
    mount();
    click(true);
    click(true);
    actions.dbl({ position: {} });
    expect(handleDoubleClick).toHaveBeenCalledOnce();
  });
});
