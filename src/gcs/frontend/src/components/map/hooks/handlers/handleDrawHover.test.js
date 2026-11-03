import { beforeEach, expect, it, vi } from 'vitest';
import { handleDrawHover } from './handleDrawHover';
import { pickCartographic } from '../../utils/entityPicking';
import { rectanglePreview } from '../../../../utils/rectangleDraw';

vi.mock('../../utils/entityPicking', () => ({ pickCartographic: vi.fn() }));

const a = { lat: 40.18, lon: 44.51 };
const b = { lat: 40.183, lon: 44.524 };
const cursor = { lat: 40.177, lon: 44.527 };
const movement = { endPosition: { x: 5, y: 6 } };
let viewer;

beforeEach(() => {
  pickCartographic.mockReset();
  pickCartographic.mockReturnValue(cursor);
  viewer = { scene: { requestRender: vi.fn() } };
});

const cbs = (over) => ({ current: { editable: true, isDrawing: true, drawShape: 'rectangle', polygon: [a, b], ...over } });

it('previews the rectangle the next click would make', () => {
  const previewRef = { current: [] };
  handleDrawHover('Cesium', viewer, movement, cbs(), previewRef);
  expect(pickCartographic).toHaveBeenCalledWith('Cesium', viewer, movement.endPosition);
  expect(previewRef.current).toEqual(rectanglePreview([a, b], cursor));
  expect(previewRef.current).toHaveLength(4);
  expect(viewer.scene.requestRender).toHaveBeenCalled();
});

it('previews nothing for the polygon shape or a finished zone, and clears an old preview', () => {
  for (const over of [{ drawShape: 'polygon' }, { polygon: [a, b, cursor] }, { isDrawing: false }, { editable: false }]) {
    const previewRef = { current: [a, b] };
    viewer.scene.requestRender.mockClear();
    handleDrawHover('Cesium', viewer, movement, cbs(over), previewRef);
    expect(previewRef.current).toEqual([]);
    expect(viewer.scene.requestRender).toHaveBeenCalledOnce();
  }
  expect(pickCartographic).not.toHaveBeenCalled();
});
