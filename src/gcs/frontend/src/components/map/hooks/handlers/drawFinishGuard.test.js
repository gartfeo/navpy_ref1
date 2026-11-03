import { expect, it } from 'vitest';
import { createDrawFinishGuard } from './drawFinishGuard';

const a = { lat: 40.18, lon: 44.51 };
const b = { lat: 40.183, lon: 44.524 };
const rect = [a, b, { lat: 40.177, lon: 44.527 }, { lat: 40.174, lon: 44.513 }];
// Callbacks as seen at a LEFT_CLICK: the width click, then any click after it.
const widthClick = { editable: true, isDrawing: true, drawShape: 'rectangle', polygon: [a, b] };
const afterFinish = { editable: true, isDrawing: false, drawShape: 'rectangle', polygon: rect };

it('drops the double-click whose first click finished the rectangle', () => {
  const guard = createDrawFinishGuard();
  guard.onClick(widthClick);
  guard.onClick(afterFinish);
  expect(guard.skipDoubleClick()).toBe(true);
  // Only that one.
  guard.onClick(afterFinish);
  guard.onClick(afterFinish);
  expect(guard.skipDoubleClick()).toBe(false);
});

it('keeps a later double-click on a corner (vertex delete still works)', () => {
  const guard = createDrawFinishGuard();
  guard.onClick(widthClick);
  guard.onClick(afterFinish); // a single click elsewhere
  guard.onClick(afterFinish); // then a double-click: its two clicks
  guard.onClick(afterFinish);
  expect(guard.skipDoubleClick()).toBe(false);
});

it('never skips in the polygon flow', () => {
  const guard = createDrawFinishGuard();
  const poly = { ...widthClick, drawShape: 'polygon' };
  guard.onClick(poly);
  guard.onClick(poly);
  expect(guard.skipDoubleClick()).toBe(false);
  expect(createDrawFinishGuard().skipDoubleClick()).toBe(false);
});

it('a drag-ending click counts in the sequence but never finishes', () => {
  const guard = createDrawFinishGuard();
  guard.onClick(widthClick, { handled: false });
  guard.onClick(afterFinish, { handled: false });
  expect(guard.skipDoubleClick()).toBe(false);
  guard.onClick(widthClick);
  guard.onClick(afterFinish, { handled: false });
  expect(guard.skipDoubleClick()).toBe(true);
});
