// The first-person views' route drawing (routeOverlay.js) with a fake Cesium.
import { describe, it, expect } from 'vitest';
import { createRouteOverlay } from './routeOverlay';

function fakeCesium() {
  class Collection {
    constructor() { this.items = []; }
    add(o) { this.items.push(o); return o; }
    removeAll() { this.items = []; }
  }
  return {
    PolylineCollection: class extends Collection {},
    PointPrimitiveCollection: class extends Collection {},
    LabelCollection: class extends Collection {},
    Color: { fromCssColorString: (css) => ({ css, withAlpha: (a) => ({ css, a }) }), WHITE: 'white', BLACK: 'black' },
    Material: { fromType: (type, u) => ({ type, ...u }) },
    LabelStyle: { FILL_AND_OUTLINE: 'fill+outline' },
    Cartesian2: class { constructor(x, y) { this.x = x; this.y = y; } },
  };
}
function fakeScene() {
  const list = [];
  return { list, primitives: { add: (c) => { list.push(c); return c; }, remove: (c) => list.splice(list.indexOf(c), 1) } };
}
const route = { zone: 0, positions: ['a', 'b', 'c'], track: [{}, {}, {}], next: 1 };

describe('createRouteOverlay', () => {
  it('a line, a dot per waypoint (the next one larger) and every number', () => {
    const scene = fakeScene();
    const overlay = createRouteOverlay(fakeCesium(), scene);
    expect(overlay.update(route, { line: true, labels: 'all' })).toBe(true);
    const [lines, dots, labels] = scene.list;
    expect(lines.items).toHaveLength(1);
    expect(lines.items[0].positions).toBe(route.positions);
    expect(dots.items.map((d) => d.pixelSize)).toEqual([8, 13, 8]);
    expect(labels.items.map((l) => l.text)).toEqual(['1', '2', '3']);
    // Over the terrain, like the main map's track markers.
    for (const d of [...dots.items, ...labels.items]) expect(d.disableDepthTestDistance).toBe(Number.POSITIVE_INFINITY);
  });

  it("'next' numbers only the next waypoint; no line when the host draws its own", () => {
    const scene = fakeScene();
    const overlay = createRouteOverlay(fakeCesium(), scene);
    overlay.update(route, { line: false, labels: 'next' });
    const [lines, dots, labels] = scene.list;
    expect(lines.items).toHaveLength(0);
    expect(dots.items).toHaveLength(3);
    expect(labels.items.map((l) => l.text)).toEqual(['2']);
  });

  it('redraws only when the route, its next waypoint or the options change', () => {
    const scene = fakeScene();
    const overlay = createRouteOverlay(fakeCesium(), scene);
    const opts = { line: true, labels: 'all' };
    overlay.update(route, opts);
    expect(overlay.update({ ...route }, opts)).toBe(false);
    expect(overlay.update({ ...route, next: 2 }, opts)).toBe(true);
    expect(overlay.update({ ...route, next: 2, positions: ['x', 'y', 'z'] }, opts)).toBe(true);
    expect(overlay.update(null, opts)).toBe(true);
    expect(scene.list.every((c) => c.items.length === 0)).toBe(true);
    expect(overlay.update(null, opts)).toBe(false);
  });

  it('destroy removes its collections from the scene', () => {
    const scene = fakeScene();
    createRouteOverlay(fakeCesium(), scene).destroy();
    expect(scene.list).toHaveLength(0);
  });
});
