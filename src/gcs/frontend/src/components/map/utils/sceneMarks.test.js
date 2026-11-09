// The Synthetic view's marks (sceneMarks.js) with a fake Cesium.
import { describe, expect, it } from 'vitest';
import { createSceneMarks } from './sceneMarks';

function fakeCesium() {
  class Collection {
    constructor(opts) { this.opts = opts; this.items = []; }
    add(o) { const item = { ...o }; this.items.push(item); return item; }
    removeAll() { this.items = []; }
  }
  return {
    BillboardCollection: class extends Collection {},
    PointPrimitiveCollection: class extends Collection {},
    LabelCollection: class extends Collection {},
    Cartesian3: { fromDegrees: (lon, lat, h) => ({ lon, lat, h }) },
    Cartesian2: class { constructor(x, y) { this.x = x; this.y = y; } },
    Color: { fromCssColorString: (css) => ({ css }), WHITE: 'white', BLACK: 'black' },
    LabelStyle: { FILL_AND_OUTLINE: 'fill+outline', FILL: 'fill' },
    HeightReference: { CLAMP_TO_GROUND: 'clamp' },
  };
}
function fakeScene() {
  const list = [];
  return { list, primitives: { add: (c) => { list.push(c); return c; }, remove: (c) => list.splice(list.indexOf(c), 1) } };
}
const dock = { lat: 40.3, lon: 44.4 };
const hub = { lat: 40.2, lon: 44.5, type: 'building' };

describe('createSceneMarks', () => {
  it('the dock highlighted with its distance and the hub, clamped to the ground over the terrain', () => {
    const scene = fakeScene();
    const marks = createSceneMarks(fakeCesium(), scene);
    expect(marks.update({ dock, dockText: 'Dock · 420 m', hub, hubText: 'DDH', fleet: [] })).toBe(true);
    const [billboards, labels] = scene.list;
    expect(billboards.items).toHaveLength(2);
    expect(labels.items.map((l) => l.text)).toEqual(['DDH', 'Dock · 420 m']);
    expect(labels.items[1].showBackground).toBe(true);
    for (const m of [...billboards.items, ...labels.items]) {
      expect(m.heightReference).toBe('clamp');
      expect(m.disableDepthTestDistance).toBe(Number.POSITIVE_INFINITY);
    }
    // The collections clamp through the scene.
    expect(billboards.opts.scene).toBe(scene);
  });

  it('a new distance only changes the label; the same marks change nothing', () => {
    const scene = fakeScene();
    const marks = createSceneMarks(fakeCesium(), scene);
    marks.update({ dock, dockText: 'Dock · 420 m', hub: null, fleet: [] });
    const [billboards, labels] = scene.list;
    const label = labels.items[0];
    expect(marks.update({ dock, dockText: 'Dock · 400 m', hub: null, fleet: [] })).toBe(true);
    expect(labels.items[0]).toBe(label);
    expect(label.text).toBe('Dock · 400 m');
    expect(billboards.items).toHaveLength(1);
    expect(marks.update({ dock, dockText: 'Dock · 400 m', hub: null, fleet: [] })).toBe(false);
  });

  it('each other fleet UAV as a dot in its zone colour with its name', () => {
    const scene = fakeScene();
    const marks = createSceneMarks(fakeCesium(), scene);
    const position = { x: 1, y: 2, z: 3 };
    marks.update({ dock: null, hub: null, fleet: [{ position, label: 's1-u2', zone: 1 }] });
    const [, , dots, labels] = scene.list;
    expect(dots.items).toHaveLength(1);
    expect(dots.items[0].position).toBe(position);
    expect(labels.items[0].text).toBe('s1-u2');
    expect(labels.items[0].fillColor).toEqual(dots.items[0].color);
    marks.update({ dock: null, hub: null, fleet: [] });
    expect(dots.items).toHaveLength(0);
  });

  it('destroy removes its collections', () => {
    const scene = fakeScene();
    const marks = createSceneMarks(fakeCesium(), scene);
    marks.update({ dock, dockText: 'Dock · 1 m', hub, hubText: 'DDH', fleet: [{ position: { x: 1, y: 2, z: 3 }, label: 'u2', zone: 0 }] });
    expect(scene.list).toHaveLength(4);
    expect(scene.list.every((c) => c.items.length > 0)).toBe(true);
    marks.destroy();
    expect(scene.list).toHaveLength(0);
  });
});
