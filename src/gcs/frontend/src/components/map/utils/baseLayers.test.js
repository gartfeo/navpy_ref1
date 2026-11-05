import { describe, it, expect, vi } from 'vitest';
import { ImageryLayer } from 'cesium';
import {
  BASE_LAYER_KEY, DEFAULT_BASE_LAYER, loadBaseLayer, saveBaseLayer,
  createBaseLayer, applyBaseLayer, watchBaseLayer, layerAfterFailure, secondViewLayer, SECOND_VIEW_FALLBACK_LAYER,
} from './baseLayers';

const memoryStorage = (init = {}) => {
  const data = { ...init };
  return {
    getItem: (k) => (k in data ? data[k] : null),
    setItem: (k, v) => { data[k] = String(v); },
    data,
  };
};

const throwingStorage = {
  getItem: () => { throw new Error('blocked'); },
  setItem: () => { throw new Error('blocked'); },
};
const of = (storage) => () => storage;

// Minimal Cesium stand-in: each constructor records its options.
function fakeCesium() {
  class Rec { constructor(opts) { this.opts = opts; } }
  class ImageryLayer extends Rec {
    constructor(opts) { super(opts); this.errorEvent = { addEventListener: vi.fn() }; }
  }
  ImageryLayer.fromProviderAsync = vi.fn((p) => ({ async: p }));
  return {
    ImageryLayer,
    UrlTemplateImageryProvider: class extends Rec {},
    OpenStreetMapImageryProvider: class extends Rec {},
    IonImageryProvider: { fromAssetId: vi.fn((id) => ({ ionAsset: id })) },
    Credit: class { constructor(html, onScreen) { this.html = html; this.onScreen = onScreen; } },
  };
}

describe('base layer choice persistence', () => {
  it('defaults to terrain when nothing is saved', () => {
    expect(DEFAULT_BASE_LAYER).toBe('terrain');
    expect(loadBaseLayer(true, of(memoryStorage()))).toBe('terrain');
  });

  it('restores a saved layer', () => {
    const s = memoryStorage();
    saveBaseLayer('street', of(s));
    expect(s.data[BASE_LAYER_KEY]).toBe('street');
    expect(loadBaseLayer(false, of(s))).toBe('street');
  });

  it('falls back to the default for satellite without an Ion token', () => {
    const s = memoryStorage({ [BASE_LAYER_KEY]: 'satellite' });
    expect(loadBaseLayer(true, of(s))).toBe('satellite');
    expect(loadBaseLayer(false, of(s))).toBe('terrain');
  });

  it('ignores unknown saved values and blocked storage', () => {
    expect(loadBaseLayer(true, of(memoryStorage({ [BASE_LAYER_KEY]: 'bogus' })))).toBe('terrain');
    expect(loadBaseLayer(true, of(throwingStorage))).toBe('terrain');
    expect(() => saveBaseLayer('street', of(throwingStorage))).not.toThrow();
  });

  it('survives window.localStorage itself throwing (blocked site data)', () => {
    const getter = vi.spyOn(globalThis, 'localStorage', 'get')
      .mockImplementation(() => { throw new DOMException('denied', 'SecurityError'); });
    expect(loadBaseLayer(true)).toBe('terrain');
    expect(() => saveBaseLayer('street')).not.toThrow();
    expect(getter).toHaveBeenCalled();
  });
});

describe('createBaseLayer', () => {
  it('terrain is OpenTopoMap with its CC-BY-SA credit on screen', () => {
    const layer = createBaseLayer(fakeCesium(), 'terrain');
    const provider = layer.opts;
    expect(provider.opts.url).toBe('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png');
    expect(provider.opts.maximumLevel).toBe(17);
    expect(provider.opts.credit.html).toContain('OpenTopoMap');
    expect(provider.opts.credit.html).toContain('CC-BY-SA');
    expect(provider.opts.credit.html).toContain('OpenStreetMap');
    expect(provider.opts.credit.onScreen).toBe(true);
  });

  it('street is OpenStreetMap with its credit on screen', () => {
    const provider = createBaseLayer(fakeCesium(), 'street').opts;
    expect(provider.opts.url).toBe('https://tile.openstreetmap.org/');
    expect(provider.opts.credit.html).toContain('OpenStreetMap');
    expect(provider.opts.credit.onScreen).toBe(true);
  });

  it('satellite is Ion imagery asset 3, loaded async', () => {
    const C = fakeCesium();
    const layer = createBaseLayer(C, 'satellite');
    expect(C.IonImageryProvider.fromAssetId).toHaveBeenCalledWith(3);
    expect(layer).toEqual({ async: { ionAsset: 3 } });
  });
});

describe('base layer load failure', () => {
  const rejected = () => {
    let reject;
    const promise = new Promise((_, r) => { reject = r; });
    return { promise, reject };
  };

  it('reports a provider that fails to load (real Cesium ImageryLayer)', async () => {
    const d = rejected();
    const onFailed = vi.fn();
    watchBaseLayer(ImageryLayer.fromProviderAsync(d.promise), onFailed);
    d.reject(new Error('401'));
    await vi.waitFor(() => expect(onFailed).toHaveBeenCalledTimes(1));
    expect(onFailed.mock.calls[0][0].message).toBe('401');
  });

  it('stays quiet for a layer already swapped out', async () => {
    const d = rejected();
    const onFailed = vi.fn();
    const layer = watchBaseLayer(ImageryLayer.fromProviderAsync(d.promise), onFailed);
    layer.destroy();
    d.reject(new Error('401'));
    await new Promise((r) => setTimeout(r, 0));
    expect(onFailed).not.toHaveBeenCalled();
  });

  it('falls back to the default only while the failed layer is selected', () => {
    expect(layerAfterFailure('satellite', 'satellite')).toBe(DEFAULT_BASE_LAYER);
    expect(layerAfterFailure('street', 'satellite')).toBe('street');
    expect(layerAfterFailure('terrain', 'terrain')).toBe('terrain');
  });
});

describe('applyBaseLayer', () => {
  it('replaces only the bottom imagery layer and requests a render', () => {
    const stack = ['old-base', 'overlay'];
    const viewer = {
      imageryLayers: {
        get length() { return stack.length; },
        get: (i) => stack[i],
        remove: vi.fn((l) => { stack.splice(stack.indexOf(l), 1); }),
        add: vi.fn((l, i) => { stack.splice(i, 0, l); }),
      },
      scene: { requestRender: vi.fn() },
    };
    applyBaseLayer(fakeCesium(), viewer, 'street', vi.fn());
    expect(viewer.imageryLayers.remove).toHaveBeenCalledWith('old-base', true);
    expect(stack[0].errorEvent.addEventListener).toHaveBeenCalled();
    expect(stack).toHaveLength(2);
    expect(stack[0].opts.opts.url).toBe('https://tile.openstreetmap.org/');
    expect(stack[1]).toBe('overlay');
    expect(viewer.scene.requestRender).toHaveBeenCalled();
  });
});

describe("secondViewLayer (the vehicle window's Synthetic)", () => {
  it('follows the map, except Satellite, whose Bing credits show in one view only: terrain there', () => {
    expect(secondViewLayer('terrain')).toBe('terrain');
    expect(secondViewLayer('street')).toBe('street');
    expect(secondViewLayer('satellite')).toBe(SECOND_VIEW_FALLBACK_LAYER);
    expect(SECOND_VIEW_FALLBACK_LAYER).toBe('terrain');
  });
});
