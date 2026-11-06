import { describe, it, expect, vi } from 'vitest';
import { ImageryLayer, Credit } from 'cesium';
import {
  BASE_LAYER_KEY, DEFAULT_BASE_LAYER, loadBaseLayer, saveBaseLayer,
  createBaseLayer, applyBaseLayer, watchBaseLayer, layerAfterFailure, syntheticBaseLayer,
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
    IonImageryProvider: { fromAssetId: vi.fn((id) => Promise.resolve({ ionAsset: id, getTileCredits: () => undefined })) },
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

  it('satellite is Ion imagery asset 3, loaded async', async () => {
    const C = fakeCesium();
    const layer = createBaseLayer(C, 'satellite');
    expect(C.IonImageryProvider.fromAssetId).toHaveBeenCalledWith(3);
    expect(await layer.async).toMatchObject({ ionAsset: 3 });
  });

  it('gives each satellite layer its own tile credits, so two views keep theirs (real Cesium Credit)', async () => {
    // Cesium caches Bing metadata across providers, so every instance
    // returns the same Credit objects, each owning one DOM element.
    const bing = [new Credit('© Microsoft'), new Credit('© Maxar')];
    const C = {
      ...fakeCesium(),
      Credit,
      IonImageryProvider: { fromAssetId: vi.fn(async () => ({ getTileCredits: () => bing.slice() })) },
    };
    const mapProvider = await createBaseLayer(C, 'satellite').async;
    const syntheticProvider = await createBaseLayer(C, 'satellite').async;
    const onMap = mapProvider.getTileCredits(1, 2, 3);
    const onSynthetic = syntheticProvider.getTileCredits(1, 2, 3);
    expect(onMap.map((c) => c.html)).toEqual(['© Microsoft', '© Maxar']);
    expect(onSynthetic.map((c) => c.html)).toEqual(['© Microsoft', '© Maxar']);
    onMap.forEach((c, i) => {
      expect(c).not.toBe(onSynthetic[i]);
      expect(c.element).not.toBe(onSynthetic[i].element);
      expect(c).not.toBe(bing[i]);
    });
    // Stable per view, so its credit display does not churn.
    expect(mapProvider.getTileCredits(4, 5, 6)[0]).toBe(onMap[0]);
  });

  it('passes no tile credits through as none', async () => {
    const C = { ...fakeCesium(), Credit };
    const provider = await createBaseLayer(C, 'satellite').async;
    expect(provider.getTileCredits(0, 0, 0)).toBeUndefined();
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

describe("syntheticBaseLayer (the vehicle window's Synthetic)", () => {
  it("shows the map's layer, Satellite included", () => {
    expect(syntheticBaseLayer('terrain', true)).toBe('terrain');
    expect(syntheticBaseLayer('street', false)).toBe('street');
    expect(syntheticBaseLayer('satellite', true)).toBe('satellite');
  });

  it('shows the default for a layer the map cannot show', () => {
    expect(syntheticBaseLayer('satellite', false)).toBe(DEFAULT_BASE_LAYER);
    expect(syntheticBaseLayer('bogus', true)).toBe(DEFAULT_BASE_LAYER);
  });
});
