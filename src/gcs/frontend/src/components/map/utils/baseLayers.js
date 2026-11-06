/**
 * Base map (2D imagery) layers the operator can pick, like Google Maps' layer
 * switch. The choice changes imagery only; the 3D terrain elevation (Cesium
 * World Terrain, when an Ion token is configured) is loaded separately.
 *
 * Sources and their terms:
 * - terrain: OpenTopoMap, CC-BY-SA, free incl. commercial use with the credit
 *   below (https://opentopomap.org/about). Server-rendered to zoom 17.
 * - satellite: Cesium Ion imagery asset 3 (Bing aerial with labels), only with
 *   a configured Ion token; Ion adds its own credits.
 * - street: OpenStreetMap standard tiles, ODbL data, credit required
 *   (https://operations.osmfoundation.org/policies/tiles/).
 */

export const BASE_LAYER_KEY = 'gcs-base-layer';
export const DEFAULT_BASE_LAYER = 'terrain';

export const BASE_LAYERS = [
  { id: 'terrain', labelKey: 'mapLayers.terrain', needsIon: false },
  { id: 'satellite', labelKey: 'mapLayers.satellite', needsIon: true },
  { id: 'street', labelKey: 'mapLayers.street', needsIon: false },
];

const OSM_CREDIT = '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors';
const OPENTOPOMAP_CREDIT = `Map data: ${OSM_CREDIT}, SRTM | Map style: © `
  + '<a href="https://opentopomap.org">OpenTopoMap</a> '
  + '(<a href="https://creativecommons.org/licenses/by-sa/3.0/">CC-BY-SA</a>)';

export function isBaseLayerAvailable(id, hasIon) {
  const layer = BASE_LAYERS.find((l) => l.id === id);
  return !!layer && (!layer.needsIon || hasIon);
}

/** The layer a second view (the vehicle window's Synthetic) shows: the map's, or the default when unavailable. */
export function syntheticBaseLayer(id, hasIon) {
  return isBaseLayerAvailable(id, hasIon) ? id : DEFAULT_BASE_LAYER;
}

/**
 * Give `provider` its own copy of each tile credit. Cesium caches Bing
 * metadata, and with it each attribution's Credit, across provider instances
 * (BingMapsImageryProvider requestMetadata/build), and a Credit owns one DOM
 * element that each CreditDisplay re-parents. Two views on Satellite then
 * took each other's credits (todo 275). Cesium clones its own logo credit per
 * display the same way (CreditDisplay constructor, Credit.clone).
 */
export function withOwnTileCredits(Cesium, provider) {
  const copies = new Map();
  const shared = provider.getTileCredits.bind(provider);
  provider.getTileCredits = (x, y, level) => shared(x, y, level)?.map((credit) => {
    let copy = copies.get(credit);
    if (!copy) {
      copy = Cesium.Credit.clone(credit);
      copies.set(credit, copy);
    }
    return copy;
  });
  return provider;
}

// Reading window.localStorage itself can throw (blocked site data), so the
// lookup stays inside the callers' try blocks.
const browserStorage = () => globalThis.localStorage;

/** The operator's saved layer, or the default when unset, unknown or unavailable. */
export function loadBaseLayer(hasIon, getStorage = browserStorage) {
  try {
    const saved = getStorage()?.getItem(BASE_LAYER_KEY);
    if (isBaseLayerAvailable(saved, hasIon)) return saved;
  } catch {}
  return DEFAULT_BASE_LAYER;
}

export function saveBaseLayer(id, getStorage = browserStorage) {
  try {
    getStorage()?.setItem(BASE_LAYER_KEY, id);
  } catch {}
}

/**
 * Layer to show after `failedId` failed to load while `current` is selected:
 * the default, unless the operator already moved on (or the default failed).
 */
export function layerAfterFailure(current, failedId) {
  return current === failedId && failedId !== DEFAULT_BASE_LAYER ? DEFAULT_BASE_LAYER : current;
}

/** Build the Cesium ImageryLayer for a layer id (unknown ids get the default). */
export function createBaseLayer(Cesium, id) {
  switch (id) {
    case 'satellite':
      return Cesium.ImageryLayer.fromProviderAsync(Cesium.IonImageryProvider.fromAssetId(3)
        .then((provider) => withOwnTileCredits(Cesium, provider)));
    case 'street':
      return new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({
        url: 'https://tile.openstreetmap.org/',
        credit: new Cesium.Credit(OSM_CREDIT, true),
      }));
    default:
      return new Cesium.ImageryLayer(new Cesium.UrlTemplateImageryProvider({
        url: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
        subdomains: ['a', 'b', 'c'],
        maximumLevel: 17,
        credit: new Cesium.Credit(OPENTOPOMAP_CREDIT, true),
      }));
  }
}

/**
 * Call `onFailed` if the layer's provider fails to load (e.g. the Ion asset
 * request is refused), unless the layer was already swapped out.
 */
export function watchBaseLayer(layer, onFailed) {
  layer.errorEvent.addEventListener((err) => {
    if (!layer.isDestroyed()) onFailed(err);
  });
  return layer;
}

/** Replace the bottom (base) imagery layer; other imagery layers stay. */
export function applyBaseLayer(Cesium, viewer, id, onFailed) {
  const layers = viewer.imageryLayers;
  if (layers.length > 0) layers.remove(layers.get(0), true);
  layers.add(watchBaseLayer(createBaseLayer(Cesium, id), onFailed), 0);
  viewer.scene.requestRender();
}
