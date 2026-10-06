/**
 * Per-entity terrain height cache.
 *
 * Samples Cesium terrain once per significant horizontal move (>~55 m)
 * and caches the ellipsoidal height so callers can position entities at
 * `terrainHeight + altitudeAGL` without async latency on every frame.
 */

/** Minimum horizontal movement (degrees) before re-sampling terrain. ~55 m. */
const RESAMPLE_DEG = 0.0005;
/** Maximum cache age (ms) before forcing a re-sample. */
const RESAMPLE_AGE_MS = 10_000;

/**
 * Decide whether terrain needs re-sampling for a given position.
 * @param {object|undefined} cached  Previous cache entry {lat, lon, terrainHeight, ts}
 * @param {number} lat               Current latitude (degrees)
 * @param {number} lon               Current longitude (degrees)
 * @param {number} now               Current timestamp (ms)
 * @returns {boolean}
 */
export function needsTerrainSample(cached, lat, lon, now) {
  if (!cached || cached.terrainHeight == null) return true;
  if ((now - cached.ts) > RESAMPLE_AGE_MS) return true;
  if (Math.abs(cached.lat - lat) > RESAMPLE_DEG) return true;
  if (Math.abs(cached.lon - lon) > RESAMPLE_DEG) return true;
  return false;
}

/**
 * Request a terrain height sample and update the cache.
 * The next render cycle picks up the corrected height automatically,
 * producing a smooth transition instead of a sudden jump.
 *
 * @param {object}       Cesium  CesiumJS namespace
 * @param {object}       viewer  Cesium Viewer instance
 * @param {object}       cache   Mutable cache object (keyed by id)
 * @param {number|string} id     Entity identifier (e.g. sys_id)
 * @param {number}       lat     Latitude (degrees)
 * @param {number}       lon     Longitude (degrees)
 */
export function sampleTerrain(Cesium, viewer, cache, id, lat, lon) {
  const now = Date.now();
  const prev = cache[id];
  // Stamp cache immediately to prevent duplicate in-flight requests
  cache[id] = { lat, lon, terrainHeight: prev?.terrainHeight ?? 0, ts: now };

  const carto = Cesium.Cartographic.fromDegrees(lon, lat);
  Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, [carto])
    .then(([sampled]) => {
      cache[id] = { lat, lon, terrainHeight: sampled.height || 0, ts: Date.now() };
    })
    .catch(() => {
      // Remove stamped entry so next update retries
      delete cache[id];
    });
}
