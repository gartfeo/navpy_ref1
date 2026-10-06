import { flatDist } from '../../../utils/geo';

const DEFAULT_LIFT_M = 2;
const SAMPLE_DEBOUNCE_MS = 300;
const MAX_SEGMENT_M = 75;

/** True if the scene can render native Cesium ground-clamped polylines
 * (GroundPolylinePrimitive — requires WebGL depth-texture support). */
export function supportsPolylinesOnTerrain(viewer, Cesium) {
  return Cesium.Entity.supportsPolylinesOnTerrain(viewer.scene);
}

/** Normalize a point (either {lon,lat[,alt]} or a Cesium.Cartesian3) to {lon, lat}. */
export function toLonLat(point, Cesium) {
  if (point instanceof Cesium.Cartesian3) {
    const carto = Cesium.Cartographic.fromCartesian(point);
    return {
      lon: Cesium.Math.toDegrees(carto.longitude),
      lat: Cesium.Math.toDegrees(carto.latitude),
    };
  }
  return { lon: point.lon, lat: point.lat };
}

/**
 * Insert intermediate points so consecutive points are at most maxSegmentM
 * apart. Prevents a fallback (non-clamped) line from cutting under terrain
 * between widely-spaced vertices. Does not mutate the input.
 */
export function densifyLonLatPath(points, { maxSegmentM = MAX_SEGMENT_M, loop = false } = {}) {
  if (points.length < 2) return points.slice();
  const pts = loop ? [...points, points[0]] : points;
  const out = [pts[0]];
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1];
    const b = pts[i];
    const segments = Math.max(1, Math.ceil(flatDist(a, b) / maxSegmentM));
    for (let s = 1; s <= segments; s++) {
      const t = s / segments;
      out.push({ lon: a.lon + (b.lon - a.lon) * t, lat: a.lat + (b.lat - a.lat) * t });
    }
  }
  return out;
}

/**
 * Height to render a fallback point at: prefer the live height from the
 * currently-loaded terrain tile, then the debounced sampled-height cache,
 * then sea level — always lifted slightly to avoid z-fighting with terrain.
 */
export function resolveFallbackHeight({ liveHeight, cachedHeight, liftM = DEFAULT_LIFT_M }) {
  const base = Number.isFinite(liveHeight) ? liveHeight
    : Number.isFinite(cachedHeight) ? cachedHeight
    : 0;
  return base + liftM;
}

/** Cache key for a sampled terrain height, rounded to ~0.1m precision. */
export function roundCacheKey(lon, lat) {
  return `${lon.toFixed(6)},${lat.toFixed(6)}`;
}

/** Stable signature for a point set, used to detect "positions didn't
 * actually change" across repeated per-frame evaluations. */
export function positionsSignature(pts) {
  return pts.map((p) => roundCacheKey(p.lon, p.lat)).join('|');
}

const disposers = new WeakMap();

/**
 * Add a polyline that follows the ground: native Cesium clampToGround where
 * the scene supports it, otherwise a terrain-sampled absolute-height
 * fallback so the line still renders instead of silently disappearing
 * (clampToGround alone is not cross-browser reliable).
 *
 * getPositions() is called every frame and may return either {lon,lat}
 * pairs or Cesium.Cartesian3 — return [] for "nothing to draw yet".
 */
export function addGroundPolyline(viewer, Cesium, { getPositions, width, material, loop = false }) {
  const capable = supportsPolylinesOnTerrain(viewer, Cesium);

  function currentLonLat() {
    const raw = getPositions();
    if (!raw || raw.length === 0) return [];
    return raw.map((p) => toLonLat(p, Cesium));
  }

  if (capable) {
    return viewer.entities.add({
      polyline: {
        positions: new Cesium.CallbackProperty(() => {
          const pts = currentLonLat();
          if (pts.length === 0) return [];
          const closed = loop ? [...pts, pts[0]] : pts;
          return closed.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));
        }, false),
        width,
        material,
        clampToGround: true,
      },
    });
  }

  // Fallback: sample + lift instead of clamping.
  const heightCache = new Map();
  let cachedProvider = null;
  let providerGeneration = 0;
  let lastScheduledSignature = null;
  let debounceTimer = null;
  let disposed = false;

  function scheduleSample(pts) {
    if (disposed) return;
    // No terrain provider yet (first ~1s after viewer creation, before Ion
    // terrain replaces the initial undefined provider — see
    // useCesiumViewer.js). Skip without recording a signature so the same
    // unchanged positions are retried once a provider is set.
    if (!viewer.terrainProvider) return;
    // The CallbackProperty below runs every rendered frame even when
    // positions haven't changed (the scene clock ticks continuously). Only
    // (re)schedule when the actual point set changed, otherwise a
    // continuously-rendering scene would keep postponing this forever and
    // sampleTerrainMostDetailed would never fire.
    const signature = positionsSignature(pts);
    if (signature === lastScheduledSignature) return;
    lastScheduledSignature = signature;

    if (debounceTimer) clearTimeout(debounceTimer);
    const generationAtSchedule = providerGeneration;
    debounceTimer = setTimeout(() => {
      const provider = viewer.terrainProvider;
      const cartos = pts.map((p) => Cesium.Cartographic.fromDegrees(p.lon, p.lat));
      Cesium.sampleTerrainMostDetailed(provider, cartos)
        .then((sampled) => {
          // Drop results from a sample kicked off against a terrain provider
          // that has since been swapped out (e.g. OSM placeholder -> Ion
          // terrain at startup) — an in-flight sample resolving after the
          // swap must not repopulate the cache with stale heights.
          if (disposed || generationAtSchedule !== providerGeneration) return;
          sampled.forEach((c, i) => {
            heightCache.set(roundCacheKey(pts[i].lon, pts[i].lat), c.height ?? 0);
          });
          viewer.scene.requestRender();
        })
        .catch(() => {
          // Let a transient sampling failure retry on the next evaluation
          // instead of permanently blocking this position set.
          if (generationAtSchedule === providerGeneration) lastScheduledSignature = null;
        });
    }, SAMPLE_DEBOUNCE_MS);
  }

  const entity = viewer.entities.add({
    polyline: {
      positions: new Cesium.CallbackProperty(() => {
        const pts = currentLonLat();
        if (pts.length === 0) return [];

        // Terrain provider swapped (e.g. OSM placeholder -> Ion terrain at
        // startup) — drop heights sampled against the old provider and force
        // a fresh sample even if positions themselves haven't changed.
        if (viewer.terrainProvider !== cachedProvider) {
          cachedProvider = viewer.terrainProvider;
          heightCache.clear();
          providerGeneration += 1;
          lastScheduledSignature = null;
        }

        const dense = densifyLonLatPath(pts, { loop });
        scheduleSample(dense);
        return dense.map((p) => {
          const carto = Cesium.Cartographic.fromDegrees(p.lon, p.lat);
          const liveHeight = viewer.scene.globe.getHeight(carto);
          const cachedHeight = heightCache.get(roundCacheKey(p.lon, p.lat));
          const height = resolveFallbackHeight({ liveHeight, cachedHeight });
          return Cesium.Cartesian3.fromDegrees(p.lon, p.lat, height);
        });
      }, false),
      width,
      material,
      clampToGround: false,
    },
  });

  disposers.set(entity, () => {
    disposed = true;
    if (debounceTimer) clearTimeout(debounceTimer);
  });

  return entity;
}

/**
 * Remove an entity created by addGroundPolyline. Always use this instead of
 * viewer.entities.remove for these entities so the fallback's debounce
 * timer / in-flight terrain sample can't leak past removal (safe no-op
 * disposer if the entity has none).
 */
export function removeGroundPolyline(viewer, entity) {
  if (!entity) return;
  try {
    disposers.get(entity)?.();
  } finally {
    disposers.delete(entity);
    try { viewer.entities.remove(entity); } catch {}
  }
}
