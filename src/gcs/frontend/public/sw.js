// Service Worker — offline tile cache + static asset cache
const CACHE_VERSION = 'gcs-v1';
const TILE_CACHE = `tiles-${CACHE_VERSION}`;
const STATIC_CACHE = `static-${CACHE_VERSION}`;

const MAX_TILE_ENTRIES = 2000;

// Tile URL patterns (network-first, cache fallback)
const TILE_PATTERNS = [
  /tile\.openstreetmap\.org/,
  /\.virtualearth\.net/,
  /assets\.ion\.cesium\.com/,
  /api\.cesium\.com/,
];

// Static asset patterns (cache-first — fingerprinted/immutable)
// Only Cesium library files — NOT /assets/ (Vite app bundles change on rebuild)
const STATIC_PATTERNS = [
  /\/Cesium\//,
];

function isTileRequest(url) {
  return TILE_PATTERNS.some((re) => re.test(url));
}

function isStaticAsset(url) {
  return STATIC_PATTERNS.some((re) => re.test(url));
}

// LRU eviction: delete oldest entries when cache exceeds limit
async function evictOldEntries(cacheName, maxEntries) {
  const cache = await caches.open(cacheName);
  const keys = await cache.keys();
  if (keys.length <= maxEntries) return;
  const toDelete = keys.length - maxEntries;
  for (let i = 0; i < toDelete; i++) {
    await cache.delete(keys[i]);
  }
}

// Cache-first for static assets. A failing cache step falls back to the
// network: CacheStorage can reject (Chrome: "Unexpected internal error" on a
// profile it cannot write), and answering undefined then would turn every
// /Cesium/ file into a network error (todo 118). A network failure after a
// cache miss stays a network error.
async function cacheFirst(event) {
  let cache = null;
  try {
    cache = await caches.open(STATIC_CACHE);
    const cached = await cache.match(event.request);
    if (cached) return cached;
  } catch {
    cache = null;
  }
  const response = await fetch(event.request);
  if (cache && response.ok) {
    event.waitUntil(cache.put(event.request, response.clone()).catch(() => {}));
  }
  return response;
}

// Install — pre-cache nothing, just activate immediately
self.addEventListener('install', (event) => {
  self.skipWaiting();
});

// Activate — clean up old caches from previous versions
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((names) =>
      Promise.all(
        names
          .filter((n) => n !== TILE_CACHE && n !== STATIC_CACHE)
          .map((n) => caches.delete(n))
      )
    ).then(() => self.clients.claim())
  );
});

// Fetch — route to appropriate cache strategy
self.addEventListener('fetch', (event) => {
  const url = event.request.url;

  // Static assets: cache-first (fingerprinted, immutable)
  if (isStaticAsset(url)) {
    event.respondWith(cacheFirst(event));
    return;
  }

  // Tile requests: network-first with cache fallback; a cache failure never
  // reaches the page, and an uncached tile offline is a network error.
  if (isTileRequest(url)) {
    event.respondWith(
      fetch(event.request)
        .then((response) => {
          if (response.ok) {
            const clone = response.clone();
            event.waitUntil(
              caches.open(TILE_CACHE)
                .then((cache) => cache.put(event.request, clone))
                .then(() => evictOldEntries(TILE_CACHE, MAX_TILE_ENTRIES))
                .catch(() => {})
            );
          }
          return response;
        })
        .catch(() => caches.match(event.request).then((cached) => cached || Response.error()))
    );
    return;
  }

  // All other requests: default browser behavior
});
