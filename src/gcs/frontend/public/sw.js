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
    event.respondWith(
      caches.open(STATIC_CACHE).then((cache) =>
        cache.match(event.request).then((cached) => {
          if (cached) return cached;
          return fetch(event.request).then((response) => {
            if (response.ok) {
              cache.put(event.request, response.clone());
            }
            return response;
          });
        })
      ).catch(() => caches.match(event.request))
    );
    return;
  }

  // Tile requests: network-first with cache fallback
  if (isTileRequest(url)) {
    event.respondWith(
      fetch(event.request)
        .then((response) => {
          if (response.ok) {
            const clone = response.clone();
            caches.open(TILE_CACHE).then((cache) => {
              cache.put(event.request, clone);
              evictOldEntries(TILE_CACHE, MAX_TILE_ENTRIES);
            });
          }
          return response;
        })
        .catch(() => caches.match(event.request))
    );
    return;
  }

  // All other requests: default browser behavior
});
