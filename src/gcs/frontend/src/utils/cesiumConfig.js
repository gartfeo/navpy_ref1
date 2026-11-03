/**
 * Runtime configuration for the Cesium Ion layers (satellite imagery, 3D
 * terrain). The backend serves the access token from outside the repository
 * (GET /api/map-config; sources in gcs/backend/map_config.py), so every clone
 * and worktree on a machine gets it without a rebuild. The repository is
 * public: the token is never committed or built into the bundle.
 */

// The map waits at most this long for the optional Ion upgrade before it
// starts without it (a hung backend must not hold the map back).
export const MAP_CONFIG_TIMEOUT_MS = 5000;

/** @returns {string|null} the token in a /api/map-config body, or null. */
export function parseIonToken(config) {
  const raw = config?.cesium_ion_token;
  if (typeof raw !== 'string') return null;
  const trimmed = raw.trim();
  return trimmed === '' ? null : trimmed;
}

/** Fetch the Ion token; null when unconfigured, unreachable or too slow. */
export async function fetchCesiumIonToken() {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), MAP_CONFIG_TIMEOUT_MS);
  try {
    const res = await fetch('/api/map-config', { signal: controller.signal });
    if (!res.ok) return null;
    return parseIonToken(await res.json());
  } catch {
    return null;
  } finally {
    clearTimeout(timer);
  }
}

let pending = null;

/** One shared fetch per page load: the viewer and the layer picker both wait on it. */
export function loadCesiumIonToken() {
  if (!pending) pending = fetchCesiumIonToken();
  return pending;
}
