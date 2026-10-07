/**
 * Build-time configuration for the Cesium Ion imagery/terrain upgrade.
 *
 * The Ion access token is supplied as `VITE_CESIUM_ION_TOKEN` at build time
 * (e.g. in an untracked `src/gcs/frontend/.env.local`). It must never be
 * committed: the repository is public. Absent by default, which keeps the map
 * on the OpenStreetMap base layer without Ion imagery or terrain; changing it
 * needs a frontend rebuild.
 */

/** @returns {string|null} the configured Ion token, or null when unconfigured. */
export function cesiumIonToken(env = import.meta.env) {
  const raw = env?.VITE_CESIUM_ION_TOKEN;
  if (typeof raw !== 'string') return null;
  const trimmed = raw.trim();
  return trimmed === '' ? null : trimmed;
}

export default cesiumIonToken;
