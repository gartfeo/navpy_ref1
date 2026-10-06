/**
 * Build-time configuration for the single Jetson tracking-overlay video feed.
 *
 * One public, credential-free WHEP endpoint, supplied as `VITE_VIDEO_WHEP_URL`
 * at build time (e.g. `http://192.168.144.10:8889/tracking/whep`). It is absent
 * by default, which hides the video panel; changing it needs a frontend
 * rebuild. There is deliberately no settings field and no backend route: this
 * is a fixed property of the installed device, not an operator preference, and
 * the URL must never carry credentials.
 */

/** @returns {string|null} the configured WHEP URL, or null when unconfigured. */
export function whepUrl(env = import.meta.env) {
  const raw = env?.VITE_VIDEO_WHEP_URL;
  if (typeof raw !== 'string') return null;
  const trimmed = raw.trim();
  return trimmed === '' ? null : trimmed;
}

export default whepUrl;
