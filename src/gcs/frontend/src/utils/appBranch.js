// Current git branch, injected at build/dev-server start by vite.config.js
// (`define: { __APP_BRANCH__ }`). Shown in the tab title and topbar so it's
// obvious which app version / which chat's instance is running.
// eslint-disable-next-line no-undef
export const APP_BRANCH =
  typeof __APP_BRANCH__ !== 'undefined' && __APP_BRANCH__ ? __APP_BRANCH__ : '';

/**
 * Deterministic tag color for a branch name so every branch / chat instance
 * gets a stable, distinct color — the same name always maps to the same hue.
 * Pure (no React, no DOM): hue is an FNV-1a hash of the string folded into
 * 0..359, returned as HSL strings tuned so the text stays legible on the dark
 * topbar. Tested via tests/gcs/test_app_branch_js.py.
 */
export function branchColor(branch) {
  const name = String(branch || '');
  let hash = 2166136261; // FNV-1a 32-bit offset basis
  for (let i = 0; i < name.length; i++) {
    hash ^= name.charCodeAt(i);
    hash = Math.imul(hash, 16777619);
  }
  const hue = (hash >>> 0) % 360;
  return {
    hue,
    bg: `hsl(${hue}, 60%, 20%)`,
    border: `hsl(${hue}, 65%, 40%)`,
    fg: `hsl(${hue}, 85%, 78%)`,
    dot: `hsl(${hue}, 75%, 55%)`,
  };
}
