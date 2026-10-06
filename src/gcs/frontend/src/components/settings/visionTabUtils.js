// Pure helpers for the Vision-tab per-class preset editor (altitude_m /
// min_pixel_size). Extracted from VisionTab so the override-buffer semantics are
// unit-testable (tests/gcs/test_vision_tab_utils_js.py) without mounting React.
//
// Dock presets are owned by the active vision profile (vision_profiles.json).
// `catalogPresets` is the catalog base for the active profile; `presetOverrides`
// is the live edit buffer flushed to PUT /api/vision-profiles/{profile} on Apply
// and discarded on Reset / profile switch.

/**
 * Apply a single per-class field edit to the override buffer.
 *
 * Merges the catalog base for that class, any existing override for it, then the
 * new field value — so editing one field (e.g. min_pixel_size) carries the
 * class's OTHER preset fields (altitude_m, label) into the buffer that Apply
 * flushes, and never drops a sibling field. Returns a new buffer (no mutation).
 */
export function applyPresetEdit(catalogPresets, presetOverrides, key, field, value) {
  const base = catalogPresets[key] || {};
  const prev = presetOverrides[key] || {};
  return {
    ...presetOverrides,
    [key]: { ...base, ...prev, [field]: value },
  };
}

/**
 * Merge the catalog presets with the live override buffer into the values the
 * grid displays. Overrides win per field; a class present only in the override
 * buffer still appears. An empty buffer yields the catalog values unchanged
 * (Reset / profile-switch discards the buffer, so the grid reverts to catalog).
 */
export function mergeDisplayPresets(catalogPresets, presetOverrides) {
  const presets = {};
  for (const k of Object.keys(catalogPresets)) {
    presets[k] = { ...catalogPresets[k], ...(presetOverrides[k] || {}) };
  }
  for (const k of Object.keys(presetOverrides)) {
    if (!presets[k]) presets[k] = { ...presetOverrides[k] };
  }
  return presets;
}
