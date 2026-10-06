/**
 * Check whether terrain-sampled heights should be applied to track positions.
 * Returns false when all heights are zero (terrain provider not yet loaded)
 * or when the array is empty.
 */
export function shouldApplyTerrainSample(sampledHeights) {
  if (!Array.isArray(sampledHeights) || sampledHeights.length === 0) return false;
  return sampledHeights.some((h) => (h ?? 0) !== 0);
}
