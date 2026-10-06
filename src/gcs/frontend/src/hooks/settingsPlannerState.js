export function resolvePlannerState(settingsData, catalog) {
  if (!settingsData) {
    return {
      plannerReady: false,
      plannerLoadError: 'Failed to load settings.',
      plannerProfiles: null,
      plannerDetectorClassDimensions: null,
    };
  }

  if (!catalog?.profiles) {
    return {
      plannerReady: false,
      plannerLoadError: 'Failed to load vision profiles.',
      plannerProfiles: null,
      plannerDetectorClassDimensions: null,
    };
  }

  // The top-level detector_class_dimensions block is the single source for per-class
  // sizes. Require a finite positive size for every detect class an operator
  // preset maps to (medium/large -> 0, small -> 4) so the planner never
  // silently computes off the JS fallback for any dock preset.
  const detectorClassDimensions = catalog.detector_class_dimensions;
  const hasValidSize = (id) => {
    const size = detectorClassDimensions?.[id]?.size_m;
    return typeof size === 'number' && Number.isFinite(size) && size > 0;
  };
  const detectorClassDimensionsValid = (
    detectorClassDimensions
    && typeof detectorClassDimensions === 'object'
    && hasValidSize('0')  // medium / large
    && hasValidSize('4')  // small (also sets MIN_CLASS_SIZE)
  );
  if (!detectorClassDimensionsValid) {
    return {
      plannerReady: false,
      plannerLoadError: 'Vision profiles missing detector class dimensions.',
      plannerProfiles: null,
      plannerDetectorClassDimensions: null,
    };
  }

  return {
    plannerReady: true,
    plannerLoadError: null,
    plannerProfiles: catalog.profiles,
    // Top-level per-class characteristic sizes — the single source of truth.
    plannerDetectorClassDimensions: detectorClassDimensions,
  };
}
