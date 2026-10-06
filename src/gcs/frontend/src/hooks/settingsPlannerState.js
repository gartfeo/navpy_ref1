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
  // sizes. Require a finite positive size for the dock detect class ('0',
  // plannerConfig DOCK_DETECT_CLASS_ID) so the planner never silently computes
  // off the JS fallback for the dock preset.
  const detectorClassDimensions = catalog.detector_class_dimensions;
  const hasValidSize = (id) => {
    const size = detectorClassDimensions?.[id]?.size_m;
    return typeof size === 'number' && Number.isFinite(size) && size > 0;
  };
  const detectorClassDimensionsValid = (
    detectorClassDimensions
    && typeof detectorClassDimensions === 'object'
    && hasValidSize('0')  // dock
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
