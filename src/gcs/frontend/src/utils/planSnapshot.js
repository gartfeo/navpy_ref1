/**
 * Pure utilities for plan snapshot dirty-detection and deep-copy.
 *
 * buildPlanSnapshot is the single source of truth for which fields constitute
 * the plan baseline.  isPlanDirty compares a normalized snapshot of current
 * state against the stored snapshot — no manual field list to maintain.
 */

/**
 * Build a deep-copied, normalized snapshot of all plan-local state that
 * affects monitor display, vehicle upload, or saved plan behavior.
 *
 * Global settings (settings.fallback_delivery_locations, altitude, FOV) are excluded —
 * they survive plan discard by design.
 */
export function buildPlanSnapshot(current) {
  return {
    plan:                 current.plan ? JSON.parse(JSON.stringify(current.plan)) : null,
    polygon:              (current.polygon || []).map(p => ({ lat: p.lat, lon: p.lon })),
    searchPattern:               current.searchPattern ?? null,
    dockClasses:        current.dockClasses ? [...current.dockClasses] : [],
    perUavDockClasses:  current.perUavDockClasses ? JSON.parse(JSON.stringify(current.perUavDockClasses)) : {},
    analysis:             current.analysis ? JSON.parse(JSON.stringify(current.analysis)) : null,
    uavCount:             current.uavCount ?? null,
    partitionAngleDeg:    current.partitionAngleDeg ?? null,
    routeOffsetM:         current.routeOffsetM ?? null,
    setLaunchPoints:      JSON.parse(JSON.stringify(current.setLaunchPoints || [null])),
    setCorridorPointsArr: JSON.parse(JSON.stringify(current.setCorridorPointsArr || [[]])),
    fallbackLocationAssignments:       current.fallbackLocationAssignments ? [...current.fallbackLocationAssignments] : [],
    simDockWps:         current.simDockWps ? JSON.parse(JSON.stringify(current.simDockWps)) : {},
    detectAfterWps:       current.detectAfterWps ? JSON.parse(JSON.stringify(current.detectAfterWps)) : {},
    // Vehicle-observable fence geometry: both ride the upload payload, so
    // editing them must mark the plan dirty like any other plan geometry.
    fenceCustomVertices:  current.fenceCustomVertices ? JSON.parse(JSON.stringify(current.fenceCustomVertices)) : null,
    exclusionPolygons:    current.exclusionPolygons ? JSON.parse(JSON.stringify(current.exclusionPolygons)) : [],
  };
}

/**
 * Returns true if current plan state differs from the stored snapshot.
 * Returns false when no snapshot exists (nothing to compare against).
 */
export function isPlanDirty(current, snapshot) {
  if (!snapshot) return false;
  return JSON.stringify(buildPlanSnapshot(current)) !== JSON.stringify(buildPlanSnapshot(snapshot));
}
