/**
 * Pure utilities for plan snapshots and the two questions asked of them:
 * is this the plan on the UAVs, and is it the plan in the file last loaded or
 * saved.
 *
 * buildPlanSnapshot is the single source of truth for which fields constitute
 * the plan baseline. planEditKey and planSyncKey say which of them count as a
 * change.
 */

/**
 * Build a deep-copied, normalized snapshot of all plan-local state that
 * affects monitor display, vehicle upload, or saved plan behavior.
 *
 * Global settings (settings.default_delivery_hubs, altitude, FOV) are excluded —
 * they survive plan discard by design.
 */
export function buildPlanSnapshot(current) {
  return {
    plan:                 current.plan ? JSON.parse(JSON.stringify(current.plan)) : null,
    polygon:              (current.polygon || []).map(p => ({ lat: p.lat, lon: p.lon })),
    searchPattern:               current.searchPattern ?? null,
    analysis:             current.analysis ? JSON.parse(JSON.stringify(current.analysis)) : null,
    uavCount:             current.uavCount ?? null,
    // The count the plan is generated for: an unset count follows the area.
    effectiveUavCount:    current.effectiveUavCount ?? current.uavCount ?? null,
    partitionAngleDeg:    current.partitionAngleDeg ?? null,
    routeOffsetM:         current.routeOffsetM ?? null,
    setLaunchPoints:      JSON.parse(JSON.stringify(current.setLaunchPoints || [null])),
    setCorridorPointsArr: JSON.parse(JSON.stringify(current.setCorridorPointsArr || [[]])),
    deliveryHubAssignments:       current.deliveryHubAssignments ? [...current.deliveryHubAssignments] : [],
    manualDeliveryHubEdit: !!current.manualDeliveryHubEdit,
    simDockWps:         current.simDockWps ? JSON.parse(JSON.stringify(current.simDockWps)) : {},
    detectAfterWps:       current.detectAfterWps ? JSON.parse(JSON.stringify(current.detectAfterWps)) : {},
    // The fence decision, only as the operator made it (see planEditKey).
    fenceTouched:         !!current.fenceTouched,
    fenceEnabled:         !!current.fenceEnabled,
    fenceOffsetM:         current.fenceOffsetM ?? null,
    // The fence lifecycle, restored with the plan (in no key): a request not
    // yet sent, what the vehicles acknowledged, and what they reported.
    fenceIntent:          current.fenceIntent ? { ...current.fenceIntent } : null,
    fenceAcknowledged:    current.fenceAcknowledged ? { ...current.fenceAcknowledged } : null,
    fenceObservations:    current.fenceObservations ?? null,
    // Vehicle-observable fence geometry: both ride the upload payload, so
    // editing them must mark the plan dirty like any other plan geometry.
    fenceCustomVertices:  current.fenceCustomVertices ? JSON.parse(JSON.stringify(current.fenceCustomVertices)) : null,
    exclusionPolygons:    current.exclusionPolygons ? JSON.parse(JSON.stringify(current.exclusionPolygons)) : [],
  };
}

const latLon = (p) => (p ? [p.lat, p.lon] : null);

/** Drop trailing entries the per-set arrays only pad with (one slot per set). */
function trimTrailing(items, isEmpty) {
  let end = items.length;
  while (end > 0 && isEmpty(items[end - 1])) end -= 1;
  return items.slice(0, end);
}

/** A zone-keyed selection map without its empty entries, in key order. */
function selectionMap(map) {
  return Object.keys(map || {})
    .filter((k) => map[k] != null && !(Array.isArray(map[k]) && map[k].length === 0))
    .sort()
    .map((k) => [k, map[k]]);
}

/**
 * What the operator authored: the inputs a plan file stores. Values the GCS
 * derives by itself are left out, so they never read as a change: the area
 * analysis, an unset UAV count (it follows the area), automatic home hub
 * assignments, the demo fence default, and the empty per-set slots the set
 * count pads. A plan just loaded from a file, or just saved, keeps this key
 * until the operator edits it.
 */
export function planEditKey(current) {
  const s = buildPlanSnapshot(current || {});
  return JSON.stringify({
    polygon: s.polygon.map(latLon),
    searchPattern: s.searchPattern,
    uavCount: s.effectiveUavCount,
    partitionAngleDeg: s.partitionAngleDeg,
    routeOffsetM: s.routeOffsetM,
    launchPoints: trimTrailing(s.setLaunchPoints.map(latLon), (p) => p == null),
    corridors: trimTrailing(s.setCorridorPointsArr.map((c) => (c || []).map(latLon)), (c) => c.length === 0),
    hubAssignments: s.manualDeliveryHubEdit ? s.deliveryHubAssignments : null,
    simDockWps: selectionMap(s.simDockWps),
    detectAfterWps: selectionMap(s.detectAfterWps),
    fence: s.fenceTouched ? [s.fenceEnabled, s.fenceOffsetM] : null,
    fenceCustomVertices: s.fenceCustomVertices ? s.fenceCustomVertices.map(latLon) : null,
    exclusions: s.exclusionPolygons.filter((r) => Array.isArray(r) && r.length >= 3).map((r) => r.map(latLon)),
  });
}

/**
 * What the UAVs fly from this plan: the authored inputs and the generated
 * routes (a planner setting can move the routes without any input changing).
 */
export function planSyncKey(current) {
  const s = buildPlanSnapshot(current || {});
  return JSON.stringify({
    edit: planEditKey(s),
    altitude: s.plan?.altitude_m ?? null,
    zones: (s.plan?.zones || []).map((z) => [z.sys_id ?? null, z.set_index ?? 0, z.altitude_m ?? null, z.track || []]),
  });
}

/** True when the plan holds anything worth keeping: an area, a launch point or routes. */
export function planHasContent(current) {
  const s = buildPlanSnapshot(current || {});
  return s.polygon.length >= 3
    || s.setLaunchPoints.some((p) => p != null)
    || (s.plan?.zones?.length || 0) > 0;
}

/**
 * Returns true if current plan state differs from the stored snapshot in what
 * the UAVs would fly (planSyncKey). Returns false when no snapshot exists.
 */
export function isPlanDirty(current, snapshot) {
  if (!snapshot) return false;
  return planSyncKey(current) !== planSyncKey(snapshot);
}
