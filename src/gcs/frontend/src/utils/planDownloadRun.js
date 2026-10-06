/**
 * Lineage tag for plans published by a fleet mission-download run.
 *
 * A fleet download publishes the plan progressively — one zone per vehicle as
 * each mission arrives — and must abort if *other* work replaces the plan out
 * from under it (e.g. an operator generates a fresh plan mid-download). It
 * cannot detect that by comparing plan object identity, because the run's own
 * corridor trim (derivePlanPolygon) legitimately rebuilds the plan object via
 * `{ ...prev, zones }` whenever a downloaded mission carries a takeoff corridor
 * (corridor_end_index > 0). That self-inflicted identity change used to be
 * misread as an external replacement, cancelling the run after the first UAV.
 *
 * Tagging each published plan with the run — an enumerable Symbol that survives
 * object spread (so the trim carries it forward) but is invisible to JSON (so
 * it never leaks into saved-plan snapshots) — lets the run recognize plans
 * descended from its own publishes, including after the trim.
 */
const PLAN_DOWNLOAD_RUN = Symbol('planDownloadRun');

/** Add/remove request-specific owners for the operator-visible download set. */
export function addDownloadOwner(registry, ids, owner) {
  for (const sysId of ids) {
    if (!registry.has(sysId)) registry.set(sysId, new Set());
    registry.get(sysId).add(owner);
  }
}

export function removeDownloadOwner(registry, ids, owner) {
  for (const sysId of ids) {
    const owners = registry.get(sysId);
    owners?.delete(owner);
    if (owners?.size === 0) registry.delete(sysId);
  }
}

export function clearDownloadOwners(registry, ids) {
  for (const sysId of ids) registry.delete(sysId);
}

export function activeDownloadIds(registry) {
  return new Set(registry.keys());
}

/** Return a copy of `plan` tagged as belonging to `run` (null-safe). */
export function tagPlanForRun(plan, run) {
  if (!plan) return plan;
  return { ...plan, [PLAN_DOWNLOAD_RUN]: run };
}

/** True when `plan` descends from a publish by `run` (survives the trim). */
export function planBelongsToRun(plan, run) {
  return !!plan && !!run && !run.cancelled && plan[PLAN_DOWNLOAD_RUN] === run;
}

/**
 * Select vehicles newly visible to a page-refresh mission run.
 *
 * Backend auto-connect can publish the connected roster incrementally, so the
 * first non-empty snapshot is not the complete fleet. Keep enlisting unseen
 * sys_ids while the rendered plan still belongs to this startup run. A plan
 * that predates the first publish, or replaces a published startup plan, is
 * operator-owned and permanently cancels the continuation.
 */
export function selectStartupMissionVehicles(run, vehicles, plan) {
  if (!run || run.cancelled) return [];

  const planWasReplaced = run.hasPublished
    ? !planBelongsToRun(plan, run)
    : !!plan;
  if (planWasReplaced) {
    run.cancelled = true;
    return [];
  }

  const visibleVehicles = vehicles || [];
  const visibleIds = [];
  const seenIds = new Set();
  for (const vehicle of visibleVehicles) {
    const sysId = vehicle?.sys_id;
    if (sysId == null || seenIds.has(sysId) || run.excludedIds?.has(sysId)) continue;
    seenIds.add(sysId);
    visibleIds.push(sysId);
  }
  // vehicleList is the upload authority: zone i is later uploaded to vehicle i.
  // Put the latest canonical roster first, while retaining an in-flight ID that
  // is temporarily absent until the disconnect path explicitly excludes it.
  const orderedIds = [
    ...visibleIds,
    ...(run.orderedIds || []).filter((sysId) => (
      !seenIds.has(sysId) && !run.excludedIds?.has(sysId)
    )),
  ];
  if (run.orderedIds) run.orderedIds.splice(0, run.orderedIds.length, ...orderedIds);

  const selected = [];
  for (const vehicle of visibleVehicles) {
    const sysId = vehicle?.sys_id;
    if (sysId == null || run.requestedIds.has(sysId) || run.excludedIds?.has(sysId)) continue;
    run.requestedIds.add(sysId);
    selected.push(vehicle);
  }
  return selected;
}

/**
 * Mark an internally emptied startup plan as unpublished.
 *
 * Disconnecting the only mission currently rendered may legitimately clear the
 * plan while other startup requests are still pending. Only a plan carrying
 * this run's lineage may authorize that reset; an operator-owned replacement
 * must continue to cancel the startup continuation.
 */
export function resetStartupPlanAfterDisconnect(run, plan) {
  if (!run || run.cancelled || !planBelongsToRun(plan, run)) return false;
  run.hasPublished = false;
  return true;
}

/** Invalidate startup side effects that captured an older connected roster. */
export function markStartupRosterChanged(run) {
  if (!run) return;
  run.rosterRevision = (run.rosterRevision || 0) + 1;
}

/** True when no disconnect changed the startup roster since `revision`. */
export function startupRosterIsCurrent(run, revision) {
  return !run || run.rosterRevision === revision;
}

/** Normalize a startup mission response into the standard client diagnostic. */
export function startupMissionResultDiagnostic(sysId, result) {
  const rejected = !!result?.error;
  return {
    event: rejected ? 'mission_result_rejected' : 'mission_result_accepted',
    fields: {
      request_id: result?._diagnostic_request_id,
      sys_id: sysId,
      outcome: result?.error === 'timeout' ? 'timeout' : rejected ? 'failure' : 'accepted',
      total: result?.waypoints?.length || 0,
    },
  };
}

/** True while a startup response is still eligible to affect client state. */
export function startupMissionRequestIsActive(run, sysId) {
  return !run || (!run.cancelled && !run.excludedIds?.has(sysId));
}

/**
 * Whether disconnect should use the legacy positional fallback for zones that
 * lack sys_id. Startup zones always carry sys_id; an unchanged count there can
 * mean a concurrent startup publish already removed the requested UAV.
 */
export function shouldFallbackDisconnectZoneRemoval(run, plan, oldZones, remaining, removedCount) {
  return oldZones.length > 0
    && removedCount > 0
    && remaining.length === oldZones.length
    && !planBelongsToRun(plan, run);
}

/**
 * Rebuild untrimmed zones from raw startup missions in roster order.
 *
 * Rendered plan zones are not a safe accumulator: derivePlanPolygon trims their
 * corridor prefix and resets corridor_end_index. Reconstructing from the raw
 * mission snapshots preserves waypoint-number conversion inputs and assigns
 * positional zone_index values after ordering.
 */
export function startupZonesFromMissions(run) {
  const zones = [];
  for (const sysId of run?.orderedIds || []) {
    if (run.excludedIds?.has(sysId)) continue;
    const mission = run.missionsBySysId?.get(sysId);
    if (!mission?.waypoints?.length) continue;
    zones.push({
      zone_index: zones.length,
      polygon: [],
      track: mission.waypoints.map((wp) => ({
        lat: wp.lat,
        lon: wp.lon,
        ...(wp.alt != null && { alt: wp.alt }),
      })),
      sys_id: sysId,
      corridor_end_index: mission.corridor_end_index ?? 0,
      altitude_m: mission.altitude_m,
    });
  }
  return zones;
}

/** First available, non-excluded mission in startup roster order. */
export function startupMissionAuthority(run) {
  for (const sysId of run?.orderedIds || []) {
    if (run.excludedIds?.has(sysId)) continue;
    const mission = run.missionsBySysId?.get(sysId);
    if (mission) return { sysId, mission };
  }
  return null;
}

/** Aggregate plan-level fields using the prior fleet assembler's semantics. */
export function startupPlanMetadata(run, fallbackAltitude = 100) {
  const missions = [];
  for (const sysId of run?.orderedIds || []) {
    if (run.excludedIds?.has(sysId)) continue;
    const mission = run.missionsBySysId?.get(sysId);
    if (mission?.waypoints?.length) missions.push(mission);
  }
  let altitude = fallbackAltitude;
  for (const mission of missions) {
    if (mission.altitude_m) altitude = mission.altitude_m;
  }
  const dockClasses = missions.find((mission) => mission.dock_classes?.length > 0)
    ?.dock_classes || [];
  return { altitude, dockClasses };
}

/** Return startup metadata only while `plan` is still owned by this run. */
export function startupPlanMetadataForOwnedPlan(run, plan, fallbackAltitude = 100) {
  return planBelongsToRun(plan, run)
    ? startupPlanMetadata(run, fallbackAltitude)
    : null;
}

/** Return startup mission fallback locations in the same order as plan zones. */
export function startupMissionFallbackLocationsForZones(run, zones) {
  return (zones || []).map((zone) => (
    run?.missionFallbackLocationsBySysId?.has(zone.sys_id)
      ? run.missionFallbackLocationsBySysId.get(zone.sys_id)
      : null
  ));
}

/**
 * Serialize startup plan side effects while allowing a later task to continue
 * if an earlier best-effort postprocessor fails.
 */
export function enqueueStartupPostProcess(run, task) {
  const previous = run.postProcessQueue || Promise.resolve();
  const queued = previous.catch(() => {}).then(task);
  run.postProcessQueue = queued;
  return queued;
}
