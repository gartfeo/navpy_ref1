/**
 * Regression coverage for fleet mission-download plan lineage.
 *
 * A fleet download publishes the plan progressively (one zone per UAV). When a
 * downloaded mission carries a takeoff corridor (corridor_end_index > 0), the
 * run's own corridor trim (derivePlanPolygon) rebuilds the plan object via
 * `{ ...prev, zones }`. The stale-plan guard used to compare object identity,
 * so that self-inflicted rebuild read as an external replacement and cancelled
 * the run after the FIRST UAV — leaving UAVs 2 and 3 undrawn. Lineage tagging
 * fixes it: the tag survives the spread, so the run recognizes its own plan.
 *
 * Run via: node tests/gcs/frontend/test_plan_download_run_logic.js
 */
const assert = require('assert');
const path = require('path');
const { readFileSync } = require('fs');

function loadEsm(relPath) {
  const abs = path.resolve(__dirname, '..', '..', '..', 'src', 'gcs', 'frontend', 'src', relPath);
  let src = readFileSync(abs, 'utf8');
  src = src.replace(/export\s+(async\s+)?(const|function|class)\s+/g, '$1$2 ');
  src = src.replace(/export\s*\{[^}]*\}/g, '');
  src = src.replace(/export\s+default\s+/g, '');
  const names = [];
  src.replace(/^(?:async\s+)?(?:const|function|class)\s+(\w+)/gm, (_, n) => { names.push(n); return _; });
  src += '\nmodule.exports = { ' + names.join(', ') + ' };\n';
  const m = { exports: {} };
  new Function('module', 'exports', 'require', src)(m, m.exports, require);
  return m.exports;
}

/** Emulate derivePlanPolygon's corridor trim: rebuild the plan object. */
function corridorTrim(plan) {
  return {
    ...plan,
    zones: plan.zones.map((z) => ({
      ...z,
      track: z.track.slice(z.corridor_end_index || 0),
      corridor_end_index: 0,
    })),
  };
}

async function main() {
  const {
    addDownloadOwner,
    removeDownloadOwner,
    clearDownloadOwners,
    activeDownloadIds,
    tagPlanForRun,
    planBelongsToRun,
    selectStartupMissionVehicles,
    resetStartupPlanAfterDisconnect,
    markStartupRosterChanged,
    startupRosterIsCurrent,
    startupMissionResultDiagnostic,
    startupMissionRequestIsActive,
    shouldFallbackDisconnectZoneRemoval,
    startupZonesFromMissions,
    startupMissionAuthority,
    startupPlanMetadata,
    startupPlanMetadataForOwnedPlan,
    startupMissionFallbackLocationsForZones,
    enqueueStartupPostProcess,
  } = loadEsm('utils/planDownloadRun.js');

  const downloadOwners = new Map();
  const startupOwner = {};
  const replacementOwner = {};
  addDownloadOwner(downloadOwners, [1], startupOwner);
  addDownloadOwner(downloadOwners, [1], replacementOwner);
  removeDownloadOwner(downloadOwners, [1], startupOwner);
  assert.deepStrictEqual(
    [...activeDownloadIds(downloadOwners)],
    [1],
    'settled cancelled request cannot clear a newer download owner',
  );
  removeDownloadOwner(downloadOwners, [1], replacementOwner);
  assert.deepStrictEqual([...activeDownloadIds(downloadOwners)], [], 'last owner clears busy state');
  addDownloadOwner(downloadOwners, [2], startupOwner);
  clearDownloadOwners(downloadOwners, [2]);
  assert.deepStrictEqual([...activeDownloadIds(downloadOwners)], [], 'disconnect clears every owner for a UAV');

  const run = { id: 'run-A' };
  const otherRun = { id: 'run-B' };
  const base = { zones: [{ track: [{}, {}, {}], corridor_end_index: 1 }], altitude_m: 100 };

  // Basic tagging + rejection.
  const tagged = tagPlanForRun(base, run);
  assert.ok(planBelongsToRun(tagged, run), 'tagged plan belongs to its run');
  assert.ok(!planBelongsToRun(tagged, otherRun), 'tagged plan rejects a different run');
  assert.ok(!planBelongsToRun(base, run), 'untagged source plan does not belong to the run');

  // Null-safe.
  assert.strictEqual(tagPlanForRun(null, run), null);
  assert.ok(!planBelongsToRun(null, run));
  assert.ok(!planBelongsToRun(undefined, run));

  const cancelledRun = { id: 'cancelled-run', cancelled: false };
  const cancelledTaggedPlan = tagPlanForRun(base, cancelledRun);
  cancelledRun.cancelled = true;
  assert.ok(
    !planBelongsToRun(cancelledTaggedPlan, cancelledRun),
    'copied lineage from a cancelled startup run is inert',
  );

  // Core regression: the corridor trim changes object identity (what broke the
  // old identity guard) but lineage must survive the `{ ...prev }` spread.
  const trimmed = corridorTrim(tagged);
  assert.notStrictEqual(trimmed, tagged, 'trim changes identity (the old-guard trigger)');
  assert.ok(planBelongsToRun(trimmed, run), 'trimmed plan still belongs to the run');

  // Operator replacement: a freshly generated plan is untagged → rejected.
  assert.ok(!planBelongsToRun({ zones: [], altitude_m: 150 }, run), 'operator-replaced plan is rejected');

  // The tag must never leak into saved-plan snapshots (JSON round-trip drops it).
  const snapshot = JSON.parse(JSON.stringify(trimmed));
  assert.ok(!planBelongsToRun(snapshot, run), 'JSON snapshot drops the lineage tag');
  assert.deepStrictEqual(
    snapshot,
    { zones: [{ track: [{}, {}], corridor_end_index: 0 }], altitude_m: 100 },
    'snapshot carries only real plan data',
  );

  // End-to-end: three UAVs publish progressively, each followed by a corridor
  // trim. The guard (lineage, not identity) must never cancel the run, so all
  // three zones survive to the final plan.
  const fleetRun = { id: 'run-3uav', hasPublished: false };
  let planRefCurrent = null;
  let cancelled = false;
  const publish = (zoneCount) => {
    // Guard, exactly as in onResultsChanged.
    if (fleetRun.hasPublished && !planBelongsToRun(planRefCurrent, fleetRun)) {
      cancelled = true;
      return;
    }
    const next = tagPlanForRun({
      zones: Array.from({ length: zoneCount }, () => ({ track: [{}, {}], corridor_end_index: 1 })),
      altitude_m: 100,
    }, fleetRun);
    planRefCurrent = next;
    fleetRun.hasPublished = true;
    // derivePlanPolygon trim runs after publish and rebuilds the object.
    planRefCurrent = corridorTrim(next);
  };
  publish(1);
  publish(2);
  publish(3);
  assert.ok(!cancelled, 'run is NOT cancelled by its own corridor trim');
  assert.strictEqual(planRefCurrent.zones.length, 3, 'all 3 UAV zones survive to the final plan');

  // Sanity: an actual external replacement between publishes still cancels.
  const guardRun = { id: 'run-guard', hasPublished: true };
  const externallyReplaced = { zones: [{ track: [{}] }], altitude_m: 100 }; // untagged
  assert.ok(!planBelongsToRun(externallyReplaced, guardRun), 'external replacement is detected → run would cancel');

  // Page-refresh regression: backend auto-connect exposes the roster
  // incrementally. The startup run must enlist each newly visible UAV exactly
  // once instead of latching permanently to the first non-empty snapshot.
  const startupRun = {
    id: 'startup-3uav',
    cancelled: false,
    hasPublished: false,
    requestedIds: new Set(),
    orderedIds: [],
    excludedIds: new Set(),
    missionFallbackLocationsBySysId: new Map(),
    missionsBySysId: new Map(),
    postProcessQueue: Promise.resolve(),
  };
  assert.deepStrictEqual(
    selectStartupMissionVehicles(startupRun, [{ sys_id: 1 }], null).map((v) => v.sys_id),
    [1],
    'first roster snapshot enlists UAV 1',
  );
  startupRun.hasPublished = true;
  const startupPlan = tagPlanForRun({ zones: [{ sys_id: 1 }] }, startupRun);
  assert.deepStrictEqual(
    selectStartupMissionVehicles(startupRun, [{ sys_id: 1 }, { sys_id: 2 }], startupPlan).map((v) => v.sys_id),
    [2],
    'second roster snapshot enlists only new UAV 2',
  );
  assert.deepStrictEqual(
    selectStartupMissionVehicles(
      startupRun,
      [{ sys_id: 1 }, { sys_id: 2 }, { sys_id: 3 }],
      startupPlan,
    ).map((v) => v.sys_id),
    [3],
    'third roster snapshot enlists only new UAV 3',
  );
  assert.deepStrictEqual(
    selectStartupMissionVehicles(
      startupRun,
      [{ sys_id: 1 }, { sys_id: 2 }, { sys_id: 3 }],
      startupPlan,
    ),
    [],
    'unchanged roster does not duplicate downloads',
  );

  // The newest vehicleList order is canonical because upload assigns zone i to
  // vehicle i. First seeing UAV 3 must not permanently pin it ahead of 1 and 2.
  const outOfOrderRun = {
    cancelled: false,
    hasPublished: false,
    requestedIds: new Set(),
    orderedIds: [],
    excludedIds: new Set(),
  };
  assert.deepStrictEqual(
    selectStartupMissionVehicles(outOfOrderRun, [{ sys_id: 3 }], null).map((v) => v.sys_id),
    [3],
    'first out-of-order snapshot still starts its available download',
  );
  assert.deepStrictEqual(
    selectStartupMissionVehicles(
      outOfOrderRun,
      [{ sys_id: 1 }, { sys_id: 2 }, { sys_id: 3 }],
      null,
    ).map((v) => v.sys_id),
    [1, 2],
    'later canonical roster enlists only newly visible UAVs',
  );
  assert.deepStrictEqual(
    outOfOrderRun.orderedIds,
    [1, 2, 3],
    'startup zone order realigns to the canonical upload roster',
  );

  // Disconnect may internally clear the sole published startup zone while
  // another requested mission is pending. That null must remain a valid first-
  // publish state, while an untagged operator plan must not authorize a reset.
  const disconnectRun = {
    cancelled: false,
    hasPublished: true,
    requestedIds: new Set([1, 2]),
    orderedIds: [1, 2],
    excludedIds: new Set([1]),
  };
  const soleStartupPlan = tagPlanForRun({ zones: [{ sys_id: 1 }] }, disconnectRun);
  assert.strictEqual(
    resetStartupPlanAfterDisconnect(disconnectRun, soleStartupPlan),
    true,
    'disconnect recognizes an internally owned plan clear',
  );
  assert.strictEqual(disconnectRun.hasPublished, false, 'pending result may publish as the new first plan');
  assert.deepStrictEqual(
    selectStartupMissionVehicles(disconnectRun, [{ sys_id: 2 }], null),
    [],
    'internal null plan does not cancel pending startup continuation',
  );
  assert.strictEqual(disconnectRun.cancelled, false, 'startup run remains active after internal clear');

  const operatorPlanRun = { cancelled: false, hasPublished: true };
  assert.strictEqual(
    resetStartupPlanAfterDisconnect(operatorPlanRun, { zones: [{ sys_id: 99 }] }),
    false,
    'operator-owned plan cannot reset startup lineage',
  );
  assert.strictEqual(operatorPlanRun.hasPublished, true, 'operator replacement remains detectable');

  // An async postprocessor must not restore stale zone-indexed parameter maps
  // after a different UAV disconnects while its fence/params await is pending.
  const revisionRun = { rosterRevision: 0 };
  const capturedRevision = revisionRun.rosterRevision;
  assert.ok(startupRosterIsCurrent(revisionRun, capturedRevision), 'captured roster starts current');
  markStartupRosterChanged(revisionRun);
  assert.ok(
    !startupRosterIsCurrent(revisionRun, capturedRevision),
    'disconnect invalidates side effects captured from the older roster',
  );
  const refreshedRevision = revisionRun.rosterRevision;
  const refreshes = [];
  await enqueueStartupPostProcess(revisionRun, async () => {
    if (startupRosterIsCurrent(revisionRun, capturedRevision)) refreshes.push('stale');
  });
  await enqueueStartupPostProcess(revisionRun, async () => {
    if (startupRosterIsCurrent(revisionRun, refreshedRevision)) refreshes.push('reduced-roster');
  });
  assert.deepStrictEqual(
    refreshes,
    ['reduced-roster'],
    'disconnect skips stale effects and a queued reduced-roster pass still runs',
  );
  assert.ok(startupRosterIsCurrent(null, undefined), 'non-startup post-processing remains unaffected');

  assert.deepStrictEqual(
    startupMissionResultDiagnostic(2, {
      _diagnostic_request_id: 'mission-2',
      waypoints: [{}, {}],
    }),
    {
      event: 'mission_result_accepted',
      fields: { request_id: 'mission-2', sys_id: 2, outcome: 'accepted', total: 2 },
    },
    'startup success preserves the standard accepted diagnostic',
  );
  assert.deepStrictEqual(
    startupMissionResultDiagnostic(3, {
      _diagnostic_request_id: 'mission-3',
      error: 'timeout',
    }),
    {
      event: 'mission_result_rejected',
      fields: { request_id: 'mission-3', sys_id: 3, outcome: 'timeout', total: 0 },
    },
    'startup timeout preserves the standard rejected diagnostic',
  );
  assert.ok(
    startupMissionRequestIsActive({ cancelled: false, excludedIds: new Set() }, 2),
    'connected startup request remains eligible for result diagnostics',
  );
  assert.ok(
    !startupMissionRequestIsActive({ cancelled: true, excludedIds: new Set() }, 2),
    'cancelled startup request cannot emit a result diagnostic',
  );
  assert.ok(
    !startupMissionRequestIsActive({ cancelled: false, excludedIds: new Set([2]) }, 2),
    'disconnected startup request cannot emit a result diagnostic',
  );

  const disconnectFallbackRun = { id: 'disconnect-fallback' };
  const alreadyReducedStartupPlan = tagPlanForRun({
    zones: [{ sys_id: 2 }, { sys_id: 3 }],
  }, disconnectFallbackRun);
  assert.strictEqual(
    shouldFallbackDisconnectZoneRemoval(
      disconnectFallbackRun,
      alreadyReducedStartupPlan,
      alreadyReducedStartupPlan.zones,
      alreadyReducedStartupPlan.zones,
      1,
    ),
    false,
    'startup plan already reduced during disconnect await does not lose another UAV',
  );
  const legacyUnmatchedPlan = { zones: [{ track: [{}] }, { track: [{}] }] };
  assert.strictEqual(
    shouldFallbackDisconnectZoneRemoval(
      disconnectFallbackRun,
      legacyUnmatchedPlan,
      legacyUnmatchedPlan.zones,
      legacyUnmatchedPlan.zones,
      1,
    ),
    true,
    'untagged zones without sys_id retain the positional disconnect fallback',
  );

  // A plan outside the startup lineage is operator-owned. It cancels the
  // continuation before a late UAV can overwrite that newer work.
  const replacedPlan = { zones: [{ sys_id: 99 }] };
  assert.deepStrictEqual(
    selectStartupMissionVehicles(startupRun, [{ sys_id: 4 }], replacedPlan),
    [],
    'operator replacement blocks late startup additions',
  );
  assert.strictEqual(startupRun.cancelled, true, 'operator replacement cancels the startup run');

  // Mission responses may finish in any order, but zones and fleet-wide
  // metadata remain tied to startup roster order.
  const orderedRun = {
    orderedIds: [1, 2, 3],
    excludedIds: new Set(),
    missionsBySysId: new Map([
      [2, { search_pattern: 'from-uav-2' }],
      [1, { search_pattern: 'from-uav-1' }],
      [3, { search_pattern: 'from-uav-3' }],
    ]),
  };
  assert.deepStrictEqual(
    startupMissionAuthority(orderedRun),
    { sysId: 1, mission: { search_pattern: 'from-uav-1' } },
    'first roster mission is the deterministic fleet metadata authority',
  );
  orderedRun.excludedIds.add(1);
  assert.deepStrictEqual(
    startupMissionAuthority(orderedRun),
    { sysId: 2, mission: { search_pattern: 'from-uav-2' } },
    'authority advances deterministically when the first UAV disconnects',
  );

  const planMetadataRun = {
    orderedIds: [1, 2, 3],
    excludedIds: new Set(),
    missionsBySysId: new Map([
      [1, { waypoints: [{}], altitude_m: 140 }],
      [2, { waypoints: [{}], altitude_m: 120 }],
      [3, { waypoints: [{}], altitude_m: 100 }],
    ]),
  };
  assert.deepStrictEqual(
    startupPlanMetadata(planMetadataRun, 80),
    { altitude: 100 },
    'fleet base altitude is last ordered altitude',
  );
  planMetadataRun.excludedIds.add(3);
  assert.deepStrictEqual(
    startupPlanMetadata(planMetadataRun, 80),
    { altitude: 120 },
    'fleet metadata recomputes after the lowest mission disconnects',
  );
  const metadataOwnedPlan = tagPlanForRun({ zones: [], altitude_m: 80 }, planMetadataRun);
  assert.deepStrictEqual(
    startupPlanMetadataForOwnedPlan(planMetadataRun, metadataOwnedPlan, 80),
    { altitude: 120 },
    'startup-owned plan may receive recomputed mission metadata',
  );
  assert.strictEqual(
    startupPlanMetadataForOwnedPlan(
      planMetadataRun,
      { zones: [{ sys_id: 99 }], altitude_m: 65 },
      65,
    ),
    null,
    'operator-owned plan rejects stale startup metadata during disconnect',
  );

  const rawZoneRun = {
    orderedIds: [1, 2],
    excludedIds: new Set(),
    missionsBySysId: new Map([
      [2, {
        waypoints: [{ lat: 20, lon: 20, alt: 120 }],
        corridor_end_index: 0,
        altitude_m: 120,
      }],
      [1, {
        waypoints: [{ lat: 10, lon: 10 }, { lat: 11, lon: 11 }],
        corridor_end_index: 1,
        altitude_m: 100,
      }],
    ]),
  };
  const rawZones = startupZonesFromMissions(rawZoneRun);
  assert.deepStrictEqual(rawZones.map((z) => z.sys_id), [1, 2], 'raw zones follow roster order');
  assert.deepStrictEqual(rawZones.map((z) => z.zone_index), [0, 1], 'zone indices match sorted positions');
  assert.strictEqual(rawZones[0].corridor_end_index, 1, 'raw corridor offset survives prior rendered-plan trims');
  assert.strictEqual(rawZones[0].track.length, 2, 'raw corridor waypoints remain available to parameter mapping');

  // Concurrent mission completions retain fleet metadata by sys_id rather than
  // rebuilding earlier POIs from a stale React closure.
  const metadataRun = { missionFallbackLocationsBySysId: new Map() };
  const poi1 = { lat: 1, lon: 1 };
  const poi2 = { lat: 2, lon: 2 };
  metadataRun.missionFallbackLocationsBySysId.set(2, poi2); // completion order differs
  metadataRun.missionFallbackLocationsBySysId.set(1, poi1);
  assert.deepStrictEqual(
    startupMissionFallbackLocationsForZones(metadataRun, [{ sys_id: 1 }, { sys_id: 2 }, { sys_id: 3 }]),
    [poi1, poi2, null],
    'fallback locations accumulate in zone order regardless of completion order',
  );

  // Side effects are serialized so a slow earlier subset cannot finish after
  // and overwrite the final three-zone parameter state.
  const queueRun = { postProcessQueue: Promise.resolve() };
  const order = [];
  const first = enqueueStartupPostProcess(queueRun, async () => {
    order.push('first-start');
    await Promise.resolve();
    order.push('first-end');
  });
  const second = enqueueStartupPostProcess(queueRun, async () => {
    order.push('second');
  });
  await Promise.all([first, second]);
  assert.deepStrictEqual(order, ['first-start', 'first-end', 'second'], 'startup post-processing is serialized');

  // A disconnected/excluded vehicle is never enlisted again by a later roster
  // snapshot from stale telemetry.
  const exclusionRun = {
    cancelled: false,
    hasPublished: false,
    requestedIds: new Set(),
    orderedIds: [],
    excludedIds: new Set([2]),
  };
  assert.deepStrictEqual(
    selectStartupMissionVehicles(exclusionRun, [{ sys_id: 1 }, { sys_id: 2 }], null).map((v) => v.sys_id),
    [1],
    'excluded UAV is not enlisted by stale roster telemetry',
  );

  console.log('Plan download-run lineage regression passed');
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
