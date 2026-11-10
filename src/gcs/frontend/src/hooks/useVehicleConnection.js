import { useCallback, useEffect, useRef, useState } from 'react';
import { decodeBitmaskArray } from '../utils/bitmask';
import { reindexZoneMap } from '../utils/simDockReindex';
import { assembleMissionsFromResults, mergeDownloadedMissionZones, missionAnswerKind } from '../utils/missionAssembly';
import { reconcileMissionDownloads, waitForBackendMissionSettlement } from '../utils/missionReconciliation';
import {
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
  startupMissionDeliveryHubsForZones,
  enqueueStartupPostProcess,
} from '../utils/planDownloadRun';
import { prunePendingVehicles, upsertPendingVehicles } from '../utils/pendingVehicles';
import { fenceObservationFromDownload } from '../utils/fenceIntent';
import { emitDiagnostic } from '../utils/diagnostics';
import { requestDeadline } from '../utils/simDeadline';

// The best-effort AAS read after a plan download; its sim-time deadline on lockstep.
const PARAMS_TIMEOUT_MS = 10000;
// Tags a plan a single UAV's download published (a connect, not a run).

/**
 * True when one download pass found no mission on any connected UAV: every
 * vehicle connected now was asked in this pass and answered with an empty
 * mission (a failed answer is not "no mission").
 */
export function fleetHasNoMission(answerKinds, connectedIds) {
  const ids = new Set(connectedIds);
  for (const id of answerKinds.keys()) ids.add(id);
  if (ids.size === 0) return false;
  for (const id of ids) {
    if (answerKinds.get(id) !== 'empty') return false;
  }
  return true;
}

/**
 * Vehicle connect, disconnect, scan, and download plan handlers.
 */
export default function useVehicleConnection({
  mission,
  vehicleList, removeVehicle, api, telemetryStoreRef,
  derivePlanPolygon, onPlanSynced,
  // Whether the editor holds exactly the plan on the UAVs, and forgetting
  // that plan once no UAV is left to carry it (usePlanSnapshot).
  isPlanSynced, forgetPlanSync,
  // A download's baseline is what it published, never an edit made while it
  // ran (usePlanSnapshot): beginPlanDownload as it starts, notePlanPublished
  // in the update that publishes its plan and derives the area.
  beginPlanDownload, notePlanPublished,
  // Called when a connect finds no mission on any connected UAV.
  onFleetWithoutMission,
  settings,
  setVehicleTargWps, setVehicleNavLastWp, setNotification,
}) {
  const {
    polygon, plan,
    setPlan, setPolygon, setAnalysis,
    setSetLaunchPoints, setSetCorridorPoints, setActiveSetIndex,
    deliveryHubAssignments,
    setSimDockWps, setDetectAfterWps,
    setFenceEnabled, setFenceTouched, setFenceCustomVertices,
    setExclusionPolygons,
    beginFenceObservation, recordFenceObservation, resetFenceObservations,
    reserveFenceObservations, clearFenceIntent, fenceObservations,
  } = mission;

  // Read one vehicle's stored geofence into the OBSERVATION map.
  //
  // Deliberately display-only. Writing it into fenceCustomVertices/fenceEnabled
  // (as this used to) made one vehicle's stored ring look like geometry the
  // operator had drawn, and marked the fence "touched" — so the very next
  // upload re-sent an explicit fence request nobody made: the downloaded ring
  // pushed to the whole fleet, or, for a vehicle with an empty fence table, an
  // explicit disable of everyone's fence.
  //
  // The request token is taken BEFORE the request, so a result that lands after
  // a plan reset, a roster change, an operator edit, or a newer read of the
  // same vehicle is discarded instead of overwriting current state.
  const observeFenceFromVehicle = useCallback(async (sysId, isActive = () => true) => {
    const token = beginFenceObservation();
    // Reserve AT this request's sequence, so starting a read also raises the
    // vehicle's ordering watermark: an older read still in flight can no longer
    // land just because this one has not answered yet.
    reserveFenceObservations?.([sysId], token.seq);
    const fence = await api.downloadFence?.(sysId);
    if (!isActive()) return;
    // A missing response is an unreadable fence, not an absent one.
    recordFenceObservation(sysId, fenceObservationFromDownload(fence), token);
  }, [api.downloadFence, beginFenceObservation, recordFenceObservation,
      reserveFenceObservations]);

  // Each vehicle carries its own fence — observe them all, never extrapolate
  // one vehicle's answer onto the fleet.
  const observeFleetFence = useCallback(async (vehicles, isActive = () => true) => {
    const sysIds = (vehicles || []).map((v) => v.sys_id).filter((id) => id != null);
    // Reserve first: between publishing a plan and hearing back, the fleet's
    // fence must already read as "asked, not yet known".
    reserveFenceObservations?.(sysIds);
    await Promise.all(sysIds.map((sysId) => observeFenceFromVehicle(sysId, isActive)));
  }, [observeFenceFromVehicle, reserveFenceObservations]);

  // A loaded plan file starts an observation lifecycle that reads the fleet
  // (resetFenceObservations readFleet): read every connected vehicle, and
  // every vehicle that joins the roster later in it, a reconnect included.
  // Answers from a later lifecycle are dropped by their token.
  const fleetReadRef = useRef({ epoch: null, roster: new Set() });
  const observationEpoch = fenceObservations?.epoch;
  const readsFleet = !!fenceObservations?.readsFleet;
  const rosterKey = vehicleList.map((v) => v.sys_id).join(',');
  useEffect(() => {
    if (!readsFleet) return;
    const read = fleetReadRef.current;
    const seen = read.epoch === observationEpoch ? read.roster : new Set();
    fleetReadRef.current = {
      epoch: observationEpoch, roster: new Set(vehicleList.map((v) => v.sys_id)),
    };
    const joined = vehicleList.filter((v) => !seen.has(v.sys_id));
    if (joined.length > 0) observeFleetFence(joined);
  }, [observationEpoch, readsFleet, rosterKey]);
  // Ref tracks latest plan to avoid stale closures after awaited API calls
  const planRef = useRef(plan);
  planRef.current = plan;
  const connectedIdsRef = useRef([]);
  connectedIdsRef.current = vehicleList.map((v) => v.sys_id);
  const reportIfFleetHasNoMission = useCallback((answerKinds) => {
    if (fleetHasNoMission(answerKinds, connectedIdsRef.current)) onFleetWithoutMission?.();
  }, [onFleetWithoutMission]);
  // Track which sys_ids have an active mission download
  const [downloadingSysIds, setDownloadingSysIds] = useState(() => new Set());
  const [pendingVehicles, setPendingVehicles] = useState(() => []);
  const downloadOwnersRef = useRef(new Map());
  const addDownloading = useCallback((ids, owner) => {
    addDownloadOwner(downloadOwnersRef.current, ids, owner);
    setDownloadingSysIds(activeDownloadIds(downloadOwnersRef.current));
  }, []);
  const removeDownloading = useCallback((ids, owner) => {
    removeDownloadOwner(downloadOwnersRef.current, ids, owner);
    setDownloadingSysIds(activeDownloadIds(downloadOwnersRef.current));
  }, []);
  const clearDownloading = useCallback((ids) => {
    clearDownloadOwners(downloadOwnersRef.current, ids);
    setDownloadingSysIds(activeDownloadIds(downloadOwnersRef.current));
  }, []);
  const missionDownloadRunRef = useRef(null);
  const startupMissionStateRef = useRef({ disabled: false, run: null });

  const cancelMissionDownloadRun = useCallback(() => {
    const run = missionDownloadRunRef.current;
    if (run) {
      run.cancelled = true;
      run.controller.abort();
      removeDownloading(run.sysIds, run.downloadOwner);
    }
    missionDownloadRunRef.current = null;
  }, [removeDownloading]);

  const beginMissionDownloadRun = useCallback((sysIds) => {
    cancelMissionDownloadRun();
    const run = {
      cancelled: false,
      controller: new AbortController(),
      sysIds: [...sysIds],
      downloadOwner: {},
      hasPublished: false,
    };
    missionDownloadRunRef.current = run;
    addDownloading(sysIds, run.downloadOwner);
    return run;
  }, [addDownloading, cancelMissionDownloadRun]);

  const waitForBackendSettlement = useCallback((sysId, run) => (
    waitForBackendMissionSettlement(telemetryStoreRef.current, sysId, run.controller.signal)
  ), [telemetryStoreRef]);

  const disableStartupMissionReconciliation = useCallback(() => {
    const state = startupMissionStateRef.current;
    state.disabled = true;
    const run = state.run;
    if (!run || run.cancelled) return;
    run.cancelled = true;
    run.controller.abort();
  }, []);
  const rememberPending = useCallback((vehicles) => {
    setPendingVehicles((prev) => upsertPendingVehicles(prev, vehicles));
  }, []);
  const forgetPending = useCallback((ids) => {
    setPendingVehicles((prev) => prunePendingVehicles(prev, [], ids));
  }, []);

  useEffect(() => {
    setPendingVehicles((prev) => prunePendingVehicles(prev, vehicleList));
  }, [vehicleList]);

  useEffect(() => () => cancelMissionDownloadRun(), [cancelMissionDownloadRun]);
  // Fetch vehicle params and apply simDockWps / detectAfterWps / vehicleTargWps.
  // Returns the dock and detection waypoints it restored, or null.
  const applySimParams = useCallback(async (zones, isActive = () => true) => {
    if (!zones?.length) return null;
    const sysIds = [...new Set(zones.map((z) => z.sys_id).filter(Boolean))];
    if (sysIds.length === 0) return null;
    const paramResults = await Promise.all(
      sysIds.map((sid) => {
        // Best-effort + bounded: a hung /params fetch must not stall the caller
        // (this runs after the download flag is cleared, but we still don't want
        // a dangling request); a failure/timeout just yields null.
        const t = requestDeadline(PARAMS_TIMEOUT_MS);
        return fetch(`/api/vehicles/${sid}/params`, { signal: t.signal, headers: t.headers })
          .then((r) => r.ok ? r.json() : null).catch(() => null)
          .finally(t.clear)
          .then((r) => ({
            sys_id: sid,
            targ_wps: parseInt(r?.params?.targ_wps) || 0,
            nav_last_wp: parseInt(r?.params?.nav_last_wp) || 0,
          }));
      })
    );
    const freshTargWps = {};
    const freshNavLastWp = {};
    for (const p of paramResults) {
      if (p.targ_wps > 0) freshTargWps[p.sys_id] = p.targ_wps;
      if (p.nav_last_wp > 0) freshNavLastWp[p.sys_id] = p.nav_last_wp;
    }
    if (!isActive()) return null;
    // Convert targ_wps bitmasks → simDockWps (track-index based)
    const newSimDockWps = {};
    for (let zi = 0; zi < zones.length; zi++) {
      const zone = zones[zi];
      const bitmask = freshTargWps[zone.sys_id];
      if (!bitmask) continue;
      const corridorLen = zone.corridor_end_index || 0;
      const trackLen = zone.track.length - corridorLen;
      const wpNums = decodeBitmaskArray(bitmask);
      const indices = [];
      for (const wpNum of wpNums) {
        const trackIdx = wpNum - 1 - corridorLen;
        if (trackIdx >= 0 && trackIdx < trackLen) indices.push(trackIdx);
      }
      if (indices.length > 0) newSimDockWps[zi] = indices;
    }
    setSimDockWps(newSimDockWps);
    const restored = { simDockWps: newSimDockWps };
    // Convert nav_last_wp → detectAfterWps (track-index based)
    if (setDetectAfterWps) {
      const newDetectAfterWps = {};
      for (let zi = 0; zi < zones.length; zi++) {
        const zone = zones[zi];
        const navLast = freshNavLastWp[zone.sys_id];
        if (!navLast || navLast <= 0) continue;
        const corridorLen = zone.corridor_end_index || 0;
        const trackIdx = navLast - 1 - corridorLen;
        const trackLen = zone.track.length - corridorLen;
        if (trackIdx >= 0 && trackIdx < trackLen) {
          newDetectAfterWps[zi] = trackIdx;
        }
      }
      setDetectAfterWps(newDetectAfterWps);
      restored.detectAfterWps = newDetectAfterWps;
    }
    if (setVehicleTargWps && Object.keys(freshTargWps).length > 0) {
      setVehicleTargWps((prev) => ({ ...prev, ...freshTargWps }));
    }
    return restored;
  }, [setSimDockWps, setDetectAfterWps, setVehicleTargWps]);

  // Apply fleet-wide startup metadata from the latest raw mission roster. A
  // roster revision guard prevents an older async fence/parameter request from
  // writing after disconnect cleanup; the disconnect path queues a fresh pass.
  // Returns the baseline source for onPlanSynced, or null when not applied.
  const applyStartupPlanEffects = useCallback(async (run, isRequestActive = () => true) => {
    if (!run || !isRequestActive() || !planBelongsToRun(planRef.current, run)) return null;
    const rosterRevision = run.rosterRevision;
    const zones = startupZonesFromMissions(run);
    const authority = startupMissionAuthority(run);
    const planMetadata = startupPlanMetadata(run, planRef.current?.altitude_m || 100);
    if (!zones.length || !authority) return null;
    const isCurrent = () => (
      isRequestActive()
      && startupRosterIsCurrent(run, rosterRevision)
      && planBelongsToRun(planRef.current, run)
    );
    const metadataMission = authority.mission;
    reserveFenceObservations?.(zones.map((z) => z.sys_id));
    derivePlanPolygon(
      zones,
      metadataMission.search_pattern || 'distributed',
      metadataMission.polygon,
      metadataMission.corridor_backbone,
      metadataMission.launch_point,
      startupMissionDeliveryHubsForZones(run, zones),
    );
    notePlanPublished?.(run.planToken);
    await observeFleetFence(
      zones.map((z) => ({ sys_id: z.sys_id })).filter((v) => v.sys_id != null),
      isCurrent,
    );
    if (!isCurrent()) return null;
    const restored = await applySimParams(zones, isCurrent);
    return isCurrent() ? { download: run.planToken, ...restored } : null;
  }, [applySimParams, derivePlanPolygon, notePlanPublished, observeFleetFence, reserveFenceObservations]);

  const downloadFleetMissions = useCallback(async (vehicles) => {
    if (vehicles.length === 0) return;
    const run = beginMissionDownloadRun(vehicles.map((v) => v.sys_id));
    const planToken = beginPlanDownload?.() ?? null;
    let applyQueue = Promise.resolve();
    const answerKinds = new Map();

    await reconcileMissionDownloads({
      vehicles,
      downloadMission: api.downloadMission,
      isActive: () => missionDownloadRunRef.current === run && !run.cancelled,
      waitForBackendSettlement: (sysId) => waitForBackendSettlement(sysId, run),
      onAwaitingLate: (sysId) => {
        const vehicle = vehicles.find((v) => v.sys_id === sysId);
        setNotification?.(`Mission download still processing for ${vehicle?.name || 'UAV ' + sysId}`);
      },
      onResultsChanged: (missionResults) => {
        const assembled = assembleMissionsFromResults(missionResults);
        if (!assembled) return;
        // Serialize plan-side effects so a faster later result cannot be
        // overwritten by an earlier result's slower parameter fetch.
        applyQueue = applyQueue.then(async () => {
          if (missionDownloadRunRef.current !== run || run.cancelled) return;
          // A late cache result belongs only to the plan produced by this run.
          // If another operator action replaced that plan, cancel instead of
          // overwriting newer work with a stale download continuation. Compare
          // by run lineage, not object identity: this run's own corridor trim
          // (derivePlanPolygon) rebuilds the plan object, and that must NOT read
          // as an external replacement — otherwise the run cancels itself after
          // the first UAV and later vehicles never draw.
          if (run.hasPublished && !planBelongsToRun(planRef.current, run)) {
            cancelMissionDownloadRun();
            return;
          }
          const nextPlan = tagPlanForRun({
            zones: assembled.zones,
            altitude_m: assembled.altitude,
          }, run);
          // Claim the WHOLE roster's fence state in the same tick that
          // publishes the plan, and before it. Reserving only when the fence
          // requests go out leaves a window in which a published plan has no
          // fence observation, and the demo default fills it; reserving for
          // vehicles that never publish a plan would instead suppress that
          // default for a genuinely new one.
          reserveFenceObservations?.(vehicles.map((v) => v.sys_id));
          setAnalysis(null);
          setPlan(nextPlan);
          // Keep the ref current before React's next render so another result
          // already queued in this tick recognizes our own plan publication.
          planRef.current = nextPlan;
          run.hasPublished = true;
          derivePlanPolygon(assembled.zones, assembled.searchPattern, assembled.polygon, assembled.corridorBackbone, assembled.launchPoint, assembled.missionDeliveryHubs);
          notePlanPublished?.(planToken);
          // Observe every vehicle's own fence. They are configured per vehicle
          // and can legitimately disagree, so the first vehicle's answer is not
          // the fleet's answer.
          await observeFleetFence(vehicles, () => (
            missionDownloadRunRef.current === run && !run.cancelled
          ));
          setNotification?.(`Missions downloaded from ${assembled.zones.length} vehicle${assembled.zones.length > 1 ? 's' : ''}`);
          const stillThisRun = () => (
            missionDownloadRunRef.current === run
            && !run.cancelled
            && planBelongsToRun(planRef.current, run)
          );
          const restored = await applySimParams(assembled.zones, stillThisRun);
          // The synced baseline includes the dock and detection waypoints the
          // vehicles' parameters just restored, or it would read as edited.
          if (stillThisRun()) onPlanSynced?.({ download: planToken, ...restored });
        });
      },
      onVehicleSettled: (sysId, result) => {
        removeDownloading([sysId], run.downloadOwner);
        answerKinds.set(sysId, missionAnswerKind(result));
        emitDiagnostic(result.error ? 'mission_result_rejected' : 'mission_result_accepted', {
          request_id: result._diagnostic_request_id, sys_id: sysId,
          outcome: result.error === 'timeout' ? 'timeout' : result.error ? 'failure' : 'accepted',
          total: result.waypoints?.length || 0,
        });
        const label = vehicles.find((v) => v.sys_id === sysId)?.name || 'UAV ' + sysId;
        if (result.error) {
          setNotification?.(`Mission download ${result.error === 'timeout' ? 'timed out' : 'failed'} for ${label}`);
        } else if (missionAnswerKind(result) === 'empty') {
          setNotification?.(`No mission on ${label}`);
        }
      },
    });
    await applyQueue;
    if (missionDownloadRunRef.current === run) {
      run.sysIds = [];
      missionDownloadRunRef.current = null;
      if (!run.cancelled) reportIfFleetHasNoMission(answerKinds);
    }
  }, [api.downloadMission, applySimParams, beginMissionDownloadRun, beginPlanDownload, cancelMissionDownloadRun, derivePlanPolygon, notePlanPublished, observeFleetFence, onPlanSynced, removeDownloading, reportIfFleetHasNoMission, reserveFenceObservations, setAnalysis, setNotification, setPlan, waitForBackendSettlement]);

  // Download missions from all connected vehicles -> assemble into plan.
  const handleDownloadPlan = useCallback(async () => {
    disableStartupMissionReconciliation();
    await downloadFleetMissions(vehicleList);
  }, [disableStartupMissionReconciliation, downloadFleetMissions, vehicleList]);

  // Disconnect + remove from telemetry state + remove zone from plan
  const handleDisconnect = useCallback(async (sysId) => {
    // The remaining UAVs keep their missions: a plan that was on the fleet
    // stays on it once the leaving UAV's zone is dropped.
    const wasSynced = !!isPlanSynced?.();
    if (missionDownloadRunRef.current?.sysIds.includes(sysId)) cancelMissionDownloadRun();
    // A page-refresh request can outlive the disconnect HTTP call. Exclude the
    // vehicle before awaiting so a late mission response cannot resurrect its
    // zone after the disconnect path removes it.
    const startupRun = startupMissionStateRef.current.run;
    markStartupRosterChanged(startupRun);
    startupRun?.excludedIds.add(sysId);
    startupRun?.missionDeliveryHubsBySysId.delete(sysId);
    startupRun?.missionsBySysId.delete(sysId);
    startupRun?.answerKinds?.delete(sysId);
    const result = await api.disconnectVehicle(sysId);
    // An edit made while the request was out is not on the fleet.
    const keepSynced = wasSynced && !!isPlanSynced?.();
    const removedIds = result?.removed || [sysId];
    for (const id of removedIds) {
      startupRun?.excludedIds.add(id);
      startupRun?.missionDeliveryHubsBySysId.delete(id);
      startupRun?.missionsBySysId.delete(id);
      startupRun?.answerKinds?.delete(id);
    }
    const removedSet = new Set(removedIds);
    for (const id of removedIds) removeVehicle(id);
    // Clear any in-flight "downloading" flag for the disconnected vehicle(s) so
    // a mid-download disconnect can't leave the plan/launch controls stuck.
    clearDownloading(removedIds);
    // Read latest plan via ref (avoids stale closure after await)
    const currentPlan = planRef.current;
    const oldZones = currentPlan?.zones || [];
    // Build kept indices — zones whose sys_id is NOT in the removed set.
    // Zones without sys_id are unmatched; handled by the fallback below.
    const keptOldIndices = [];
    for (let i = 0; i < oldZones.length; i++) {
      if (!removedSet.has(oldZones[i].sys_id)) keptOldIndices.push(i);
    }
    let remaining = oldZones
      .filter((z) => !removedSet.has(z.sys_id))
      .map((z, i) => ({ ...z, zone_index: i }));
    // Robustness: if sys_id filtering didn't remove anything (zones lack sys_id),
    // fall back to removing the last N zones to match the reduced vehicle count.
    if (shouldFallbackDisconnectZoneRemoval(
      startupRun, currentPlan, oldZones, remaining, removedIds.length,
    )) {
      const keepCount = Math.max(0, oldZones.length - removedIds.length);
      remaining = oldZones.slice(0, keepCount).map((z, i) => ({ ...z, zone_index: i }));
      keptOldIndices.length = 0;
      for (let i = 0; i < keepCount; i++) keptOldIndices.push(i);
    }
    if (remaining.length === 0) {
      // Clearing the last zone is an internal continuation of this startup run,
      // not an operator replacement. Reset its publication state synchronously
      // so another mission already in flight can become the new first publish.
      if (resetStartupPlanAfterDisconnect(startupRun, currentPlan)) {
        planRef.current = null;
      }
      setPlan(null);
      setPolygon([]);
      setAnalysis(null);
      setSetLaunchPoints([null]);
      setSetCorridorPoints([[]]);
      setActiveSetIndex(0);
      setSimDockWps({});
      if (setDetectAfterWps) setDetectAfterWps({});
      // Fence + keep-outs are plan geometry — clean them with the rest when
      // the last vehicle disconnects (same lifecycle as the zone). The
      // observations and any unacknowledged request belong to that plan too.
      setFenceCustomVertices(null);
      setExclusionPolygons([]);
      setFenceEnabled(false);
      setFenceTouched(false);
      clearFenceIntent?.();
      resetFenceObservations?.();
      // No UAV is left to carry the synced plan.
      forgetPlanSync?.();
    } else {
      const startupMetadata = startupPlanMetadataForOwnedPlan(
        startupRun, currentPlan, currentPlan?.altitude_m || 100,
      );
      const nextPlan = currentPlan ? {
        ...currentPlan,
        zones: remaining,
        ...(startupMetadata && { altitude_m: startupMetadata.altitude }),
      } : null;
      planRef.current = nextPlan;
      setPlan(nextPlan);
      // Reindex zone-keyed maps to match new zone positions.
      setSimDockWps((prev) => reindexZoneMap(prev, keptOldIndices, 'array'));
      if (setDetectAfterWps) {
        setDetectAfterWps((prev) => reindexZoneMap(prev, keptOldIndices, 'scalar'));
      }
      derivePlanPolygon(remaining);
      // Not on the fleet: the stored baseline still holds the leaving UAV's
      // zone, so it can be neither compared with nor restored.
      if (!keepSynced) forgetPlanSync?.();
      const startupOwnsPlan = !!startupRun && planBelongsToRun(nextPlan, startupRun);
      if (keepSynced && !startupOwnsPlan) onPlanSynced?.();
      if (startupOwnsPlan) {
        void enqueueStartupPostProcess(startupRun, async () => {
          const applied = await applyStartupPlanEffects(startupRun, () => (
            !startupRun.cancelled && planBelongsToRun(planRef.current, startupRun)
          ));
          if (applied && keepSynced) onPlanSynced?.(applied);
        }).catch((e) => console.warn('Startup plan refresh after disconnect failed:', e));
      }
    }
  }, [api.disconnectVehicle, applyStartupPlanEffects, cancelMissionDownloadRun, clearDownloading, removeVehicle, setPlan, setPolygon, setAnalysis, setSetLaunchPoints, setSetCorridorPoints, setActiveSetIndex, derivePlanPolygon, onPlanSynced, isPlanSynced, forgetPlanSync, setSimDockWps, setDetectAfterWps, setFenceCustomVertices, setExclusionPolygons, setFenceEnabled, setFenceTouched, clearFenceIntent, resetFenceObservations]);

  // Download mission for a single vehicle (background, tracked by downloadingSysIds)
  const downloadSingleMission = useCallback(async (sysId, name, startupRun = null) => {
    const downloadOwner = {};
    const startupRequestIsActive = () => (
      startupMissionRequestIsActive(startupRun, sysId)
    );
    // A page-load restore's token is its run's; a single connect is a download
    // of its own. Whether the editor held the UAVs' plan as it started: a
    // mission merged into it is the UAVs' plan only then.
    const planToken = startupRun ? null : (beginPlanDownload?.() ?? null);
    const syncedAtStart = !!isPlanSynced?.();
    addDownloading([sysId], downloadOwner);
    let m;
    try {
      m = await api.downloadMission(sysId);
      if (startupRun && m?.error === 'timeout') {
        setNotification?.(`Mission download still processing for ${name || 'UAV ' + sysId}`);
        await waitForBackendSettlement(sysId, startupRun);
        if (!startupRequestIsActive()) return;
        m = await api.downloadMission(sysId);
      }
    } finally {
      // Clear the indicator as soon as the download itself settles. Everything
      // below (plan wiring + best-effort applySimParams, which does its own
      // /params fetch) must NOT hold the "Downloading…" state.
      removeDownloading([sysId], downloadOwner);
    }
    if (!startupRequestIsActive()) return;
    if (startupRun && m) {
      const diagnostic = startupMissionResultDiagnostic(sysId, m);
      emitDiagnostic(diagnostic.event, diagnostic.fields);
    }
    // A page-load restore asks the UAVs one by one, so its answers gather in
    // the run; a single connect is a pass of its own.
    if (startupRun && !startupRun.answerKinds) startupRun.answerKinds = new Map();
    const answerKinds = startupRun ? startupRun.answerKinds : new Map();
    answerKinds.set(sysId, missionAnswerKind(m));
    if (missionAnswerKind(m) === 'empty') reportIfFleetHasNoMission(answerKinds);
    if (m?.error) {
      // Bounded failure (incl. the 45 s timeout) — surface it instead of leaving
      // the operator guessing.
      setNotification?.(`Mission download ${m.error === 'timeout' ? 'timed out' : 'failed'} for ${name || 'UAV ' + sysId}`);
      return;
    }
    if (!m?.waypoints?.length) {
      // An empty vehicle is not a failure: the startup restore stays quiet and
      // an operator's download says so.
      if (!startupRun && missionAnswerKind(m) === 'empty') setNotification?.(`No mission on ${name || 'UAV ' + sysId}`);
      return;
    }
    // Same ordering rule as the fleet path: this vehicle's fence state is
    // claimed before its mission can publish a plan. A vehicle that carries no
    // mission publishes nothing and is not claimed, so drawing a new plan still
    // gets the demo default.
    reserveFenceObservations?.([sysId]);
    try {
      // Merge against the latest plan via ref (avoids stale closure after awaits).
      // Two vehicles connected in quick succession run this concurrently; without
      // a synchronous ref update each would read the same pre-merge plan and the
      // later setPlan would drop the earlier vehicle's zone (last-write-wins).
      const currentPlan = planRef.current;
      if (startupRun) {
        const planWasReplaced = startupRun.hasPublished
          ? !planBelongsToRun(currentPlan, startupRun)
          : !!currentPlan;
        if (planWasReplaced) {
          startupRun.cancelled = true;
          startupRun.controller.abort();
          return;
        }
        startupRun.missionDeliveryHubsBySysId.set(sysId, m.default_delivery_hub || null);
        startupRun.missionsBySysId.set(sysId, m);
      }
      const mergedZones = startupRun
        ? startupZonesFromMissions(startupRun)
        : mergeDownloadedMissionZones(currentPlan?.zones, m, sysId);
      // The other zones' hubs below come from the editor: merged into a plan
      // the operator edited (before or during this download), the result is
      // not the UAVs' plan.
      const mergedIntoUavPlan = !currentPlan?.zones?.length
        || (syncedAtStart && !!isPlanSynced?.());
      const startupMetadata = startupRun
        ? startupPlanMetadata(startupRun, currentPlan?.altitude_m || 100)
        : null;
      let nextPlan = {
        ...currentPlan,
        zones: mergedZones,
        altitude_m: startupMetadata?.altitude || m.altitude_m || currentPlan?.altitude_m || 100,
      };
      if (startupRun) {
        nextPlan = tagPlanForRun(nextPlan, startupRun);
        startupRun.hasPublished = true;
      } else if (!currentPlan?.zones?.length) {
        // The UAV's own mission, and nothing else: the plan on the UAVs
        // (downloadRunOf), each download its own run. Merged into a plan that
        // already came from a download, the spread above keeps that plan's
        // tag; merged into the operator's own zones, it is not the UAVs' plan
        // and stays untagged.
        nextPlan = tagPlanForRun(nextPlan, { single: true });
      }
      // Publish the ref before React's next render so a concurrent single-vehicle
      // download merges on top of this zone instead of racing on a stale read.
      planRef.current = nextPlan;
      setPlan(nextPlan);
      const postProcess = async () => {
        if (!startupRequestIsActive()) return;
        if (startupRun && !planBelongsToRun(planRef.current, startupRun)) return;

        if (startupRun) {
          const applied = await applyStartupPlanEffects(startupRun, startupRequestIsActive);
          if (!applied) return;
          onPlanSynced?.(applied);
          setNotification?.(`Mission downloaded from ${name || 'UAV ' + sysId}`);
          return;
        }

        // Non-startup single-vehicle downloads retain their existing merge and
        // post-processing behavior.
        const zonesForEffects = mergedZones;
        const existingDeliveryHubs = settings?.default_delivery_hubs || [];
        const pois = zonesForEffects.map((z, i) => {
          if (i === zonesForEffects.length - 1) return m.default_delivery_hub || null;
          const deliveryHubIdx = deliveryHubAssignments?.[i];
          if (deliveryHubIdx != null && existingDeliveryHubs[deliveryHubIdx]) {
            return { lat: existingDeliveryHubs[deliveryHubIdx].lat, lon: existingDeliveryHubs[deliveryHubIdx].lon };
          }
          return null;
        });
        const metadataMission = m;
        derivePlanPolygon(
          zonesForEffects,
          metadataMission.search_pattern || "distributed",
          metadataMission.polygon,
          metadataMission.corridor_backbone,
          metadataMission.launch_point,
          pois,
        );
        notePlanPublished?.(planToken);
        await observeFenceFromVehicle(sysId, startupRequestIsActive);
        if (!startupRequestIsActive()) return;
        const restored = await applySimParams(zonesForEffects, startupRequestIsActive);
        if (!startupRequestIsActive()) return;
        if (mergedIntoUavPlan) onPlanSynced?.({ download: planToken, ...restored });
        else forgetPlanSync?.();
        setNotification?.(`Mission downloaded from ${name || 'UAV ' + sysId}`);
      };
      if (startupRun) {
        await enqueueStartupPostProcess(startupRun, postProcess);
      } else {
        await postProcess();
      }
    } catch (e) {
      console.warn('Mission post-processing failed (vehicle connected ok):', e);
    }
  }, [api.downloadMission, setPlan, derivePlanPolygon, beginPlanDownload, notePlanPublished, isPlanSynced, forgetPlanSync, observeFenceFromVehicle, onPlanSynced, deliveryHubAssignments, reserveFenceObservations, settings, applySimParams, applyStartupPlanEffects, setNotification, waitForBackendSettlement, reportIfFleetHasNoMission]);

  // Connect a single vehicle + auto-download its mission (only if no active plan)
  const handleConnect = useCallback(async (device, sysId, name) => {
    const connectOwner = {};
    // Mark as busy immediately so the sidebar shows "Connecting..." during the connect call
    rememberPending([{ sys_id: sysId, name }]);
    addDownloading([sysId], connectOwner);
    const result = await api.connectVehicle(device, sysId, name);
    if (result?.error) { removeDownloading([sysId], connectOwner); forgetPending([sysId]); return result; }
    const currentZones = planRef.current?.zones || [];
    const vehicleInPlan = currentZones.some((z) => z.sys_id === sysId);
    // Skip if vehicle already has a zone in the plan
    if (vehicleInPlan) {
      removeDownloading([sysId], connectOwner);
      return result;
    }
    // Skip if user is drawing a polygon but no plan exists yet (active planning)
    if (currentZones.length === 0 && polygon.length >= 3) {
      removeDownloading([sysId], connectOwner);
      return result;
    }
    disableStartupMissionReconciliation();
    removeDownloading([sysId], connectOwner);
    // Fire-and-forget: download runs in background, tracked by downloadingSysIds.
    // downloadSingleMission calls addDownloading/removeDownloading internally.
    downloadSingleMission(sysId, name);
    return result;
  }, [api.connectVehicle, polygon, downloadSingleMission, rememberPending, forgetPending, disableStartupMissionReconciliation]);

  // Download missions from all given vehicles (background, tracked by downloadingSysIds)
  const downloadScannedMissions = useCallback(async (vehicles) => {
    await downloadFleetMissions(vehicles);
  }, [downloadFleetMissions]);

  // Scan: discover + connect all + auto-download missions (only if no active plan)
  const handleScan = useCallback(async (device) => {
    const result = await api.discoverVehicles(device);
    if (result?.found > 0 && result.vehicles) {
      rememberPending(result.vehicles);
      // Skip auto-download only if user is actively planning with no uploaded plan
      const currentZones = planRef.current?.zones || [];
      if (currentZones.length === 0 && polygon.length >= 3) return result;
      disableStartupMissionReconciliation();
      // Fire-and-forget: download runs in background, tracked by downloadingSysIds
      downloadScannedMissions(result.vehicles);
    }
    return result;
  }, [api.discoverVehicles, polygon, downloadScannedMissions, rememberPending, disableStartupMissionReconciliation]);

  // Auto-fetch per-vehicle targ_wps / nav_last_wp once per connect.
  // vehicleList is a new array on every telemetry update (~5 Hz), so a sys_id
  // is claimed when its fetch starts, not when it answers: otherwise every
  // update during the pending fetch sends another one. A sys_id that leaves
  // the list (disconnect) is forgotten, so a reconnect fetches again, and an
  // answer from before that is dropped.
  const targWpsFetchByIdRef = useRef(new Map());
  useEffect(() => {
    const claims = targWpsFetchByIdRef.current;
    const live = new Set(vehicleList.map((v) => v.sys_id));
    for (const sid of [...claims.keys()]) {
      if (!live.has(sid)) claims.delete(sid);
    }
    const newIds = vehicleList.filter((v) => !claims.has(v.sys_id));
    if (newIds.length === 0) return;
    const claim = {};
    newIds.forEach((v) => claims.set(v.sys_id, claim));
    Promise.all(
      newIds.map((v) =>
        fetch(`/api/vehicles/${v.sys_id}/params`).then((r) => r.ok ? r.json() : null).catch(() => null)
      )
    ).then((results) => {
      const twUpdates = {};
      const nlUpdates = {};
      results.forEach((r, i) => {
        const sid = newIds[i].sys_id;
        if (claims.get(sid) !== claim) return;
        const tw = parseInt(r?.params?.targ_wps) || 0;
        const nl = parseInt(r?.params?.nav_last_wp) || 0;
        if (tw > 0) twUpdates[sid] = tw;
        if (nl > 0) nlUpdates[sid] = nl;
      });
      if (Object.keys(twUpdates).length > 0) {
        setVehicleTargWps((prev) => ({ ...prev, ...twUpdates }));
      }
      if (Object.keys(nlUpdates).length > 0) {
        setVehicleNavLastWp((prev) => ({ ...prev, ...nlUpdates }));
      }
    });
  }, [vehicleList]);

  // Auto-download missions on page refresh. Backend auto-connect can expose the
  // roster one UAV at a time, so keep enlisting newly visible sys_ids instead
  // of latching permanently to the first non-empty vehicleList snapshot.
  useEffect(() => {
    const state = startupMissionStateRef.current;
    if (state.disabled) return;
    if (!state.run) {
      state.run = {
        cancelled: false,
        controller: new AbortController(),
        requestedIds: new Set(),
        orderedIds: [],
        excludedIds: new Set(),
        missionDeliveryHubsBySysId: new Map(),
        missionsBySysId: new Map(),
        answerKinds: new Map(),
        postProcessQueue: Promise.resolve(),
        rosterRevision: 0,
        hasPublished: false,
        // The operator's decisions as the restore starts (usePlanSnapshot).
        planToken: beginPlanDownload?.() ?? null,
      };
    }
    const run = state.run;
    const newVehicles = selectStartupMissionVehicles(run, vehicleList, planRef.current);
    if (run.cancelled) {
      run.controller.abort();
      return;
    }
    for (const vehicle of newVehicles) {
      // Each result merges synchronously through planRef, so concurrent arrivals
      // accumulate without last-write-wins loss. The startup lineage guard in
      // downloadSingleMission blocks a late result after operator replacement.
      downloadSingleMission(vehicle.sys_id, vehicle.name, run);
    }
  }, [vehicleList, plan, downloadSingleMission]);

  return {
    handleConnect,
    handleDisconnect,
    handleScan,
    handleDownloadPlan,
    // Exposed so the fence observation path is testable on its own and so a
    // caller can refresh one vehicle's observed fence without a plan download.
    observeFenceFromVehicle,
    observeFleetFence,
    cancelStartupMissionReconciliation: disableStartupMissionReconciliation,
    downloadingSysIds,
    pendingVehicles,
  };
}
