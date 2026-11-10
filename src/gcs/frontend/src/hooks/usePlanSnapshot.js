import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { buildPlanSnapshot, planEditKey, planHasContent, planSyncKey } from '../utils/planSnapshot';
import { downloadRunOf } from '../utils/planDownloadRun';
import useConfirm from './useConfirm';

const NO_SYNC = { key: null, run: null };

/**
 * The plan the UAVs carry once a request completed, from what the request
 * itself sent or read: never the editor as it is at completion, which may hold
 * an edit made while the request was out.
 *
 * - An upload: the editor as the upload read it (`upload`, taken before
 *   anything was awaited) with the plan and fence request it sent.
 * - A download: the editor as the download published its plan and derived
 *   the area from it (`download`, a beginPlanDownload token, see
 *   publishedState), with the dock and detection waypoints its parameter read
 *   restored, when it did.
 * - Neither (a UAV left a fleet whose plan the editor still holds): the editor.
 *
 * What the UAVs acknowledged and report about their fences is the latest, as
 * the editor holds it.
 */
function syncedState(editor, source) {
  const latestFence = {
    fenceAcknowledged: editor.fenceAcknowledged,
    fenceObservations: editor.fenceObservations,
  };
  if (source?.upload) return { ...source.upload, ...latestFence };
  if (source?.download) {
    const published = source.download.state ?? editor;
    return {
      ...published,
      simDockWps: source.simDockWps ?? published.simDockWps,
      detectAfterWps: source.detectAfterWps ?? published.detectAfterWps,
      ...latestFence,
    };
  }
  return editor;
}

/**
 * The editor a download's publication leaves, without the edits the operator
 * made while that download was running. A download publishes progressively
 * (one UAV's mission at a time), so a later publication starts from the state
 * recorded for the same download's plan (`chain`), not from the editor, and
 * takes from the editor only what the publication itself changed (`before` is
 * the editor just before it). The plan is always the published one: a plan
 * replaced by anything else ends the download before its baseline is taken.
 */
function publishedState(before, after, chain) {
  const run = downloadRunOf(after.plan);
  const base = run != null && chain?.run === run ? chain.state : before;
  const state = { ...base, plan: after.plan };
  for (const key of Object.keys(after)) {
    if (!Object.is(after[key], before[key])) state[key] = after[key];
  }
  return state;
}

/**
 * The plan's two baselines and leaving plan mode.
 *
 * - Synced: the plan on the UAVs, taken when an upload or download completes
 *   (onPlanSynced) from what that request sent or read (syncedState).
 *   `planSynced` says the editor still holds exactly that plan, or a plan a
 *   download published whose baseline has not landed yet (downloadRunOf); the
 *   monitor offers the start control only then. An upload reads the editor
 *   before it awaits anything (capturePlanState) and drops the baseline when
 *   it starts writing (beginPlanUpload): until it completes, what the UAVs
 *   carry is not known. A download marks the editor it published
 *   (beginPlanDownload).
 * - Saved: the plan in the file last saved (markPlanSaved, taken at once) or
 *   loaded (markPlanLoaded, taken once the load has committed).
 *
 * Leaving plan mode with a plan that is in neither baseline asks whether to
 * save it first. Leaving with a plan that is not the one on the UAVs returns
 * the map to the UAVs' plan, so the monitor always shows what they fly.
 */
export default function usePlanSnapshot({
  // Captured values
  plan, polygon, searchPattern, analysis, uavCount, effectiveUavCount,
  partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr,
  deliveryHubAssignments, manualDeliveryHubEdit, simDockWps, detectAfterWps,
  fenceTouched, fenceEnabled, fenceOffsetM, fenceIntent, fenceAcknowledged, fenceObservations,
  fenceCustomVertices, exclusionPolygons,
  // The settings' home hubs (not restored: only the assigned positions count).
  deliveryHubs,
  // Restore setters
  setPlan, setPolygon, setSearchPattern, setAnalysis,
  setUavCount, setPartitionAngleDeg, setRouteOffsetM, setSetLaunchPoints,
  setSetCorridorPoints, setDeliveryHubAssignments, setManualDeliveryHubEdit,
  setSimDockWps, setDetectAfterWps, restoreFenceState,
  setFenceCustomVertices, setExclusionPolygons,
  // Side-effect deps (drawingRef avoids hook-ordering issues with useDrawing)
  drawingRef, suppressRegenRef, goToMonitor, undoRef,
  // Writes the plan file (usePlanPersistence, created after this hook).
  savePlanRef,
}) {
  const { t } = useTranslation();
  const confirm = useConfirm();
  const vehicleSnapshotRef = useRef(null);
  const stateForSnapshotRef = useRef({});
  // Each request for a synced baseline, with what it sent or read (syncedState).
  const [snapshotTrigger, setSnapshotTrigger] = useState({ n: 0, source: null });
  // Downloads that published a plan whose editor state is not captured yet.
  const [publishTrigger, setPublishTrigger] = useState(0);
  const pendingDownloadsRef = useRef([]);
  // The last state recorded for a downloaded plan ({ run, state }): the next
  // publication of that plan builds on it (publishedState).
  const downloadChainRef = useRef(null);
  // The synced baseline's key, and the download run of the plan it was taken
  // from: a download's provisional claim ends once its own baseline is taken.
  const [sync, setSync] = useState(NO_SYNC);
  const [savedTrigger, setSavedTrigger] = useState(0);
  const savedKeyRef = useRef(null);

  const current = {
    plan, polygon, searchPattern, analysis, uavCount, effectiveUavCount,
    partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr,
    deliveryHubAssignments, manualDeliveryHubEdit, simDockWps, detectAfterWps,
    fenceTouched, fenceEnabled, fenceOffsetM, fenceIntent, fenceAcknowledged, fenceObservations,
    fenceCustomVertices, exclusionPolygons, deliveryHubs,
  };
  stateForSnapshotRef.current = current;

  // `source` is what the completed request sent or read (syncedState).
  const onPlanSynced = useCallback((source = null) => setSnapshotTrigger((prev) => ({
    n: prev.n + 1, source,
  })), []);
  // The editor as it is now: what an upload builds its request from.
  const capturePlanState = useCallback(() => ({ ...stateForSnapshotRef.current }), []);
  // Called by a download in the same update that publishes its plan and
  // derives the area: the returned token holds the editor once that update
  // has committed, for onPlanSynced({ download: token }).
  const beginPlanDownload = useCallback(() => {
    // The updates of this publication are not rendered yet: the ref holds the
    // editor just before it.
    const token = { before: { ...stateForSnapshotRef.current }, state: null };
    pendingDownloadsRef.current.push(token);
    setPublishTrigger((n) => n + 1);
    return token;
  }, []);
  // Saved: the file holds the plan as it is now (leaving plan mode may restore
  // the UAVs' plan right after). Loaded: called in the same update that
  // applies the file, so the key is taken once that update has committed.
  const markPlanSaved = useCallback(() => {
    savedKeyRef.current = planEditKey(stateForSnapshotRef.current);
  }, []);
  const markPlanLoaded = useCallback(() => setSavedTrigger(t => t + 1), []);
  // What the UAVs carry is no longer known (an upload is writing) or no UAV is
  // left to carry it: nothing to compare with or restore. The plan in the
  // editor loses any download's claim too.
  const forgetPlanSync = useCallback(() => {
    vehicleSnapshotRef.current = null;
    downloadChainRef.current = null;
    setSync({ key: null, run: downloadRunOf(stateForSnapshotRef.current.plan) });
  }, []);
  // An upload starts writing: what the UAVs carry is unknown until it completes.
  const beginPlanUpload = forgetPlanSync;

  // Declared before the baseline effect: a download that publishes and
  // completes within one update is captured first.
  useEffect(() => {
    if (publishTrigger === 0) return;
    const after = stateForSnapshotRef.current;
    for (const token of pendingDownloadsRef.current) {
      token.state = publishedState(token.before, after, downloadChainRef.current);
      downloadChainRef.current = { run: downloadRunOf(after.plan), state: token.state };
    }
    pendingDownloadsRef.current = [];
  }, [publishTrigger]);

  useEffect(() => {
    if (snapshotTrigger.n === 0) return;
    const state = syncedState(stateForSnapshotRef.current, snapshotTrigger.source);
    vehicleSnapshotRef.current = buildPlanSnapshot(state);
    const run = downloadRunOf(state.plan);
    // A later download into this plan builds on the baseline just taken.
    downloadChainRef.current = run != null ? { run, state } : null;
    setSync({ key: planSyncKey(state), run });
  }, [snapshotTrigger]);

  useEffect(() => {
    if (savedTrigger === 0) return;
    savedKeyRef.current = planEditKey(stateForSnapshotRef.current);
  }, [savedTrigger]);

  const currentSyncKey = useMemo(
    () => planSyncKey(current),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [plan, polygon, searchPattern, effectiveUavCount, partitionAngleDeg, routeOffsetM,
      setLaunchPoints, setCorridorPointsArr, deliveryHubAssignments, manualDeliveryHubEdit,
      simDockWps, detectAfterWps, fenceTouched, fenceEnabled, fenceOffsetM,
      fenceCustomVertices, exclusionPolygons, deliveryHubs],
  );
  // A plan a download just published is the UAVs' plan before its baseline
  // lands (the download's post-processing derives the area and dock picks
  // first), or the monitor would call it "not on the UAVs" meanwhile. Once
  // the baseline was taken from that download, only the key decides: an edit
  // (picks, hubs, fence) must read as a change, also on a copy of the plan.
  const run = downloadRunOf(plan);
  const provisional = run != null && run !== sync.run;
  const planSynced = (sync.key != null && currentSyncKey === sync.key) || provisional;
  const planSyncedRef = useRef(planSynced);
  planSyncedRef.current = planSynced;
  const isPlanSynced = useCallback(() => planSyncedRef.current, []);

  const restoreVehiclePlan = useCallback((snap) => {
    suppressRegenRef.current = true;
    setPlan(snap.plan);
    setPolygon(snap.polygon);
    setSearchPattern(snap.searchPattern);
    setAnalysis(snap.analysis);
    setUavCount(snap.uavCount);
    setPartitionAngleDeg(snap.partitionAngleDeg);
    setRouteOffsetM(snap.routeOffsetM);
    setSetLaunchPoints(snap.setLaunchPoints);
    setSetCorridorPoints(snap.setCorridorPointsArr);
    setDeliveryHubAssignments(snap.deliveryHubAssignments);
    setManualDeliveryHubEdit?.(snap.manualDeliveryHubEdit);
    setSimDockWps(snap.simDockWps);
    setDetectAfterWps(snap.detectAfterWps);
    restoreFenceState?.({
      enabled: snap.fenceEnabled, offsetM: snap.fenceOffsetM,
      touched: snap.fenceTouched, intent: snap.fenceIntent,
      acknowledged: snap.fenceAcknowledged, observations: snap.fenceObservations,
    });
    setFenceCustomVertices?.(snap.fenceCustomVertices ?? null);
    setExclusionPolygons?.(snap.exclusionPolygons ?? []);
    drawingRef.current.loadVertices(snap.polygon);
    // loadVertices resets useDrawing's undo stack; keep the orchestrator's
    // undo stack in sync so a later re-enter + Undo can't pop a stale entry.
    if (undoRef) undoRef.current = [];
  }, [setPlan, setPolygon, setSearchPattern, setAnalysis, setUavCount, setPartitionAngleDeg,
    setRouteOffsetM, setSetLaunchPoints, setSetCorridorPoints, setDeliveryHubAssignments,
    setManualDeliveryHubEdit, setSimDockWps, setDetectAfterWps, restoreFenceState,
    setFenceCustomVertices, setExclusionPolygons]);

  const handleExitPlanning = useCallback(async () => {
    const state = stateForSnapshotRef.current;
    const snap = vehicleSnapshotRef.current;
    const onVehicles = snap != null && planSyncKey(state) === planSyncKey(snap);
    const inFile = savedKeyRef.current != null && planEditKey(state) === savedKeyRef.current;
    if (!onVehicles && !inFile && planHasContent(state)) {
      const answer = await confirm(
        snap ? t('planning.savePrompt.messageRestore') : t('planning.savePrompt.message'),
        {
          dialog: {
            title: t('planning.savePrompt.title'),
            confirmLabel: t('planning.savePrompt.save'),
            altLabel: t('planning.savePrompt.dontSave'),
            cancelLabel: t('planning.savePrompt.keepEditing'),
          },
        },
      );
      if (answer === false) return;
      if (answer === true) {
        savePlanRef?.current?.();
        // The file holds the plan as it is now, before the UAVs' plan is
        // restored below.
        savedKeyRef.current = planEditKey(state);
      }
    }
    if (snap && !onVehicles) restoreVehiclePlan(snap);
    goToMonitor();
  }, [goToMonitor, confirm, t, restoreVehiclePlan, savePlanRef]);

  return {
    onPlanSynced, capturePlanState, beginPlanUpload, beginPlanDownload,
    markPlanSaved, markPlanLoaded, forgetPlanSync, handleExitPlanning, planSynced, isPlanSynced,
  };
}
