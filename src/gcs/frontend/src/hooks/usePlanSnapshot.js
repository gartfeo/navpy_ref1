import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { buildPlanSnapshot, planEditKey, planHasContent, planSyncKey } from '../utils/planSnapshot';
import { planFromDownload } from '../utils/planDownloadRun';
import useConfirm from './useConfirm';

const NO_SYNC = { key: null, plan: null };

/**
 * The plan's two baselines and leaving plan mode.
 *
 * - Synced: the plan on the UAVs, captured on upload and download
 *   (onPlanSynced). `planSynced` says the editor still holds exactly that
 *   plan, or a plan a download published whose baseline has not landed yet
 *   (planFromDownload); the monitor offers the start control only then. An
 *   upload drops the baseline when it starts writing (forgetPlanSync): until
 *   it completes, what the UAVs carry is not known.
 * - Saved: the plan in the file last loaded or saved (markPlanSaved).
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
  fenceTouched, fenceEnabled, fenceOffsetM, fenceIntent,
  fenceCustomVertices, exclusionPolygons,
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
  const [snapshotTrigger, setSnapshotTrigger] = useState(0);
  // The synced baseline's key, and the plan object it was taken from: a
  // download's provisional claim ends once its own baseline is taken.
  const [sync, setSync] = useState(NO_SYNC);
  const [savedTrigger, setSavedTrigger] = useState(0);
  const savedKeyRef = useRef(null);

  const current = {
    plan, polygon, searchPattern, analysis, uavCount, effectiveUavCount,
    partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr,
    deliveryHubAssignments, manualDeliveryHubEdit, simDockWps, detectAfterWps,
    fenceTouched, fenceEnabled, fenceOffsetM, fenceIntent,
    fenceCustomVertices, exclusionPolygons,
  };
  stateForSnapshotRef.current = current;

  const onPlanSynced = useCallback(() => setSnapshotTrigger(t => t + 1), []);
  // Called in the same update that loads or saves the plan file: the baseline
  // is taken once that update has committed.
  const markPlanSaved = useCallback(() => setSavedTrigger(t => t + 1), []);
  // What the UAVs carry is no longer known (an upload is writing) or no UAV is
  // left to carry it: nothing to compare with or restore. The plan in the
  // editor loses any download's claim too.
  const forgetPlanSync = useCallback(() => {
    vehicleSnapshotRef.current = null;
    setSync({ key: null, plan: stateForSnapshotRef.current.plan ?? null });
  }, []);

  useEffect(() => {
    if (snapshotTrigger === 0) return;
    const state = stateForSnapshotRef.current;
    vehicleSnapshotRef.current = buildPlanSnapshot(state);
    setSync({ key: planSyncKey(state), plan: state.plan ?? null });
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
      fenceCustomVertices, exclusionPolygons],
  );
  // A plan a download just published is the UAVs' plan before its baseline
  // lands (the download's post-processing derives the area and dock picks
  // first), or the monitor would call it "not on the UAVs" meanwhile. Once
  // the baseline was taken from it, only the key decides: an edit that leaves
  // the plan object alone (picks, hubs, fence) must still read as a change.
  const provisional = planFromDownload(plan) && plan !== sync.plan;
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
        // restored below (a deferred markPlanSaved would see the restore).
        savedKeyRef.current = planEditKey(state);
      }
    }
    if (snap && !onVehicles) restoreVehiclePlan(snap);
    goToMonitor();
  }, [goToMonitor, confirm, t, restoreVehiclePlan, savePlanRef]);

  return { onPlanSynced, markPlanSaved, forgetPlanSync, handleExitPlanning, planSynced, isPlanSynced };
}
