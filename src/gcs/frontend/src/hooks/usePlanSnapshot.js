import { useCallback, useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { buildPlanSnapshot, isPlanDirty } from '../utils/planSnapshot';

/**
 * Manages the plan snapshot lifecycle: capture baseline on sync,
 * detect dirty state, and restore on discard + exit planning.
 */
export default function usePlanSnapshot({
  // Captured values
  plan, polygon, searchPattern, analysis, uavCount,
  partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr,
  fallbackLocationAssignments, simDockWps, detectAfterWps,
  fenceCustomVertices, exclusionPolygons,
  // Restore setters
  setPlan, setPolygon, setSearchPattern, setAnalysis,
  setUavCount, setPartitionAngleDeg, setRouteOffsetM, setSetLaunchPoints,
  setSetCorridorPoints, setFallbackLocationAssignments, setSimDockWps,
  setDetectAfterWps, setFenceCustomVertices, setExclusionPolygons,
  // Side-effect deps (drawingRef avoids hook-ordering issues with useDrawing)
  drawingRef, suppressRegenRef, goToMonitor, undoRef,
}) {
  const { t } = useTranslation();
  const vehicleSnapshotRef = useRef(null);
  const stateForSnapshotRef = useRef({});
  const [snapshotTrigger, setSnapshotTrigger] = useState(0);

  stateForSnapshotRef.current = {
    plan, polygon, searchPattern, analysis, uavCount,
    partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr,
    fallbackLocationAssignments, simDockWps, detectAfterWps,
    fenceCustomVertices, exclusionPolygons,
  };

  const onPlanSynced = useCallback(() => setSnapshotTrigger(t => t + 1), []);

  useEffect(() => {
    if (snapshotTrigger === 0) return;
    vehicleSnapshotRef.current = buildPlanSnapshot(stateForSnapshotRef.current);
  }, [snapshotTrigger]);

  const handleExitPlanning = useCallback(() => {
    const snap = vehicleSnapshotRef.current;
    if (snap) {
      if (isPlanDirty(stateForSnapshotRef.current, snap)) {
        if (!window.confirm(t('planning.exitDirtyConfirm'))) return;
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
        setFallbackLocationAssignments(snap.fallbackLocationAssignments);
        setSimDockWps(snap.simDockWps);
        setDetectAfterWps(snap.detectAfterWps);
        setFenceCustomVertices?.(snap.fenceCustomVertices ?? null);
        setExclusionPolygons?.(snap.exclusionPolygons ?? []);
        drawingRef.current.loadVertices(snap.polygon);
        // loadVertices resets useDrawing's undo stack; keep the orchestrator's
        // undo stack in sync so a later re-enter + Undo can't pop a stale entry.
        if (undoRef) undoRef.current = [];
      }
    }
    goToMonitor();
  }, [goToMonitor, setPlan, setPolygon, setSearchPattern, setAnalysis, setUavCount, setPartitionAngleDeg,
      setRouteOffsetM, setSetLaunchPoints, setSetCorridorPoints, setFallbackLocationAssignments,
      setSimDockWps, setDetectAfterWps, setFenceCustomVertices, setExclusionPolygons]);

  return { onPlanSynced, handleExitPlanning };
}
