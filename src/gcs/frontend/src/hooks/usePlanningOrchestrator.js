import { useCallback, useEffect, useRef } from 'react';
import useDrawing from '../components/map/useDrawing';
import usePlanGeneration from './usePlanGeneration';
import useCorridorPath from './useCorridorPath';
import usePlanPersistence from './usePlanPersistence';
import useFallbackLocationPlacement from './useFallbackLocationPlacement';
import usePlanSnapshot from './usePlanSnapshot';
import usePlacementModes from './usePlacementModes';
import usePlanDrag from './usePlanDrag';
import useExclusionDrawing from './useExclusionDrawing';
import { toggleSimDock } from '../utils/planningModes';

export default function usePlanningOrchestrator({
  mission, settings, handleSaveSettings, settingsVersion, plannerReady,
}) {
  const {
    phase, polygon, setPolygon, searchPattern, setSearchPattern,
    dockClasses, setDockClasses,
    perUavDockClasses, setPerUavDockClasses,
    analysis, setAnalysis,
    plan, setPlan, launchPoint, setLaunchPoint,
    corridorPoints, setCorridorPoints,
    setLaunchPoints, setSetLaunchPoints,
    setCorridorPointsArr, setSetCorridorPoints,
    activeSetIndex, setActiveSetIndex,
    uavCount, setUavCount, uavCountLocked, setUavCountLocked,
    partitionAngleDeg, setPartitionAngleDeg,
    routeOffsetM, setRouteOffsetM,
    goToPlanning, goToMonitor,
    fallbackLocationAssignments, setFallbackLocationAssignments,
    manualFallbackLocationEdit, setManualFallbackLocationEdit,
    simDockWps, setSimDockWps,
    detectAfterWps, setDetectAfterWps,
    fenceEnabled, setFenceEnabled,
    fenceOffsetM, setFenceOffsetM,
    fenceTouched, setFenceTouched,
    clearFenceIntent, resetFenceObservations,
    fenceCustomVertices, setFenceCustomVertices,
    exclusionPolygons, setExclusionPolygons,
  } = mission;
  // ---- Refs ----
  const suppressRegenRef = useRef(false);
  const regenTimerRef = useRef(null);
  const undoRef = useRef([]);
  const draggingRef = useRef(false);

  // ---- Drawing ref (for usePlanSnapshot, which needs loadVertices at call time) ----
  const drawingRef = useRef(null);

  // ---- Plan snapshot (dirty-check + exit planning) ----
  const { onPlanSynced, handleExitPlanning } = usePlanSnapshot({
    plan, polygon, searchPattern, dockClasses, perUavDockClasses, analysis, uavCount,
    partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr,
    fallbackLocationAssignments, simDockWps, detectAfterWps,
    fenceCustomVertices, exclusionPolygons,
    setPlan, setPolygon, setSearchPattern, setDockClasses, setPerUavDockClasses, setAnalysis,
    setUavCount, setPartitionAngleDeg, setRouteOffsetM, setSetLaunchPoints,
    setSetCorridorPoints, setFallbackLocationAssignments, setSimDockWps,
    setDetectAfterWps, setFenceCustomVertices, setExclusionPolygons,
    drawingRef, suppressRegenRef, goToMonitor, undoRef,
  });

  // ---- Phase ref for effects ----
  const phaseRef = useRef(phase);
  phaseRef.current = phase;

  // ---- fallback location Placement ----
  const {
    placingFallbackLocation, setPlacingFallbackLocation,
    placingFallbackLocationType, setPlacingFallbackLocationType,
    handlePlaceFallbackLocation, handleRemoveFallbackLocation, handleMoveFallbackLocation,
  } = useFallbackLocationPlacement({ settings, handleSaveSettings, setFallbackLocationAssignments, phase });

  // ---- Plan generation ----
  const {
    effectiveUavCount, effectiveSets,
    localAnalyze, localGenerate,
    approachPoint, corridorPath,
    handlePoiChange,
  } = usePlanGeneration({
    polygon, setPolygon,
    searchPattern,
    dockClasses, setDockClasses,
    analysis, setAnalysis,
    plan, setPlan,
    uavCount, setUavCount,
    uavCountLocked,
    launchPoint,
    corridorPoints,
    setLaunchPoints,
    setCorridorPointsArr,
    setSetLaunchPoints,
    setSetCorridorPoints,
    setActiveSetIndex,
    partitionAngleDeg, setPartitionAngleDeg,
    routeOffsetM,
    phase,
    draggingRef,
    suppressRegenRef,
    regenTimerRef,
    settings,
    settingsVersion,
    plannerReady,
    setFallbackLocationAssignments,
    manualFallbackLocationEdit,
  });

  // ---- Drawing ----
  const drawing = useDrawing({
    setPolygon,
    analyze: localAnalyze,
    dockClasses,
  });
  drawingRef.current = drawing;

  // ---- Placement modes ----
  const {
    placingLaunchPoint, setPlacingLaunchPoint,
    placingCorridor, setPlacingCorridor,
    placingExclusion, setPlacingExclusion,
    startDrawExclusive,
    toggleCorridorPlacement,
    toggleFallbackLocationPlacement,
    toggleExclusionPlacement,
    startPlacingFallbackLocationFromSettings,
  } = usePlacementModes({
    drawing, placingFallbackLocation, setPlacingFallbackLocation,
    searchPattern, analysis, polygon,
  });

  // ---- Keep-out (exclusion) drawing ----
  const {
    draftExclusion,
    handlePlaceExclusionVertex,
    handleFinishExclusion,
    handleRemoveExclusion,
    handleClearExclusions,
    handleExclusionVertexDrag,
    handleExclusionMidpointInsert,
    handleExclusionVertexDelete,
  } = useExclusionDrawing({ placingExclusion, exclusionPolygons, setExclusionPolygons });

  const handleMapClick = useCallback((latlon) => {
    drawing.onMapClick(latlon);
    undoRef.current = [...undoRef.current, { type: 'vertex' }];
  }, [drawing.onMapClick]);

  // ---- Corridor path ----
  const {
    handlePlaceCorridorPoint,
    handleCorridorMidpointInsert,
    handleCorridorPointDrag,
    handleCorridorPointDelete,
    handleClearCorridor,
    handleSetCorridorPointDrag,
    handleCorridorDragRecord,
  } = useCorridorPath({
    launchPoint,
    setLaunchPoint,
    corridorPoints,
    setCorridorPoints,
    setSetLaunchPoints,
    setSetCorridorPoints,
    undoRef,
  });

  // ---- Plan persistence ----
  const {
    handleSavePolygon,
    handleLoadPolygon,
    derivePlanPolygon,
  } = usePlanPersistence({
    polygon,
    launchPoint,
    corridorPoints,
    searchPattern,
    dockClasses,
    perUavDockClasses,
    setPolygon,
    setSearchPattern,
    setDockClasses,
    setPerUavDockClasses,
    setLaunchPoint,
    setCorridorPoints,
    setSetLaunchPoints,
    setSetCorridorPoints,
    setActiveSetIndex,
    setAnalysis,
    setPlan,
    setCorridorPointsArr,
    setLaunchPoints,
    drawing,
    localAnalyze,
    undoRef,
    suppressRegenRef,
    fallbackLocationAssignments,
    setFallbackLocationAssignments,
    simDockWps,
    setSimDockWps,
    detectAfterWps,
    setDetectAfterWps,
    fenceEnabled,
    fenceOffsetM,
    fenceTouched,
    fenceCustomVertices,
    exclusionPolygons,
    setFenceEnabled,
    setFenceOffsetM,
    setFenceTouched,
    authorFenceIntent: mission.authorFenceIntent,
    fenceIntent: mission.fenceIntent,
    clearFenceIntent,
    notifyFenceGeometryReplaced: mission.notifyFenceGeometryReplaced,
    resetFenceObservations,
    setFenceCustomVertices,
    setExclusionPolygons,
    settings,
    handleSaveSettings,
    plannerReady,
  });

  // ---- Drag handlers ----
  const {
    handlePolygonDragRecord,
    handleDragStart, handleDragEnd,
    handleSuppressRegen,
    handleVertexDrag, handlePolygonMove,
    handlePartitionAngleDrag,
  } = usePlanDrag({
    drawing,
    polygon, dockClasses, searchPattern,
    launchPoint, corridorPoints,
    uavCountLocked, effectiveUavCount, approachPoint,
    plan, analysis,
    localAnalyze, localGenerate,
    plannerReady,
    draggingRef, regenTimerRef, undoRef, suppressRegenRef,
    setPartitionAngleDeg,
  });

  // ---- Search pattern change ----
  const handleSearchPatternChange = useCallback((newSearchPattern) => {
    if (newSearchPattern === searchPattern) return;
    const hasPlan = plan?.zones?.length > 0 || polygon.length >= 3;
    const corridorSwitch = searchPattern === 'corridor' || newSearchPattern === 'corridor';
    let polygonCleared = false;
    if (corridorSwitch && hasPlan) {
      if (!window.confirm('Switching search pattern will clear the current plan. Continue?')) return;
      setPolygon([]);
      setPlan(null);
      setAnalysis(null);
      // The plan geometry a hand-edited fence ring was shaped against is gone —
      // drop the stale ring so it can't render/upload over the next plan.
      setFenceCustomVertices(null);
      setSetLaunchPoints([null]);
      setSetCorridorPoints([[]]);
      setActiveSetIndex(0);
      setPartitionAngleDeg(null);
      undoRef.current = [];
      setPlacingLaunchPoint(false);
      setPlacingCorridor(false);
      setPlacingFallbackLocation(false);
      if (drawing.isDrawing) drawing.stopDraw();
      polygonCleared = true;
      setSimDockWps({});
      setDetectAfterWps({});
    } else {
      setPlan(null);
      setPartitionAngleDeg(null);
      setSimDockWps({});
      setDetectAfterWps({});
    }
    setSearchPattern(newSearchPattern);
    if (newSearchPattern === 'corridor') {
      setPlacingCorridor(true);
    } else if (newSearchPattern === 'distributed' && (polygon.length === 0 || polygonCleared)) {
      drawing.startDraw();
    }
  }, [searchPattern, plan, polygon, setSearchPattern, setPolygon, setPlan, setAnalysis, setLaunchPoint, setCorridorPoints, setFenceCustomVertices, drawing]);

  // ---- Enter planning with auto-draw for distributed ----
  const handleGoToPlanning = useCallback(() => {
    goToPlanning();
    if (searchPattern === 'distributed' && polygon.length === 0) {
      drawing.startDraw();
    }
  }, [goToPlanning, searchPattern, polygon, drawing]);

  // ---- Launch point ----
  const handlePlaceLaunchPoint = useCallback((latlon) => {
    setLaunchPoint({ lat: latlon.lat, lon: latlon.lon });
    setPlacingLaunchPoint(false);
  }, []);

  const handleRemoveLaunchPoint = useCallback((setIdx) => {
    if (setIdx != null) {
      setSetLaunchPoints((prev) => { const n = [...prev]; n[setIdx] = null; return n; });
      setSetCorridorPoints((prev) => { const n = [...prev]; n[setIdx] = []; return n; });
    } else {
      setLaunchPoint(null);
      setCorridorPoints([]);
    }
    setPlacingLaunchPoint(false);
    undoRef.current = undoRef.current.filter((e) => e.type === 'vertex');
    setPlacingCorridor(false);
    if (searchPattern === 'corridor') {
      setPlan(null);
      setPlacingCorridor(true);
    } else if (analysis && polygon.length >= 3) {
      localGenerate(polygon, dockClasses, searchPattern, effectiveUavCount, null, null);
    }
  }, [analysis, polygon, dockClasses, searchPattern, effectiveUavCount, localGenerate, setPlan, setLaunchPoint, setCorridorPoints, setSetLaunchPoints, setSetCorridorPoints]);

  // ---- Undo ----
  const handleUndo = useCallback(() => {
    const stack = undoRef.current;
    if (stack.length === 0) return;
    const last = stack[stack.length - 1];
    undoRef.current = stack.slice(0, -1);
    if (last.type === 'vertex') {
      drawing.undo();
    } else if (last.type === 'corridor') {
      setCorridorPoints((prev) => prev.slice(0, -1));
    } else if (last.type === 'corridorReplace') {
      setCorridorPoints(last.prev);
    } else if (last.type === 'launch') {
      setLaunchPoint(null);
      setCorridorPoints([]);
      undoRef.current = undoRef.current.filter((e) => e.type === 'vertex');
    } else if (last.type === 'corridorDrag') {
      if (last.corridorIndex === -1) {
        setSetLaunchPoints((prev) => {
          const next = [...prev];
          next[last.setIdx] = last.oldLatLon;
          return next;
        });
      } else {
        setSetCorridorPoints((prev) => {
          const next = [...prev];
          next[last.setIdx] = (next[last.setIdx] || []).map((p, i) =>
            i === last.corridorIndex ? last.oldLatLon : p
          );
          return next;
        });
      }
    } else if (last.type === 'polygonDrag') {
      drawing.restoreVertices(last.oldPolygon);
    }
  }, [drawing, setSetLaunchPoints, setSetCorridorPoints]);

  const canUndo = undoRef.current.length > 0;

  // ---- Clear all ----
  const handleClearAll = useCallback(() => {
    drawing.clear();
    setSetLaunchPoints([null]);
    setSetCorridorPoints([[]]);
    setActiveSetIndex(0);
    setPartitionAngleDeg(null);
    setRouteOffsetM(null);
    undoRef.current = [];
    setPlan(null);
    setAnalysis(null);
    setPlacingLaunchPoint(false);
    setPlacingFallbackLocation(false);
    setPlacingExclusion(false);
    setPlacingCorridor(searchPattern === 'corridor');
    setSimDockWps({});
    setDetectAfterWps({});
    // Reset the fence to its default (off, untouched) so demo auto-enable can
    // re-derive it when a new search zone is drawn. Clearing starts a new plan
    // lifecycle: an unacknowledged request and the previous plan's vehicle
    // observations both belong to the plan being discarded.
    setFenceEnabled(false);
    setFenceTouched(false);
    clearFenceIntent?.();
    resetFenceObservations?.();
    setFenceCustomVertices(null);
    // Keep-out zones are part of the drawn plan geometry — wipe them too.
    setExclusionPolygons([]);
  }, [drawing, setSetLaunchPoints, setSetCorridorPoints, setActiveSetIndex, setPlan, setAnalysis, searchPattern, setFenceEnabled, setFenceTouched, clearFenceIntent, resetFenceObservations, setFenceCustomVertices, setPlacingExclusion, setExclusionPolygons]);

  // Clear sim POI and detect-after selections when zone count changes
  // during planning (plan regenerated with different zones). Skip during
  // monitor — download sets these directly and must not be overwritten.
  const prevZoneCountRef = useRef(plan?.zones?.length || 0);
  useEffect(() => {
    const zc = plan?.zones?.length || 0;
    if (prevZoneCountRef.current !== 0 && zc !== prevZoneCountRef.current) {
      if (phaseRef.current === 'PLANNING') {
        setSimDockWps({});
        setDetectAfterWps({});
      }
    }
    prevZoneCountRef.current = zc;
  }, [plan]);

  // ---- Sim POI toggle ----
  const handleToggleSimDock = useCallback((zoneIndex, wpIndex) => {
    setSimDockWps((prev) => toggleSimDock(prev, zoneIndex, wpIndex));
  }, []);

  return {
    // For useMissionUpload / useVehicleConnection (stay in App)
    derivePlanPolygon,
    onPlanSynced,
    effectiveUavCount,
    effectiveSets,
    localGenerate,
    approachPoint,
    corridorPath,

    // Drawing object
    drawing,

    // Map interaction handlers
    handleMapClick,
    handleVertexDrag,
    handlePolygonMove,
    handleDragStart, handleDragEnd,
    handleSuppressRegen,
    handlePolygonDragRecord,
    handlePartitionAngleDrag,

    // Corridor handlers
    handlePlaceCorridorPoint,
    handleCorridorMidpointInsert,
    handleCorridorPointDrag,
    handleCorridorPointDelete,
    handleClearCorridor,
    handleSetCorridorPointDrag,
    handleCorridorDragRecord,

    // Launch point handlers
    handlePlaceLaunchPoint,
    handleRemoveLaunchPoint,

    // Planning-phase actions
    handleSearchPatternChange,
    handlePoiChange,
    handleGoToPlanning,
    handleExitPlanning,
    handleToggleSimDock,

    // Toolbar state & actions
    handleUndo, canUndo, handleClearAll,
    handleSavePolygon, handleLoadPolygon,
    startDrawExclusive,
    toggleCorridorPlacement,
    toggleFallbackLocationPlacement,
    toggleExclusionPlacement,

    // SettingsModal entrypoint
    startPlacingFallbackLocationFromSettings,

    // Placement mode state
    placingLaunchPoint, setPlacingLaunchPoint,
    placingCorridor, setPlacingCorridor,
    placingFallbackLocation, setPlacingFallbackLocation,
    placingFallbackLocationType, setPlacingFallbackLocationType,
    placingExclusion,

    // fallback location handlers
    handlePlaceFallbackLocation, handleRemoveFallbackLocation, handleMoveFallbackLocation,

    // Keep-out (exclusion) drawing + zone-style ring editing
    draftExclusion,
    handlePlaceExclusionVertex,
    handleFinishExclusion,
    handleRemoveExclusion,
    handleClearExclusions,
    handleExclusionVertexDrag,
    handleExclusionMidpointInsert,
    handleExclusionVertexDelete,
  };
}
