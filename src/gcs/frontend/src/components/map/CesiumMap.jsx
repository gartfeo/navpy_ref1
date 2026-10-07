import React, { useRef, useMemo, useEffect } from 'react';
import { pointInPolygon } from './utils/entityPicking';
import useCesiumViewer from './hooks/useCesiumViewer';
import useMapInteraction from './hooks/useMapInteraction';
import usePlanSync from './hooks/usePlanSync';
import usePolygonLayer from './hooks/usePolygonLayer';
import useZoneLayer from './hooks/useZoneLayer';
import useTrackLayer from './hooks/useTrackLayer';
import useLaunchZoneLayer from './hooks/useLaunchZoneLayer';
import useCorridorLayer from './hooks/useCorridorLayer';
import useFenceLayer from './hooks/useFenceLayer';
import useExclusionLayer from './hooks/useExclusionLayer';
import useCorridorWaypoints from './hooks/useCorridorWaypoints';
import usePartitionHandle from './hooks/usePartitionHandle';
import useUavMarkers from './hooks/useUavMarkers';
import useUavTrails from './hooks/useUavTrails';
import useCoverageLayer from './hooks/useCoverageLayer';
import useAssignmentMarkers from './hooks/useAssignmentMarkers';
import useAvailableTaskMarkers from './hooks/useAvailableTaskMarkers';
import useDockMarkers from './hooks/useDockMarkers';
import useDetectionMarkers from './hooks/useDetectionMarkers';
import useDeliveryHubLayer from './hooks/useDeliveryHubLayer';
import useFollowUav from './hooks/useFollowUav';
import useTrackWpMarkers from './hooks/useTrackWpMarkers';
import { resolveSimDocks, resolveSimDocksFromPlan, resolveDetectionStart, resolveDetectionStartFromPlan } from './utils/dockResolver';

export { pointInPolygon };

// Safety net: hard cap on how long the loading overlay may stay up waiting for
// the base map's first tiles. Normally tilesReady fires well before this; it only
// applies if the tile-progress signal never arrives (e.g. imagery-provider error)
// so the overlay can't hang forever.
const LOADING_FALLBACK_MS = 8000;

function CesiumMap({
  polygon,
  plan,
  analysis,
  searchPattern,
  fencePolygon,
  showFence,
  fenceWarning,
  onFenceVertexDrag,
  onFenceMidpointInsert,
  onFenceVertexDelete,
  exclusionPolygons,
  exclusionDraft,
  exclusionConflicts,
  placingExclusion,
  onPlaceExclusionVertex,
  onFinishExclusion,
  onExclusionVertexDrag,
  onExclusionMidpointInsert,
  onExclusionVertexDelete,
  showZones,
  showTracks,
  showLaunchZone,
  showCoverage,
  showVision,
  showTrails,
  trailResetKey,
  coverageResetKey,
  plannerReady,
  vehicleList,
  storeRef,
  trackResetKey,
  phase,
  isDrawing,
  onMapClick,
  onVertexDrag,
  onVertexDelete,
  onMidpointInsert,
  onPolygonMove,
  onFinishDraw,
  onRemoveLaunchPoint,
  placingLaunchPoint,
  onPlaceLaunchPoint,
  placingCorridor,
  onPlaceCorridorPoint,
  onCorridorMidpointInsert,
  onCorridorPointDelete,
  onDragStart,
  onDragEnd,
  onSuppressRegen,
  setLaunchPoints,
  setCorridorPoints: setCorridorPointsArr,
  activeSetIndex,
  setActiveSetIndex,
  onSetCorridorPointDrag,
  onCorridorDragRecord,
  onPolygonDragRecord,
  partitionAngleDeg,
  onPartitionAngleDrag,
  effectiveSets,
  resetViewRef,
  flyToRef,
  pendingConfirms,
  assignments,
  availableTasks,
  simMode,
  simDockWps,
  onToggleSimDock,
  vehicleTargWps,
  vehicleNavLastWp,
  detectAfterWps,
  mapDefaults,
  deliveryHubs,
  deliveryHubAssignments,
  placingDeliveryHub,
  onPlaceDeliveryHub,
  onRemoveDeliveryHub,
  onMoveDeliveryHub,
  followSysId,
  fpvMode,
  onViewerReady,
}) {
  // Entity refs for cleanup
  const entitiesRef = useRef({
    polygon: null,
    polygonOutline: null,
    vertices: [],
    midpoints: [],
    zones: [],
    tracks: [],
    launchZone: [],
    minLaunchZone: [],
    launchPointMarkers: [],
    corridors: [],
    fence: [],
    fenceVertices: [],
    fenceMidpoints: [],
    exclusions: [],
    exclusionVertices: [],
    exclusionMidpoints: [],
    partitionHandle: null,
    partitionLine: null,
    uavMap: {},
    trailMap: {},
    simDocks: [],
    trackWpMarkers: [],
    detectionStart: [],
  });

  // Store latest callbacks in refs so handlers don't need recreation
  const callbacksRef = useRef({});
  callbacksRef.current = {
    isDrawing,
    editable: phase === 'PLANNING',
    searchPattern,
    polygon,
    onMapClick,
    onVertexDrag,
    onVertexDelete,
    onMidpointInsert,
    onPolygonMove,
    onFinishDraw,
    onRemoveLaunchPoint,
    placingLaunchPoint,
    onPlaceLaunchPoint,
    placingCorridor,
    onPlaceCorridorPoint,
    onCorridorMidpointInsert,
    onCorridorPointDelete,
    onDragStart,
    onDragEnd,
    onSuppressRegen,
    onSetCorridorPointDrag,
    onCorridorDragRecord,
    onPolygonDragRecord,
    onPartitionAngleDrag,
    onSwitchSet: setActiveSetIndex,
    placingDeliveryHub,
    onPlaceDeliveryHub,
    onRemoveDeliveryHub,
    onMoveDeliveryHub,
    placingExclusion,
    onPlaceExclusionVertex,
    onFinishExclusion,
    onExclusionVertexDrag,
    onExclusionMidpointInsert,
    onExclusionVertexDelete,
    onFenceVertexDrag,
    onFenceMidpointInsert,
    onFenceVertexDelete,
    simMode,
    onToggleSimDock,
  };

  // Derive active set values
  const launchPoint = (setLaunchPoints || [])[activeSetIndex ?? 0] ?? null;
  const corridorPoints = (setCorridorPointsArr || [[]])[activeSetIndex ?? 0] || [];

  // Shared refs for smooth CallbackProperty reads during drag
  const polygonRef = useRef(polygon || []);
  polygonRef.current = polygon || [];

  const setLaunchPointsRef = useRef(setLaunchPoints || []);
  setLaunchPointsRef.current = setLaunchPoints || [];

  const setCorridorPointsRef = useRef(setCorridorPointsArr || [[]]);
  setCorridorPointsRef.current = setCorridorPointsArr || [[]];

  const partitionAngleDegRef = useRef(partitionAngleDeg);
  partitionAngleDegRef.current = partitionAngleDeg;

  const vehiclePosRef = useRef({});
  const corridorWpRef = useRef({});

  // Structural keys
  const zoneCount = plan?.zones?.length || 0;
  const lzSrc = analysis || plan;
  const lzLen = lzSrc?.launch_zone?.length || 0;
  const minLzLen = lzSrc?.min_launch_zone?.length || 0;
  const setCorridorCounts = (setCorridorPointsArr || [[]]).map(a => (a || []).length).join(',');
  const numSets = plan?.zones ? Math.max(...plan.zones.map(z => z.set_index ?? 0)) + 1 : 1;
  const setLaunchPointCount = (setLaunchPoints || []).filter(lp => lp != null).length;

  // Hooks — each manages one concern
  const { containerRef, viewerRef, cesiumRef, readyRef, viewerReady, terrainReady, tilesReady } = useCesiumViewer(resetViewRef, flyToRef, mapDefaults);

  const { planRef, trackPosRef, zonePosRef, lzPosRef, minLzPosRef, clampOuterRef, terrainBaseRef } =
    usePlanSync(cesiumRef, viewerRef, plan, analysis, terrainReady);

  useMapInteraction(cesiumRef, viewerRef, readyRef, callbacksRef, {
    polygonRef, setLaunchPointsRef, setCorridorPointsRef, clampOuterRef, activeSetIndex,
  });

  usePolygonLayer(cesiumRef, viewerRef, entitiesRef, polygonRef, polygon, phase, viewerReady);
  useFenceLayer(cesiumRef, viewerRef, entitiesRef, fencePolygon, showFence, viewerReady, phase === 'PLANNING', fenceWarning);
  useExclusionLayer(cesiumRef, viewerRef, entitiesRef, exclusionPolygons, exclusionDraft, exclusionConflicts, viewerReady, phase === 'PLANNING');
  useZoneLayer(cesiumRef, viewerRef, entitiesRef, zonePosRef, zoneCount, showZones, searchPattern, viewerReady);

  useTrackLayer(
    cesiumRef, viewerRef, entitiesRef, trackPosRef, planRef, vehiclePosRef,
    storeRef, plan, searchPattern, polygon, launchPoint, corridorPoints,
    zoneCount, showTracks, trackResetKey, phase, viewerReady,
  );

  useLaunchZoneLayer(cesiumRef, viewerRef, entitiesRef, lzPosRef, minLzPosRef, lzLen, minLzLen, showLaunchZone, viewerReady);

  useCorridorLayer(
    cesiumRef, viewerRef, entitiesRef, corridorWpRef,
    setLaunchPointsRef, setCorridorPointsRef, trackPosRef, planRef,
    setLaunchPoints, setCorridorPointsArr, numSets, zoneCount,
    showLaunchZone, showTracks, searchPattern, setLaunchPointCount, phase, setCorridorCounts,
    terrainReady, viewerReady,
  );

  useCorridorWaypoints(
    cesiumRef, viewerRef, corridorWpRef,
    setCorridorPointsRef, setLaunchPointsRef,
    setCorridorPointsArr, setLaunchPoints,
    phase, showTracks, numSets, setLaunchPointCount, setCorridorCounts,
    terrainReady,
  );

  usePartitionHandle(
    cesiumRef, viewerRef, entitiesRef, polygonRef, partitionAngleDegRef,
    polygon, partitionAngleDeg, effectiveSets, searchPattern, phase, viewerReady,
  );

  useUavMarkers(cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady);
  useFollowUav(cesiumRef, viewerRef, entitiesRef, followSysId, viewerReady, fpvMode);
  useUavTrails(cesiumRef, viewerRef, entitiesRef, storeRef, viewerReady, showTrails, `${trackResetKey}:${trailResetKey}`);
  useCoverageLayer(cesiumRef, viewerRef, storeRef, plan, showCoverage, showVision, trackResetKey, coverageResetKey, viewerReady, plannerReady);
  useAssignmentMarkers(cesiumRef, viewerRef, assignments || {}, storeRef, viewerReady);
  useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks || {}, viewerReady);

  // Stable key from vehicle sys_ids — avoids recomputing on 5 Hz telemetry updates
  const vehicleSysIdKey = (vehicleList || []).map(v => v.sys_id).join(',');

  const simDockWpsKey = JSON.stringify(simDockWps || {});
  const simDocks = useMemo(() => {
    if (!simMode) return null;
    if (phase === 'PLANNING' && simDockWps && Object.keys(simDockWps).length > 0) {
      return resolveSimDocksFromPlan(plan, simDockWps);
    }
    return resolveSimDocks(plan, setCorridorPointsArr, searchPattern, vehicleTargWps, vehicleList);
  }, [simMode, phase, plan, setCorridorPointsArr, searchPattern, vehicleTargWps, vehicleSysIdKey, simDockWpsKey]);

  // Cache POIs so they persist even after a vehicle disconnects mid-flight.
  // Clear the cache when all vehicles are gone (full disconnect).
  const poiCacheRef = useRef(new Map());
  const stablePois = useMemo(() => {
    // Planning-mode POIs (user-selected, no vehicles needed) — pass through directly
    if (phase === 'PLANNING' && simDocks?.length > 0) return simDocks;

    if (!vehicleList || vehicleList.length === 0) {
      poiCacheRef.current.clear();
      return [];
    }
    // Prune entries for connected vehicles — they'll be re-added from simDocks.
    // Disconnected vehicle entries survive (the cache's purpose).
    const connectedIds = new Set(vehicleList.map(v => v.sys_id));
    for (const [key, t] of poiCacheRef.current) {
      if (connectedIds.has(t.sys_id)) poiCacheRef.current.delete(key);
    }
    for (const t of (simDocks || [])) {
      poiCacheRef.current.set(`${t.sys_id}_${t.wpNumber}`, t);
    }
    return [...poiCacheRef.current.values()];
  }, [simDocks, vehicleSysIdKey, phase]);

  useDockMarkers(cesiumRef, viewerRef, entitiesRef, stablePois, viewerReady);

  const detectAfterWpsKey = JSON.stringify(detectAfterWps || {});
  const detectionPoints = useMemo(() => {
    if (!simMode) return null;
    if (phase === 'PLANNING' && detectAfterWps && Object.keys(detectAfterWps).length > 0) {
      return resolveDetectionStartFromPlan(plan, detectAfterWps);
    }
    return resolveDetectionStart(plan, setCorridorPointsArr, searchPattern, vehicleNavLastWp, vehicleList);
  }, [simMode, phase, plan, setCorridorPointsArr, searchPattern, vehicleNavLastWp, vehicleSysIdKey, detectAfterWpsKey]);
  useDetectionMarkers(cesiumRef, viewerRef, entitiesRef, detectionPoints, viewerReady);
  useTrackWpMarkers(cesiumRef, viewerRef, entitiesRef, trackPosRef, plan, simMode, phase, simDockWps, viewerReady);

  useDeliveryHubLayer(cesiumRef, viewerRef, deliveryHubs, deliveryHubAssignments, plan, viewerReady, terrainReady, trackPosRef, terrainBaseRef, showTracks);

  // Dismiss the loading overlay as soon as the base map has painted its first
  // tiles (tilesReady) — NOT after the slower, network-bound Ion terrain upgrade
  // (terrainReady), which used to hold the "AAS LOADING" screen up for the whole
  // upgrade. Still gated on a painted base map so there's no blue/blank flash.
  // LOADING_FALLBACK_MS is a safety net: if the tile-progress signal never
  // arrives (e.g. an imagery-provider error), the overlay must not hang forever.
  useEffect(() => {
    if (!onViewerReady || !viewerReady) return undefined;
    if (tilesReady) { onViewerReady(true); return undefined; }
    const fallback = setTimeout(() => onViewerReady(true), LOADING_FALLBACK_MS);
    return () => clearTimeout(fallback);
  }, [viewerReady, tilesReady, onViewerReady]);

  return <div ref={containerRef} style={{ flex: 1, height: '100%', position: 'relative' }} />;
}

function cesiumMapPropsEqual(prev, next) {
  const keys = Object.keys(next);
  for (let i = 0; i < keys.length; i++) {
    if (prev[keys[i]] !== next[keys[i]]) return false;
  }
  return true;
}

export default React.memo(CesiumMap, cesiumMapPropsEqual);
