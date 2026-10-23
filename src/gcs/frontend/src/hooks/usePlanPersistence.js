import { useCallback, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { analyzeArea } from '../utils/planner';
import { convexHull } from '../utils/geo';
import { buildDeliveryHubsFromDownload, normalizePlanFileHubs } from '../utils/deliveryHubAssignment';
import { MAX_UAV_COUNT, ROUTE_OFFSET_MAX_M, ROUTE_OFFSET_MIN_M } from '../utils/planLimits';

// Generator inputs, besides geometry, that shape the tracks: a plan file must
// carry them to regenerate the same lines (and so keep its sim-dock waypoint
// indices valid). null = auto. Present values must lie in the ranges the
// planning sidebar offers, or the file is rejected before any state changes.
const PLAN_SHAPE_FIELDS = {
  route_offset_m: (v) => Number.isFinite(v) && v >= ROUTE_OFFSET_MIN_M && v <= ROUTE_OFFSET_MAX_M,
  partition_angle_deg: (v) => Number.isFinite(v),
  uav_count: (v) => Number.isInteger(v) && v >= 1 && v <= MAX_UAV_COUNT,
};

function hasInvalidPlanShape(raw) {
  return Object.entries(PLAN_SHAPE_FIELDS).some(([key, isValid]) =>
    raw[key] != null && !isValid(raw[key]));
}

// Every hub needs a position, and every assignment must be a hub index (or
// null). The indices refer to the file's hubs when it carries some, else to
// the settings hubs the load keeps.
function hasInvalidDeliveryHubs(raw, settingsHubCount) {
  const hubs = raw.default_delivery_hubs;
  if (hubs != null && normalizePlanFileHubs(hubs) == null) return true;
  const assignments = raw.delivery_hub_assignments;
  if (assignments == null) return false;
  if (!Array.isArray(assignments)) return true;
  const hubCount = hubs?.length > 0 ? hubs.length : settingsHubCount;
  return assignments.some((idx) => idx != null
    && !(Number.isInteger(idx) && idx >= 0 && idx < hubCount));
}

/**
 * Save/load plan geometry to/from JSON files, and derive polygon from downloaded plan tracks.
 */
export default function usePlanPersistence({
  polygon,
  launchPoint,
  corridorPoints,
  searchPattern,
  uavCount,
  partitionAngleDeg,
  routeOffsetM,
  setPolygon,
  setSearchPattern,
  setUavCount,
  setPartitionAngleDeg,
  setRouteOffsetM,
  setLaunchPoint,
  setCorridorPoints,
  setLaunchPoints,
  setSetLaunchPoints,
  setSetCorridorPoints,
  setActiveSetIndex,
  setAnalysis,
  setPlan,
  setCorridorPointsArr,
  drawing,
  regenerateLoadedPlan,
  resetZoneSelectionBaseline,
  undoRef,
  holdDownloadedPlan,
  releaseDownloadedPlan,
  deliveryHubAssignments,
  setDeliveryHubAssignments,
  setManualDeliveryHubEdit,
  simDockWps,
  setSimDockWps,
  detectAfterWps,
  setDetectAfterWps,
  fenceEnabled,
  fenceOffsetM,
  fenceTouched,
  fenceIntent,
  authorFenceIntent,
  clearFenceIntent,
  notifyFenceGeometryReplaced,
  resetFenceObservations,
  fenceCustomVertices,
  exclusionPolygons,
  setFenceEnabled,
  setFenceOffsetM,
  setFenceTouched,
  setFenceCustomVertices,
  setExclusionPolygons,
  settings,
  handleSaveSettings,
  plannerReady,
  pendingPlanFile,
  setPendingPlanFile,
}) {
  const { t } = useTranslation();

  // Save plan geometry to file
  const handleSavePolygon = useCallback(() => {
    const hasPolygon = polygon && polygon.length >= 3;
    const hasCorridor = launchPoint != null;
    if (!hasPolygon && !hasCorridor) return;
    const data = {
      polygon: polygon.map((p) => ({ lat: p.lat, lon: p.lon })),
      search_pattern: searchPattern,
      uav_count: uavCount ?? null,
      partition_angle_deg: partitionAngleDeg ?? null,
      route_offset_m: routeOffsetM ?? null,
    };
    // Save per-set launch points and corridors
    const savedLps = setLaunchPoints.map((lp) => lp ? { lat: lp.lat, lon: lp.lon } : null);
    const savedCps = (setCorridorPointsArr ?? [[]]).map((cp) => (cp || []).map((p) => ({ lat: p.lat, lon: p.lon })));
    if (savedLps.some((lp) => lp != null)) data.set_launch_points = savedLps;
    if (savedCps.some((cp) => cp.length > 0)) data.set_corridors = savedCps;
    if (deliveryHubAssignments?.length > 0) data.delivery_hub_assignments = deliveryHubAssignments;
    if (simDockWps && Object.keys(simDockWps).length > 0) data.sim_dock_wps = simDockWps;
    if (detectAfterWps && Object.keys(detectAfterWps).length > 0) data.detect_after_wps = detectAfterWps;
    // Geofence: settings always; the ring itself only when operator-edited
    // (the auto ring re-derives from the polygon + offset on load). Keep-outs
    // are plan geometry like the corridor — persist their rings. Only persist
    // when the operator authored the fence — otherwise every plan would carry a
    // fence block, and loading it would mark the fence "touched" and suppress
    // demo auto-enable. Observed vehicle state is deliberately NOT authored
    // state, so an unknown/mixed download serializes no fence block at all.
    if (fenceEnabled || fenceTouched || fenceIntent) {
      data.fence = { enabled: !!fenceEnabled, offset_m: fenceOffsetM };
      if (fenceCustomVertices?.length >= 3) {
        data.fence.custom_vertices = fenceCustomVertices.map((p) => ({ lat: p.lat, lon: p.lon }));
      }
    }
    if (exclusionPolygons?.length > 0) {
      data.exclusions = exclusionPolygons
        .filter((r) => Array.isArray(r) && r.length >= 3)
        .map((r) => r.map((p) => ({ lat: p.lat, lon: p.lon })));
    }
    const deliveryHubs = settings?.default_delivery_hubs || [];
    if (deliveryHubs.length > 0) data.default_delivery_hubs = deliveryHubs;
    const json = JSON.stringify(data, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'plan.json';
    a.click();
    URL.revokeObjectURL(url);
  }, [polygon, launchPoint, corridorPoints, searchPattern, uavCount, partitionAngleDeg, routeOffsetM, setLaunchPoints, setCorridorPointsArr, deliveryHubAssignments, simDockWps, detectAfterWps, settings, fenceEnabled, fenceOffsetM, fenceTouched, fenceIntent, fenceCustomVertices, exclusionPolygons]);

  // Apply a validated plan file. It needs the planner configuration: the
  // loaded area is analyzed (auto UAV count, set count) and its tracks
  // regenerated from it.
  const applyPlanFile = useCallback((raw) => {
    try {
      // A hand-edited fence ring / keep-outs belong to the plan they were
      // shaped against — loading a plan replaces them with THAT plan's
      // saved rings (or clears them when the file has none). Never keep a
      // stale ring from the previous plan.
      setFenceCustomVertices?.(
        raw?.fence?.custom_vertices?.length >= 3
          ? raw.fence.custom_vertices.map((p) => ({ lat: p.lat, lon: p.lon }))
          : null,
      );
      setExclusionPolygons?.(
        Array.isArray(raw?.exclusions)
          ? raw.exclusions
              .filter((r) => Array.isArray(r) && r.length >= 3)
              .map((r) => r.map((p) => ({ lat: p.lat, lon: p.lon })))
          : [],
      );
      // The file's area replaces the current one, even when it has none (a
      // corridor): a kept area would be re-analyzed later and bring back the
      // replaced plan's UAV count.
      const poly = raw.polygon.length >= 3 ? raw.polygon : [];
      const regenerates = poly.length >= 3 && raw.search_pattern !== 'corridor';
      setPolygon(poly);
      drawing.loadVertices(poly);
      // Analyze the loaded area at the loaded spacing in this same update:
      // the auto UAV count, and from it the set count that sizes the
      // per-set launch points below, must come from THIS plan, not from
      // the analysis of the plan being replaced. A plan without an area
      // (a corridor with no polygon) has no analysis, so its auto count is
      // the planner default — never the replaced area's requirement.
      setAnalysis(poly.length >= 3
        ? analyzeArea(poly.map((p) => ({ lat: p.lat, lon: p.lon })), undefined,
          { trackSpacingOverrideM: raw.route_offset_m ?? null })
        : null);
      if (raw.search_pattern) setSearchPattern(raw.search_pattern);
      // Older files carry no spacing / partition angle: load them as auto.
      // Without a saved UAV count the session's current count stays.
      setRouteOffsetM(raw.route_offset_m ?? null);
      setPartitionAngleDeg(raw.partition_angle_deg ?? null);
      if (Object.hasOwn(raw, 'uav_count')) setUavCount(raw.uav_count);
      // Restore per-set state.
      if (raw.set_launch_points?.length > 0) {
        setSetLaunchPoints(raw.set_launch_points.map((lp) => lp ? { lat: lp.lat, lon: lp.lon } : null));
      } else {
        setSetLaunchPoints([null]);
      }
      if (raw.set_corridors?.length > 0) {
        setSetCorridorPoints(raw.set_corridors.map((cp) => (cp || []).map((p) => ({ lat: p.lat, lon: p.lon }))));
      } else {
        setSetCorridorPoints([[]]);
      }
      setActiveSetIndex(0);
      if (raw.default_delivery_hubs?.length > 0 && handleSaveSettings) {
        handleSaveSettings({ default_delivery_hubs: normalizePlanFileHubs(raw.default_delivery_hubs) });
      }
      // The file's assignment is the saved operator decision: keep it through
      // the regeneration below (manual), or start from none (auto) when the
      // file has none — never the replaced plan's.
      const fileAssignments = raw.delivery_hub_assignments?.length > 0
        ? raw.delivery_hub_assignments : null;
      setDeliveryHubAssignments?.(fileAssignments ?? []);
      setManualDeliveryHubEdit?.(fileAssignments != null);
      // The waypoint selections describe the plan regenerated from this
      // file, not the zones of the plan being replaced. Save omits empty
      // maps, so an omitted map means "none" — never the previous plan's.
      resetZoneSelectionBaseline?.();
      setSimDockWps?.(raw.sim_dock_wps ?? {});
      setDetectAfterWps?.(raw.detect_after_wps ?? {});
      if (raw.fence && setFenceEnabled) {
        // A saved fence block is an operator decision from that plan —
        // load it as authored intent (including a legacy explicit off), so
        // it is re-sent once and then consumed on acknowledgement.
        if (authorFenceIntent) {
          authorFenceIntent(!!raw.fence.enabled);
        } else {
          setFenceTouched?.(true);
          setFenceEnabled(!!raw.fence.enabled);
        }
        if (raw.fence.offset_m != null && setFenceOffsetM) setFenceOffsetM(raw.fence.offset_m);
      } else {
        // No fence block: the loaded plan carries no fence decision. A
        // request still pending belongs to the plan being replaced — keeping
        // it would send the discarded plan's fence on the next upload — and
        // the previous toggle state is not this plan's either.
        clearFenceIntent?.();
        setFenceEnabled?.(false);
        setFenceTouched?.(false);
      }
      // This is a replacement plan: the previous plan's vehicle fence
      // observations no longer describe it. The connected fleet is asked
      // again, and the demo default waits for its answer.
      resetFenceObservations?.({ readFleet: true });
      undoRef.current = [];
      releaseDownloadedPlan?.();
      if (regenerates) regenerateLoadedPlan();
    } catch {
      window.alert(t('planFile.invalidData'));
    }
  }, [t, setAnalysis, setPolygon, drawing.loadVertices, regenerateLoadedPlan, resetZoneSelectionBaseline, setSearchPattern, setUavCount, setPartitionAngleDeg, setRouteOffsetM, setLaunchPoint, setCorridorPoints, authorFenceIntent, clearFenceIntent, setFenceEnabled, setFenceTouched, resetFenceObservations, handleSaveSettings, setSetLaunchPoints, setSetCorridorPoints, setActiveSetIndex, setDeliveryHubAssignments, setManualDeliveryHubEdit, setSimDockWps, setDetectAfterWps, setFenceOffsetM, setFenceCustomVertices, setExclusionPolygons, undoRef, releaseDownloadedPlan]);

  // A file loaded before the planner is ready is held, not applied: applied
  // early, its analysis would be empty and the readiness transition would
  // re-derive the UAV count and set count without it. It is applied, as
  // loaded, once the planner is ready. While it is held, plan generation
  // treats the planner as not ready (see usePlanningOrchestrator), so the
  // readiness transition cannot regenerate the plan the file replaces.
  const plannerReadyRef = useRef(plannerReady);
  plannerReadyRef.current = plannerReady;
  useEffect(() => {
    if (!plannerReady || !pendingPlanFile) return;
    setPendingPlanFile(null);
    applyPlanFile(pendingPlanFile);
  }, [plannerReady, pendingPlanFile, applyPlanFile]);

  // Load plan geometry from file
  const handleLoadPolygon = useCallback(() => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = (e) => {
      const file = e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = (ev) => {
        let raw;
        try {
          raw = JSON.parse(ev.target.result);
        } catch {
          window.alert(t('planFile.invalidData'));
          return;
        }
        if (!raw || !Array.isArray(raw.polygon)
          || ['poi_classes', 'delivery_docks', 'delivery_dock_assignments', 'tactic'].some((key) => Object.hasOwn(raw, key))
          || hasInvalidPlanShape(raw)
          || hasInvalidDeliveryHubs(raw, settings?.default_delivery_hubs?.length ?? 0)) {
          window.alert(t('planFile.invalidData'));
          return;
        }
        if (plannerReadyRef.current) applyPlanFile(raw);
        else setPendingPlanFile(raw);
      };
      reader.readAsText(file);
    };
    input.click();
  }, [t, applyPlanFile, setPendingPlanFile, settings]);

  // Derive polygon + launch/corridor from plan tracks using corridor_end_index
  const derivePlanPolygon = useCallback((zones, downloadedSearchPattern, downloadedPolygon, downloadedCorridorBackbone, downloadedLaunchPoint, downloadedMissionDeliveryHubs) => {
    const tracks = zones.filter((z) => z.track?.length);
    if (tracks.length === 0) return;
    undoRef.current = [];

    // Keep the vehicles' missions on screen until the operator changes what
    // plans are generated from (see usePlanGeneration).
    holdDownloadedPlan?.();
    // The plan flies one UAV per downloaded mission; the area's auto count
    // would re-zone it.
    setUavCount(zones.length);

    // Restoring a downloaded plan replaces the geometry a hand-edited fence
    // ring was shaped against — drop the stale ring (fence reverts to auto).
    setFenceCustomVertices?.(null);
    // The auto ring therefore moves to a shape nobody authored. Say so, or the
    // changed ring reads as an operator edit and the next untouched upload
    // re-sends the fence.
    notifyFenceGeometryReplaced?.();

    if (downloadedSearchPattern) setSearchPattern(downloadedSearchPattern);

    const getCorridorIdx = (z) => {
      if ((z.corridor_end_index ?? 0) > 0) return z.corridor_end_index;
      return 0;
    };

    // Launch point from metadata (preferred) or heuristic fallback. The
    // download replaces every set's launch point and corridor: slots left
    // from the replaced plan are not the vehicles' geometry.
    let heuristicSkip = 0;
    let launch = null;
    let corridor = [];
    if (downloadedLaunchPoint) {
      launch = { lat: downloadedLaunchPoint.lat, lon: downloadedLaunchPoint.lon };
      const firstCidx = getCorridorIdx(tracks[0]);
      if (downloadedCorridorBackbone?.length > 0) {
        corridor = downloadedCorridorBackbone.map((p) => ({ lat: p.lat, lon: p.lon }));
      } else {
        corridor = firstCidx > 0 ? tracks[0].track.slice(0, firstCidx) : [];
      }
    } else {
      // Heuristic fallback: detect shared first waypoint across zones
      const first = tracks[0].track[0];
      const allShareFirst = tracks.length > 1 && tracks.every((z) =>
        Math.abs(z.track[0].lat - first.lat) < 1e-7 &&
        Math.abs(z.track[0].lon - first.lon) < 1e-7
      );
      if (allShareFirst && getCorridorIdx(tracks[0]) === 0) {
        heuristicSkip = 1;
      }

      const firstCidx = getCorridorIdx(tracks[0]) || heuristicSkip;
      if (firstCidx > 0 && tracks[0].track.length > firstCidx) {
        launch = { lat: tracks[0].track[0].lat, lon: tracks[0].track[0].lon };
        if (downloadedCorridorBackbone?.length > 0) {
          corridor = downloadedCorridorBackbone.map((p) => ({ lat: p.lat, lon: p.lon }));
        } else {
          corridor = firstCidx > 1 ? tracks[0].track.slice(1, firstCidx) : [];
        }
      }
    }
    setSetLaunchPoints([launch]);
    setSetCorridorPoints([corridor]);
    setActiveSetIndex(0);

    // Use downloaded polygon if available, otherwise convex-hull heuristic
    let resolvedPoly = null;
    if (downloadedPolygon?.length >= 3) {
      resolvedPoly = downloadedPolygon.map((p) => ({ lat: p.lat, lon: p.lon }));
      setPolygon(resolvedPoly);
      drawing.loadVertices(resolvedPoly);
    } else if (downloadedSearchPattern !== 'corridor') {
      const allPts = tracks.flatMap((z) => {
        const ci = getCorridorIdx(z) || heuristicSkip;
        return z.track.slice(ci);
      });
      if (allPts.length < 3) return;
      const hull = convexHull(allPts);
      if (hull.length < 3) return;
      resolvedPoly = hull;
      setPolygon(hull);
      drawing.loadVertices(hull);
    }
    if (plannerReady && resolvedPoly && resolvedPoly.length >= 3 && downloadedSearchPattern !== 'corridor') {
      const a = analyzeArea(
        resolvedPoly.map((p) => ({ lat: p.lat, lon: p.lon })),
      );
      setAnalysis(a);
    }

    // Restore delivery hubs from downloaded delivery hubs
    if (downloadedMissionDeliveryHubs) {
      const existingDeliveryHubs = settings?.default_delivery_hubs || [];
      const { deliveryHubs, assignments } = buildDeliveryHubsFromDownload(downloadedMissionDeliveryHubs, existingDeliveryHubs);
      if (deliveryHubs.length > 0) {
        handleSaveSettings({ default_delivery_hubs: deliveryHubs });
        setDeliveryHubAssignments(assignments);
      }
    }

    // Trim corridor waypoints from zone tracks
    setPlan((prev) => {
      if (!prev?.zones) return prev;
      let changed = false;
      const trimmed = prev.zones.map((z) => {
        const ci = getCorridorIdx(z);
        if (ci > 0 && z.track?.length > ci) {
          changed = true;
          return { ...z, track: z.track.slice(ci), corridor_end_index: 0 };
        }
        return z;
      });
      return changed ? { ...prev, zones: trimmed } : prev;
    });
  }, [plannerReady, holdDownloadedPlan, setUavCount, setPolygon, drawing.loadVertices, setPlan, setSearchPattern, setAnalysis, setSetLaunchPoints, setSetCorridorPoints, setActiveSetIndex, handleSaveSettings, setDeliveryHubAssignments, setFenceCustomVertices, notifyFenceGeometryReplaced, settings]);

  return {
    handleSavePolygon,
    handleLoadPolygon,
    derivePlanPolygon,
  };
}
