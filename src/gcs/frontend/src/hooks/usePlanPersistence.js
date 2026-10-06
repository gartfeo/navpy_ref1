import { useCallback } from 'react';
import { useTranslation } from 'react-i18next';
import { analyzeArea } from '../utils/planner';
import { convexHull } from '../utils/geo';
import { buildFallbackLocationsFromDownload } from '../utils/fallbackLocationAssignment';

/**
 * Save/load plan geometry to/from JSON files, and derive polygon from downloaded plan tracks.
 */
export default function usePlanPersistence({
  polygon,
  launchPoint,
  corridorPoints,
  searchPattern,
  setPolygon,
  setSearchPattern,
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
    };
    // Save per-set launch points and corridors
    const savedLps = setLaunchPoints.map((lp) => lp ? { lat: lp.lat, lon: lp.lon } : null);
    const savedCps = (setCorridorPointsArr ?? [[]]).map((cp) => (cp || []).map((p) => ({ lat: p.lat, lon: p.lon })));
    if (savedLps.some((lp) => lp != null)) data.set_launch_points = savedLps;
    if (savedCps.some((cp) => cp.length > 0)) data.set_corridors = savedCps;
    if (fallbackLocationAssignments?.length > 0) data.fallback_location_assignments = fallbackLocationAssignments;
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
    const fallbackLocations = settings?.fallback_delivery_locations || [];
    if (fallbackLocations.length > 0) data.fallback_delivery_locations = fallbackLocations;
    const json = JSON.stringify(data, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'plan.json';
    a.click();
    URL.revokeObjectURL(url);
  }, [polygon, launchPoint, corridorPoints, searchPattern, setLaunchPoints, setCorridorPointsArr, fallbackLocationAssignments, simDockWps, detectAfterWps, settings, fenceEnabled, fenceOffsetM, fenceTouched, fenceIntent, fenceCustomVertices, exclusionPolygons]);

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
          || ['poi_classes', 'delivery_docks', 'delivery_dock_assignments', 'tactic'].some((key) => Object.hasOwn(raw, key))) {
          window.alert(t('planFile.invalidData'));
          return;
        }
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
          const poly = raw.polygon;
          if (poly && Array.isArray(poly) && poly.length >= 3) {
            setPolygon(poly);
            drawing.loadVertices(poly);
          }
          if (raw.search_pattern) setSearchPattern(raw.search_pattern);
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
          if (raw.fallback_delivery_locations?.length > 0 && handleSaveSettings) {
            handleSaveSettings({ fallback_delivery_locations: raw.fallback_delivery_locations });
          }
          if (raw.fallback_location_assignments?.length > 0 && setFallbackLocationAssignments) {
            setFallbackLocationAssignments(raw.fallback_location_assignments);
          }
          if (raw.sim_dock_wps && setSimDockWps) setSimDockWps(raw.sim_dock_wps);
          if (raw.detect_after_wps && setDetectAfterWps) setDetectAfterWps(raw.detect_after_wps);
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
          // observations no longer describe it.
          resetFenceObservations?.();
          undoRef.current = [];
          if (poly?.length >= 3 && raw.search_pattern !== 'corridor') localAnalyze(poly);
        } catch {
          window.alert(t('planFile.invalidData'));
        }
      };
      reader.readAsText(file);
    };
    input.click();
  }, [t, setPolygon, drawing.loadVertices, localAnalyze, setSearchPattern, setLaunchPoint, setCorridorPoints, authorFenceIntent, clearFenceIntent, setFenceEnabled, setFenceTouched, resetFenceObservations]);

  // Derive polygon + launch/corridor from plan tracks using corridor_end_index
  const derivePlanPolygon = useCallback((zones, downloadedSearchPattern, downloadedPolygon, downloadedCorridorBackbone, downloadedLaunchPoint, downloadedMissionFallbackLocations) => {
    const tracks = zones.filter((z) => z.track?.length);
    if (tracks.length === 0) return;
    undoRef.current = [];

    // Suppress auto-regeneration
    suppressRegenRef.current = true;

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

    // Launch point from metadata (preferred) or heuristic fallback
    let heuristicSkip = 0;
    if (downloadedLaunchPoint) {
      setLaunchPoint({ lat: downloadedLaunchPoint.lat, lon: downloadedLaunchPoint.lon });
      const firstCidx = getCorridorIdx(tracks[0]);
      if (downloadedCorridorBackbone?.length > 0) {
        setCorridorPoints(downloadedCorridorBackbone.map((p) => ({ lat: p.lat, lon: p.lon })));
      } else {
        setCorridorPoints(firstCidx > 0 ? tracks[0].track.slice(0, firstCidx) : []);
      }
    } else {
      // Heuristic fallback: detect shared first waypoint across zones
      heuristicSkip = 0;
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
        setLaunchPoint({ lat: tracks[0].track[0].lat, lon: tracks[0].track[0].lon });
        if (downloadedCorridorBackbone?.length > 0) {
          setCorridorPoints(downloadedCorridorBackbone.map((p) => ({ lat: p.lat, lon: p.lon })));
        } else {
          setCorridorPoints(firstCidx > 1 ? tracks[0].track.slice(1, firstCidx) : []);
        }
      } else {
        setLaunchPoint(null);
        setCorridorPoints([]);
      }
    }

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

    // Restore fallback locations from downloaded fallback locations
    if (downloadedMissionFallbackLocations) {
      const existingFallbackLocations = settings?.fallback_delivery_locations || [];
      const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(downloadedMissionFallbackLocations, existingFallbackLocations);
      if (fallbackLocations.length > 0) {
        handleSaveSettings({ fallback_delivery_locations: fallbackLocations });
        setFallbackLocationAssignments(assignments);
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
  }, [plannerReady, setPolygon, drawing.loadVertices, setLaunchPoint, setCorridorPoints, setPlan, setSearchPattern, setAnalysis, setSetLaunchPoints, setSetCorridorPoints, handleSaveSettings, setFallbackLocationAssignments, setFenceCustomVertices, notifyFenceGeometryReplaced, settings]);

  return {
    handleSavePolygon,
    handleLoadPolygon,
    derivePlanPolygon,
  };
}
