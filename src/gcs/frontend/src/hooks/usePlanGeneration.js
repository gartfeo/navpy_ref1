import { useCallback, useEffect, useRef } from 'react';
import { analyzeArea, generatePlan, computeSets, setTrackSpacingOverride } from '../utils/planner';
import { autoAssignFallbackLocations } from '../utils/fallbackLocationAssignment';
import { PHASES } from './useMissionState';

/**
 * Manages local (instant) plan analysis and generation, plus auto-regen effects.
 */
export default function usePlanGeneration({
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
}) {
  const effectiveUavCount = uavCount ?? analysis?.required_uavs ?? 3;
  const effectiveSets = computeSets(effectiveUavCount);

  // Keep phase accessible to effects without making it a dependency
  const phaseRef = useRef(phase);
  phaseRef.current = phase;

  // Resize per-set arrays when effective sets count changes
  useEffect(() => {
    setSetLaunchPoints((prev) => {
      if (prev.length === effectiveSets) return prev;
      if (prev.length < effectiveSets) return [...prev, ...Array(effectiveSets - prev.length).fill(null)];
      return prev.slice(0, effectiveSets);
    });
    setSetCorridorPoints((prev) => {
      if (prev.length === effectiveSets) return prev;
      if (prev.length < effectiveSets) return [...prev, ...Array(effectiveSets - prev.length).fill(null).map(() => [])];
      return prev.slice(0, effectiveSets);
    });
    setActiveSetIndex((prev) => Math.min(prev, effectiveSets - 1));
  }, [effectiveSets]);

  // Per-set approach points: last corridor point for each set, or its launch point
  const setApproachPoints = setLaunchPoints.map((lp, si) => {
    const cp = (setCorridorPointsArr ?? [[]])[si] ?? [];
    if (cp.length > 0) return cp[cp.length - 1];
    return lp;
  });

  const localAnalyze = useCallback((poly, selectedDockClasses) => {
    if (!plannerReady) { setAnalysis(null); return; }
    if (!poly || poly.length < 3) { setAnalysis(null); return; }
    const polyObj = poly.map((p) => ({ lat: p.lat, lon: p.lon }));
    const a = analyzeArea(polyObj, selectedDockClasses);
    setAnalysis(a);
    if (searchPattern !== 'corridor' && poly.length >= 3) {
      let count = uavCount ?? a.required_uavs ?? 3;
      if (!uavCountLocked && a.required_uavs > (uavCount ?? a.required_uavs)) {
        setUavCount(a.required_uavs);
        count = a.required_uavs;
      }
      const ap = corridorPoints.length > 0
        ? corridorPoints[corridorPoints.length - 1] : launchPoint;
      const cp = null; // non-corridor — no corridor path
      const result = generatePlan(
        polyObj, selectedDockClasses, searchPattern, count,
        ap ? { lat: ap.lat, lon: ap.lon } : null, cp,
        setApproachPoints, partitionAngleDeg,
      );
      setPlan(result);
    }
  }, [plannerReady, setAnalysis, setPlan, searchPattern, uavCount, uavCountLocked, setUavCount, launchPoint, corridorPoints, setApproachPoints, partitionAngleDeg]);

  const localGenerate = useCallback((poly, selectedDockClasses, localSearchPattern, count, lp, corridor) => {
    if (!plannerReady) return null;
    if (localSearchPattern === 'corridor') {
      if (!corridor || corridor.length < 2) return null;
      if (!poly || poly.length < 3) poly = corridor;
    } else if (!poly || poly.length < 3) {
      return null;
    }
    const result = generatePlan(
      poly.map((p) => ({ lat: p.lat, lon: p.lon })),
      selectedDockClasses, localSearchPattern, count,
      lp ? { lat: lp.lat, lon: lp.lon } : null,
      corridor?.map((p) => ({ lat: p.lat, lon: p.lon })) || null,
      setApproachPoints, partitionAngleDeg,
    );
    // Apply altitude separation per zone so tracks + corridors render at staggered heights
    if (result?.zones?.length > 1 && result.altitude_separation_m) {
      const alt = result.altitude_m || 150;
      const sep = result.altitude_separation_m;
      result.zones = result.zones.map((z, i) => ({
        ...z,
        altitude_m: alt + (result.zones.length - 1 - i) * sep,
      }));
    }
    setPlan(result);
    // Auto-assign fallback locations when plan has zones and fallback locations exist (skip if manually edited)
    const fallbackLocations = settings?.fallback_delivery_locations;
    if (result?.zones?.length && fallbackLocations?.length && setFallbackLocationAssignments && !manualFallbackLocationEdit) {
      setFallbackLocationAssignments(autoAssignFallbackLocations(result.zones, fallbackLocations));
    }
    return result;
  }, [plannerReady, setPlan, setApproachPoints, partitionAngleDeg, settings, setFallbackLocationAssignments, manualFallbackLocationEdit]);

  // Approach point for zigzag direction: last corridor point, or launch point
  const approachPoint = corridorPoints.length > 0
    ? corridorPoints[corridorPoints.length - 1]
    : launchPoint;

  // Corridor search pattern: full path = launch + corridor points
  const corridorPath = searchPattern === 'corridor' && launchPoint && corridorPoints.length > 0
    ? [launchPoint, ...corridorPoints] : null;

  // Sync uavCount from analysis — set initial count only
  useEffect(() => {
    if (!analysis?.required_uavs) return;
    if (uavCount == null) {
      setUavCount(analysis.required_uavs);
    }
  }, [analysis]);

  // Auto-generate plan when searchPattern or uavCount changes (PLANNING only)
  useEffect(() => {
    if (!plannerReady) return;
    if (phaseRef.current !== PHASES.PLANNING) return;
    if (suppressRegenRef.current) return;
    if (searchPattern === 'corridor') {
      if (corridorPath) {
        const polyArg = polygon.length >= 3 ? polygon : corridorPath;
        localGenerate(polyArg, dockClasses, searchPattern, effectiveUavCount, approachPoint, corridorPath);
      }
    } else if (analysis && polygon.length >= 3) {
      localGenerate(polygon, dockClasses, searchPattern, effectiveUavCount, approachPoint, corridorPath);
    }
  }, [plannerReady, searchPattern, effectiveUavCount, partitionAngleDeg]);

  // Demo-mode route offset: push the override into the planner config, then
  // re-analyze + regenerate so tracks reflect the new spacing. The override is
  // applied even outside PLANNING so any later generation uses it; only the
  // regen itself is gated. Skipped on mount (null override is already active).
  const routeOffsetMountRef = useRef(true);
  useEffect(() => {
    setTrackSpacingOverride(routeOffsetM);
    if (routeOffsetMountRef.current) { routeOffsetMountRef.current = false; return; }
    if (!plannerReady) return;
    if (phaseRef.current !== PHASES.PLANNING) return;
    if (suppressRegenRef.current) return; // snapshot restore — keep the restored plan
    if (searchPattern === 'corridor') {
      if (corridorPath) {
        const polyArg = polygon.length >= 3 ? polygon : corridorPath;
        localGenerate(polyArg, dockClasses, searchPattern, effectiveUavCount, approachPoint, corridorPath);
      }
    } else if (polygon.length >= 3) {
      localAnalyze(polygon, dockClasses);
    }
  }, [routeOffsetM]);

  // Clear plan/analysis when polygon is removed (PLANNING only)
  useEffect(() => {
    if (phaseRef.current !== PHASES.PLANNING) return;
    if (searchPattern === 'corridor') return;
    if (!polygon || polygon.length < 3) {
      setPlan((prev) => {
        if (!prev?.zones) return null;
        const isDownloaded = prev.zones.every((z) => !z.polygon || z.polygon.length === 0);
        return isDownloaded ? prev : null;
      });
      setAnalysis(null);
      setSetLaunchPoints([null]);
      setSetCorridorPoints([[]]);
      setActiveSetIndex(0);
      setPartitionAngleDeg(null);
    }
  }, [polygon, searchPattern]);

  // When entering PLANNING, clear suppress flag and re-analyze if needed
  useEffect(() => {
    if (!plannerReady) return;
    if (phase === PHASES.PLANNING) {
      suppressRegenRef.current = false;
      if (polygon.length >= 3 && !analysis) {
        if (plan) {
          // Plan exists (downloaded/uploaded) — analyze for UI (launch zones etc.)
          // but do NOT regenerate: preserves tracks and simDockWps indices.
          const a = analyzeArea(
            polygon.map((p) => ({ lat: p.lat, lon: p.lon })),
            dockClasses,
          );
          setAnalysis(a);
        } else {
          localAnalyze(polygon, dockClasses);
        }
      }
    }
  }, [plannerReady, phase]);

  // Re-generate plan when approach point changes (PLANNING only, throttled for live drag)
  const lpThrottleRef = useRef(0);
  useEffect(() => {
    if (!plannerReady) return;
    if (phaseRef.current !== PHASES.PLANNING) return;
    if (suppressRegenRef.current) {
      suppressRegenRef.current = false;
      return;
    }
    const curCorridorPath = searchPattern === 'corridor' && launchPoint && corridorPoints.length > 0
      ? [launchPoint, ...corridorPoints] : null;
    if (searchPattern === 'corridor') {
      if (!curCorridorPath) { setPlan(null); return; }
    } else {
      const anyApproach = approachPoint || setLaunchPoints.some(lp => lp != null);
      if (!analysis || polygon.length < 3 || !anyApproach) return;
    }
    const now = Date.now();
    const elapsed = now - lpThrottleRef.current;
    const delay = elapsed >= 100 ? 0 : 100 - elapsed;
    const polyArg = searchPattern === 'corridor' ? (polygon.length >= 3 ? polygon : curCorridorPath) : polygon;
    if (regenTimerRef.current) clearTimeout(regenTimerRef.current);
    regenTimerRef.current = setTimeout(() => {
      regenTimerRef.current = null;
      lpThrottleRef.current = Date.now();
      localGenerate(polyArg, dockClasses, searchPattern, effectiveUavCount, approachPoint, curCorridorPath);
    }, delay);
    return () => { clearTimeout(regenTimerRef.current); regenTimerRef.current = null; };
  }, [plannerReady, launchPoint, corridorPoints, setLaunchPoints, setCorridorPointsArr]);

  // Re-analyze when settings change (PLANNING only)
  useEffect(() => {
    if (!plannerReady) return;
    if (phaseRef.current !== PHASES.PLANNING) return;
    if (!settingsVersion) return; // skip initial mount
    if (polygon.length >= 3) {
      localAnalyze(polygon, dockClasses);
    }
  }, [plannerReady, settingsVersion]);

  // Re-analyze when dock classes change
  const handleTargetChange = useCallback(
    (newTargets) => {
      if (polygon.length >= 3) {
        localAnalyze(polygon, newTargets);
      }
    },
    [polygon, localAnalyze]
  );

  return {
    effectiveUavCount,
    effectiveSets,
    localAnalyze,
    localGenerate,
    approachPoint,
    corridorPath,
    handleTargetChange,
  };
}
