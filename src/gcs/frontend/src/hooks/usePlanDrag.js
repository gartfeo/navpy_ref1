import { useCallback, useRef } from 'react';
import { analyzeArea } from '../utils/planner';
import { pointInPolygon } from '../components/map/CesiumMap';

/**
 * Manages polygon/vertex drag lifecycle: start/end with regen control,
 * UAV-count-locked vertex constraint, min-launch-zone polygon constraint,
 * undo recording, suppress-regen flag, and partition angle drag.
 */
export default function usePlanDrag({
  drawing,
  polygon, dockClasses, searchPattern,
  launchPoint, corridorPoints,
  uavCountLocked, effectiveUavCount, approachPoint,
  plan, analysis,
  localAnalyze, localGenerate,
  plannerReady,
  draggingRef, regenTimerRef, undoRef, suppressRegenRef,
  setPartitionAngleDeg,
}) {
  const dragMlzRef = useRef(null);

  const handlePolygonDragRecord = useCallback((oldPolygon) => {
    undoRef.current = [...undoRef.current, { type: 'polygonDrag', oldPolygon }];
  }, []);

  const handleDragStart = useCallback(() => {
    draggingRef.current = true;
    if (uavCountLocked) {
      dragMlzRef.current = (plan || analysis)?.min_launch_zone || null;
    }
    if (regenTimerRef.current) { clearTimeout(regenTimerRef.current); regenTimerRef.current = null; }
  }, [uavCountLocked, plan, analysis]);

  const handleDragEnd = useCallback((skipRegen) => {
    draggingRef.current = false;
    dragMlzRef.current = null;
    if (regenTimerRef.current) { clearTimeout(regenTimerRef.current); regenTimerRef.current = null; }
    if (!plannerReady) return;
    if (!skipRegen) {
      if (polygon.length >= 3) {
        localAnalyze(polygon, dockClasses);
      } else if (searchPattern === 'corridor' && launchPoint && corridorPoints.length > 0) {
        const cp = [launchPoint, ...corridorPoints];
        localGenerate(cp, dockClasses, searchPattern, effectiveUavCount, approachPoint, cp);
      }
    }
  }, [plannerReady, polygon, dockClasses, localAnalyze, searchPattern, launchPoint, corridorPoints, effectiveUavCount, approachPoint, localGenerate]);

  const handleSuppressRegen = useCallback(() => {
    suppressRegenRef.current = true;
  }, []);

  // Vertex drag — reject if locked and candidate needs more UAVs
  const handleVertexDrag = useCallback((index, newLatLon) => {
    if (!plannerReady || !uavCountLocked || polygon.length < 3) {
      drawing.onVertexDrag(index, newLatLon);
      return;
    }
    if (uavCountLocked && polygon.length >= 3) {
      const candidate = polygon.map((p, i) =>
        i === index ? { lat: newLatLon.lat, lon: newLatLon.lon } : { lat: p.lat, lon: p.lon }
      );
      if (analyzeArea(candidate, dockClasses).min_uavs > effectiveUavCount) {
        return;
      }
    }
    drawing.onVertexDrag(index, newLatLon);
  }, [plannerReady, uavCountLocked, polygon, dockClasses, effectiveUavCount, drawing.onVertexDrag]);

  // Polygon move — reject if any vertex would exit frozen min_launch_zone
  const handlePolygonMove = useCallback((dlat, dlon) => {
    if (uavCountLocked) {
      const mlz = dragMlzRef.current;
      if (mlz && mlz.length >= 3) {
        const anyOutside = polygon.some(v =>
          !pointInPolygon({ lat: v.lat + dlat, lon: v.lon + dlon }, mlz)
        );
        if (anyOutside) return;
      }
    }
    drawing.onPolygonMove(dlat, dlon);
  }, [uavCountLocked, polygon, drawing.onPolygonMove]);

  // Partition angle drag handler
  const handlePartitionAngleDrag = useCallback((deg) => {
    setPartitionAngleDeg(deg);
  }, []);

  return {
    handlePolygonDragRecord,
    handleDragStart,
    handleDragEnd,
    handleSuppressRegen,
    handleVertexDrag,
    handlePolygonMove,
    handlePartitionAngleDrag,
  };
}
