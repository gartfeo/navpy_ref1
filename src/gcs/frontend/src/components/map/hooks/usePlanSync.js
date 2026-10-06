import { useRef, useEffect } from 'react';
import { shouldApplyTerrainSample } from '../utils/terrainGuard';

/**
 * Plan data sync + terrain sampling.
 * Syncs pre-computed Cartesian3 positions from plan/analysis and debounces
 * terrain sampling for track altitude accuracy.
 */
export default function usePlanSync(cesiumRef, viewerRef, plan, analysis, terrainReady) {
  const planRef = useRef(null);
  planRef.current = plan;
  const analysisRef = useRef(null);
  analysisRef.current = analysis;

  const zonePosRef = useRef([]);
  const trackPosRef = useRef([]);
  const lzPosRef = useRef([]);
  const minLzPosRef = useRef([]);
  const clampOuterRef = useRef(null);
  const terrainTimerRef = useRef(null);
  const terrainBaseRef = useRef([]);

  // Track previous plan/analysis identity to avoid overwriting terrain-sampled
  // positions on unrelated re-renders (e.g. 5 Hz telemetry updates).
  const prevSyncPlanRef = useRef(null);
  const prevSyncAnalysisRef = useRef(null);

  // Sync pre-computed positions during render — only when plan/analysis actually
  // changes (reference identity).
  {
    const Cesium = cesiumRef.current;
    if (Cesium) {
      const planChanged = plan !== prevSyncPlanRef.current;
      const analysisChanged = analysis !== prevSyncAnalysisRef.current;

      if (planChanged) {
        prevSyncPlanRef.current = plan;
        if (plan?.zones) {
          const baseAlt = plan.altitude_m || 150;
          zonePosRef.current = plan.zones.map((z) =>
            z.polygon?.length >= 3 ? z.polygon.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat)) : []
          );
          trackPosRef.current = plan.zones.map((z, i) => {
            const zoneAlt = z.altitude_m || baseAlt;
            const tH = terrainBaseRef.current[i] || 0;
            return z.track?.length >= 2 ? z.track.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat, tH + (p.alt ?? zoneAlt))) : [];
          });
        } else {
          zonePosRef.current = [];
          trackPosRef.current = [];
        }
      }

      if (planChanged || analysisChanged) {
        prevSyncAnalysisRef.current = analysis;
        const src = analysis || plan;
        const ld = src?.launch_zone;
        lzPosRef.current = Array.isArray(ld) && ld.length >= 3
          ? ld.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat)) : [];
        const mld = src?.min_launch_zone;
        minLzPosRef.current = Array.isArray(mld) && mld.length >= 3
          ? mld.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat)) : [];
        // Sync clamp boundary — same data as green line
        clampOuterRef.current = (Array.isArray(ld) && ld.length >= 3) ? ld
          : (Array.isArray(mld) && mld.length >= 3) ? mld : null;
      }
    }
  }

  // Debounced terrain sampling — updates track positions after drag settles.
  // Waits for terrainReady so the provider returns real heights, not zeros.
  useEffect(() => {
    if (!terrainReady) return;
    if (terrainTimerRef.current) clearTimeout(terrainTimerRef.current);
    terrainTimerRef.current = setTimeout(() => {
      const Cesium = cesiumRef.current;
      const viewer = viewerRef.current;
      if (!Cesium || !viewer || !plan?.zones) return;
      const baseAlt = plan.altitude_m || 150;
      plan.zones.forEach((zone, i) => {
        if (!zone.track || zone.track.length < 2) return;
        const zoneAlt = zone.altitude_m || baseAlt;
        const cartos = zone.track.map((p) => Cesium.Cartographic.fromDegrees(p.lon, p.lat));
        Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, cartos)
          .then((sampled) => {
            const heights = sampled.map((c) => c.height || 0);
            if (!shouldApplyTerrainSample(heights)) return;
            trackPosRef.current[i] = sampled.map((c, j) =>
              Cesium.Cartesian3.fromRadians(c.longitude, c.latitude, (c.height || 0) + (zone.track[j].alt ?? zoneAlt))
            );
            const avgH = heights.reduce((s, h) => s + h, 0) / (heights.length || 1);
            terrainBaseRef.current[i] = avgH;
          })
          .catch((err) => { console.warn('Terrain sampling failed:', err); });
      });
    }, 300);
    return () => { if (terrainTimerRef.current) clearTimeout(terrainTimerRef.current); };
  }, [plan, terrainReady]);

  return { planRef, trackPosRef, zonePosRef, lzPosRef, minLzPosRef, clampOuterRef, terrainBaseRef };
}
