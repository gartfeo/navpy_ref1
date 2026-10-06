import { useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid, setColors } from '../../../styles';
import { LAUNCH_POINT_ICON, makeLaunchIcon } from '../constants/icons';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';

/**
 * Launch markers + corridor lines.
 * Creates launch point markers (per set) and both ground-clamped + terrain-sampled
 * corridor polylines.
 */
export default function useCorridorLayer(
  cesiumRef, viewerRef, entitiesRef, corridorWpRef,
  setLaunchPointsRef, setCorridorPointsRef, trackPosRef, planRef,
  setLaunchPoints, setCorridorPointsArr, numSets, zoneCount,
  showLaunchZone, showTracks, searchPattern, setLaunchPointCount, phase, setCorridorCounts,
  terrainReady, viewerReady
) {
  const corridorAltRef = useRef(0);
  const { t, i18n } = useTranslation();

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;
    let cancelled = false;

    const ents = entitiesRef.current;
    ents.launchPointMarkers.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.corridors.forEach((e) => removeGroundPolyline(viewer, e));
    ents.launchPointMarkers = [];
    ents.corridors = [];

    const slps = setLaunchPoints || [];
    const anyLP = slps.some((lp) => lp != null);
    if (!anyLP) return;

    // Launch point markers + ground-clamped corridor lines for all sets
    const editable = phase === 'PLANNING';

    slps.forEach((slp, setIdx) => {
      if (!slp) return;
      const capturedIdx = setIdx;
      const sColor = numSets > 1
        ? setColors[setIdx % setColors.length]
        : colors.warning;

      // Launch point markers — gated on showLaunchZone
      if (showLaunchZone) {
        const lpPrefix = t('mapMarkers.launchPoint');
        const label = numSets > 1 ? `${lpPrefix}${setIdx + 1}` : lpPrefix;

        // Static position in MONITOR mode for reliable heightReference clamping;
        // CallbackProperty in PLANNING for smooth drag updates.
        const lpPosition = editable
          ? new Cesium.CallbackProperty(() => {
              const lp = setLaunchPointsRef.current[capturedIdx];
              return lp ? Cesium.Cartesian3.fromDegrees(lp.lon, lp.lat) : Cesium.Cartesian3.ZERO;
            }, false)
          : Cesium.Cartesian3.fromDegrees(slp.lon, slp.lat);

        const lpEnt = viewer.entities.add({
          position: lpPosition,
          billboard: {
            image: numSets > 1 ? makeLaunchIcon(sColor, 28) : LAUNCH_POINT_ICON,
            width: numSets > 1 ? 28 : 24,
            height: numSets > 1 ? 28 : 24,
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: {
            text: label,
            font: '12px sans-serif',
            fillColor: Cesium.Color.fromCssColorString(sColor),
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: Cesium.VerticalOrigin.TOP,
            pixelOffset: new Cesium.Cartesian2(0, 14),
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            pixelOffsetScaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { isSetCorridorPoint: true, setIdx: setIdx, corridorIndex: -1 },
        });
        ents.launchPointMarkers.push(lpEnt);
      }

      // Ground-clamped corridor arrows — one per segment for direction.
      // Ground-following via addGroundPolyline (not raw clampToGround) so
      // they still render on browsers/GPUs without WebGL depth-texture
      // support.
      if (showTracks) {
        const setCps = (setCorridorPointsArr || [[]])[setIdx] || [];
        const allPts = [slp, ...setCps];
        const segColor = Cesium.Color.fromCssColorString(sColor).withAlpha(0.5);
        for (let seg = 0; seg < allPts.length - 1; seg++) {
          const capturedSeg = seg;
          const getPositions = editable
            ? () => {
                const lp = setLaunchPointsRef.current[capturedIdx];
                if (!lp) return [];
                const cps = setCorridorPointsRef.current[capturedIdx] || [];
                const pts = [lp, ...cps];
                const a = pts[capturedSeg];
                const b = pts[capturedSeg + 1];
                if (!a || !b) return [];
                return [a, b];
              }
            : () => [allPts[seg], allPts[seg + 1]];
          const lineEnt = addGroundPolyline(viewer, Cesium, {
            getPositions,
            width: 4,
            material: new Cesium.PolylineArrowMaterialProperty(segColor),
          });
          ents.corridors.push(lineEnt);
        }
      }
    });

    // Terrain-sampled approach corridor lines (per zone)
    // Skip for corridor search pattern — tracks ARE the corridor, no approach lines needed.
    // The ground-clamped line above already shows the backbone path.
    if (showTracks && searchPattern !== 'corridor') {
      const curPlan = planRef.current;
      if (curPlan?.zones && anyLP) {
        const baseAlt = curPlan.altitude_m || 150;

        // Group zones by set_index
        const setZones = {};
        curPlan.zones.forEach((zone, i) => {
          if (!zone.track || zone.track.length < 1) return;
          const si = zone.set_index ?? 0;
          if (!setZones[si]) setZones[si] = [];
          setZones[si].push({ zoneIdx: i, first: zone.track[0], alt: zone.altitude_m || baseAlt });
        });

        Object.entries(setZones).forEach(([siStr, trackStarts]) => {
          const si = parseInt(siStr);
          const setLp = slps[si];
          if (!setLp) return;
          const setCp = (setCorridorPointsArr || [[]])[si] || [];

          const allSamplePts = [
            { lon: setLp.lon, lat: setLp.lat },
            ...setCp,
          ];
          trackStarts.forEach((ts) => {
            allSamplePts.push({ lon: ts.first.lon, lat: ts.first.lat });
          });

          const cartos = allSamplePts.map((p) => Cesium.Cartographic.fromDegrees(p.lon, p.lat));
          const capturedSi = si;

          const createSetCorridors = (sampledHeights) => {
            if (cancelled) return;
            const lpTerrainH = sampledHeights[0];
            corridorAltRef.current = lpTerrainH + baseAlt;

            trackStarts.forEach((ts, j) => {
              const capturedZoneIdx = ts.zoneIdx;
              const colorStr = zoneColorsSolid[capturedZoneIdx % zoneColorsSolid.length];
              const corridorAlt = ts.alt;
              const endSampleIdx = setCp.length + 1 + j;
              const endTerrainH = sampledHeights[endSampleIdx];
              const endPosFallback = Cesium.Cartesian3.fromDegrees(
                ts.first.lon, ts.first.lat, endTerrainH + corridorAlt
              );
              const cpTerrainHeights = setCp.map((_, ci) => sampledHeights[1 + ci]);

              // Per-segment arrow polylines for approach corridor
              const numApproachSegs = setCp.length + 1; // LP→CP1, ..., CPn→trackStart
              const arrowColor = Cesium.Color.fromCssColorString(colorStr).withAlpha(0.7);
              for (let seg = 0; seg < numApproachSegs; seg++) {
                const capturedSeg = seg;
                const e = viewer.entities.add({
                  polyline: {
                    positions: new Cesium.CallbackProperty(() => {
                      const lp = setLaunchPointsRef.current[capturedSi];
                      if (!lp) return [];
                      const cps = setCorridorPointsRef.current[capturedSi] || [];
                      const pts = [Cesium.Cartesian3.fromDegrees(lp.lon, lp.lat, lpTerrainH)];
                      cps.forEach((cp, ci) => {
                        const h = ci < cpTerrainHeights.length ? cpTerrainHeights[ci] : lpTerrainH;
                        pts.push(Cesium.Cartesian3.fromDegrees(cp.lon, cp.lat, h + corridorAlt));
                      });
                      const tp = trackPosRef.current[capturedZoneIdx];
                      pts.push(tp && tp.length > 0 ? tp[0] : endPosFallback);
                      if (capturedSeg + 1 >= pts.length) return [];
                      return [pts[capturedSeg], pts[capturedSeg + 1]];
                    }, false),
                    width: 4,
                    material: new Cesium.PolylineArrowMaterialProperty(arrowColor),
                  },
                });
                ents.corridors.push(e);
              }
            });
          };

          if (terrainReady) {
            Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, cartos)
              .then((sampled) => createSetCorridors(sampled.map((s) => s.height || 0)))
              .catch(() => createSetCorridors(allSamplePts.map(() => 0)));
          } else {
            createSetCorridors(allSamplePts.map(() => 0));
          }
        });
      }
    }
    return () => { cancelled = true; };
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [numSets, zoneCount, showLaunchZone, showTracks, searchPattern, setLaunchPointCount, phase, setCorridorCounts, terrainReady, viewerReady, i18n.language]);
}
