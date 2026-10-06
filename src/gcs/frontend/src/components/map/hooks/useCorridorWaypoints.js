import { useEffect } from 'react';
import { colors, setColors } from '../../../styles';
import { MIDPOINT_ICON, makeCorridorIcon } from '../constants/icons';

/**
 * Per-set waypoint markers + midpoint handles.
 * Only recreated for the set that changed — prevents flickering other sets.
 */
export default function useCorridorWaypoints(
  cesiumRef, viewerRef, corridorWpRef,
  setCorridorPointsRef, setLaunchPointsRef,
  setCorridorPointsArr, setLaunchPoints,
  phase, showTracks, numSets, setLaunchPointCount, setCorridorCounts,
  terrainReady
) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const slps = setLaunchPoints || [];
    const scps = setCorridorPointsArr || [[]];
    const editable = phase === 'PLANNING';
    const maxSi = Math.max(slps.length, ...Object.keys(corridorWpRef.current).map(Number), 0) + 1;

    for (let si = 0; si < maxSi; si++) {
      const hasLp = slps[si] != null;
      const cpCount = hasLp ? (scps[si] || []).length : 0;
      const key = `${hasLp}:${cpCount}:${editable}:${showTracks}:${numSets}`;
      const prev = corridorWpRef.current[si];
      if (prev && prev.key === key) continue;

      // Tear down old entities for this set
      if (prev) {
        prev.wps.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
        prev.mids.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
      }

      if (!hasLp || !showTracks) {
        delete corridorWpRef.current[si];
        continue;
      }

      const setData = { key, wps: [], mids: [] };
      const sColor = numSets > 1 ? setColors[si % setColors.length] : colors.warning;
      const setCps = scps[si] || [];

      // Corridor waypoint markers
      setCps.forEach((cp, ci) => {
        const capturedSetIdx = si;
        const capturedCorridorIdx = ci;
        // Static position in MONITOR for reliable heightReference; CallbackProperty in PLANNING for drag.
        const cpPosition = editable
          ? new Cesium.CallbackProperty(() => {
              const arr = setCorridorPointsRef.current[capturedSetIdx];
              const p = arr ? arr[capturedCorridorIdx] : null;
              return p ? Cesium.Cartesian3.fromDegrees(p.lon, p.lat) : Cesium.Cartesian3.ZERO;
            }, false)
          : Cesium.Cartesian3.fromDegrees(cp.lon, cp.lat);

        const cpEnt = viewer.entities.add({
          position: cpPosition,
          billboard: {
            image: makeCorridorIcon(sColor, 18),
            width: 18,
            height: 18,
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: {
            text: `КТ${ci + 1}`,
            font: '10px sans-serif',
            fillColor: Cesium.Color.fromCssColorString(sColor),
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: Cesium.VerticalOrigin.TOP,
            pixelOffset: new Cesium.Cartesian2(0, 12),
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            pixelOffsetScaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { isSetCorridorPoint: true, setIdx: si, corridorIndex: ci },
        });
        setData.wps.push(cpEnt);
      });

      // Corridor midpoint handles (PLANNING phase only)
      if (editable && setCps.length > 0) {
        const capturedSetIdx = si;
        const lp = slps[si];
        for (let i = 0; i < setCps.length; i++) {
          const capturedMidIdx = i;
          const a = i === 0 ? lp : setCps[i - 1];
          const b = setCps[i];
          const e = viewer.entities.add({
            position: (a && b)
              ? new Cesium.CallbackProperty(() => {
                  const pts = setCorridorPointsRef.current[capturedSetIdx] || [];
                  const pa = capturedMidIdx === 0
                    ? setLaunchPointsRef.current[capturedSetIdx]
                    : pts[capturedMidIdx - 1];
                  const pb = pts[capturedMidIdx];
                  if (!pa || !pb) return Cesium.Cartesian3.ZERO;
                  return Cesium.Cartesian3.fromDegrees((pa.lon + pb.lon) / 2, (pa.lat + pb.lat) / 2);
                }, false)
              : Cesium.Cartesian3.ZERO,
            billboard: {
              image: MIDPOINT_ICON,
              width: 12,
              height: 12,
              heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
              disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
            properties: { isCorridorMidpoint: true, setIdx: si, corridorMidIndex: i },
          });
          setData.mids.push(e);
        }
      }

      corridorWpRef.current[si] = setData;
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [setCorridorCounts, phase, showTracks, numSets, setLaunchPointCount, terrainReady]);
}
