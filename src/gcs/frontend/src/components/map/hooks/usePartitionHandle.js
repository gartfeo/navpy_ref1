import { useEffect } from 'react';
import { flatDist, offsetLatLon } from '../../../utils/geo';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';

/**
 * Partition rotation handle — rendered when effectiveSets > 1, polygon >= 3,
 * non-corridor search pattern. Dragging rotates the partition angle.
 */
export default function usePartitionHandle(
  cesiumRef, viewerRef, entitiesRef, polygonRef, partitionAngleDegRef,
  polygon, partitionAngleDeg, effectiveSets, searchPattern, phase, viewerReady
) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    if (ents.partitionHandle) { try { viewer.entities.remove(ents.partitionHandle); } catch {} }
    if (ents.partitionLine) { removeGroundPolyline(viewer, ents.partitionLine); }
    ents.partitionHandle = null;
    ents.partitionLine = null;

    const poly = polygon || [];
    if (effectiveSets <= 1 || poly.length < 3 || searchPattern === 'corridor' || phase !== 'PLANNING') return;

    const ROTATION_ICON = `data:image/svg+xml,${encodeURIComponent(
      '<svg xmlns="http://www.w3.org/2000/svg" width="28" height="28">' +
      '<circle cx="14" cy="14" r="12" fill="rgba(255,255,255,0.15)" stroke="white" stroke-width="1.5"/>' +
      '<path d="M14 5a9 9 0 0 1 7.8 4.5" stroke="white" stroke-width="2" fill="none" stroke-linecap="round"/>' +
      '<path d="M20.5 6.5l1.8 3.5-3.5-1.3" stroke="white" stroke-width="1.5" fill="none" stroke-linecap="round"/>' +
      '</svg>'
    )}`;

    // Handle billboard at polygon centroid
    ents.partitionHandle = viewer.entities.add({
      position: new Cesium.CallbackProperty(() => {
        const p = polygonRef.current;
        if (p.length < 3) return Cesium.Cartesian3.ZERO;
        const cLat = p.reduce((s, pt) => s + pt.lat, 0) / p.length;
        const cLon = p.reduce((s, pt) => s + pt.lon, 0) / p.length;
        return Cesium.Cartesian3.fromDegrees(cLon, cLat);
      }, false),
      billboard: {
        image: ROTATION_ICON,
        width: 28,
        height: 28,
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      properties: { isPartitionHandle: true },
    });

    // Direction line — short polyline from centroid along partition angle.
    // Ground-following via addGroundPolyline (not raw clampToGround) so it
    // still renders on browsers/GPUs without WebGL depth-texture support.
    ents.partitionLine = addGroundPolyline(viewer, Cesium, {
      getPositions: () => {
        const p = polygonRef.current;
        if (p.length < 3) return [];
        const cLat = p.reduce((s, pt) => s + pt.lat, 0) / p.length;
        const cLon = p.reduce((s, pt) => s + pt.lon, 0) / p.length;
        const center = { lat: cLat, lon: cLon };
        let maxDist = 0;
        for (const pt of p) {
          const d = flatDist(center, pt);
          if (d > maxDist) maxDist = d;
        }
        const lineLen = maxDist * 0.3;
        const angleDeg = partitionAngleDegRef.current;
        const perpRad = (angleDeg != null ? angleDeg : 0) * Math.PI / 180;
        const endPt = offsetLatLon(cLat, cLon, lineLen * Math.sin(perpRad), lineLen * Math.cos(perpRad));
        const dLat = endPt.lat - cLat;
        const dLon = endPt.lon - cLon;
        return [
          { lon: cLon - dLon, lat: cLat - dLat },
          { lon: cLon + dLon, lat: cLat + dLat },
        ];
      },
      width: 2,
      material: Cesium.Color.WHITE.withAlpha(0.6),
    });
  }, [polygon, effectiveSets, searchPattern, partitionAngleDeg, phase, viewerReady]);
}
