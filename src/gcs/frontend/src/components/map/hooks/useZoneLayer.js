import { useEffect } from 'react';
import { zoneColorsSolid } from '../../../styles';

/**
 * Zone polygon overlays — structural recreation only, CallbackProperty for positions.
 */
export default function useZoneLayer(cesiumRef, viewerRef, entitiesRef, zonePosRef, zoneCount, showZones, searchPattern, viewerReady) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    ents.zones.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.zones = [];

    if (!showZones || !zoneCount) return;

    for (let i = 0; i < zoneCount; i++) {
      const idx = i;
      const colorStr = zoneColorsSolid[i % zoneColorsSolid.length];
      const e = viewer.entities.add({
        polygon: {
          hierarchy: new Cesium.CallbackProperty(() => {
            const pos = zonePosRef.current[idx];
            if (!pos || pos.length < 3) return new Cesium.PolygonHierarchy([]);
            return new Cesium.PolygonHierarchy(pos);
          }, false),
          material: Cesium.Color.fromCssColorString(colorStr).withAlpha(0.15),
          classificationType: Cesium.ClassificationType.TERRAIN,
        },
      });
      ents.zones.push(e);
    }
    // viewerReady gates re-run so zones created before the async Cesium
    // viewer finished init aren't lost — same fresh-load race as
    // usePolygonLayer.
  }, [zoneCount, showZones, searchPattern, viewerReady]);
}
