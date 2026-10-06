import { useEffect } from 'react';
import { colors } from '../../../styles';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';

/**
 * Launch zone polygon entities — fill, outline, min launch zone dashed line.
 */
export default function useLaunchZoneLayer(cesiumRef, viewerRef, entitiesRef, lzPosRef, minLzPosRef, lzLen, minLzLen, showLaunchZone, viewerReady) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    ents.launchZone.forEach((e) => removeGroundPolyline(viewer, e));
    ents.minLaunchZone.forEach((e) => removeGroundPolyline(viewer, e));
    ents.launchZone = [];
    ents.minLaunchZone = [];

    if (!showLaunchZone || !lzLen) return;

    // Polygon fill
    const polyEntity = viewer.entities.add({
      polygon: {
        hierarchy: new Cesium.CallbackProperty(() => {
          const pos = lzPosRef.current;
          if (!pos || pos.length < 3) return new Cesium.PolygonHierarchy([]);
          return new Cesium.PolygonHierarchy(pos);
        }, false),
        material: Cesium.Color.fromCssColorString(colors.success).withAlpha(0.08),
        classificationType: Cesium.ClassificationType.TERRAIN,
      },
    });
    ents.launchZone.push(polyEntity);

    // Polyline border — ground-following via addGroundPolyline (not raw
    // clampToGround) so it still renders on browsers/GPUs without WebGL
    // depth-texture support.
    const lineEntity = addGroundPolyline(viewer, Cesium, {
      getPositions: () => (lzPosRef.current && lzPosRef.current.length >= 3 ? lzPosRef.current : []),
      width: 3,
      material: Cesium.Color.fromCssColorString(colors.success),
      loop: true,
    });
    ents.launchZone.push(lineEntity);

    // Min launch zone (red dashed)
    if (minLzLen >= 3) {
      const minLineEntity = addGroundPolyline(viewer, Cesium, {
        getPositions: () => (minLzPosRef.current && minLzPosRef.current.length >= 3 ? minLzPosRef.current : []),
        width: 3,
        material: new Cesium.PolylineDashMaterialProperty({
          color: Cesium.Color.fromCssColorString(colors.error),
          dashLength: 16,
        }),
        loop: true,
      });
      ents.minLaunchZone.push(minLineEntity);
    }
    // viewerReady gates re-run so launch-zone entities created before the
    // async Cesium viewer finished init aren't lost — same fresh-load race as
    // usePolygonLayer.
  }, [lzLen, minLzLen, showLaunchZone, viewerReady]);
}
