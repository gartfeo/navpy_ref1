import { useEffect, useRef } from 'react';
import { colors } from '../../../styles';
import { VERTEX_ICON, MIDPOINT_ICON } from '../constants/icons';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';
import { planVertexBillboardPlacement } from '../utils/planVertexPlacement';

/**
 * User polygon entities — fill, outline, vertices, midpoint handles.
 * Uses CallbackProperty for fill/outline so position changes during drag
 * don't require entity recreation. Vertices/midpoints updated in-place
 * when count hasn't changed; full recreation only on count or phase change.
 */
export default function usePolygonLayer(cesiumRef, viewerRef, entitiesRef, polygonRef, polygon, phase, viewerReady) {
  const prevPolyLenRef = useRef(0);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    const editable = phase === 'PLANNING';
    const poly = polygon || [];
    const prevLen = prevPolyLenRef.current;
    const countChanged = poly.length !== prevLen || ents.vertices.length !== (editable ? poly.length : 0);
    prevPolyLenRef.current = poly.length;

    // Fast path: same vertex count — update positions in-place
    if (!countChanged && poly.length >= 3 && ents.polygon) {
      if (editable) {
        poly.forEach((pt, i) => {
          if (ents.vertices[i]) {
            const placement = planVertexBillboardPlacement(Cesium, pt);
            ents.vertices[i].position = placement.position;
            ents.vertices[i].billboard.heightReference = placement.heightReference;
          }
        });
        for (let i = 0; i < poly.length; i++) {
          const next = poly[(i + 1) % poly.length];
          if (ents.midpoints[i]) {
            ents.midpoints[i].position = Cesium.Cartesian3.fromDegrees(
              (poly[i].lon + next.lon) / 2, (poly[i].lat + next.lat) / 2
            );
          }
        }
      }
      // Polygon fill + outline use CallbackProperty — auto-update from ref
      return;
    }

    // Full recreation: vertex count changed, phase changed, or first render
    if (ents.polygon) { try { viewer.entities.remove(ents.polygon); } catch {} }
    if (ents.polygonOutline) { removeGroundPolyline(viewer, ents.polygonOutline); }
    ents.vertices.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.midpoints.forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.polygon = null;
    ents.polygonOutline = null;
    ents.vertices = [];
    ents.midpoints = [];

    if (poly.length < 1) return;

    // Vertices — only in PLANNING phase
    if (editable) {
      poly.forEach((pt, i) => {
        const placement = planVertexBillboardPlacement(Cesium, pt);
        const e = viewer.entities.add({
          position: placement.position,
          billboard: {
            image: VERTEX_ICON,
            width: 16,
            height: 16,
            heightReference: placement.heightReference,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { vertexIndex: i },
        });
        ents.vertices.push(e);
      });
    }

    if (poly.length < 3) return;

    // Filled polygon — CallbackProperty reads from ref for smooth drag
    ents.polygon = viewer.entities.add({
      polygon: {
        hierarchy: new Cesium.CallbackProperty(() => {
          const p = polygonRef.current;
          if (p.length < 3) return new Cesium.PolygonHierarchy([]);
          return new Cesium.PolygonHierarchy(
            p.map((pt) => Cesium.Cartesian3.fromDegrees(pt.lon, pt.lat))
          );
        }, false),
        material: Cesium.Color.fromCssColorString(colors.accent).withAlpha(0.15),
        classificationType: Cesium.ClassificationType.BOTH,
      },
      properties: { isPolygonFill: true },
    });

    // Polygon outline — reads from ref for smooth drag. Ground-following via
    // addGroundPolyline (not raw clampToGround) so the outline still renders
    // on browsers/GPUs without WebGL depth-texture support.
    ents.polygonOutline = addGroundPolyline(viewer, Cesium, {
      getPositions: () => (polygonRef.current.length >= 3 ? polygonRef.current : []),
      width: 2,
      material: Cesium.Color.fromCssColorString(colors.accent),
      loop: true,
    });

    // Midpoint handles — only in PLANNING phase
    if (editable) {
      for (let i = 0; i < poly.length; i++) {
        const next = poly[(i + 1) % poly.length];
        const mid = {
          lat: (poly[i].lat + next.lat) / 2,
          lon: (poly[i].lon + next.lon) / 2,
        };
        const e = viewer.entities.add({
          position: Cesium.Cartesian3.fromDegrees(mid.lon, mid.lat),
          billboard: {
            image: MIDPOINT_ICON,
            width: 10,
            height: 10,
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { midpointIndex: i },
        });
        ents.midpoints.push(e);
      }
    }
    // viewerReady gates re-run: on a fresh page load the auto-downloaded
    // polygon can be set before the async Cesium viewer finishes init, so the
    // first run bails on `!viewer`. Without viewerReady in the deps the effect
    // would never re-run (polygon's reference is stable), leaving the outline
    // uncreated until something else changed it (e.g. a manual Download Plan).
  }, [polygon, phase, viewerReady]);
}
