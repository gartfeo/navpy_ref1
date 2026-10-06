import { useEffect, useRef } from 'react';
import { VERTEX_ICON, MIDPOINT_ICON } from '../constants/icons';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';
import { planVertexBillboardPlacement } from '../utils/planVertexPlacement';

// Amber dashed boundary — distinct from the red search zone and green launch zone.
const FENCE_COLOR = '#ff9800';
// Hazard red while an operator-edited ring is unsafe (coverage gap or
// self-intersection) — the gesture happens on the map, so the cue must too.
const FENCE_WARN_COLOR = '#ff1744';

/**
 * Geofence layer — the inclusion polygon drawn as a dashed amber ground
 * outline with a faint fill, plus (in PLANNING) vertex/midpoint handles so the
 * operator can reshape it exactly like the search zone. Fill/outline positions
 * are read live from a ref so slider- or drag-driven changes update without
 * re-adding entities; handles move in place while the vertex count is
 * unchanged (same fast path as usePolygonLayer) and are recreated on count
 * change.
 */
export default function useFenceLayer(cesiumRef, viewerRef, entitiesRef, fencePolygon, showFence, viewerReady, editable, warning) {
  const posRef = useRef([]);

  const visible = !!showFence && (fencePolygon?.length ?? 0) >= 3;
  const handlesOn = visible && !!editable;
  const count = visible ? fencePolygon.length : 0;
  const warn = !!warning;

  // Keep Cartesian positions fresh for the CallbackProperty reads, and move
  // existing handles in place. The viewer runs with requestRenderMode, so
  // mutating posRef alone won't repaint — request a render so a slider- or
  // drag-driven change is visible immediately (same pattern as useUavMarkers).
  useEffect(() => {
    const Cesium = cesiumRef.current;
    if (!Cesium) { posRef.current = []; return; }
    const poly = fencePolygon || [];
    posRef.current = poly.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));

    // Fast path: same vertex count — move handles without recreating them.
    const ents = entitiesRef.current;
    if ((ents.fenceVertices?.length || 0) === poly.length && poly.length > 0) {
      poly.forEach((pt, i) => {
        const v = ents.fenceVertices[i];
        if (v) {
          const placement = planVertexBillboardPlacement(Cesium, pt);
          v.position = placement.position;
          v.billboard.heightReference = placement.heightReference;
        }
        const next = poly[(i + 1) % poly.length];
        const m = ents.fenceMidpoints?.[i];
        if (m) {
          m.position = Cesium.Cartesian3.fromDegrees(
            (pt.lon + next.lon) / 2, (pt.lat + next.lat) / 2,
          );
        }
      });
    }
    viewerRef.current?.scene?.requestRender();
  }, [fencePolygon, viewerReady]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    (ents.fence || []).forEach((e) => removeGroundPolyline(viewer, e));
    ents.fence = [];
    (ents.fenceVertices || []).forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    (ents.fenceMidpoints || []).forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.fenceVertices = [];
    ents.fenceMidpoints = [];

    if (!visible) return;

    const color = Cesium.Color.fromCssColorString(warn ? FENCE_WARN_COLOR : FENCE_COLOR);

    // Faint fill so the enclosed area reads as "inside the fence".
    const fillEntity = viewer.entities.add({
      polygon: {
        hierarchy: new Cesium.CallbackProperty(() => {
          const pos = posRef.current;
          if (!pos || pos.length < 3) return new Cesium.PolygonHierarchy([]);
          return new Cesium.PolygonHierarchy(pos);
        }, false),
        material: color.withAlpha(0.06),
        classificationType: Cesium.ClassificationType.TERRAIN,
      },
    });
    ents.fence.push(fillEntity);

    // Dashed boundary — ground-following (see useLaunchZoneLayer / the
    // cross-browser clamp note for why addGroundPolyline over raw clampToGround).
    const lineEntity = addGroundPolyline(viewer, Cesium, {
      getPositions: () => (posRef.current && posRef.current.length >= 3 ? posRef.current : []),
      width: 2.5,
      material: new Cesium.PolylineDashMaterialProperty({ color, dashLength: 16 }),
      loop: true,
    });
    ents.fence.push(lineEntity);

    // Editable handles (PLANNING only) — amber-tinted so they read as fence
    // controls, not search-zone controls.
    if (handlesOn) {
      const poly = fencePolygon || [];
      poly.forEach((pt, i) => {
        const placement = planVertexBillboardPlacement(Cesium, pt);
        ents.fenceVertices.push(viewer.entities.add({
          position: placement.position,
          billboard: {
            image: VERTEX_ICON,
            width: 14,
            height: 14,
            color,
            heightReference: placement.heightReference,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { fenceVertexIndex: i },
        }));
      });
      for (let i = 0; i < poly.length; i++) {
        const next = poly[(i + 1) % poly.length];
        ents.fenceMidpoints.push(viewer.entities.add({
          position: Cesium.Cartesian3.fromDegrees(
            (poly[i].lon + next.lon) / 2, (poly[i].lat + next.lat) / 2,
          ),
          billboard: {
            image: MIDPOINT_ICON,
            width: 9,
            height: 9,
            color,
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { fenceMidpointIndex: i },
        }));
      }
    }
    // viewerReady gates re-run so entities created before the async viewer
    // finished init aren't lost — same fresh-load race as usePolygonLayer.
    // `count` recreates handles when a vertex is inserted or deleted; `warn`
    // recreates for the amber↔red hazard switch.
  }, [visible, handlesOn, count, warn, viewerReady]);
}
