import { useEffect, useRef } from 'react';
import { VERTEX_ICON, MIDPOINT_ICON } from '../constants/icons';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';
import { planVertexBillboardPlacement } from '../utils/planVertexPlacement';

// No-entry red — deliberately distinct from the amber inclusion fence
// (#ff9800, dashed) and the search zone. Keep-outs get a striped "hazard"
// fill (where terrain-materials are supported) plus a solid bold outline so
// they always read as forbidden areas.
const KEEPOUT_COLOR = '#ff3b30';
const KEEPOUT_CONFLICT_COLOR = '#ff1744';

/**
 * Keep-out (exclusion) layer — renders every finished exclusion ring plus the
 * in-progress draft ring, and (in PLANNING) zone-style vertex/midpoint handles
 * so the operator can reshape any ring like the search polygon.
 *
 * Ring positions are read live from a ref via CallbackProperty so vertex drags
 * update without recreating fills/outlines; handles move in place while the
 * ring structure (ring count + per-ring vertex counts) is unchanged and are
 * recreated on structural change (insert/delete/new ring) — the same fast-path
 * split as usePolygonLayer / useFenceLayer. Rings the planned flight
 * intersects (`exclusionConflicts` by exclusionIndex) are emphasized.
 */
export default function useExclusionLayer(
  cesiumRef, viewerRef, entitiesRef,
  exclusionPolygons, draftExclusion, exclusionConflicts, viewerReady, editable,
) {
  const conflictIdx = new Set((exclusionConflicts || []).map((c) => c.exclusionIndex));

  // Live positions for CallbackProperty reads + in-place handle moves.
  const ringsRef = useRef([]);
  ringsRef.current = exclusionPolygons || [];
  const posRef = useRef([]); // per ring: Cartesian3[]

  // Structural signatures: recreate entities only when the SHAPE of the data
  // changes (ring/vertex counts, conflicts, draft, editability) — plain vertex
  // drags keep the same structure and take the in-place path.
  const structSig = (exclusionPolygons || []).map((r) => (r || []).length).join(',');
  const draftSig = JSON.stringify(draftExclusion || []);
  const conflictSig = [...conflictIdx].sort((a, b) => a - b).join(',');
  const handlesOn = !!editable;

  // Position refresh + in-place handle moves (viewer runs requestRenderMode).
  useEffect(() => {
    const Cesium = cesiumRef.current;
    if (!Cesium) { posRef.current = []; return; }
    const rings = ringsRef.current;
    posRef.current = rings.map((ring) =>
      (ring || []).map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat)));

    const ents = entitiesRef.current;
    rings.forEach((ring, ri) => {
      const verts = ents.exclusionVertices?.[ri];
      const mids = ents.exclusionMidpoints?.[ri];
      if (!verts || verts.length !== (ring || []).length) return;
      ring.forEach((pt, i) => {
        const v = verts[i];
        if (v) {
          const placement = planVertexBillboardPlacement(Cesium, pt);
          v.position = placement.position;
          v.billboard.heightReference = placement.heightReference;
        }
        const next = ring[(i + 1) % ring.length];
        const m = mids?.[i];
        if (m) {
          m.position = Cesium.Cartesian3.fromDegrees(
            (pt.lon + next.lon) / 2, (pt.lat + next.lat) / 2,
          );
        }
      });
    });
    viewerRef.current?.scene?.requestRender();
  }, [exclusionPolygons, viewerReady]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    (ents.exclusions || []).forEach((e) => removeGroundPolyline(viewer, e));
    ents.exclusions = [];
    (ents.exclusionVertices || []).flat().forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    (ents.exclusionMidpoints || []).flat().forEach((e) => { try { viewer.entities.remove(e); } catch {} });
    ents.exclusionVertices = [];
    ents.exclusionMidpoints = [];

    const stripesSupported = Cesium.GroundPrimitive.supportsMaterials(viewer.scene);
    const rings = ringsRef.current;

    rings.forEach((ring, ri) => {
      if (!Array.isArray(ring) || ring.length < 3) {
        ents.exclusionVertices.push([]);
        ents.exclusionMidpoints.push([]);
        return;
      }
      const conflict = conflictIdx.has(ri);
      const color = Cesium.Color.fromCssColorString(conflict ? KEEPOUT_CONFLICT_COLOR : KEEPOUT_COLOR);

      // Filled hazard area — hierarchy reads live positions so drags track.
      const material = stripesSupported
        ? new Cesium.StripeMaterialProperty({
            evenColor: color.withAlpha(conflict ? 0.55 : 0.4),
            oddColor: Cesium.Color.TRANSPARENT,
            repeat: 28,
            orientation: Cesium.StripeOrientation.HORIZONTAL,
          })
        : color.withAlpha(conflict ? 0.28 : 0.18);
      const fill = viewer.entities.add({
        polygon: {
          hierarchy: new Cesium.CallbackProperty(() => {
            const pos = posRef.current[ri];
            if (!pos || pos.length < 3) return new Cesium.PolygonHierarchy([]);
            return new Cesium.PolygonHierarchy(pos);
          }, false),
          material,
          classificationType: Cesium.ClassificationType.TERRAIN,
        },
      });
      ents.exclusions.push(fill);

      // Outline — solid + bold; reads live lat/lon so drags track.
      const line = addGroundPolyline(viewer, Cesium, {
        getPositions: () => ringsRef.current[ri] || [],
        width: conflict ? 3.5 : 2.5,
        material: color,
        loop: true,
      });
      ents.exclusions.push(line);

      // Zone-style edit handles (PLANNING only) — red-tinted so they read as
      // keep-out controls, not zone/fence controls.
      const verts = [];
      const mids = [];
      if (handlesOn) {
        ring.forEach((pt, i) => {
          const placement = planVertexBillboardPlacement(Cesium, pt);
          verts.push(viewer.entities.add({
            position: placement.position,
            billboard: {
              image: VERTEX_ICON,
              width: 14,
              height: 14,
              color,
              heightReference: placement.heightReference,
              disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
            properties: { exclusionVertexRing: ri, exclusionVertexIndex: i },
          }));
        });
        for (let i = 0; i < ring.length; i++) {
          const next = ring[(i + 1) % ring.length];
          mids.push(viewer.entities.add({
            position: Cesium.Cartesian3.fromDegrees(
              (ring[i].lon + next.lon) / 2, (ring[i].lat + next.lat) / 2,
            ),
            billboard: {
              image: MIDPOINT_ICON,
              width: 9,
              height: 9,
              color,
              heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
              disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
            properties: { exclusionMidRing: ri, exclusionMidIndex: i },
          }));
        }
      }
      ents.exclusionVertices.push(verts);
      ents.exclusionMidpoints.push(mids);
    });

    // In-progress draft — vertex dots so the first clicks are visible, plus an
    // open dashed outline once there are at least two vertices. (The ring
    // closes at 3 points, so the draft never exceeds 2.)
    const draft = draftExclusion || [];
    draft.forEach((p) => {
      const dot = viewer.entities.add({
        position: Cesium.Cartesian3.fromDegrees(p.lon, p.lat),
        point: {
          pixelSize: 8,
          color: Cesium.Color.fromCssColorString(KEEPOUT_COLOR),
          outlineColor: Cesium.Color.WHITE,
          outlineWidth: 1,
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      });
      ents.exclusions.push(dot);
    });
    if (draft.length >= 2) {
      const color = Cesium.Color.fromCssColorString(KEEPOUT_COLOR);
      const line = addGroundPolyline(viewer, Cesium, {
        getPositions: () => draft,
        width: 2.5,
        material: new Cesium.PolylineDashMaterialProperty({ color, dashLength: 12 }),
        loop: false,
      });
      ents.exclusions.push(line);
    }

    viewer.scene.requestRender();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [structSig, draftSig, conflictSig, handlesOn, viewerReady]);
}
