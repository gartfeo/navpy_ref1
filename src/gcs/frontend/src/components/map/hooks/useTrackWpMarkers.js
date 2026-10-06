import { useEffect } from 'react';
import { zoneColorsSolid } from '../../../styles';
import { makeTrackWpIcon } from '../constants/icons';

const DOT_SIZE = 16;
const DOT_SIZE_SELECTED = 20;

/**
 * Renders clickable circle billboards at every track waypoint when
 * simMode + PLANNING phase + plan has zones.
 *
 * Selected waypoints (present in simDockWps) appear larger and fully opaque;
 * unselected ones are semi-transparent.
 *
 * Each entity has PropertyBag with { isTrackWp, zoneIndex, wpIndex } for picking.
 */
export default function useTrackWpMarkers(
  cesiumRef, viewerRef, entitiesRef, trackPosRef,
  plan, simMode, phase, simDockWps, viewerReady,
) {
  const zoneCount = plan?.zones?.length || 0;
  const active = simMode && phase === 'PLANNING' && zoneCount > 0;
  const selectionKey = JSON.stringify(simDockWps || {});
  const trackLengths = plan?.zones?.map(z => z.track?.length || 0).join(',') || '';

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    const existing = ents.trackWpMarkers || [];
    for (const e of existing) {
      try { viewer.entities.remove(e); } catch {}
    }

    if (!active) {
      ents.trackWpMarkers = [];
      return;
    }

    const selectedSet = new Set();
    if (simDockWps) {
      for (const [zi, wps] of Object.entries(simDockWps)) {
        for (const wi of (wps || [])) selectedSet.add(`${zi}_${wi}`);
      }
    }

    const added = [];
    for (let zi = 0; zi < zoneCount; zi++) {
      const colorHex = zoneColorsSolid[zi % zoneColorsSolid.length];

      const wpCount = trackPosRef.current[zi]?.length || 0;
      for (let wi = 0; wi < wpCount; wi++) {
        const capturedZi = zi;
        const capturedWi = wi;
        const selected = selectedSet.has(`${zi}_${wi}`);
        const size = selected ? DOT_SIZE_SELECTED : DOT_SIZE;
        const icon = makeTrackWpIcon(colorHex, size, selected);

        const e = viewer.entities.add({
          position: new Cesium.CallbackProperty(() => {
            const p = trackPosRef.current[capturedZi];
            return p?.[capturedWi] || Cesium.Cartesian3.ZERO;
          }, false),
          billboard: {
            image: icon,
            width: size,
            height: size,
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.4),
            heightReference: Cesium.HeightReference.NONE,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          properties: { isTrackWp: true, zoneIndex: zi, wpIndex: wi },
        });
        added.push(e);
      }
    }

    ents.trackWpMarkers = added;
  }, [active, zoneCount, selectionKey, trackLengths, viewerReady]);
}
