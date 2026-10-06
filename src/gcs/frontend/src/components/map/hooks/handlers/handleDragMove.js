import { pickCartographic, clampToRing } from '../../utils/entityPicking';

/**
 * MOUSE_MOVE: dispatch drag to appropriate handler.
 * Mutates dragState.pendingLatLon for corridor drags.
 */
export function handleDragMove(Cesium, viewer, movement, dragState, callbacksRef, refs) {
  if (!dragState) return;
  const cb = callbacksRef.current;
  const latlon = pickCartographic(Cesium, viewer, movement.endPosition);
  if (!latlon) return;

  if (dragState.type === 'setCorridorPoint') {
    const clamped = clampToRing(latlon, refs.clampOuterRef);
    if (dragState.corridorIndex === -1) {
      const lps = refs.setLaunchPointsRef.current;
      if (lps) lps[dragState.setIdx] = { lat: clamped.lat, lon: clamped.lon };
    } else {
      const setCps = refs.setCorridorPointsRef.current[dragState.setIdx];
      if (setCps) setCps[dragState.corridorIndex] = { lat: clamped.lat, lon: clamped.lon };
    }
    dragState.pendingLatLon = clamped;
    // Only commit state for the last point during drag (determines approach direction).
    // Non-last points update only the ref (smooth CallbackProperty feedback);
    // state is committed in handleDragEnd to avoid regen-induced jumping.
    if (dragState.isLastPoint && cb.onSetCorridorPointDrag) {
      cb.onSetCorridorPointDrag(dragState.setIdx, dragState.corridorIndex, clamped);
    }
  } else if (dragState.type === 'partitionHandle' && cb.onPartitionAngleDrag) {
    const p = refs.polygonRef.current;
    if (p.length >= 3) {
      const cLat = p.reduce((s, pt) => s + pt.lat, 0) / p.length;
      const cLon = p.reduce((s, pt) => s + pt.lon, 0) / p.length;
      const dx = (latlon.lon - cLon) * Math.cos(cLat * Math.PI / 180);
      const dy = latlon.lat - cLat;
      const deg = Math.atan2(dy, dx) * 180 / Math.PI;
      cb.onPartitionAngleDrag(deg);
    }
  } else if (dragState.type === 'fallbackLocation' && dragState.entity && cb.placingFallbackLocation) {
    // Pick terrain cartesian directly so icon sits on terrain surface
    const ray = viewer.camera.getPickRay(movement.endPosition);
    const cartesian = ray && viewer.scene.globe.pick(ray, viewer.scene);
    if (Cesium.defined(cartesian)) {
      dragState.entity.position = cartesian;
    }
    dragState.lastLatLon = latlon;
  } else if (dragState.type === 'vertex' && cb.onVertexDrag) {
    cb.onVertexDrag(dragState.index, latlon);
  } else if (dragState.type === 'fenceVertex' && cb.onFenceVertexDrag) {
    cb.onFenceVertexDrag(dragState.index, latlon);
  } else if (dragState.type === 'exclusionVertex' && cb.onExclusionVertexDrag) {
    cb.onExclusionVertexDrag(dragState.ring, dragState.index, latlon);
  } else if (dragState.type === 'polygon' && cb.onPolygonMove) {
    const dlat = latlon.lat - dragState.startLatLon.lat;
    const dlon = latlon.lon - dragState.startLatLon.lon;
    cb.onPolygonMove(dlat, dlon);
    dragState.startLatLon = latlon;
  }
}
