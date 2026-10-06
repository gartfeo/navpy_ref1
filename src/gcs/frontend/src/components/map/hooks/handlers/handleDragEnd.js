/**
 * LEFT_UP: end drag. Returns true if a drag was active.
 */
export function handleDragEnd(viewer, dragState, callbacksRef) {
  if (!dragState) return false;
  viewer.scene.screenSpaceCameraController.enableRotate = true;
  const cb = callbacksRef.current;

  if (dragState.type === 'fallbackLocation') {
    if (dragState.lastLatLon && cb.onMoveFallbackLocation && cb.placingFallbackLocation) {
      cb.onMoveFallbackLocation(dragState.index, dragState.lastLatLon);
    }
  } else if (dragState.type === 'fenceVertex' || dragState.type === 'exclusionVertex') {
    // Fence/keep-out reshaping never affects the plan — skip the regen in onDragEnd.
  } else if (dragState.type === 'setCorridorPoint') {
    if (dragState.isLastPoint) {
      // Last point: full regen (approach direction changed)
      if (cb.onDragEnd) cb.onDragEnd();
    } else if (dragState.pendingLatLon) {
      // Non-last: commit position
      // Corridor search pattern: don't suppress regen (all points shape the track)
      if (cb.searchPattern !== 'corridor' && cb.onSuppressRegen) cb.onSuppressRegen();
      if (cb.onSetCorridorPointDrag) {
        cb.onSetCorridorPointDrag(dragState.setIdx, dragState.corridorIndex, dragState.pendingLatLon);
      }
      const skipRegen = cb.searchPattern !== 'corridor';
      if (cb.onDragEnd) cb.onDragEnd(skipRegen);
    }
  } else {
    if (cb.onDragEnd) cb.onDragEnd();
  }

  return true;
}
