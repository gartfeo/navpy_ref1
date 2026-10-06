import { pickEntity } from '../../utils/entityPicking';

/**
 * LEFT_DOUBLE_CLICK: finish drawing / delete vertex or corridor point.
 */
export function handleDoubleClick(Cesium, viewer, click, callbacksRef, activeSetIndex) {
  const cb = callbacksRef.current;
  if (!cb.editable) return;

  // Fence/keep-out vertex delete works in EVERY mode (their handles stay
  // grabbable in placement modes too — see handleDragStart), so check the pick
  // before the keep-out branch or the gesture would silently end ring shaping.
  const early = pickEntity(Cesium, viewer, click.position);
  if (early && early.type === 'fenceVertex' && cb.onFenceVertexDelete) {
    cb.onFenceVertexDelete(early.index);
    return;
  }
  if (early && early.type === 'exclusionVertex' && cb.onExclusionVertexDelete) {
    cb.onExclusionVertexDelete(early.ring, early.index);
    return;
  }

  // Keep-out (exclusion) drawing — the ring already closed at 3 points (same
  // logic as the zone); double-click just stops shaping the current ring and
  // stays in the mode so the next click starts another keep-out.
  if (cb.placingExclusion && cb.onFinishExclusion) {
    cb.onFinishExclusion();
    return;
  }

  if (cb.isDrawing) {
    if (cb.polygon && cb.polygon.length >= 3 && cb.onFinishDraw) {
      cb.onFinishDraw();
    }
    return;
  }

  // Not drawing — check if double-clicked a vertex, corridor point, or launch point to delete
  const hit = pickEntity(Cesium, viewer, click.position);
  if (hit && hit.type === 'fallbackLocation' && cb.placingFallbackLocation && cb.onRemoveFallbackLocation) {
    cb.onRemoveFallbackLocation(hit.index);
  } else if (hit && hit.type === 'vertex' && cb.onVertexDelete) {
    cb.onVertexDelete(hit.index);
  } else if (hit && hit.type === 'setCorridorPoint') {
    // Switch to the target set so convenience setters act on the right set
    if (hit.setIdx !== (activeSetIndex ?? 0) && cb.onSwitchSet) cb.onSwitchSet(hit.setIdx);
    if (hit.corridorIndex === -1 && cb.onRemoveLaunchPoint) {
      cb.onRemoveLaunchPoint(hit.setIdx);
    } else if (hit.corridorIndex >= 0 && cb.onCorridorPointDelete) {
      cb.onCorridorPointDelete(hit.corridorIndex, hit.setIdx);
    }
  }
}
