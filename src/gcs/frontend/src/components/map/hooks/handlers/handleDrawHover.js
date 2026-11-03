import { pickCartographic } from '../../utils/entityPicking';
import { rectanglePreview } from '../../../../utils/rectangleDraw';

/**
 * MOUSE_MOVE with no drag: while a rectangle zone is drafted, put the ring
 * the next click would make in previewRef (read by useDrawPreviewLayer).
 */
export function handleDrawHover(Cesium, viewer, movement, callbacksRef, previewRef) {
  const cb = callbacksRef.current;
  const drafting = cb.editable && cb.isDrawing && cb.drawShape === 'rectangle'
    && cb.polygon && cb.polygon.length > 0 && cb.polygon.length < 3;
  const hadPreview = previewRef.current.length > 0;
  if (!drafting) {
    if (hadPreview) {
      previewRef.current = [];
      viewer.scene.requestRender();
    }
    return;
  }
  const latlon = pickCartographic(Cesium, viewer, movement.endPosition);
  if (!latlon) return;
  previewRef.current = rectanglePreview(cb.polygon, latlon);
  viewer.scene.requestRender();
}
