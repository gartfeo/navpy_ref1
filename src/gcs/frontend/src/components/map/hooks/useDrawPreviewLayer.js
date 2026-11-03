import { useEffect } from 'react';
import { colors } from '../../../styles';
import { addGroundPolyline, removeGroundPolyline } from '../utils/groundPolyline';

/**
 * Rubber-band outline of the rectangle being drawn. Reads previewRef every
 * frame (handleDrawHover fills it), so the pointer moves it without React
 * renders. One entity for the viewer's life; an empty ref draws nothing.
 * The ref is cleared when drawing stops or the draft changes, so a finished
 * or dropped draft leaves no stale outline.
 */
export default function useDrawPreviewLayer(cesiumRef, viewerRef, previewRef, isDrawing, polygon, viewerReady) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return undefined;
    const entity = addGroundPolyline(viewer, Cesium, {
      getPositions: () => previewRef.current,
      width: 2,
      material: new Cesium.PolylineDashMaterialProperty({
        color: Cesium.Color.fromCssColorString(colors.accent),
        dashLength: 12,
      }),
      loop: true,
    });
    return () => removeGroundPolyline(viewer, entity);
  }, [viewerReady]);

  useEffect(() => {
    previewRef.current = [];
    viewerRef.current?.scene?.requestRender();
  }, [isDrawing, polygon]);
}
