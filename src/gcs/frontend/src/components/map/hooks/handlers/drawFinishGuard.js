/**
 * The width click finishes a rectangle and stops drawing. If that click was
 * the first of a double-click, the LEFT_DOUBLE_CLICK that follows would land
 * outside drawing and delete the corner under the pointer. This guard drops
 * exactly that double-click: the one whose first click finished a rectangle.
 * Cesium fires LEFT_CLICK for both clicks of a double-click, then
 * LEFT_DOUBLE_CLICK, so "one click since the finishing click" identifies it.
 */
export function createDrawFinishGuard() {
  let clicksSinceFinish = Infinity;
  return {
    /** Call with the callbacks as they were when the LEFT_CLICK arrived. */
    onClick(cb) {
      const finishes = cb.editable && cb.isDrawing && cb.drawShape === 'rectangle'
        && !!cb.polygon && cb.polygon.length === 2;
      clicksSinceFinish = finishes ? 0 : clicksSinceFinish + 1;
    },
    /** True when this LEFT_DOUBLE_CLICK belongs to the finishing click. */
    skipDoubleClick() {
      const skip = clicksSinceFinish === 1;
      if (skip) clicksSinceFinish = Infinity;
      return skip;
    },
  };
}
