/**
 * The width click finishes a rectangle and stops drawing. If that click was
 * the first of a double-click, the LEFT_DOUBLE_CLICK that follows would land
 * outside drawing and delete the corner under the pointer. This guard drops
 * exactly that double-click: the one whose first click finished a rectangle.
 * Cesium fires LEFT_CLICK for both clicks of a double-click, then
 * LEFT_DOUBLE_CLICK, so "one click since the finishing click" identifies it.
 * The second click may land on the new corner and start a vertex drag; it
 * still counts.
 */
export function createDrawFinishGuard() {
  let clicksSinceFinish = Infinity;
  return {
    /**
     * Call on every LEFT_CLICK, with the callbacks as they were when it
     * arrived. handled=false: the click ended a drag and never reached
     * handleClick, so it cannot have finished a rectangle, but it still is
     * one click of the sequence.
     */
    onClick(cb, { handled = true } = {}) {
      const finishes = handled && cb.editable && cb.isDrawing && cb.drawShape === 'rectangle'
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
