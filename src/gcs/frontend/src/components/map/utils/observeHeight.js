/**
 * Report an element's height now and whenever it changes (ResizeObserver).
 * Used for Cesium credit lines, which the vehicle window and the swapped map
 * must sit above. Returns the disconnect function; a no-op without an element
 * or a layout engine.
 */
export function observeHeight(el, onHeight) {
  if (!el || typeof ResizeObserver === 'undefined') return () => {};
  let last = -1;
  const measure = () => {
    const h = el.offsetHeight;
    if (h !== last) {
      last = h;
      onHeight(h);
    }
  };
  measure();
  const observer = new ResizeObserver(measure);
  observer.observe(el);
  return () => observer.disconnect();
}
