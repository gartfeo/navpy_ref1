/**
 * Pure state-machine logic for fade-out transitions.
 * Given the desired visibility and whether the component is currently mounted,
 * returns the next { mounted, fading } state.
 */
export function computeFadeState(visible, prevMounted) {
  if (visible) return { mounted: true, fading: false };
  if (prevMounted) return { mounted: true, fading: true };
  return { mounted: false, fading: false };
}
