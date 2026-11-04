/**
 * The backend's clock as the app last heard it: the `ts` (seconds) of the
 * latest telemetry message. On hardware that is wall time; on a lockstep run
 * it is sim time (deterministic_dev_sim.md section 5: no app-side clock
 * decides anything sent to the run). It advances with each telemetry message,
 * not between them. Before the first one it is the page's own clock.
 */
let lastBackendMs = null;
const listeners = new Set();

export function noteBackendTs(tsSeconds) {
  if (typeof tsSeconds !== 'number' || !Number.isFinite(tsSeconds)) return;
  lastBackendMs = tsSeconds * 1000;
  for (const listener of [...listeners]) listener(lastBackendMs);
}

/**
 * Call `listener` on each new backend time, as it arrives: a deadline on this
 * clock is seen the moment the clock passes it, never on a wall timer's next
 * turn. Returns the unsubscribe.
 */
export function onBackendTime(listener) {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

/** Epoch ms on the backend's clock. */
export function backendNowMs() {
  return lastBackendMs ?? Date.now();
}

/** Tests: forget the backend's clock. */
export function resetBackendClock() {
  lastBackendMs = null;
  listeners.clear();
}
