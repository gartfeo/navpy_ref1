/**
 * Shared requestAnimationFrame scheduler.
 * Consolidates multiple independent RAF loops into a single coordinated loop.
 */
const callbacks = new Map();
let rafId = null;

function loop() {
  for (const fn of callbacks.values()) fn();
  if (callbacks.size > 0) {
    rafId = requestAnimationFrame(loop);
  } else {
    rafId = null;
  }
}

export function scheduleRaf(id, drawFn) {
  callbacks.set(id, drawFn);
  if (rafId === null) {
    rafId = requestAnimationFrame(loop);
  }
}

export function cancelRaf(id) {
  callbacks.delete(id);
  if (callbacks.size === 0 && rafId !== null) {
    cancelAnimationFrame(rafId);
    rafId = null;
  }
}
