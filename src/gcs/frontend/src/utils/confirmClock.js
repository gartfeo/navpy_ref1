import { backendNowMs, onBackendTime } from './backendClock';

/**
 * The one clock of the task-confirm countdown: receipt time, the card, the
 * summary chip, the order and the alert sound all read it (todos 67, 68).
 * It is the backend's clock (utils/backendClock.js), so on a lockstep run
 * the countdown counts sim time and its auto-fire lands at the sim time it
 * shows.
 */
export function confirmNow() {
  return backendNowMs();
}

/** Each advance of the confirm clock (the countdown's auto-fire check). */
export const onConfirmTime = onBackendTime;
