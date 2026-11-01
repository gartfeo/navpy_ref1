/**
 * The one clock of the task-confirm countdown: receipt time, the card, the
 * summary chip, the order and the alert sound all read it (todos 67, 68).
 * deterministic_dev_sim.md plans to move the countdown to sim time; that
 * swap happens here.
 */
export function confirmNow() {
  return Date.now();
}
