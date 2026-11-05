/**
 * Resolve how the launch START button should behave for a given state.
 *
 * A launch START (container HOLD TO LAUNCH, bungee START MISSION) arms or
 * launches the connected UAVs, so it must never start on a single stray click.
 * When `armReady` is set, the ready path is gated behind a press-and-hold
 * ("arm"); on completion the caller shows a confirm modal before the start
 * actually runs. Both GCS start buttons set it; `armReady` false is a plain
 * single-click start. The not-ready path is always a hold-to-force regardless
 * of `armReady` (unchanged existing flow).
 *
 * @param {{armReady?: boolean, ready: boolean, disabled?: boolean}} params
 * @returns {{clickToStart: boolean, holdEnabled: boolean, holdMode: ('arm'|'force'|null)}}
 *   - clickToStart: a single click starts the mission immediately (ready, `armReady` false)
 *   - holdEnabled: a press-and-hold is active
 *   - holdMode: what completing the hold does — 'arm' requests confirmation,
 *     'force' force-launches after an explicit confirm, null does nothing
 */
export function resolveStartButtonMode({ armReady = false, ready, disabled = false }) {
  if (disabled) return { clickToStart: false, holdEnabled: false, holdMode: null };
  if (ready) {
    return armReady
      ? { clickToStart: false, holdEnabled: true, holdMode: 'arm' }
      : { clickToStart: true, holdEnabled: false, holdMode: null };
  }
  // Not ready: force path — hold, then explicit confirm.
  return { clickToStart: false, holdEnabled: true, holdMode: 'force' };
}
