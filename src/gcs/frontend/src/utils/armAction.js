export const LONG_PRESS_MS = 1000;
export const AIRBORNE_ALT_THRESHOLD = 3;

/**
 * Which keyboard keys arm a press-and-hold. Only Enter and Space, matching
 * native button activation keys — so a keyboard hold is deliberate and no
 * other key can start (or, via the button's suppressed click, shortcut) it.
 * @param {string} key - KeyboardEvent.key
 * @returns {boolean}
 */
export function isHoldKey(key) {
  return key === 'Enter' || key === ' ';
}

/**
 * Determine arm/disarm action requirements.
 * @param {{ armed: boolean, prearmSeverity: string, altRel: number|null|undefined }} params
 * @returns {{ requiresLongPress: boolean, hint: string|null, force: boolean }}
 */
export function getArmAction({ armed, prearmSeverity, altRel }) {
  if (armed) {
    const airborne = altRel != null && altRel >= AIRBORNE_ALT_THRESHOLD;
    if (airborne) {
      return { requiresLongPress: true, hint: 'armAction.holdToForceDisarm', force: true };
    }
    return { requiresLongPress: false, hint: null, force: false };
  }

  if (prearmSeverity === 'warn') {
    return { requiresLongPress: true, hint: 'armAction.holdToArm', force: true };
  }
  if (prearmSeverity === 'fail') {
    return { requiresLongPress: true, hint: 'armAction.holdToForceArm', force: true };
  }
  return { requiresLongPress: false, hint: null, force: false };
}
