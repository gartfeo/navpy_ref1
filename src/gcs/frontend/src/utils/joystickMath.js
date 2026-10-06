/** Pure math utilities for virtual joystick → MAVLink MANUAL_CONTROL. */

/**
 * Clamp (x, y) to the unit square [-1, 1] × [-1, 1].
 * Each axis is clamped independently so corners are reachable.
 * @param {number} x  -1..1
 * @param {number} y  -1..1
 * @returns {{x: number, y: number}}
 */
export function clampToSquare(x, y) {
  return {
    x: Math.max(-1, Math.min(1, x)),
    y: Math.max(-1, Math.min(1, y)),
  };
}

/**
 * Map a symmetric axis value [-1, 1] → MAVLink range [-1000, 1000].
 * @param {number} value  -1..1
 * @returns {number} integer in [-1000, 1000]
 */
export function axisToMavlink(value) {
  const clamped = Math.max(-1, Math.min(1, value));
  return Math.round(clamped * 1000);
}

/**
 * Map throttle [-1, 1] → MAVLink range [0, 1000].
 * Center (0) maps to 500 (hover / mid-throttle).
 * @param {number} value  -1..1
 * @returns {number} integer in [0, 1000]
 */
export function throttleToMavlink(value) {
  const clamped = Math.max(-1, Math.min(1, value));
  return Math.round((clamped + 1) * 500);
}

/**
 * Convert RC PWM value (1000-2000) to joystick stick Y position [-1, 1].
 * 1000 → -1, 1500 → 0, 2000 → 1.
 * @param {number|null|undefined} pwm  RC channel PWM value
 * @returns {number} stick Y in [-1, 1]
 */
export function rcPwmToStickY(pwm) {
  return Math.max(-1, Math.min(1, ((pwm ?? 1000) - 1000) / 500 - 1));
}

/**
 * Build a MANUAL_CONTROL payload from left/right joystick positions.
 *
 * Left stick:  throttle (y → z), yaw (x → r)
 * Right stick: pitch (y → x), roll (x → y)
 *
 * @param {{x: number, y: number}} left   left joystick position, each -1..1
 * @param {{x: number, y: number}} right  right joystick position, each -1..1
 * @returns {{x: number, y: number, z: number, r: number}}
 */
export function buildManualControlPayload(left, right) {
  return {
    x: axisToMavlink(right.y),   // pitch: right stick up/down
    y: axisToMavlink(right.x),   // roll:  right stick left/right
    z: throttleToMavlink(left.y), // throttle: left stick up/down
    r: axisToMavlink(left.x),    // yaw:   left stick left/right
  };
}
