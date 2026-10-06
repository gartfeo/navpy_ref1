/**
 * Accelerometer / level calibration — shared constants and pure state logic.
 *
 * Position codes match ArduPilot's AccelCalibrator vehicle-position enum and
 * the backend MAV_CMD_ACCELCAL_VEHICLE_POS param1 values (1=level .. 6=back).
 * Kept free of React so the transitions can be unit-tested via Node.js and
 * reused by both the useAccelCal hook and the CalibrationTab component.
 *
 * Re-authored from feat/uav-accel-cal (466 commits behind dev, INTEG-04,
 * D-04) — drop-in logic, no dev-side collision.
 */

// Ordered as ArduPilot prompts them; `key` indexes the i18n strings.
export const ACCEL_CAL_POSITIONS = [
  { code: 1, key: 'level' },
  { code: 2, key: 'left' },
  { code: 3, key: 'right' },
  { code: 4, key: 'noseDown' },
  { code: 5, key: 'noseUp' },
  { code: 6, key: 'back' },
];

export const ACCEL_CAL_POS_KEY_BY_CODE = ACCEL_CAL_POSITIONS.reduce((m, p) => {
  m[p.code] = p.key;
  return m;
}, {});

/** Fresh per-vehicle state when the operator kicks off a calibration. */
export function initCalState(mode) {
  return { active: true, mode, step: null, prompt: '', result: null, busy: true };
}

/**
 * Fold an incoming `accel_cal_step` WebSocket message into the next
 * per-vehicle calibration state.
 *
 * msg: { status: 'prompt'|'success'|'failed', step: number|null, prompt_text }
 *
 * A `prompt` while idle (re)opens a session — this keeps the wizard in sync if
 * a cal was started elsewhere or the page reloaded mid-cal. A `success`/`failed`
 * closes the active session; when idle it is ignored (stale late result).
 */
export function reduceAccelCalStep(state, msg) {
  const status = msg && msg.status;
  if (!state || !state.active) {
    if (status === 'prompt') {
      return {
        active: true,
        mode: (state && state.mode) || 'full',
        step: msg.step ?? null,
        prompt: msg.prompt_text || '',
        result: null,
        busy: false,
      };
    }
    return state || null;
  }
  if (status === 'prompt') {
    return {
      ...state,
      step: msg.step ?? null,
      prompt: msg.prompt_text || '',
      result: null,
      busy: false,
    };
  }
  if (status === 'success' || status === 'failed') {
    return {
      ...state,
      active: false,
      step: null,
      prompt: msg.prompt_text || state.prompt,
      result: status,
      busy: false,
    };
  }
  return state;
}
