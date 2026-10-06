/**
 * Client-side pre-arm warning checks.
 * Pure utility — no React dependencies.
 */

/**
 * Resolve the fine-grained pre-arm state for a vehicle.
 * Prefers the backend `prearm_check_state`; falls back to the legacy tri-state
 * `prearm_ok` when the field is absent (older snapshot). The fallback cannot
 * tell "checks disabled" from "no data", so it maps the indeterminate case to
 * `no_sys_status` — the same behaviour as before this field existed.
 * @returns {'no_sys_status'|'not_reported'|'checks_disabled'|'failed'|'ok'}
 */
function prearmState(vehicle) {
  if (typeof vehicle.prearm_check_state === 'string') {
    return vehicle.prearm_check_state;
  }
  if (vehicle.prearm_ok === true) return 'ok';
  if (vehicle.prearm_ok === false) return 'failed';
  return 'no_sys_status';
}

/**
 * Default launch-readiness gates. Mirrors the backend ReadinessGates defaults
 * (control.py) so the UI and the server agree when no settings are supplied.
 * The GPS 3D-fix threshold itself is fixed (protocol constant); only whether
 * the GPS check runs is toggleable.
 */
export const DEFAULT_LAUNCH_GATES = Object.freeze({
  checkPrearm: true,
  checkGps: true,
  checkGpsAcc: false,
  maxGpsHaccM: 1.0,
  checkThrottle: true,
  maxThrottleRc3: 1050,
  checkBattery: true,
  minBatteryPct: 15,
  blockOnUnknownBattery: false,
});

/**
 * Derive launch-readiness gates from GCS settings. Missing fields fall back to
 * the behavior-preserving defaults, so an older settings object still yields
 * today's checks.
 * @param {object|null|undefined} settings - GCS settings object
 * @returns {typeof DEFAULT_LAUNCH_GATES}
 */
export function launchGatesFromSettings(settings) {
  const l = (settings && settings.launch) || {};
  return {
    checkPrearm: l.check_prearm_enabled !== false,
    checkGps: l.check_gps_enabled !== false,
    checkGpsAcc: l.check_gps_acc_enabled === true,
    maxGpsHaccM: l.max_gps_hacc_m ?? DEFAULT_LAUNCH_GATES.maxGpsHaccM,
    checkThrottle: l.check_throttle_enabled !== false,
    maxThrottleRc3: l.max_throttle_rc3 ?? DEFAULT_LAUNCH_GATES.maxThrottleRc3,
    checkBattery: l.check_battery_enabled !== false,
    minBatteryPct: l.min_battery_pct ?? DEFAULT_LAUNCH_GATES.minBatteryPct,
    blockOnUnknownBattery: l.block_on_unknown_battery === true,
  };
}

/**
 * Compute pre-arm warnings from vehicle telemetry.
 *
 * `warnings` are blocking conditions (shown as errors, gate launch).
 * `advisories` are non-blocking notices (e.g. arming checks disabled) — the
 * vehicle is armable, so they neither gate launch nor count as failures.
 * @param {object|null|undefined} vehicle - telemetry object
 * @param {typeof DEFAULT_LAUNCH_GATES} [gates] - per-check enable + thresholds
 * @returns {{ warnings: Array, advisories: string[], severity: 'ok'|'warn'|'fail' }}
 */
export function computePrearmWarnings(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (!vehicle) return { warnings: [], advisories: [], severity: 'ok' };

  const warnings = [];
  const advisories = [];
  let hadModeNotArmable = false;
  const prearmDetails = [];

  // Pre-arm status from autopilot, keyed on the fine-grained state.
  const state = prearmState(vehicle);
  if (gates.checkPrearm) {
    if (state === 'no_sys_status') {
      warnings.push('prearm.waitingForStatus');
    } else if (state === 'not_reported') {
      warnings.push('prearm.statusNotReported');
    } else if (state === 'checks_disabled') {
      // Arming checks are turned off — the vehicle IS armable, so this is an
      // advisory, not a blocker.
      advisories.push('prearm.checksDisabled');
    } else if (state === 'failed') {
      // Prefer specific PreArm: messages from status_texts over the generic
      // "Pre-arm check failed" so the operator can see exactly what needs fixing.
      // (When the vehicle is armable, stale cached messages must not apply —
      // only proactive checks below.)
      if (Array.isArray(vehicle.status_texts)) {
        for (const entry of vehicle.status_texts) {
          const text = entry && entry.text;
          if (typeof text === 'string' && text.startsWith('PreArm:')) {
            const msg = text.slice(7).trim();
            if (msg && msg.toLowerCase().includes('mode not armable')) {
              hadModeNotArmable = true;
            } else if (msg && !prearmDetails.includes(msg)) {
              prearmDetails.push(msg);
            }
          }
        }
      }
      if (prearmDetails.length > 0) {
        for (const d of prearmDetails) warnings.push(d);
      } else if (!hadModeNotArmable) {
        warnings.push('prearm.prearmFailed');
      }
    } else if (state !== 'ok') {
      // Unknown/unexpected state — fail closed (mirrors the backend launch gate)
      // so the UI doesn't show "armable" for a state it doesn't understand.
      warnings.push('prearm.statusUnknown');
    }
  }

  // Throttle not zero
  if (gates.checkThrottle && vehicle.rc3 != null && vehicle.rc3 > gates.maxThrottleRc3) {
    warnings.push('prearm.throttleNotZero');
  }

  // GPS checks
  if (gates.checkGps) {
    if (vehicle.gps_fix == null) {
      warnings.push('prearm.gpsUnknown');
    } else if (vehicle.gps_fix < 3) {
      warnings.push('prearm.noGps3d');
    }
  }

  // GPS horizontal-accuracy check
  if (gates.checkGpsAcc && vehicle.gps_hacc != null && vehicle.gps_hacc > gates.maxGpsHaccM) {
    warnings.push({ key: 'prearm.gpsAccLow', acc: vehicle.gps_hacc.toFixed(1) });
  }

  // Battery check
  if (gates.checkBattery) {
    if (vehicle.battery != null && vehicle.battery < gates.minBatteryPct) {
      warnings.push({ key: 'prearm.batteryLow', pct: Math.round(vehicle.battery) });
    } else if (vehicle.battery == null && gates.blockOnUnknownBattery) {
      // Closes the gap where a missing/unconfigured battery monitor silently passes.
      warnings.push('prearm.batteryUnknown');
    }
  }

  // Determine severity. Advisories never raise severity on their own.
  const modeNotArmableOnly = gates.checkPrearm && state === 'failed'
    && hadModeNotArmable && prearmDetails.length === 0;
  let severity = 'ok';
  if (gates.checkPrearm && state === 'failed' && !modeNotArmableOnly) {
    severity = 'fail';
  } else if (warnings.length > 0) {
    severity = 'warn';
  }

  return { warnings, advisories, severity };
}

/**
 * Compute launch readiness for a vehicle.
 * Builds on computePrearmWarnings and adds launch-critical checks
 * (companion computer, mission uploaded). Advisories are passed through for
 * display but never affect `ready`.
 * @param {object|null|undefined} vehicle - telemetry object
 * @param {typeof DEFAULT_LAUNCH_GATES} [gates] - per-check enable + thresholds
 * @returns {{ issues: Array, advisories: string[], ready: boolean }}
 */
export function computeLaunchReadiness(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (!vehicle) return { issues: [], advisories: [], ready: true };

  const { warnings, advisories } = computePrearmWarnings(vehicle, gates);
  const issues = [...warnings];

  if (vehicle.companion_ok === false) {
    issues.push('prearm.companionNotConnected');
  }

  if (!vehicle.mission_uploaded && !(vehicle.mission_total > 0)) {
    issues.push('prearm.missionNotUploaded');
  }

  return { issues, advisories, ready: issues.length === 0 };
}

/**
 * Filter advisories down to those not dismissed.
 * @param {Array} advisories - advisory keys/objects from computePrearmWarnings
 * @param {Set|Array} dismissed - keys the user has dismissed
 * @returns {Array} advisories still to show
 */
export function visibleAdvisories(advisories, dismissed) {
  const skip = dismissed instanceof Set ? dismissed : new Set(dismissed || []);
  return (advisories || []).filter(
    (a) => !skip.has(typeof a === 'object' ? a.key : a),
  );
}
