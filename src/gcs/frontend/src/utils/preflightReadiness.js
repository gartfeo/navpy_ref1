/**
 * Preflight readiness checks for the "before run" panel.
 * Pure utility — no React dependencies.
 *
 * Builds on prearmChecks.js so the prearm/GPS/battery verdicts use the exact
 * same gates/thresholds as the launch gate, then ADDS rows for EKF health,
 * sensor health (gyro/accel/mag/baro), and the RC link — fields surfaced from
 * the telemetry snapshot (`vehicle.ekf`, `vehicle.sensors`) — plus the
 * launch-gate-only conditions companion computer, mission upload, and throttle
 * (see computeVehicleReadiness).
 *
 * This is an ADVISORY readiness view: it does NOT change launch gating in
 * LaunchPanel/control.py — it informs the operator. But its overall GO/NO-GO is
 * reconciled with the launch gate: computeVehicleReadiness folds
 * `computeLaunchReadiness().ready` into the verdict, so the board can never read
 * GO while the launch button is disabled. It may still be STRICTER than the gate
 * (EKF/RC/sensors/airspeed can force NO-GO for things the gate ignores).
 *
 * Each check returns `{ key, status, detail }` where status is one of:
 *   'go'   — green, ready
 *   'warn' — amber, caution but not blocking
 *   'nogo' — red, blocking
 *   'na'   — grey, not applicable / not yet reported (treated as non-blocking)
 */

import { computePrearmWarnings, computeLaunchReadiness, DEFAULT_LAUNCH_GATES } from './prearmChecks';
import { computeFleetHarmonize } from './paramHarmonize';

// EKF innovation test-ratio thresholds (0..1, from EKF_STATUS_REPORT variances).
// FAIL mirrors ArduPilot's FS_EKF_THRESH default (0.8) — the innovation level at
// which the firmware's own EKF failsafe trips. WARN (0.5) is the conventional
// Mission Planner amber level. Centralized + documented so the panel (and any
// future gate) agree on what an unhealthy EKF means.
export const EKF_VARIANCE_FAIL = 0.8;
export const EKF_VARIANCE_WARN = 0.5;

// EKF_STATUS_FLAGS bit 10: estimator has not finished initializing, so the nav
// solution is unusable regardless of variances.
const EKF_FLAG_UNINITIALIZED = 1 << 10; // 1024

// Ground airspeed sanity threshold (m/s). While disarmed and stationary the
// pitot should read ~0; a reading at/above this suggests a stale zero (or wind)
// and that airspeed should be re-zeroed with the pitot covered.
export const GROUND_AIRSPEED_WARN_MS = 3.0;

// Sensors aggregated into the single "Sensors" row (RC is its own row, GPS has
// its richer fix/sats/hacc row). Order is the display order in the tooltip.
export const PREFLIGHT_SENSORS = ['gyro', 'accel', 'mag', 'abs_pressure'];

// Ordering for the overall verdict: 'na' (no data) is the lowest so a vehicle
// reporting at least one healthy row reads as 'go', while any 'nogo' wins. Only
// an all-'na' vehicle stays 'na'. 'warn' never blocks (go stays true).
const STATUS_RANK = { na: 0, go: 1, warn: 2, nogo: 3 };

/** Return the more-severe of two statuses ('nogo' > 'warn' > 'go' > 'na'). */
function worst(a, b) {
  return STATUS_RANK[b] > STATUS_RANK[a] ? b : a;
}

/**
 * Prearm-state row. Reuses computePrearmWarnings with the non-prearm checks
 * disabled, so this row reflects ONLY the autopilot's pre-arm verdict — and
 * inherits its tested nuances (the "mode not armable" exemption and extraction
 * of specific `PreArm:` STATUSTEXT messages).
 */
export function computePrearmCheck(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (gates.checkPrearm === false) {
    return { key: 'prearm', status: 'na', detail: null };
  }
  const prearmOnly = {
    ...gates,
    checkGps: false,
    checkGpsAcc: false,
    checkThrottle: false,
    checkBattery: false,
  };
  const { warnings, advisories, severity } = computePrearmWarnings(vehicle, prearmOnly);
  let status = 'go';
  if (severity === 'fail') status = 'nogo';
  else if (warnings.length > 0 || advisories.length > 0) status = 'warn';
  return { key: 'prearm', status, detail: { warnings, advisories } };
}

/** GPS row: 3D-fix gate (matches the launch gate) plus optional HACC advisory. */
export function computeGpsCheck(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  const fix = vehicle.gps_fix;
  let status = 'go';
  if (gates.checkGps !== false) {
    // Null fix == no GPS data → not ready, same as the launch gate which blocks
    // on `gps_fix is None or < 3`.
    if (fix == null || fix < 3) status = 'nogo';
  }
  if (status === 'go' && gates.checkGpsAcc
      && vehicle.gps_hacc != null && vehicle.gps_hacc > gates.maxGpsHaccM) {
    status = 'warn';
  }
  return {
    key: 'gps',
    status,
    detail: { fix, sats: vehicle.gps_sats, hacc: vehicle.gps_hacc },
  };
}

/** EKF row from EKF_STATUS_REPORT flags + variances. */
export function computeEkfCheck(ekf) {
  if (!ekf) return { key: 'ekf', status: 'na', detail: null };
  const flags = Number(ekf.flags) || 0;
  if (flags & EKF_FLAG_UNINITIALIZED) {
    return { key: 'ekf', status: 'nogo', detail: { reason: 'uninitialized' } };
  }
  // Exclude terrain_alt_variance: it reads high whenever no terrain/rangefinder
  // is in use (a normal condition) and must not force a NO-GO.
  const variances = [
    ekf.velocity_variance,
    ekf.pos_horiz_variance,
    ekf.pos_vert_variance,
    ekf.compass_variance,
  ].map(Number).filter((x) => Number.isFinite(x));
  const maxVariance = variances.length ? Math.max(...variances) : 0;
  let status = 'go';
  if (maxVariance >= EKF_VARIANCE_FAIL) status = 'nogo';
  else if (maxVariance >= EKF_VARIANCE_WARN) status = 'warn';
  return { key: 'ekf', status, detail: { maxVariance } };
}

/** Battery row: same %/unknown gating as the launch gate. */
export function computeBatteryCheck(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (gates.checkBattery === false) {
    return { key: 'battery', status: 'na', detail: { volt: vehicle.voltage } };
  }
  const pct = vehicle.battery;
  if (pct == null) {
    return {
      key: 'battery',
      status: gates.blockOnUnknownBattery ? 'nogo' : 'warn',
      detail: { pct: null, volt: vehicle.voltage },
    };
  }
  return {
    key: 'battery',
    status: pct < gates.minBatteryPct ? 'nogo' : 'go',
    detail: { pct, volt: vehicle.voltage },
  };
}

/**
 * Airspeed (pitot) row. Presence-gated: 'na' when no differential-pressure
 * sensor is fitted. 'nogo' when the sensor is unhealthy; 'warn' (advisory nudge)
 * when it reads a real airspeed while disarmed and stationary — a hint the zero
 * offset is stale (or there's wind) and it should be re-zeroed with the pitot
 * covered. Not part of computePreflightReadiness's core checks (presence-gated),
 * but included by the readiness view for the airspeed row + banner.
 */
export function computeAirspeedCheck(vehicle) {
  if (!vehicle || vehicle.airspeed_present !== true) {
    return { key: 'airspeed', status: 'na', detail: { present: false } };
  }
  const reading = typeof vehicle.air_speed === 'number' ? vehicle.air_speed : null;
  if (vehicle.airspeed_ok === false) {
    return { key: 'airspeed', status: 'nogo', detail: { present: true, reading, reason: 'unhealthy' } };
  }
  const stationary = !vehicle.armed
    && (vehicle.ground_speed == null || vehicle.ground_speed < 1.0);
  if (stationary && reading != null && reading >= GROUND_AIRSPEED_WARN_MS) {
    return { key: 'airspeed', status: 'warn', detail: { present: true, reading, reason: 'groundReading' } };
  }
  return { key: 'airspeed', status: 'go', detail: { present: true, reading } };
}

/** RC-link row from the SYS_STATUS RC-receiver health bit. */
export function computeRcCheck(sensors) {
  const rc = sensors ? sensors.rc : undefined;
  if (rc === true) return { key: 'rc', status: 'go', detail: null };
  if (rc === false) return { key: 'rc', status: 'nogo', detail: null };
  // Not present/enabled (e.g. no RC receiver configured) → can't judge.
  return { key: 'rc', status: 'na', detail: null };
}

/** Aggregate sensor-health row (gyro/accel/mag/baro). */
export function computeSensorCheck(sensors) {
  if (!sensors) return { key: 'sensors', status: 'na', detail: null };
  const known = PREFLIGHT_SENSORS.some((k) => typeof sensors[k] === 'boolean');
  if (!known) return { key: 'sensors', status: 'na', detail: null };
  const failed = PREFLIGHT_SENSORS.filter((k) => sensors[k] === false);
  return {
    key: 'sensors',
    status: failed.length ? 'nogo' : 'go',
    detail: failed.length ? { failed } : null,
  };
}

/**
 * Full preflight readiness for one vehicle: an ordered list of check rows and
 * an overall verdict (the worst row status; `go` is true unless any row is
 * 'nogo').
 * @param {object|null|undefined} vehicle - telemetry object
 * @param {typeof DEFAULT_LAUNCH_GATES} [gates] - per-check enable + thresholds
 * @returns {{ checks: Array, overall: 'go'|'warn'|'nogo'|'na', go: boolean }}
 */
export function computePreflightReadiness(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (!vehicle) return { checks: [], overall: 'na', go: true };
  const checks = [
    computePrearmCheck(vehicle, gates),
    computeGpsCheck(vehicle, gates),
    computeEkfCheck(vehicle.ekf),
    computeBatteryCheck(vehicle, gates),
    computeRcCheck(vehicle.sensors),
    computeSensorCheck(vehicle.sensors),
  ];
  const overall = checks.reduce((acc, c) => worst(acc, c.status), 'na');
  return { checks, overall, go: overall !== 'nogo' };
}

/**
 * Companion-computer (NavPy) liveness row. Mirrors the launch gate, which blocks
 * only on a sustained "down" (backend sets companion_ok = companion_status !==
 * 'down'). A transient "checking" gap in the 1 Hz heartbeat is caution, not a
 * blocker, so it must not flap the verdict. Missing data (no status, no
 * companion_ok) is 'na' (non-blocking), matching the gate which only trips on an
 * explicit companion_ok === false.
 */
export function computeCompanionCheck(vehicle) {
  if (!vehicle) return { key: 'companion', status: 'na', detail: null };
  const s = vehicle.companion_status;
  // companion_ok === false is the launch gate's blocking signal; also treat an
  // explicit 'down' status as a blocker in case a snapshot is inconsistent.
  if (vehicle.companion_ok === false || s === 'down') {
    return { key: 'companion', status: 'nogo', detail: { status: s || 'down' } };
  }
  if (s === 'checking') return { key: 'companion', status: 'warn', detail: { status: s } };
  if (s === 'ok' || vehicle.companion_ok === true) {
    return { key: 'companion', status: 'go', detail: { status: s || 'ok' } };
  }
  return { key: 'companion', status: 'na', detail: { status: s || null } };
}

/**
 * Mission-upload row. Mirrors the launch gate: a vehicle with no verified upload
 * and no mission items on board cannot launch, so not-uploaded is a hard blocker
 * (this is why the board reads NO-GO for every vehicle until a plan is uploaded).
 */
export function computeMissionCheck(vehicle) {
  if (!vehicle) return { key: 'mission', status: 'na', detail: null };
  const uploaded = !!vehicle.mission_uploaded || (vehicle.mission_total > 0);
  return {
    key: 'mission',
    status: uploaded ? 'go' : 'nogo',
    detail: { total: vehicle.mission_total ?? null },
  };
}

/**
 * Throttle row. Mirrors the gate's throttle check: a high stick while disarmed
 * blocks arming/launch. 'na' when RC3 isn't reported or the check is disabled
 * (nothing to judge, non-blocking) — same as the gate, which only trips on a
 * real reading above the threshold.
 */
export function computeThrottleCheck(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (!vehicle || gates.checkThrottle === false || vehicle.rc3 == null) {
    return {
      key: 'throttle',
      status: 'na',
      detail: { rc3: (vehicle && vehicle.rc3 != null) ? vehicle.rc3 : null },
    };
  }
  return {
    key: 'throttle',
    status: vehicle.rc3 > gates.maxThrottleRc3 ? 'nogo' : 'go',
    detail: { rc3: vehicle.rc3 },
  };
}

/**
 * Full readiness for one vehicle. Folds in the presence-gated airspeed row plus
 * the launch-gate-only conditions (companion, mission, throttle) so the board's
 * overall verdict cannot read GO while the launch gate would block the vehicle.
 * Link-down vehicles read 'na' with no checks — there's nothing to report until
 * telemetry resumes. Single source of truth for "this vehicle's one overall
 * verdict", shared by the readiness grid and the fleet-status rollup below.
 * @returns {{ checks: Array, overall: 'go'|'warn'|'nogo'|'na' }}
 */
export function computeVehicleReadiness(vehicle, gates = DEFAULT_LAUNCH_GATES) {
  if (!vehicle || !vehicle.link_ok) return { checks: [], overall: 'na' };
  const base = computePreflightReadiness(vehicle, gates);
  const companion = computeCompanionCheck(vehicle);
  const mission = computeMissionCheck(vehicle);
  const throttle = computeThrottleCheck(vehicle, gates);
  const air = computeAirspeedCheck(vehicle);
  const checks = [...base.checks, companion, mission, throttle, air];
  let overall = checks.reduce((acc, c) => worst(acc, c.status), 'na');
  // Authoritative backstop: the launch gate is the source of truth for "can this
  // vehicle launch". Any launch blocker (throttle, GPS accuracy, unknown battery,
  // or a FUTURE gate condition) forces the header NO-GO, so the board can never
  // read GO while the launch button is disabled. Every current gate issue also
  // maps to one of the rows above, so the action strip always has a visible
  // reason — any NEW launch-gate issue MUST add a corresponding row here.
  if (!computeLaunchReadiness(vehicle, gates).ready) overall = worst(overall, 'nogo');
  return { checks, overall };
}

/**
 * Fleet-wide rollup: worst per-vehicle overall wins ('nogo' > 'warn' > 'go');
 * 'na' only when there's no vehicle with live telemetry to report on. Meant for
 * an at-a-glance indicator (e.g. the top-bar readiness button) — same verdicts
 * as the readiness grid, just collapsed to one status.
 */
export function computeFleetStatus(vehicleList, gates = DEFAULT_LAUNCH_GATES) {
  const overalls = (vehicleList || []).map((v) => computeVehicleReadiness(v, gates).overall);
  if (!overalls.length || overalls.every((o) => o === 'na')) return 'na';
  if (overalls.some((o) => o === 'nogo')) return 'nogo';
  if (overalls.some((o) => o === 'warn')) return 'warn';
  return 'go';
}

/**
 * Fleet-wide "params not synced" row (CONF-01, D-01..D-06). Reuses
 * computeFleetHarmonize (skipUavSpecific:true) instead of re-implementing the
 * per-param comparison, so this row and the Sync UAVs / Harmonize UI can never
 * disagree on what counts as divergent (D-02). Generic over EVERY shared
 * (non-identity/cal/bus) param — not confirm-mode only — so `AAS_NAV_AUTO_CM`
 * is just one of many rows that can trigger it.
 *
 * Advisory only: always returns `warn`, never `nogo` (D-03 — a preflight
 * readiness warning, not a launch blocker). `na` when fewer than two
 * connected UAVs have a loaded param snapshot (nothing to compare yet — this
 * is not itself a failure; D-06's auto-download-on-connect fills snapshots in
 * shortly after connect).
 *
 * Read-only (firmware-maintained) divergent rows are excluded from `detail`:
 * they can legitimately differ per board (STAT_* counters, *_DEVID, ...) and
 * are not writable via the Harmonize UI this row links to, so surfacing them
 * here would be non-actionable noise.
 *
 * @param {object} args
 * @param {object} args.snapshotsByVehicle - `{ [sysId]: normalisedSnapshot }`
 * @param {Array<number|string>} args.sysIds - connected vehicle ids
 * @returns {{ key: 'paramSync', status: 'go'|'warn'|'na', detail: null|{ divergent: number, rows: Array<{name: string, distinctValues: Array}> } }}
 */
export function computeFleetParamSync({ snapshotsByVehicle, sysIds } = {}) {
  const ids = Array.isArray(sysIds) ? sysIds : [];
  const snaps = snapshotsByVehicle || {};
  const loadedCount = ids.filter((sid) => !!(snaps[sid] && snaps[sid].paramsByName)).length;
  // Fewer than two loaded snapshots -> nothing to compare (never 'warn').
  if (loadedCount < 2) return { key: 'paramSync', status: 'na', detail: null };

  const { rows } = computeFleetHarmonize({
    snapshotsByVehicle: snaps, sysIds: ids, skipUavSpecific: true,
  });
  // Only non-skip-listed, non-read-only divergent rows are "not synced" in the
  // actionable sense this warning is for — identity/cal params are SUPPOSED
  // to differ per board (isUavSpecific), and read-only rows can't be written.
  const divergentRows = rows
    .filter((r) => r.status === 'divergent' && !r.isUavSpecific)
    .map((r) => ({ name: r.name, distinctValues: r.distinctValues }));
  const status = divergentRows.length > 0 ? 'warn' : 'go';
  return {
    key: 'paramSync',
    status,
    detail: { divergent: divergentRows.length, rows: divergentRows },
  };
}
