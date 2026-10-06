/**
 * Pure state helpers for onboard compass / magnetometer calibration.
 *
 * Kept free of React so the reduction logic can be unit-tested via Node and
 * reused by useCompassCal. The backend sends a single `compass_cal_progress`
 * WS message for both MAG_CAL_PROGRESS (report=false) and MAG_CAL_REPORT
 * (report=true); this module folds those into a per-vehicle view.
 *
 * Ported from feat/uav-compass-cal (INTEG-04, D-04) — drop-in, no dev-side
 * collision.
 */

// MAG_CAL_STATUS enum (pymavlink ardupilotmega).
export const MAG_CAL_STATUS = {
  NOT_STARTED: 0,
  WAITING_TO_START: 1,
  RUNNING_STEP_ONE: 2,
  RUNNING_STEP_TWO: 3,
  SUCCESS: 4,
  FAILED: 5,
  BAD_ORIENTATION: 6,
  BAD_RADIUS: 7,
};

const FAILED_STATUSES = new Set([
  MAG_CAL_STATUS.FAILED,
  MAG_CAL_STATUS.BAD_ORIENTATION,
  MAG_CAL_STATUS.BAD_RADIUS,
]);

/** i18n key suffix for a per-compass MAG_CAL_STATUS value. */
export function statusKey(status) {
  switch (status) {
    case MAG_CAL_STATUS.NOT_STARTED: return 'notStarted';
    case MAG_CAL_STATUS.WAITING_TO_START: return 'waiting';
    case MAG_CAL_STATUS.RUNNING_STEP_ONE: return 'step1';
    case MAG_CAL_STATUS.RUNNING_STEP_TWO: return 'step2';
    case MAG_CAL_STATUS.SUCCESS: return 'success';
    case MAG_CAL_STATUS.FAILED: return 'failed';
    case MAG_CAL_STATUS.BAD_ORIENTATION: return 'badOrientation';
    case MAG_CAL_STATUS.BAD_RADIUS: return 'badRadius';
    default: return 'unknown';
  }
}

/** Initial per-vehicle calibration state (no calibration seen yet). */
export function emptyCal() {
  return { compasses: {}, status: 'idle', rebootRequired: false };
}

/** State to apply when the operator starts a calibration. */
export function startCal() {
  return { compasses: {}, status: 'running', rebootRequired: false };
}

/** State to apply when the operator cancels a calibration. */
export function cancelCal(prev) {
  return { ...(prev || emptyCal()), status: 'cancelled' };
}

/**
 * Derive the vehicle-level status from its per-compass map.
 * `started` keeps a freshly-started calibration showing "running" until the
 * first progress message arrives.
 */
export function deriveStatus(compasses, started) {
  const ids = Object.keys(compasses);
  if (ids.length === 0) return started ? 'running' : 'idle';
  let anyFailed = false;
  let allReported = true;
  let allSuccess = true;
  for (const id of ids) {
    const c = compasses[id];
    if (FAILED_STATUSES.has(c.status)) anyFailed = true;
    if (!c.report) allReported = false;
    if (c.status !== MAG_CAL_STATUS.SUCCESS) allSuccess = false;
  }
  if (anyFailed) return 'failed';
  if (allReported && allSuccess) return 'success';
  return 'running';
}

/**
 * Fold a `compass_cal_progress` event into the prior per-vehicle state.
 * Pure: returns a new object, never mutates `prev`.
 */
export function reduceCompassCal(prev, event) {
  const base = prev || emptyCal();
  const compasses = { ...base.compasses };
  const id = Number(event.compass_id);
  const existing = compasses[id] || {};
  const report = !!event.report || !!existing.report;
  compasses[id] = {
    pct: event.report ? 100 : (event.pct ?? existing.pct ?? 0),
    status: event.cal_status ?? existing.status ?? MAG_CAL_STATUS.NOT_STARTED,
    fitness: event.report ? (event.fitness ?? null) : (existing.fitness ?? null),
    report,
  };
  const status = deriveStatus(compasses, true);
  return { compasses, status, rebootRequired: status === 'success' };
}
