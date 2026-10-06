/**
 * Remediation map: turns a readiness problem into a next step.
 * Pure utility — no React.
 *
 * For a non-green check it returns `{ hintKey, action }`:
 *   hintKey  i18n key ('preflight.fix.*') for the short "how to fix" text
 *   action   null                         → guidance only (wait / physical fix)
 *            { type: 'cal' }              → one-click preflight cal (gyro+baro
 *                                            [+airspeed]); pitot vehicles get a
 *                                            covered-pitot confirm at the call site
 *            { type: 'link', tool, url }  → handoff to a heavier calibration tool
 *                                            not implemented in-GCS (opens docs)
 *
 * The PreArm-reason table is intentionally small and keyword-based; grow it as
 * new ArduPilot messages come up. Unmapped states fall back to a generic hint.
 */

// Handoff targets for calibrations this GCS does not implement. Point at the
// ArduPilot docs for now; swap for an in-app tool if/when one exists.
export const CAL_DOC_URLS = {
  compass: 'https://ardupilot.org/plane/docs/common-compass-calibration-in-mission-planner.html',
  accel: 'https://ardupilot.org/plane/docs/common-accelerometer-calibration.html',
  radio: 'https://ardupilot.org/plane/docs/common-radio-control-calibration.html',
};

const CAL = { type: 'cal' };
function link(tool) {
  return { type: 'link', tool, url: CAL_DOC_URLS[tool] };
}

function prearmFix(check) {
  const msgs = (check.detail?.warnings || []).filter((m) => typeof m === 'string');
  const advisories = check.detail?.advisories || [];
  if (advisories.length && !msgs.length) {
    return { hintKey: 'preflight.fix.checksDisabled', action: null };
  }
  const text = msgs.join(' ').toLowerCase();
  if (/compass|mag/.test(text)) return { hintKey: 'preflight.fix.compass', action: link('compass') };
  if (/accel/.test(text)) return { hintKey: 'preflight.fix.accel', action: link('accel') };
  if (/gyro/.test(text)) return { hintKey: 'preflight.fix.gyro', action: CAL };
  if (/airspeed|arspd/.test(text)) return { hintKey: 'preflight.fix.airspeed', action: CAL };
  if (/baro|altitude/.test(text)) return { hintKey: 'preflight.fix.baro', action: CAL };
  if (/gps/.test(text)) return { hintKey: 'preflight.fix.gps', action: null };
  if (/throttle/.test(text)) return { hintKey: 'preflight.fix.throttle', action: null };
  if (/rc|radio|receiver/.test(text)) return { hintKey: 'preflight.fix.rc', action: link('radio') };
  return { hintKey: 'preflight.fix.prearm', action: null };
}

function companionFix(check) {
  // 'checking' is a transient heartbeat gap, not a fault — give a milder nudge
  // than the "start/connect" guidance shown for a sustained 'down'.
  return check.status === 'nogo'
    ? { hintKey: 'preflight.fix.companion', action: null }
    : { hintKey: 'preflight.fix.companionChecking', action: null };
}

function sensorsFix(check) {
  const failed = check.detail?.failed || [];
  if (failed.includes('mag')) return { hintKey: 'preflight.fix.compass', action: link('compass') };
  if (failed.includes('gyro')) return { hintKey: 'preflight.fix.gyro', action: CAL };
  if (failed.includes('abs_pressure')) return { hintKey: 'preflight.fix.baro', action: CAL };
  if (failed.includes('accel')) return { hintKey: 'preflight.fix.accel', action: link('accel') };
  return { hintKey: 'preflight.fix.sensors', action: null };
}

/**
 * @returns {{hintKey: string, action: object|null}|null} null when the check is
 * green/na (no next step).
 */
export function remediationFor(check) {
  if (!check || check.status === 'go' || check.status === 'na') return null;
  switch (check.key) {
    case 'prearm': return prearmFix(check);
    case 'companion': return companionFix(check);
    case 'mission': return { hintKey: 'preflight.fix.mission', action: null };
    case 'throttle': return { hintKey: 'preflight.fix.throttle', action: null };
    case 'gps': return { hintKey: 'preflight.fix.gps', action: null };
    case 'ekf': return { hintKey: 'preflight.fix.ekf', action: CAL };
    case 'battery': return { hintKey: 'preflight.fix.battery', action: null };
    case 'rc': return { hintKey: 'preflight.fix.rc', action: link('radio') };
    case 'sensors': return sensorsFix(check);
    case 'airspeed': return { hintKey: 'preflight.fix.airspeed', action: CAL };
    default: return null;
  }
}

// Per-vehicle "next step" priority: most blocking / most actionable first.
// Companion + mission lead because they are launch-gate blockers the operator
// must resolve before launch (companion-down is the reported bug).
const PRIORITY = ['companion', 'mission', 'prearm', 'gps', 'throttle', 'battery', 'ekf', 'airspeed', 'sensors', 'rc'];
const STATUS_RANK = { nogo: 0, warn: 1 };

// Rank for the priority tie-break. An unknown key sorts LAST (not first, which a
// raw indexOf === -1 would do), so a future un-prioritized row never hijacks the
// "next step" ahead of a known launch blocker.
function priorityRank(key) {
  const i = PRIORITY.indexOf(key);
  return i < 0 ? PRIORITY.length : i;
}

/**
 * Pick the single most important issue for a vehicle's next-step banner.
 * @returns {{check: object, hintKey: string, action: object|null}|null}
 */
export function topIssue(checks) {
  const issues = (checks || []).filter(
    (c) => c && (c.status === 'nogo' || c.status === 'warn'),
  );
  if (!issues.length) return null;
  issues.sort((a, b) => {
    const s = (STATUS_RANK[a.status] ?? 9) - (STATUS_RANK[b.status] ?? 9);
    if (s !== 0) return s;
    return priorityRank(a.key) - priorityRank(b.key);
  });
  const top = issues[0];
  return { check: top, ...remediationFor(top) };
}
