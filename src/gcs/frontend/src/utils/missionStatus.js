import { colors } from '../styles.js';

/**
 * Status definitions keyed by status id.
 * Each entry holds the display label and CSS color.
 */
const STATUS_TABLE = {
  idle:        { labelKey: 'missionStatus.idle',        color: colors.textDim },
  en_route:    { labelKey: 'missionStatus.enRoute',     color: colors.textDim },
  searching:   { labelKey: 'missionStatus.searching',   color: colors.success },
  waiting:     { labelKey: 'missionStatus.waiting',     color: colors.textDim },
  assigned:    { labelKey: 'missionStatus.assigned',    color: colors.accent },
  approaching: { labelKey: 'missionStatus.approaching', color: colors.warning },
  confirming:  { labelKey: 'missionStatus.confirming',  color: colors.warning },
  confirmed:   { labelKey: 'missionStatus.confirmed',   color: colors.error },
  homing:      { labelKey: 'missionStatus.homing',      color: colors.accent },
  recovering:  { labelKey: 'missionStatus.recovering',  color: colors.textDim },
  docked:      { labelKey: 'missionStatus.docked',      color: colors.success },
  dock_failed: { labelKey: 'missionStatus.dockFailed',  color: colors.error },
};

// NavPy's display phase (snapshot `swarm.phase`, docs/design/gcs_mission_phase.md)
// -> status id, from the phase and, for SEARCH and HOLD, mode and progress.
const PHASE_STATUS = {
  SEARCH: (v, wpOffset) => (beforeSearch(v, wpOffset) ? 'en_route' : 'searching'),
  TO_TASK: () => 'approaching',
  HOMING: () => 'homing',
  CONFIRM: () => 'confirming',
  APPROACH: () => 'confirmed',
  // ONHOLD: not started, before the detect waypoint, or mode not active.
  HOLD: (v) => (v.armed && v.mode === 'AUTO' ? 'en_route' : 'idle'),
  RECOVERING: () => 'recovering',
};
// Phases where a held outcome is the badge rather than a "Last: ..." line.
const OUTCOME_BADGE_PHASES = new Set(['HOLD', 'RECOVERING']);
// Which assignment statuses may label which phase (fail-closed: a CONFIRMED
// slot never retires and cannot be tied to the current approach).
const LABEL_STATUSES = {
  TO_TASK: new Set(['assigned']),
  CONFIRM: new Set(['assigned', 'confirming']),
};
const FAIL_REASON_KEYS = new Set([
  'nav_fail', 'no_detection', 'missed_dock', 'navigation_rejected',
  'source_command_stale', 'discontinuity_source_missing',
  'final_approach_record_unavailable', 'source_event_active_poi_missing',
  'source_event_pump',
]);

/**
 * Compute the mission lifecycle status for a UAV.
 *
 * With NavPy's phase in `vehicle.swarm` the badge comes from it; without one
 * (an old companion or trace, or no beat yet) it is guessed from mode,
 * armed state, progress and the assignment, as before.
 *
 * @param {object} vehicle - Telemetry data: { mode, armed, lat, lon, mission_progress, swarm }
 * @param {object|null} assignment - Assignment state: { taskId, taskType, lat, lon, status, doingUid } or null
 * @param {number} wpOffset - Count of pre-search waypoints (corridor approach, takeoff, etc.)
 * @param {boolean} [hasPendingConfirm=false] - Whether a confirm popup is active for this vehicle
 * @returns {{ status: string, labelKey: string, color: string, assignment: object|null,
 *   outcome: object|null, last: object|null, stale: boolean }}
 *   `assignment` is the one the row may label; `outcome` is shown under an
 *   outcome badge (its reason), `last` as a dim "Last: ..." line.
 */
export function computeMissionStatus(vehicle, assignment, wpOffset, hasPendingConfirm) {
  const swarm = vehicle.swarm;
  if (swarm && swarm.phase) return phaseStatus(vehicle, swarm, assignment, wpOffset);
  const statusId = deriveStatusId(vehicle, assignment, wpOffset, hasPendingConfirm);
  return withEntry(statusId, { assignment });
}

function withEntry(statusId, extra = {}) {
  const entry = STATUS_TABLE[statusId];
  return {
    status: statusId, labelKey: entry.labelKey, color: entry.color,
    assignment: null, outcome: null, last: null, stale: false,
    ...extra,
  };
}

/** The outcome NavPy reports, or null: `{ kind, reasonKey }`. */
function swarmOutcome(swarm) {
  if (swarm.outcome === 'docked') return { kind: 'docked', reasonKey: null };
  if (swarm.outcome === 'failed') {
    const reason = FAIL_REASON_KEYS.has(swarm.fail_reason) ? swarm.fail_reason : 'unknown';
    return { kind: 'dock_failed', reasonKey: `missionStatus.reasons.${reason}` };
  }
  return null;
}

/**
 * Status from NavPy's phase. A stale beat with no outcome falls back to the
 * mode guess. With a phase, published or expired, the badge never reads the
 * assignment or the approved card.
 */
function phaseStatus(vehicle, swarm, assignment, wpOffset) {
  const outcome = swarmOutcome(swarm);
  if (swarm.stale && !outcome) return withEntry(guessStatusId(vehicle, wpOffset));
  const stale = !!swarm.stale;
  if (outcome && OUTCOME_BADGE_PHASES.has(swarm.phase)) {
    return withEntry(outcome.kind, { outcome, stale });
  }
  const phase = PHASE_STATUS[swarm.phase];
  const statusId = phase ? phase(vehicle, wpOffset) : guessStatusId(vehicle, wpOffset);
  return withEntry(statusId, {
    assignment: labelAssignment(swarm, assignment),
    last: outcome,
    stale,
  });
}

/**
 * The assignment whose task label and coordinates the row may show:
 * an allowed status for the phase, from the helper boot that sends this beat.
 */
function labelAssignment(swarm, assignment) {
  const allowed = LABEL_STATUSES[swarm.phase];
  if (!assignment || !allowed || !allowed.has(assignment.status)) return null;
  return isCurrentSlot(assignment, swarm) ? assignment : null;
}

/**
 * Whether the helper was first assigned this slot (`assignedUid`, the "doing"
 * it was assigned by; repeats of the handshake keep it) in the heartbeat's
 * boot and after its last approach the GCS heard (`swarm.approach_seq`). The
 * helper stamps its answers and its beats from one counter, so a slot
 * assigned before that approach belongs to an earlier task.
 */
function isCurrentSlot(entry, swarm) {
  const uid = entry.assignedUid;
  if (!uid || uid.boot_id == null || uid.boot_id !== swarm.boot) return false;
  return swarm.approach_seq == null || uid.msg_seq > swarm.approach_seq;
}

/**
 * The entry a card with NavPy's phase may label: an ASSIGNED or CONFIRMING
 * slot of this vehicle from its current companion boot, never a CONFIRMED one.
 *
 * @param {object} assignments - { [key]: entry } from taskAssignmentState
 * @param {number} sysId
 * @param {object|null} swarm - snapshot `swarm`
 * @returns {object|null}
 */
export function currentTaskSlot(assignments, sysId, swarm) {
  if (!swarm) return null;
  let best = null;
  for (const entry of Object.values(assignments)) {
    if (entry.receiverId !== sysId) continue;
    if (entry.status !== 'assigned' && entry.status !== 'confirming') continue;
    if (!isCurrentSlot(entry, swarm)) continue;
    if (!best || entry.status === 'confirming') best = entry;
  }
  return best;
}

/** AUTO mission still on the pre-search waypoints (corridor, takeoff). */
function beforeSearch(vehicle, wpOffset) {
  return vehicle.mode === 'AUTO' && vehicle.mission_progress > 0
    && (vehicle.mission_progress - wpOffset) < 0;
}

/** Mode, armed state and progress only: IDLE / EN ROUTE / SEARCHING. */
function guessStatusId(vehicle, wpOffset) {
  if (!vehicle.armed) return 'idle';
  if (beforeSearch(vehicle, wpOffset)) return 'en_route';
  if (vehicle.mode === 'AUTO') return 'searching';
  return 'idle';
}

/**
 * Derive the raw status id string from vehicle and assignment state.
 * Checked in priority order — first match wins.
 */
function deriveStatusId(vehicle, assignment, wpOffset, hasPendingConfirm) {
  if (assignment) {
    if (assignment.status === 'confirmed') return 'confirmed';
    if (hasPendingConfirm) return 'confirming';
    // A WAITING helper does not fly the task until its owner applies it.
    if (assignment.status === 'waiting') return 'waiting';
    if (vehicle.mode === 'GUIDED') return 'approaching';
    return 'assigned';
  }

  return guessStatusId(vehicle, wpOffset);
}

const SWARM_BADGES = {
  busy:    { labelKey: 'vehicle.swarmBusy',    titleKey: 'vehicle.swarmBusyTitle',    color: colors.accent },
  unknown: { labelKey: 'vehicle.swarmUnknown', titleKey: 'vehicle.swarmUnknownTitle', color: colors.textDim },
};

/**
 * Badge for the companion's swarm node state (snapshot `swarm`): BUSY UAVs
 * are skipped by task auctions, UNKNOWN once the last beat outlived its TTL.
 * None before the first beat or while FREE, the normal state.
 *
 * @param {object|null} swarm - { state: 'FREE'|'BUSY', boot, seq, stale }
 * @returns {{ labelKey: string, titleKey: string, color: string }|null}
 */
export function computeSwarmBadge(swarm) {
  if (!swarm) return null;
  if (swarm.stale) return SWARM_BADGES.unknown;
  return swarm.state === 'BUSY' ? SWARM_BADGES.busy : null;
}
