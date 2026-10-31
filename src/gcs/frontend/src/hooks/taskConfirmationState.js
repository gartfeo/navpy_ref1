/**
 * Pure reducer + selectors for per-UAV task confirmation cards.
 *
 * One card per UAV (keyed by sys_id); a new confirm request supersedes the
 * prior card for that UAV. Every transition is task-id-guarded so a stale
 * event for an old task cannot mutate a newer card.
 *
 * Decided states are monotonic, mirroring the vehicle/backend intent:
 *   - pending   -> approved | denied | canceled
 *   - approved  -> canceled (Cancel task requests cancellation; idempotent on approve)
 *   - denied    -> terminal
 *   - canceled  -> terminal
 * A late approve can therefore never un-decide a canceled/denied card.
 *
 * A per-UAV abort is the destructive E-STOP command (force-disarm); its card
 * clears via the task_confirm_reset / disarm_after_guided broadcasts, not through here.
 *
 * Timestamps are passed in by callers (receivedAt / at / now) so this module
 * stays pure and deterministic (no Date.now()). All functions return the same
 * state reference when nothing changes (cheap React bailout).
 */

export const CONFIRM_STATUS = Object.freeze({
  PENDING: 'pending',
  APPROVED: 'approved',
  DENIED: 'denied',
  CANCELED: 'canceled',
});

// Operator action -> the decided status it implies.
const ACTION_STATUS = Object.freeze({
  approve: CONFIRM_STATUS.APPROVED,
  timeout_approve: CONFIRM_STATUS.APPROVED,
  deny: CONFIRM_STATUS.DENIED,
  timeout_deny: CONFIRM_STATUS.DENIED,
  cancel: CONFIRM_STATUS.CANCELED,
});

function isTerminal(status) {
  return status === CONFIRM_STATUS.DENIED || status === CONFIRM_STATUS.CANCELED;
}

/**
 * Resolve the next status when a decision (target status) is applied to a card
 * currently in `current`. Monotonic: terminal stays; approved only progresses
 * to canceled; pending takes the decided status directly.
 */
function nextStatus(current, target) {
  if (isTerminal(current)) return current;
  if (current === CONFIRM_STATUS.APPROVED) {
    return target === CONFIRM_STATUS.APPROVED
      ? CONFIRM_STATUS.APPROVED
      : CONFIRM_STATUS.CANCELED;
  }
  return target; // pending
}

export function confirmRequest(state, data) {
  return {
    ...state,
    [data.sys_id]: {
      taskId: data.task_id,
      // Identity of this confirm round, echoed back with the decision so the
      // backend records the answer against the round the operator saw. The
      // same (sysId, taskId) can be asked again (a bounded re-ask after a
      // timeout); that is a NEW round with a new uid, not this one.
      roundUid: data.round_uid ?? null,
      taskType: data.task_type ?? null,
      lat: data.lat,
      lon: data.lon,
      alt: data.alt ?? null,
      imageB64: null,
      status: CONFIRM_STATUS.PENDING,
      action: null,
      receivedAt: data.receivedAt,
      decidedAt: null,
    },
  };
}

/**
 * True when `data` belongs to a different confirm round than the card holds.
 * Only decidable when BOTH sides carry a uid; either side missing one (an
 * older backend, a sender without dedup metadata) falls back to the task-id
 * guard alone, exactly as before.
 */
function isOtherRound(entry, data) {
  return data.round_uid != null && entry.roundUid != null
    && data.round_uid !== entry.roundUid;
}

export function confirmImage(state, data) {
  const entry = state[data.sys_id];
  if (!entry || entry.taskId !== data.task_id) return state;
  // A thumbnail from a round this card has moved past is not this POI's
  // current picture.
  if (isOtherRound(entry, data)) return state;
  return { ...state, [data.sys_id]: { ...entry, imageB64: data.image_b64 } };
}

export function confirmResponse(state, data) {
  const entry = state[data.sys_id];
  if (!entry || entry.taskId !== data.task_id) return state; // task-id guard
  // A decision for an earlier round must not decide the round on screen: the
  // same POI can be asked again while a response is still being sent, and
  // marking the new card decided would also stop its countdown.
  if (isOtherRound(entry, data)) return state;
  const action = data.action || (data.is_confirmed ? 'approve' : 'deny');
  const target = ACTION_STATUS[action]
    ?? (data.is_confirmed ? CONFIRM_STATUS.APPROVED : CONFIRM_STATUS.DENIED);
  const status = nextStatus(entry.status, target);
  // If the status did not progress, the decision was blocked (terminal card)
  // or idempotent (re-approve) -- ignore it entirely, including action/time,
  // so a late approve cannot rewrite a canceled/denied card's audit fields.
  if (status === entry.status) return state;
  return { ...state, [data.sys_id]: { ...entry, status, action, decidedAt: data.at } };
}

export function disarmAfterGuided(state, data) {
  return removeCard(state, data.sys_id);
}

export function confirmReset(state, data) {
  const ids = data && data.sys_ids;
  if (!ids || ids.length === 0) return state; // no ids -> no-op (never clear-all)
  let changed = false;
  const next = { ...state };
  for (const sid of ids) {
    if (sid in next) {
      delete next[sid];
      changed = true;
    }
  }
  return changed ? next : state;
}

export function removeCard(state, sysId) {
  if (!(sysId in state)) return state;
  const next = { ...state };
  delete next[sysId];
  return next;
}

export function resetAll() {
  return {};
}

/**
 * Remove denied/canceled cards older than ttlMs.
 * Approved cards persist (cleared by disarm_after_guided / cancel / reset instead).
 */
export function expireDecided(state, now, ttlMs) {
  let changed = false;
  const next = { ...state };
  for (const sid of Object.keys(next)) {
    const e = next[sid];
    if ((e.status === CONFIRM_STATUS.DENIED || e.status === CONFIRM_STATUS.CANCELED)
        && e.decidedAt != null && now - e.decidedAt >= ttlMs) {
      delete next[sid];
      changed = true;
    }
  }
  return changed ? next : state;
}

// ---- Selectors ----

export function isPending(entry) {
  return !!entry && entry.status === CONFIRM_STATUS.PENDING;
}

export function isDecided(entry) {
  return !!entry && entry.status !== CONFIRM_STATUS.PENDING;
}

export function canCancel(entry) {
  return !!entry && entry.status === CONFIRM_STATUS.APPROVED;
}

/**
 * Identity of the confirm round an entry shows. The same POI can be asked
 * again (a bounded re-ask), so the task id alone does NOT change between
 * rounds -- the round uid is what makes it a different question. Rounds from
 * a sender without a uid fall back to receivedAt, which a re-ask also
 * refreshes.
 */
export function confirmRoundKey(entry) {
  return `${entry.taskId}:${entry.roundUid ?? entry.receivedAt ?? ''}`;
}

export function hasPendingConfirm(state, sysId) {
  return isPending(state[sysId]);
}

export function getConfirm(state, sysId) {
  return state[sysId] || null;
}

// ---- Deadline, order and summary (todos 67, 68; gcs_confirm_cue.md) ----
//
// The ONE place a confirm round's deadline is computed: the card's countdown
// and auto-fire, the list order, the summary chip and the alert sound all
// use these helpers with a `now` from utils/confirmClock.js, so the sim-time
// move (deterministic_dev_sim.md) changes the clock in one place.

/** The card's wait when the vehicle's NAV_CWT is not loaded or not valid. */
export const DEFAULT_CONFIRM_WAIT_SEC = 30;

/**
 * The wait the GCS counts down for a round: the cached NAV_CWT when it is a
 * finite number 0 or more (0 = no wait), else DEFAULT_CONFIRM_WAIT_SEC,
 * flagged `estimated` so the label can say the time is a guess.
 */
export function confirmWaitSec(waitSec) {
  const known = typeof waitSec === 'number' && Number.isFinite(waitSec) && waitSec >= 0;
  return known ? { sec: waitSec, estimated: false } : { sec: DEFAULT_CONFIRM_WAIT_SEC, estimated: true };
}

/** Epoch ms (confirmNow's clock) at which the GCS's countdown for `entry` ends. */
export function confirmDeadlineMs(entry, waitSec) {
  return entry.receivedAt + confirmWaitSec(waitSec).sec * 1000;
}

/** Milliseconds left until `deadlineMs`, never negative. */
export function msLeft(deadlineMs, now) {
  return Math.max(0, deadlineMs - now);
}

/** Whole seconds left, rounded up: what the card and the chip both show. */
export function secondsLeft(deadlineMs, now) {
  return Math.ceil(msLeft(deadlineMs, now) / 1000);
}

/**
 * Identity of one round of one UAV. confirmRoundKey alone has no UAV in it,
 * so the fired latch, the alert bookkeeping and the chip's scroll target key
 * by this.
 */
export function confirmRoundId(sysId, entry) {
  return roundIdFromKey(sysId, confirmRoundKey(entry));
}

/** confirmRoundId from a round key the caller already has. */
export function roundIdFromKey(sysId, roundKey) {
  return `${sysId}|${roundKey}`;
}

/** The phases of a pending round. */
export const CONFIRM_PHASE = Object.freeze({
  LIVE: 'live', // time left, not fired: the operator can still answer in time
  DUE: 'due', // at 0, the card has not fired yet (between ticks)
  FIRED: 'fired', // the card's auto-fire ran for this round (latched)
});

function pendingPhase(sysId, entry, waitSecFor, now, fired) {
  if (fired && fired[confirmRoundId(sysId, entry)] != null) return CONFIRM_PHASE.FIRED;
  return msLeft(confirmDeadlineMs(entry, waitSecFor(sysId)), now) > 0
    ? CONFIRM_PHASE.LIVE : CONFIRM_PHASE.DUE;
}

const DECIDED_RANK = Object.freeze({
  [CONFIRM_STATUS.APPROVED]: 1,
  [CONFIRM_STATUS.DENIED]: 2,
  [CONFIRM_STATUS.CANCELED]: 2,
});

/**
 * [numericSysId, entry] pairs in display order: pending rounds first -- the
 * fired ones by fire time, then the rest by deadline, soonest first -- then
 * approved, then denied or canceled; ties and decided groups by numeric
 * sys_id. `waitSecFor(sysId)` is the cached NAV_CWT (or undefined); `fired`
 * maps confirmRoundId -> fire time.
 */
export function orderedConfirmEntries(state, waitSecFor, fired = {}) {
  const rows = Object.entries(state || {}).map(([sid, entry]) => {
    const sysId = Number(sid);
    if (!isPending(entry)) return { sysId, entry, rank: DECIDED_RANK[entry.status] ?? 2, at: 0 };
    const firedAt = fired[confirmRoundId(sysId, entry)];
    return firedAt != null
      ? { sysId, entry, rank: -1, at: firedAt }
      : { sysId, entry, rank: 0, at: confirmDeadlineMs(entry, waitSecFor(sysId)) };
  });
  rows.sort((a, b) => (a.rank - b.rank) || (a.at - b.at) || (a.sysId - b.sysId));
  return rows.map((r) => [r.sysId, r.entry]);
}

/**
 * What the summary chip shows, or null when no round is pending. The shown
 * round is the soonest LIVE one; with none, the earliest DUE one; else the
 * first FIRED one. `sysIds` are all pending UAVs in display order, the shown
 * one first.
 */
export function pendingSummary(state, waitSecFor, now, fired = {}) {
  const pending = orderedConfirmEntries(state, waitSecFor, fired)
    .filter(([, entry]) => isPending(entry))
    .map(([sysId, entry]) => ({ sysId, entry, phase: pendingPhase(sysId, entry, waitSecFor, now, fired) }));
  if (pending.length === 0) return null;
  const shown = pending.find((p) => p.phase === CONFIRM_PHASE.LIVE)
    ?? pending.find((p) => p.phase === CONFIRM_PHASE.DUE)
    ?? pending[0];
  const deadlineMs = confirmDeadlineMs(shown.entry, waitSecFor(shown.sysId));
  return {
    count: pending.length,
    sysIds: [shown.sysId, ...pending.filter((p) => p !== shown).map((p) => p.sysId)],
    shownSysId: shown.sysId,
    shownEntry: shown.entry,
    phase: shown.phase,
    deadlineMs,
    secondsLeft: shown.phase === CONFIRM_PHASE.LIVE ? secondsLeft(deadlineMs, now) : 0,
    liveCount: pending.filter((p) => p.phase === CONFIRM_PHASE.LIVE).length,
  };
}

/** m:ss for a whole number of seconds. */
export function formatClock(sec) {
  const s = Math.max(0, Math.floor(sec));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

/**
 * The words for what the GCS sends at zero (gcs_confirm_cue.md decision 5),
 * from the RAW cached params (undefined = not loaded; no AAS_DEFAULTS here).
 * Returns the card's and the chip's locale keys and the `time` argument
 * ("0:21", "~0:21" when the wait is estimated). First match wins.
 */
export function confirmZeroLabel({ phase, cmFl, cwt, seconds }) {
  const { estimated } = confirmWaitSec(cwt);
  const time = `${estimated ? '~' : ''}${formatClock(seconds)}`;
  const same = (k) => ({ cardKey: `task.zero.${k}`, chipKey: `task.zero.${k}`, time });
  if (phase === CONFIRM_PHASE.FIRED) return same('fired');
  const approves = cmFl == null ? null : Boolean(cmFl);
  // A zero wait is due from the start; say why (decision 7) instead of "due".
  if (cwt === 0) return same(approves ? 'noWaitApprove' : 'noWaitDeny');
  if (phase === CONFIRM_PHASE.DUE) return same('due');
  const k = approves == null ? 'autoDenyUnknown' : approves ? 'autoApprove' : 'autoDeny';
  return { cardKey: `task.zero.${k}`, chipKey: `task.zeroChip.${k}`, time };
}
