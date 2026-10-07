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

export function hasPendingConfirm(state, sysId) {
  return isPending(state[sysId]);
}

export function getConfirm(state, sysId) {
  return state[sysId] || null;
}

export function visibleConfirmEntries(state) {
  // [numericSysId, entry] pairs, oldest first by receivedAt.
  return Object.entries(state)
    .map(([sid, e]) => [Number(sid), e])
    .sort((a, b) => ((a[1].receivedAt ?? 0) - (b[1].receivedAt ?? 0)) || (a[0] - b[0]));
}
