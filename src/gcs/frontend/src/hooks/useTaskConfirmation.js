import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  confirmRequest,
  confirmImage,
  confirmResponse,
  disarmAfterGuided,
  confirmReset,
  expireDecided,
  resetAll,
  confirmRoundId,
  roundIdFromKey,
} from './taskConfirmationState';
import useConfirmAlert from './useConfirmAlert';
import { confirmNow } from '../utils/confirmClock';

// Decided (denied/canceled) cards stay reviewable for this long, then auto-
// dismiss. Approved cards persist until disarm_after_guided / cancel / reset.
const DECIDED_TTL_MS = 8000;

// Operator action -> the is_confirmed value the vehicle acts on. Mirrors the
// backend table so deny/cancel/timeout_deny all reject and approve confirms.
// ("abort" is NOT a confirm action -- a per-UAV abort is the destructive
// E-STOP command, handled in useVehicleCommands.)
const ACTION_CONFIRMED = {
  approve: true,
  timeout_approve: true,
  deny: false,
  cancel: false,
  timeout_deny: false,
};

const NO_WAIT = () => undefined;

/**
 * Manages task confirmation cards: pending + decided state via the pure
 * taskConfirmationState reducer, image association, approve/deny/cancel
 * actions, lifecycle clears (disarm-after-GUIDED/reset), the fired latch and
 * the alert sound (useConfirmAlert).
 *
 * `waitSecFor(sysId)` is that UAV's cached NAV_CWT (undefined when not
 * loaded); the sound needs it to know the soonest deadline. The returned
 * `cue` is what ConfirmCardList needs for the order, the labels and the chip.
 */
export default function useTaskConfirmation({ waitSecFor = NO_WAIT } = {}) {
  const [pendingConfirms, setPendingConfirms] = useState({});
  const [forcedConfirms, setForcedConfirms] = useState({});
  // Fired latch: confirmRoundId -> the time the card's auto-fire ran for that
  // round. A fired round stays "time up" for the cue even if a later wait or
  // a backward clock jump gives it time again; it never decides the round.
  const [fired, setFired] = useState({});
  const alert = useConfirmAlert(pendingConfirms, waitSecFor, fired);
  const onArrivalRef = useRef(alert.onArrival);
  onArrivalRef.current = alert.onArrival;

  const markFired = useCallback((sysId, roundKey) => {
    const id = roundIdFromKey(sysId, roundKey);
    setFired((prev) => (prev[id] != null ? prev : { ...prev, [id]: confirmNow() }));
  }, []);

  // Drop latches of rounds no longer on screen.
  useEffect(() => {
    setFired((prev) => {
      const current = new Set(Object.entries(pendingConfirms).map(([sid, e]) => confirmRoundId(sid, e)));
      const keys = Object.keys(prev);
      if (keys.every((k) => current.has(k))) return prev;
      const next = {};
      for (const k of keys) if (current.has(k)) next[k] = prev[k];
      return next;
    });
  }, [pendingConfirms]);

  // ---- WS-driven transitions (apply to all clients) ----
  const handleConfirmRequest = useCallback((data) => {
    setPendingConfirms((prev) => confirmRequest(prev, { ...data, receivedAt: confirmNow() }));
    onArrivalRef.current(data.sys_id);
  }, []);

  const handleConfirmImage = useCallback((data) => {
    setPendingConfirms((prev) => confirmImage(prev, data));
  }, []);

  const handleConfirmResponse = useCallback((data) => {
    setPendingConfirms((prev) => confirmResponse(prev, { ...data, at: Date.now() }));
  }, []);

  const handleConfirmReset = useCallback((data) => {
    setPendingConfirms((prev) => confirmReset(prev, data));
    const ids = data && data.sys_ids;
    if (ids && ids.length) {
      setForcedConfirms((prev) => {
        let changed = false;
        const next = { ...prev };
        for (const sid of ids) {
          if (sid in next) { delete next[sid]; changed = true; }
        }
        return changed ? next : prev;
      });
    }
  }, []);

  const handleDisarmAfterGuided = useCallback((data) => {
    setPendingConfirms((prev) => disarmAfterGuided(prev, data));
    setForcedConfirms((prev) => {
      if (!(data.sys_id in prev)) return prev;
      const next = { ...prev };
      delete next[data.sys_id];
      return next;
    });
  }, []);

  // ---- CONF-03 "Ask me anyway" one-shot override (D-18/D-19/D-20) ----
  // forcedConfirms is { [sysId]: taskId } -- the GCS marks a forced POI
  // locally (no new wire field needed; the GCS already knows which POI
  // it forced). Only one card is pending per sysId at a time, so a single
  // taskId per sysId is enough; a later confirm round for a DIFFERENT
  // taskId simply fails a `forcedConfirms[sysId] === taskId` equality check
  // wherever a consumer (e.g. TaskConfirmCard) reads this map.
  const markForced = useCallback((sysId, taskId) => {
    setForcedConfirms((prev) => ({ ...prev, [sysId]: taskId }));
  }, []);

  const forceConfirm = useCallback(async (sysId, taskId) => {
    try {
      const res = await fetch('/api/control/task_confirm_override', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sys_id: sysId, task_id: taskId }),
      });
      if (!res.ok) {
        console.error('Force-confirm override rejected:', res.status);
        return;
      }
    } catch (e) {
      console.error('Force-confirm override failed:', e);
      return;
    }
    markForced(sysId, taskId);
  }, [markForced]);

  // ---- Auto-dismiss decided (denied/canceled) cards after the TTL ----
  useEffect(() => {
    const id = setInterval(() => {
      setPendingConfirms((prev) => expireDecided(prev, Date.now(), DECIDED_TTL_MS));
    }, 1000);
    return () => clearInterval(id);
  }, []);

  // ---- Operator actions (initiating client) ----
  const respond = useCallback(async (sysId, action) => {
    const entry = pendingConfirms[sysId];
    if (!entry) return;
    // Defend the lifecycle even though the UI hides buttons: approve/deny only
    // from pending; cancel only from an approved card.
    if (action === 'cancel') {
      if (entry.status !== 'approved') return;
    } else if (entry.status !== 'pending') {
      return;
    }
    const isConfirmed = ACTION_CONFIRMED[action] ?? false;
    const taskId = entry.taskId;
    // Echo the round uid so the backend binds this answer to the round the
    // operator saw, not to whatever round is open when the response lands.
    const roundUid = entry.roundUid ?? null;
    try {
      const res = await fetch('/api/control/task_confirm', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sys_id: sysId, task_id: taskId, is_confirmed: isConfirmed, action, round_uid: roundUid,
        }),
      });
      if (!res.ok) {
        console.error('Task confirm response rejected:', res.status);
        return; // do not mark decided locally if the backend refused
      }
    } catch (e) {
      console.error('Task confirm response failed:', e);
      return; // network failure -> leave the card as-is
    }
    // Optimistic decided state; the WS broadcast re-applies the same reducer
    // transition (idempotent + monotonic).
    setPendingConfirms((prev) => confirmResponse(prev, {
      sys_id: sysId, task_id: taskId, is_confirmed: isConfirmed, action,
      // The round captured when the operator acted: by the time this resolves
      // the card may already show a NEW round for the same POI, which this
      // decision does not answer.
      round_uid: roundUid, at: Date.now(),
    }));
  }, [pendingConfirms]);

  const approve = useCallback((sysId) => respond(sysId, 'approve'), [respond]);
  const deny = useCallback((sysId) => respond(sysId, 'deny'), [respond]);
  const cancel = useCallback((sysId) => respond(sysId, 'cancel'), [respond]);

  const reset = useCallback(() => {
    setPendingConfirms(resetAll());
    setFired({});
  }, []);

  const cue = useMemo(() => ({
    waitSecFor,
    fired,
    onAutoFire: markFired,
    soundState: alert.soundState,
    unlockSound: alert.unlockSound,
  }), [waitSecFor, fired, markFired, alert.soundState, alert.unlockSound]);

  return {
    pendingConfirms,
    handleConfirmRequest,
    handleConfirmImage,
    handleConfirmResponse,
    handleConfirmReset,
    handleDisarmAfterGuided,
    approve,
    deny,
    cancel,
    reset,
    forcedConfirms,
    forceConfirm,
    cue,
  };
}
