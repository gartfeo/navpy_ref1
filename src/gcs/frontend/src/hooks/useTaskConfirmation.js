import { useCallback, useEffect, useRef, useState } from 'react';
import {
  confirmRequest,
  confirmImage,
  confirmResponse,
  disarmAfterGuided,
  confirmReset,
  expireDecided,
  resetAll,
} from './taskConfirmationState';

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

/**
 * Manages task confirmation cards: pending + decided state via the pure
 * taskConfirmationState reducer, image association, approve/deny/cancel
 * actions, lifecycle clears (disarm-after-GUIDED/reset), and audio alerts.
 */
export default function useTaskConfirmation() {
  const [pendingConfirms, setPendingConfirms] = useState({});
  const [forcedConfirms, setForcedConfirms] = useState({});
  const audioCtxRef = useRef(null);

  const playAlertBeep = useCallback(() => {
    try {
      const ctx = audioCtxRef.current || new (window.AudioContext || window.webkitAudioContext)();
      audioCtxRef.current = ctx;
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.frequency.value = 880;
      gain.gain.value = 0.3;
      osc.start();
      osc.stop(ctx.currentTime + 0.15);
    } catch {
      // Audio not available
    }
  }, []);

  // ---- WS-driven transitions (apply to all clients) ----
  const handleConfirmRequest = useCallback((data) => {
    setPendingConfirms((prev) => confirmRequest(prev, { ...data, receivedAt: Date.now() }));
    playAlertBeep();
  }, [playAlertBeep]);

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
      const res = await fetch('/api/control/task_force_confirm', {
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

  const reset = useCallback(() => setPendingConfirms(resetAll()), []);

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
  };
}
