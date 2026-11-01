import { useCallback, useEffect, useRef, useState } from 'react';
import { pendingSummary, confirmWaitSec, CONFIRM_PHASE } from './taskConfirmationState';
import { confirmNow } from '../utils/confirmClock';

/**
 * The task-confirm alert sound (todo 67, gcs_confirm_cue.md decision 2).
 *
 * - Each new round: one beep at once (also for a zero wait).
 * - While a round is LIVE (time left, not fired): one beep every
 *   ALERT_REPEAT_MS; when the soonest has URGENT_LEFT_SEC or less, a double
 *   beep every URGENT_REPEAT_MS instead. Periods count from the last sound,
 *   so state updates never restart them. At most one pattern per tick or
 *   arrival burst; an urgent pattern wins.
 * - Silent with no live round (decided, fired, due, reset, disarm). A
 *   pattern already playing finishes; unmount stops it and closes the
 *   context.
 * - Browsers keep audio suspended until a user gesture: the first
 *   pointerdown or keydown (or unlockSound) resumes it, and the listeners
 *   come back whenever it is suspended again. Nothing is queued meanwhile;
 *   `soundState` tells the chip ('running' | 'suspended' | 'unavailable').
 */
export const ALERT_REPEAT_MS = 5000;
export const URGENT_REPEAT_MS = 2000;
export const URGENT_LEFT_SEC = 10;
/** Arrivals closer together than this share one beep. */
export const ARRIVAL_GAP_MS = 1000;
const TICK_MS = 1000;
const GAIN = 0.3;

/** Tones of each pattern: [frequency Hz, start offset s, duration s]. */
export const ALERT_PATTERNS = Object.freeze({
  beep: [[880, 0, 0.15]],
  urgent: [[1320, 0, 0.1], [1320, 0.2, 0.1]],
});

/**
 * Which pattern is due, or null. `soonestLiveSec` is the soonest live
 * round's whole seconds left (null: no live round); `sinceLastMs` the time
 * since the last sound (Infinity before the first).
 */
export function alertPatternDue({ soonestLiveSec, sinceLastMs, arrival = false }) {
  const live = soonestLiveSec != null;
  if (live && soonestLiveSec <= URGENT_LEFT_SEC && sinceLastMs >= URGENT_REPEAT_MS) return 'urgent';
  if (arrival && sinceLastMs >= ARRIVAL_GAP_MS) return 'beep';
  if (live && soonestLiveSec > URGENT_LEFT_SEC && sinceLastMs >= ALERT_REPEAT_MS) return 'beep';
  return null;
}

function audioContextClass() {
  if (typeof window === 'undefined') return null;
  return window.AudioContext || window.webkitAudioContext || null;
}

export default function useConfirmAlert(pendingConfirms, waitSecFor, fired) {
  const [soundState, setSoundState] = useState(() => (audioContextClass() ? 'suspended' : 'unavailable'));
  const ctxRef = useRef(null);
  const lastSoundAtRef = useRef(-Infinity);
  const oscillatorsRef = useRef(new Set());
  // The tick reads the latest inputs through refs.
  const inputsRef = useRef({ pendingConfirms, waitSecFor, fired });
  inputsRef.current = { pendingConfirms, waitSecFor, fired };

  const ensureContext = useCallback(() => {
    if (ctxRef.current) return ctxRef.current;
    const Ctx = audioContextClass();
    if (!Ctx) return null;
    try {
      const ctx = new Ctx();
      ctxRef.current = ctx;
      ctx.addEventListener?.('statechange', () => setSoundState(ctx.state === 'running' ? 'running' : 'suspended'));
      setSoundState(ctx.state === 'running' ? 'running' : 'suspended');
      return ctx;
    } catch {
      setSoundState('unavailable');
      return null;
    }
  }, []);

  const unlockSound = useCallback(() => {
    const ctx = ensureContext();
    if (!ctx || ctx.state === 'running') return;
    try {
      Promise.resolve(ctx.resume())
        .then(() => setSoundState(ctx.state === 'running' ? 'running' : 'suspended'))
        .catch(() => setSoundState('suspended')); // retried on the next gesture
    } catch {
      setSoundState('suspended');
    }
  }, [ensureContext]);

  const play = useCallback((name) => {
    const ctx = ctxRef.current;
    if (!ctx || ctx.state !== 'running') return; // nothing is queued
    lastSoundAtRef.current = confirmNow();
    try {
      const t0 = ctx.currentTime;
      for (const [freq, offset, dur] of ALERT_PATTERNS[name]) {
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.frequency.value = freq;
        gain.gain.value = GAIN;
        oscillatorsRef.current.add(osc);
        osc.onended = () => oscillatorsRef.current.delete(osc);
        osc.start(t0 + offset);
        osc.stop(t0 + offset + dur);
      }
    } catch {
      // Audio failed mid-pattern; the next due pattern tries again.
    }
  }, []);

  /** Soonest live round's whole seconds left (null: none), from the latest inputs. */
  const soonestLiveSec = (exceptSysId = null) => {
    const { pendingConfirms: pc, waitSecFor: wf, fired: fd } = inputsRef.current;
    const rest = exceptSysId == null ? pc : Object.fromEntries(
      Object.entries(pc || {}).filter(([sid]) => Number(sid) !== exceptSysId),
    );
    const summary = pendingSummary(rest, wf, confirmNow(), fd);
    return summary && summary.phase === CONFIRM_PHASE.LIVE ? summary.secondsLeft : null;
  };

  /**
   * Call from the request handler: UAV `sysId`'s new round arrived. The
   * state update is not applied yet, so the new round's full wait joins the
   * other UAVs' live rounds here (its old round, if any, is replaced).
   */
  const onArrival = useCallback((sysId) => {
    const ctx = ensureContext();
    if (ctx && ctx.state !== 'running') unlockSound(); // works with sticky activation
    const others = soonestLiveSec(sysId);
    const incoming = confirmWaitSec(inputsRef.current.waitSecFor(sysId)).sec;
    const live = [others, incoming > 0 ? incoming : null].filter((s) => s != null);
    const due = alertPatternDue({
      soonestLiveSec: live.length ? Math.min(...live) : null,
      sinceLastMs: confirmNow() - lastSoundAtRef.current,
      arrival: true,
    });
    if (due) play(due);
    // soonestLiveSec reads refs only.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ensureContext, unlockSound, play]);

  // The tick runs while a round is live. Only a change of that fact restarts
  // it, so round, image or param updates never push a due sound back; a
  // round that goes due between renders leaves it idling until the next
  // render (its fire, a decision or a longer wait) says whether to stop.
  const hasLive = soonestLiveSec() != null;
  useEffect(() => {
    if (!hasLive) return undefined;
    const id = setInterval(() => {
      const sec = soonestLiveSec();
      if (sec == null) return;
      const due = alertPatternDue({ soonestLiveSec: sec, sinceLastMs: confirmNow() - lastSoundAtRef.current });
      if (due) play(due);
    }, TICK_MS);
    return () => clearInterval(id);
    // soonestLiveSec reads refs only.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [hasLive, play]);

  // Gesture unlock while not running; back on whenever it suspends again.
  useEffect(() => {
    if (soundState !== 'suspended') return undefined;
    const opts = { capture: true, passive: true };
    window.addEventListener('pointerdown', unlockSound, opts);
    window.addEventListener('keydown', unlockSound, opts);
    return () => {
      window.removeEventListener('pointerdown', unlockSound, opts);
      window.removeEventListener('keydown', unlockSound, opts);
    };
  }, [soundState, unlockSound]);

  // Unmount: stop every oscillator at once and close the context.
  useEffect(() => () => {
    for (const osc of oscillatorsRef.current) {
      try { osc.stop(); } catch { /* already stopped */ }
    }
    oscillatorsRef.current.clear();
    try { ctxRef.current?.close?.(); } catch { /* already closed */ }
    ctxRef.current = null;
  }, []);

  return { soundState, unlockSound, onArrival };
}
