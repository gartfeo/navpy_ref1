import { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';

/** How long after a move a press that begins is refused: Chromium's dialog guard (gcs_press_guard.md). */
export const PRESS_GUARD_MS = 500;
/** At this time left or less the cooldown is off: a refused press may have no time to be repeated. */
export const EXPIRY_OPEN_MS = 2000;
/** Absorbs the fractions in rectangles and scrollTop during a scroll. */
const MOVE_EPSILON_PX = 1;
/** How long a refused button says to press again. */
const REFUSED_LABEL_MS = 1500;

const differs = (a, b) => Math.abs(a - b) >= MOVE_EPSILON_PX;

function readPlace(row, scroller) {
  const rect = row.getBoundingClientRect();
  return {
    top: rect.top,
    left: rect.left,
    width: rect.width,
    height: rect.height,
    scrollTop: scroller?.scrollTop ?? 0,
    scrollHeight: scroller?.scrollHeight ?? 0,
  };
}

/**
 * Compare a reading with the baseline. Returns whether the row moved and the
 * next baseline. Top moved by exactly the scroll, with the scroll length
 * unchanged, is the operator's scroll; top held is scroll anchoring, so the
 * scroll is taken into the baseline. A position difference under the epsilon
 * stays in the baseline, so slow drift adds up.
 */
export function comparePlace(base, next) {
  const resized = differs(base.left, next.left) || differs(base.width, next.width)
    || differs(base.height, next.height);
  const topMoved = differs(base.top, next.top);
  // The row's place in the scrolled content: unchanged by a scroll.
  const contentChanged = differs(base.top + base.scrollTop, next.top + next.scrollTop)
    || differs(base.scrollHeight, next.scrollHeight);
  if (resized || (topMoved && contentChanged)) return { moved: true, base: next };
  return {
    moved: false,
    base: { ...base, top: topMoved ? next.top : base.top, scrollTop: next.scrollTop, scrollHeight: next.scrollHeight },
  };
}

/** A click no pointer made: keyboard, or assistive input (React Aria's isVirtualEvent). */
const isVirtualClick = (e) => e.detail === 0 || e.buttons === 1;

/**
 * Refuses a pointer click on a pending card's Approve or Deny when the button
 * moved or appeared under it (todo 166, gcs_press_guard.md). A press must begin
 * on the clicked button, and at least PRESS_GUARD_MS after the last move;
 * the second part is off while `isExpiring()` holds.
 *
 * Returns the row's ref, `clickCapture(name)` for each button's onClickCapture,
 * `dimmed`, the `refused` button's name, and the `moves` count.
 */
export default function usePressGuard({ active, roundKey, scrollRef = null, isExpiring }) {
  const rowRef = useRef(null);
  const baseRef = useRef(null);
  const lastMoveRef = useRef(0);
  const pressRef = useRef(null);
  const coolTimerRef = useRef(null);
  const refusedTimerRef = useRef(null);
  const isExpiringRef = useRef(isExpiring);
  isExpiringRef.current = isExpiring;
  const [cooling, setCooling] = useState(true);
  const [moves, setMoves] = useState(0);
  const [refused, setRefused] = useState(null);

  const markMoved = useCallback(() => {
    lastMoveRef.current = performance.now();
    setMoves((n) => n + 1);
    setCooling(true);
    clearTimeout(coolTimerRef.current);
    coolTimerRef.current = setTimeout(() => setCooling(false), PRESS_GUARD_MS);
  }, []);

  const check = useCallback(() => {
    if (!rowRef.current) return;
    const next = readPlace(rowRef.current, scrollRef?.current);
    const { moved, base } = comparePlace(baseRef.current ?? next, next);
    baseRef.current = base;
    if (moved) markMoved();
  }, [scrollRef, markMoved]);

  // The buttons appear with the card, and a new round is a new question.
  useLayoutEffect(() => {
    if (!active) return;
    baseRef.current = null;
    pressRef.current = null;
    check();
    markMoved();
  }, [active, roundKey, check, markMoved]);

  useEffect(() => {
    if (!active) return undefined;
    let frame = requestAnimationFrame(function tick() {
      check();
      frame = requestAnimationFrame(tick);
    });
    // Every press replaces the record, so one begun elsewhere never inherits it.
    const onPointerDown = (e) => {
      check();
      const button = e.target.closest?.('button');
      pressRef.current = button && rowRef.current?.contains(button)
        ? { button, at: e.timeStamp } : null;
    };
    const onPointerCancel = () => { pressRef.current = null; };
    document.addEventListener('pointerdown', onPointerDown, true);
    document.addEventListener('pointercancel', onPointerCancel, true);
    return () => {
      cancelAnimationFrame(frame);
      document.removeEventListener('pointerdown', onPointerDown, true);
      document.removeEventListener('pointercancel', onPointerCancel, true);
    };
  }, [active, check]);

  useEffect(() => () => {
    clearTimeout(coolTimerRef.current);
    clearTimeout(refusedTimerRef.current);
  }, []);

  const clickCapture = (name) => (e) => {
    if (isVirtualClick(e)) return;
    check();
    const press = pressRef.current;
    pressRef.current = null;
    const sameGesture = press?.button === e.currentTarget;
    if (sameGesture && (isExpiringRef.current() || press.at - lastMoveRef.current >= PRESS_GUARD_MS)) return;
    e.stopPropagation();
    setRefused(name);
    clearTimeout(refusedTimerRef.current);
    refusedTimerRef.current = setTimeout(() => setRefused(null), REFUSED_LABEL_MS);
  };

  return { rowRef, clickCapture, dimmed: cooling && !isExpiring(), refused, moves };
}
