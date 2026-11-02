import { useState, useRef, useCallback, useEffect } from 'react';
import { isHoldKey } from '../utils/armAction';

const noop = () => {};
const NO_OP_HANDLERS = {
  onMouseDown: noop, onMouseMove: noop, onMouseUp: noop, onMouseLeave: noop,
  onTouchStart: noop, onTouchMove: noop, onTouchEnd: noop, onTouchCancel: noop, onContextMenu: noop,
  onKeyDown: noop, onKeyUp: noop, onBlur: noop,
};

/**
 * How far past its button a press still counts: Android's touch slop, which its
 * own views allow for the same check. A finger is not a point, and one that rolls
 * a little past the edge must not end the hold.
 */
const PRESS_SLOP_PX = 8;

/**
 * The press a mouse or touch event starts; null for a key press. changedTouches is
 * the finger that pressed; touches also lists one resting on a stick or the map.
 */
function pressOf(e) {
  const touch = e?.changedTouches?.[0];
  const p = touch ?? e;
  if (typeof p?.clientX !== 'number') return null;
  return { touchId: touch?.identifier ?? null, x: p.clientX, y: p.clientY };
}

/** The press after a move event: another finger moving (a thumb on a stick) leaves it as it was. */
function movedPress(press, e) {
  const p = press.touchId == null
    ? e
    : Array.from(e.changedTouches ?? []).find((t) => t.identifier === press.touchId);
  return typeof p?.clientX === 'number' ? { ...press, x: p.clientX, y: p.clientY } : press;
}

/** Whether a hold can go on: the element still has a layout box, and still lies under the press. */
function stillPressed({ el, press }) {
  if (el.getClientRects().length === 0) return false;
  if (!press) return true;
  const box = el.getBoundingClientRect();
  return press.x >= box.left - PRESS_SLOP_PX && press.x <= box.right + PRESS_SLOP_PX
    && press.y >= box.top - PRESS_SLOP_PX && press.y <= box.bottom + PRESS_SLOP_PX;
}

export default function useLongPress({ onComplete, duration = 1000, enabled = true, resetKey }) {
  const [progress, setProgress] = useState(0);
  const startRef = useRef(null);
  const rafRef = useRef(null);
  const callbackRef = useRef(onComplete);
  // The pressed element and the press, while it had a layout box when pressed. A pressed
  // button has one in a browser; jsdom has none, so a test that does not stub getClientRects skips the check.
  const pressedRef = useRef(null);

  useEffect(() => { callbackRef.current = onComplete; }, [onComplete]);

  const cancel = useCallback(() => {
    startRef.current = null;
    pressedRef.current = null;
    if (rafRef.current) cancelAnimationFrame(rafRef.current);
    rafRef.current = null;
    setProgress(0);
  }, []);

  // Abort any in-flight hold when the gesture is disabled or its meaning
  // changes (resetKey). Without this, a running animation frame keeps ticking
  // past a disable/re-enable blip or a mode flip and can complete the callback
  // without a fresh, uninterrupted hold in a single stable mode.
  useEffect(() => { cancel(); }, [enabled, resetKey, cancel]);

  const tick = useCallback(() => {
    if (startRef.current == null) return;
    // An ancestor went display: none and the element is still mounted (the
    // handheld panel collapsing), or the element moved from under the finger
    // (a confirm round arriving above it), or the finger moved off: the operator
    // no longer has this hold in hand, and a touch gets no leave event for any.
    if (pressedRef.current && !stillPressed(pressedRef.current)) {
      cancel();
      return;
    }
    const elapsed = performance.now() - startRef.current;
    const p = Math.min(elapsed / duration, 1);
    setProgress(p);
    if (p >= 1) {
      cancel();
      callbackRef.current?.();
    } else {
      rafRef.current = requestAnimationFrame(tick);
    }
  }, [duration, cancel]);

  const start = useCallback((e) => {
    const el = e?.currentTarget;
    pressedRef.current = el?.getClientRects?.().length > 0 ? { el, press: pressOf(e) } : null;
    startRef.current = performance.now();
    rafRef.current = requestAnimationFrame(tick);
  }, [tick]);

  const track = useCallback((e) => {
    const pressed = pressedRef.current;
    if (pressed?.press) pressed.press = movedPress(pressed.press, e);
  }, []);

  useEffect(() => () => {
    if (rafRef.current) cancelAnimationFrame(rafRef.current);
  }, []);

  if (!enabled) return { handlers: NO_OP_HANDLERS, progress: 0 };

  return {
    progress,
    handlers: {
      onMouseDown: start,
      onMouseMove: track,
      onMouseUp: cancel,
      onMouseLeave: cancel,
      onTouchStart: (e) => { e.preventDefault(); start(e); },
      onTouchMove: track,
      onTouchEnd: cancel,
      onTouchCancel: cancel,
      onContextMenu: (e) => e.preventDefault(),
      // Keyboard press-and-hold. preventDefault suppresses the native button
      // click so a key press never fires on activation — only a completed hold
      // does, mirroring the pointer path. Auto-repeat keydown is ignored while
      // a hold is already running.
      onKeyDown: (e) => { if (isHoldKey(e.key)) { e.preventDefault(); if (startRef.current == null) start(e); } },
      onKeyUp: (e) => { if (isHoldKey(e.key)) { e.preventDefault(); cancel(); } },
      onBlur: cancel,
    },
  };
}
