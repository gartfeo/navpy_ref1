import { useState, useRef, useCallback, useEffect } from 'react';
import { isHoldKey } from '../utils/armAction';

const noop = () => {};
const NO_OP_HANDLERS = {
  onMouseDown: noop, onMouseUp: noop, onMouseLeave: noop,
  onTouchStart: noop, onTouchEnd: noop, onContextMenu: noop,
  onKeyDown: noop, onKeyUp: noop, onBlur: noop,
};

export default function useLongPress({ onComplete, duration = 1000, enabled = true, resetKey }) {
  const [progress, setProgress] = useState(0);
  const startRef = useRef(null);
  const rafRef = useRef(null);
  const callbackRef = useRef(onComplete);

  useEffect(() => { callbackRef.current = onComplete; }, [onComplete]);

  const cancel = useCallback(() => {
    startRef.current = null;
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

  const start = useCallback(() => {
    startRef.current = performance.now();
    rafRef.current = requestAnimationFrame(tick);
  }, [tick]);

  useEffect(() => () => {
    if (rafRef.current) cancelAnimationFrame(rafRef.current);
  }, []);

  if (!enabled) return { handlers: NO_OP_HANDLERS, progress: 0 };

  return {
    progress,
    handlers: {
      onMouseDown: start,
      onMouseUp: cancel,
      onMouseLeave: cancel,
      onTouchStart: (e) => { e.preventDefault(); start(); },
      onTouchEnd: cancel,
      onContextMenu: (e) => e.preventDefault(),
      // Keyboard press-and-hold. preventDefault suppresses the native button
      // click so a key press never fires on activation — only a completed hold
      // does, mirroring the pointer path. Auto-repeat keydown is ignored while
      // a hold is already running.
      onKeyDown: (e) => { if (isHoldKey(e.key)) { e.preventDefault(); if (startRef.current == null) start(); } },
      onKeyUp: (e) => { if (isHoldKey(e.key)) { e.preventDefault(); cancel(); } },
      onBlur: cancel,
    },
  };
}
