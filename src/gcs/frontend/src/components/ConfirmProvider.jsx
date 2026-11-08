import React, { createContext, useCallback, useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import ConfirmModal from './ConfirmModal';

/**
 * The in-page replacement for window.confirm (todo 233). The native dialog
 * blocks the page, so telemetry, the HUD and E-STOP freeze while it is open,
 * and in the handheld app's WebView it brings back Android's navigation bar.
 * Here the page keeps running and only the dialog takes input.
 *
 * Outside a provider, confirm() rejects instead of falling back to the native
 * dialog, so the action is not taken and the missing provider shows up.
 */
const outsideProvider = () => Promise.reject(new Error('confirm() needs a ConfirmProvider'));

export const ConfirmContext = createContext(outsideProvider);

/**
 * A question's Confirm button takes a tap only this long after the question
 * shows: Android's double-tap timeout (ViewConfiguration.DOUBLE_TAP_TIMEOUT,
 * 300 ms), the window in which a second tap still belongs to the first. So the
 * second tap of a double tap, on the button that opened the dialog or on the
 * previous question's Confirm, can never be the yes to a new question.
 * Cancel and Escape work at once.
 */
export const CONFIRM_ARM_MS = 300;

/**
 * Provides `confirm(message, { signal }) => Promise<boolean>`: true on Confirm,
 * false on Cancel, Escape, a tap outside the dialog, or when `signal` aborts
 * (the asking component went away, hooks/useConfirm). Requests made while a
 * dialog is open wait their turn, one dialog at a time, in request order.
 *
 * A request with the same text as one still pending resolves false at once:
 * the pending dialog answers only its own caller. Unlike the native dialog
 * the page keeps running, so a second tap on the same button, or an effect
 * that re-runs on every telemetry update, would otherwise stack copies of one
 * question and act once per copy.
 *
 * When the provider itself unmounts, every pending question resolves false,
 * so no caller waits for an answer that can no longer come.
 */
export default function ConfirmProvider({ children }) {
  const { t } = useTranslation();
  const [queue, setQueue] = useState([]);
  // The pending requests, in order; the state above only renders the first.
  const pending = useRef([]);
  const nextId = useRef(0);

  // Answers a request once and drops it; later answers to it are ignored, so
  // a double tap cannot also answer the next one in line.
  const settle = useCallback((request, ok) => {
    if (!pending.current.includes(request)) return;
    pending.current = pending.current.filter((r) => r !== request);
    setQueue(pending.current);
    request.dispose();
    request.resolve(ok);
  }, []);

  const confirm = useCallback((message, { signal } = {}) => {
    if (signal?.aborted) return Promise.resolve(false);
    if (pending.current.some((r) => r.message === message)) return Promise.resolve(false);
    return new Promise((resolve) => {
      nextId.current += 1;
      const request = { id: nextId.current, message, resolve, dispose: () => {} };
      if (signal) {
        const onAbort = () => settle(request, false);
        signal.addEventListener('abort', onAbort, { once: true });
        request.dispose = () => signal.removeEventListener('abort', onAbort);
      }
      pending.current = [...pending.current, request];
      setQueue(pending.current);
    });
  }, [settle]);

  useEffect(() => () => {
    const open = pending.current;
    pending.current = [];
    open.forEach((r) => { r.dispose(); r.resolve(false); });
  }, []);

  const current = queue[0];

  return (
    <ConfirmContext.Provider value={confirm}>
      {children}
      {current && (
        // A new element per question: a tap on the previous dialog's
        // buttons can never land on the next one's.
        <Question key={current.id} request={current} settle={settle} t={t} />
      )}
    </ConfirmContext.Provider>
  );
}

/** One shown question: Confirm armed after CONFIRM_ARM_MS, Escape cancels. */
function Question({ request, settle, t }) {
  const [armed, setArmed] = useState(false);
  useEffect(() => {
    const timer = setTimeout(() => setArmed(true), CONFIRM_ARM_MS);
    return () => clearTimeout(timer);
  }, []);
  useEffect(() => {
    const onKey = (e) => { if (e.key === 'Escape') settle(request, false); };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [request, settle]);

  return (
    <ConfirmModal
      title={t('confirm.title')}
      message={request.message}
      tone="caution"
      confirmLabel={t('confirm.ok')}
      confirmDisabled={!armed}
      onConfirm={() => { if (armed) settle(request, true); }}
      onCancel={() => settle(request, false)}
    />
  );
}
