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
 * Confirm takes only a click that is not part of a double click or double
 * tap: the browser counts consecutive clicks in `event.detail` by the
 * platform's own rule (the OS double-click time on a PC, the double-tap
 * timeout on Android; measured on the UniRC, 2026-11-08: 2 for a second tap
 * within it, 1 after), and 0 for the keyboard. So the second click of a double
 * click, on the button that opened the dialog or on the previous question's
 * Confirm, can never be the yes to a new question. Cancel and Escape take any.
 */
export const isSingleClick = (event) => (event?.detail ?? 0) <= 1;

/**
 * Provides `confirm(message, { signals }) => Promise<boolean>`: true on
 * Confirm, false on Cancel, Escape, a tap outside the dialog, or when any of
 * `signals` aborts (the asking component went away, or the control the
 * question is for; hooks/useConfirm). Requests made while a dialog is open
 * wait their turn, one dialog at a time, in request order.
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

  const confirm = useCallback((message, { signals = [] } = {}) => {
    const watched = signals.filter(Boolean);
    if (watched.some((s) => s.aborted)) return Promise.resolve(false);
    if (pending.current.some((r) => r.message === message)) return Promise.resolve(false);
    return new Promise((resolve) => {
      nextId.current += 1;
      const request = { id: nextId.current, message, resolve };
      const onAbort = () => settle(request, false);
      watched.forEach((s) => s.addEventListener('abort', onAbort, { once: true }));
      // Settled any way, the request stops listening, so a long-lived signal
      // (App's mount) keeps no listener per question asked.
      request.dispose = () => watched.forEach((s) => s.removeEventListener('abort', onAbort));
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

/** One shown question: Confirm takes a single click only, Escape cancels. */
function Question({ request, settle, t }) {
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
      onConfirm={(event) => { if (isSingleClick(event)) settle(request, true); }}
      onCancel={() => settle(request, false)}
    />
  );
}
