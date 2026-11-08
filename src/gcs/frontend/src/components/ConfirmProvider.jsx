import React, { createContext, useCallback, useRef, useState } from 'react';
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
 * Provides `confirm(message) => Promise<boolean>`: true on Confirm, false on
 * Cancel or a click outside the dialog. Requests made while a dialog is open
 * wait their turn, one dialog at a time, in request order.
 *
 * A request with the same text as one still pending resolves false at once:
 * the pending dialog answers only its own caller. Unlike the native dialog
 * the page keeps running, so a second tap on the same button, or an effect
 * that re-runs on every telemetry update, would otherwise stack copies of one
 * question and act once per copy.
 */
export default function ConfirmProvider({ children }) {
  const { t } = useTranslation();
  const [queue, setQueue] = useState([]);
  // The pending requests, in order; the state above only renders the first.
  const pending = useRef([]);
  const nextId = useRef(0);

  const confirm = useCallback((message) => {
    if (pending.current.some((r) => r.message === message)) return Promise.resolve(false);
    return new Promise((resolve) => {
      nextId.current += 1;
      pending.current = [...pending.current, { id: nextId.current, message, resolve }];
      setQueue(pending.current);
    });
  }, []);

  const current = queue[0];
  // Settles only the request it was shown for, and only once, so a double
  // tap cannot also answer the next one in line.
  const settle = (request, ok) => {
    if (!pending.current.includes(request)) return;
    pending.current = pending.current.filter((r) => r !== request);
    setQueue(pending.current);
    request.resolve(ok);
  };

  return (
    <ConfirmContext.Provider value={confirm}>
      {children}
      {current && (
        // A new element per question: a tap on the previous dialog's
        // buttons can never land on the next one's.
        <ConfirmModal
          key={current.id}
          title={t('confirm.title')}
          message={current.message}
          tone="caution"
          confirmLabel={t('confirm.ok')}
          onConfirm={() => settle(current, true)}
          onCancel={() => settle(current, false)}
        />
      )}
    </ConfirmContext.Provider>
  );
}
