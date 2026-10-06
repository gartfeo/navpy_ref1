import React, { useEffect, useRef, useState } from 'react';
import { colors } from '../../styles';

export const displayDuration = (sev) => {
  if (sev <= 2) return 15000;   // EMERG, ALERT, CRIT
  if (sev <= 3) return 10000;   // ERR
  if (sev <= 4) return 7000;    // WARN
  return 5000;                   // NOTICE, INFO, DEBUG
};

const severityColor = (sev) => {
  if (sev <= 2) return '#ff4444';        // EMERG, ALERT, CRIT — bright red
  if (sev <= 3) return colors.error;     // ERR — red
  if (sev <= 4) return colors.warning;   // WARN — yellow/orange
  return colors.textBright;              // NOTICE, INFO, DEBUG — white (visible)
};

export default React.memo(function StatusStrip({ statusTexts }) {
  const [messages, setMessages] = useState([]);
  const lastTsRef = useRef(0);

  // Accumulate new status texts with per-message expiry timestamps
  useEffect(() => {
    if (!statusTexts || statusTexts.length === 0) return;
    const newMsgs = statusTexts.filter(
      (m) => m.ts > lastTsRef.current && !(m.text && m.text.toLowerCase().includes('mode not armable'))
    );
    if (newMsgs.length === 0) return;
    lastTsRef.current = newMsgs[newMsgs.length - 1].ts;
    const now = Date.now();
    const stamped = newMsgs.map((m) => ({
      ...m,
      _expiresAt: now + displayDuration(m.severity),
    }));
    setMessages((prev) => [...prev, ...stamped].slice(-5));
  }, [statusTexts]);

  // Sweep expired messages every second while messages exist
  useEffect(() => {
    if (messages.length === 0) return;
    const id = setInterval(() => {
      setMessages((prev) => {
        const alive = prev.filter((m) => m._expiresAt > Date.now());
        return alive.length === prev.length ? prev : alive;
      });
    }, 1000);
    return () => clearInterval(id);
  }, [messages.length > 0]);

  if (messages.length === 0) return null;

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      gap: 2,
      fontFamily: '"Consolas", "Monaco", "Courier New", monospace',
    }}>
      {messages.map((msg, i) => (
        <div key={`${msg.ts}-${i}`} style={{
          fontSize: 11,
          fontWeight: 600,
          color: severityColor(msg.severity),
          textShadow: '0 0 4px rgba(0,0,0,0.8)',
          whiteSpace: 'nowrap',
        }}>
          [{msg.label}] {msg.text}
        </div>
      ))}
    </div>
  );
});
