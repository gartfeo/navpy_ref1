import React from 'react';
import { colors } from '../../styles';

/**
 * Small SVG donut progress ring. Pure presentation.
 *   - determinate: `percent` 0..100 fills the arc clockwise from 12 o'clock.
 *   - indeterminate (`indeterminate`): a short arc spins continuously, for the
 *     window where work is happening but there is no measurable progress yet
 *     (e.g. MAVFTP session setup before the first byte / before the file size
 *     is known) — so the ring never sits frozen at a static 0%.
 * Used in the Parameters grid column headers to show per-UAV full-param
 * download progress.
 */
export default function CircularProgress({
  percent = 0, color = colors.accent, size = 18, stroke = 4, indeterminate = false,
}) {
  const r = 15; // matches the 36x36 viewBox
  const circ = 2 * Math.PI * r;
  if (indeterminate) {
    const arc = circ * 0.3; // ~30% visible arc that spins
    // CSS keyframes (matching LoadingScreen/TaskConfirmCard) rather than SVG
    // SMIL, so the spin can be disabled under prefers-reduced-motion.
    return (
      <svg
        width={size} height={size} viewBox="0 0 36 36"
        className="cp-spin"
        style={{ flex: '0 0 auto', transformOrigin: 'center' }}
        aria-hidden="true"
      >
        <style>{`
          @keyframes cp-spin { to { transform: rotate(360deg); } }
          .cp-spin { animation: cp-spin 0.8s linear infinite; }
          @media (prefers-reduced-motion: reduce) { .cp-spin { animation: none; } }
        `}</style>
        <circle cx="18" cy="18" r={r} fill="none" stroke={colors.surface} strokeWidth={stroke} />
        <circle
          cx="18" cy="18" r={r}
          fill="none" stroke={color} strokeWidth={stroke} strokeLinecap="round"
          strokeDasharray={`${arc} ${circ - arc}`}
        />
      </svg>
    );
  }
  const pct = Math.max(0, Math.min(100, percent));
  const offset = circ * (1 - pct / 100);
  return (
    <svg width={size} height={size} viewBox="0 0 36 36" style={{ flex: '0 0 auto' }} aria-hidden="true">
      <circle cx="18" cy="18" r={r} fill="none" stroke={colors.surface} strokeWidth={stroke} />
      <circle
        cx="18" cy="18" r={r}
        fill="none" stroke={color} strokeWidth={stroke} strokeLinecap="round"
        strokeDasharray={circ} strokeDashoffset={offset}
        transform="rotate(-90 18 18)"
        style={{ transition: 'stroke-dashoffset 120ms linear' }}
      />
    </svg>
  );
}
