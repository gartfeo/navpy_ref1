import React from 'react';
import { colors } from '../styles';

/**
 * Rings the control that fixes a problem the UI reports, so the operator
 * learns where it is (the owner prefers this to a one-click fix button). The
 * ring pulses; with reduced motion it stays still. Inactive, it adds nothing
 * but a wrapper that keeps the child's size.
 */
export default function FixHighlight({ active, children, style }) {
  return (
    <span
      data-fix-highlight={active ? 'on' : undefined}
      style={{
        display: 'inline-flex',
        borderRadius: 8,
        ...(active ? {
          boxShadow: `0 0 0 2px ${colors.warning}`,
          animation: 'fixHighlightPulse 1.4s ease-in-out infinite',
        } : null),
        ...style,
      }}
    >
      {active && (
        <style>{`
          @keyframes fixHighlightPulse {
            0%, 100% { box-shadow: 0 0 0 2px ${colors.warning}; }
            50% { box-shadow: 0 0 0 5px ${colors.warning}66; }
          }
          @media (prefers-reduced-motion: reduce) {
            [data-fix-highlight="on"] { animation: none !important; }
          }
        `}</style>
      )}
      {children}
    </span>
  );
}
