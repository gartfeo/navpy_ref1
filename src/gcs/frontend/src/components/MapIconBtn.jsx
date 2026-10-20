import React from 'react';
import { colors } from '../styles';

/**
 * The app's 32 px map button. Any other prop (aria-*, data-*, ref) goes to
 * the button as is; the styles stay these.
 *
 * - `joined`: part of a bordered group that draws the outer border; the
 *   button has no border or radius of its own, only a divider on its left,
 *   and the pressed ring is drawn inside.
 * - `unavailable`: dimmed like disabled but still focusable and clickable,
 *   for a control that says why it cannot act.
 */
export default function MapIconBtn({
  children, onClick, disabled, title, color, active, joined = false, unavailable = false, ...rest
}) {
  const dim = disabled || unavailable;
  const borderColor = active ? colors.accent : colors.border;
  return (
    <button
      type="button"
      {...rest}
      onClick={onClick}
      disabled={disabled}
      title={title}
      style={{
        width: 32,
        height: 32,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        background: active ? 'rgba(0, 210, 255, 0.2)' : 'rgba(22, 33, 62, 0.85)',
        color: dim ? colors.textDim : (color || (active ? colors.accent : colors.textBright)),
        ...(joined
          ? {
            border: 'none',
            borderLeft: `1px solid ${colors.border}`,
            borderRadius: 0,
            boxShadow: active ? `inset 0 0 0 1px ${colors.accent}` : 'none',
          }
          : { border: `1px solid ${borderColor}`, borderRadius: 6 }),
        cursor: dim ? 'not-allowed' : 'pointer',
        opacity: dim ? 0.7 : 1,
        backdropFilter: 'blur(4px)',
        padding: 0,
      }}
    >
      {children}
    </button>
  );
}
