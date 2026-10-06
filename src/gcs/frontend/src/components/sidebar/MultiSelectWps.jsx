import React, { useState, useRef, useEffect } from 'react';
import { colors } from '../../styles';

/** Multiselect dropdown for track waypoint selection. */
export default function MultiSelectWps({ trackLen, selected, onToggle }) {
  const [open, setOpen] = useState(false);
  const ref = useRef(null);

  useEffect(() => {
    if (!open) return;
    const handleClick = (e) => {
      if (ref.current && !ref.current.contains(e.target)) setOpen(false);
    };
    document.addEventListener('mousedown', handleClick);
    return () => document.removeEventListener('mousedown', handleClick);
  }, [open]);

  const count = selected.length;
  const summary = count > 0 ? `${count} WP${count !== 1 ? 's' : ''}` : '\u2014';

  return (
    <div ref={ref} style={{ position: 'relative' }}>
      <button
        onClick={() => setOpen((o) => !o)}
        style={{
          width: '100%',
          background: colors.surface,
          color: count > 0 ? colors.textBright : colors.textDim,
          border: `1px solid ${colors.border}`,
          borderRadius: 4,
          padding: '3px 6px',
          fontSize: 12,
          cursor: 'pointer',
          textAlign: 'left',
        }}
      >
        {summary}
      </button>
      {open && (
        <div style={{
          position: 'absolute',
          top: '100%',
          left: 0,
          right: 0,
          maxHeight: 150,
          overflowY: 'auto',
          background: colors.surface,
          border: `1px solid ${colors.border}`,
          borderRadius: 4,
          zIndex: 100,
          padding: 4,
        }}>
          {Array.from({ length: trackLen }, (_, wi) => (
            <label key={wi} style={{
              display: 'flex', alignItems: 'center', gap: 4,
              padding: '2px 4px', cursor: 'pointer',
              color: colors.text, fontSize: 12,
            }}>
              <input
                type="checkbox"
                checked={selected.includes(wi)}
                onChange={() => onToggle(wi)}
                style={{ accentColor: colors.accent }}
              />
              WP {wi + 1}
            </label>
          ))}
        </div>
      )}
    </div>
  );
}
