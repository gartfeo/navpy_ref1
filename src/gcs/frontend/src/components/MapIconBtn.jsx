import React from 'react';
import { colors } from '../styles';

export default function MapIconBtn({ children, onClick, disabled, title, color, active }) {
  return (
    <button
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
        color: disabled ? colors.textDim : (color || (active ? colors.accent : colors.textBright)),
        border: `1px solid ${active ? colors.accent : colors.border}`,
        borderRadius: 6,
        cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.7 : 1,
        backdropFilter: 'blur(4px)',
        padding: 0,
      }}
    >
      {children}
    </button>
  );
}
