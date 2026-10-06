import React from 'react';
import { colors } from '../../styles';

export function SectionTitle({ children }) {
  return (
    <h3 style={{ color: colors.textBright, fontSize: 14, fontWeight: 700, marginBottom: 4 }}>
      {children}
    </h3>
  );
}

export function Divider() {
  return <hr style={{ border: 'none', borderTop: `1px solid ${colors.border}`, margin: '12px 0' }} />;
}

export function Label({ children, style }) {
  return (
    <div style={{ color: colors.textDim, fontSize: 12, fontWeight: 600, marginBottom: 6, ...style }}>
      {children}
    </div>
  );
}

export function InfoRow({ label, value }) {
  return (
    <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
      <span style={{ color: colors.textDim, fontSize: 13 }}>{label}</span>
      <span style={{ color: colors.textBright, fontSize: 13, fontWeight: 600 }}>{value}</span>
    </div>
  );
}

export function StepBtn({ disabled, onClick, children }) {
  return (
    <button
      disabled={disabled}
      onClick={onClick}
      style={{
        background: colors.surface,
        color: disabled ? colors.textDim : colors.textBright,
        border: `1px solid ${colors.border}`,
        borderRadius: 4,
        width: 28,
        height: 28,
        fontSize: 16,
        cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.4 : 1,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
      }}
    >
      {children}
    </button>
  );
}