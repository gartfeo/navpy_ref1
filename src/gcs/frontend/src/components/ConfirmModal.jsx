import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';

// Accent per confirmation tone: 'danger' (E-STOP, red) is the default;
// 'caution' (container launch, reboot, amber) reads as a deliberate
// high-consequence action without borrowing E-STOP's stop-red semantics.
const TONE_ACCENT = {
  danger: colors.error,
  caution: colors.warning,
};

/**
 * Confirmation modal for high-consequence actions (E-STOP, reboot, container
 * launch). `confirmLabel` overrides the confirm button text (defaults to the
 * E-STOP label); `tone` selects the accent colour.
 */
export default function ConfirmModal({ title, message, onConfirm, onCancel, confirmLabel, tone = 'danger' }) {
  const { t } = useTranslation();
  const accent = TONE_ACCENT[tone] || colors.error;
  const confirmColor = tone === 'caution' ? '#000' : '#fff';
  return (
    <div
      style={{
        position: 'fixed',
        inset: 0,
        background: 'rgba(0,0,0,0.7)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 9999,
      }}
      onClick={onCancel}
    >
      <div
        style={{
          background: colors.bgLight,
          border: `2px solid ${accent}`,
          borderRadius: 8,
          padding: 32,
          minWidth: 320,
          textAlign: 'center',
        }}
        onClick={(e) => e.stopPropagation()}
      >
        <h2 style={{ color: accent, marginBottom: 12, fontSize: 20 }}>{title}</h2>
        <p style={{ color: colors.text, marginBottom: 24, fontSize: 14 }}>{message}</p>
        <div style={{ display: 'flex', gap: 12, justifyContent: 'center' }}>
          <button
            onClick={onCancel}
            style={{
              background: colors.surface,
              color: colors.text,
              border: `1px solid ${colors.border}`,
              borderRadius: 4,
              padding: '8px 24px',
              cursor: 'pointer',
              fontSize: 14,
            }}
          >
            {t('confirm.cancel')}
          </button>
          <button
            onClick={onConfirm}
            style={{
              background: accent,
              color: confirmColor,
              border: 'none',
              borderRadius: 4,
              padding: '8px 24px',
              fontWeight: 700,
              cursor: 'pointer',
              fontSize: 14,
            }}
          >
            {confirmLabel ?? t('confirm.confirmEstop')}
          </button>
        </div>
      </div>
    </div>
  );
}
