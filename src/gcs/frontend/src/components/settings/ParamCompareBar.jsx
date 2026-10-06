import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';

// Compare-mode control bar: shows the loaded file, the diff summary, the
// "skip UAV-specific" and "differs only" toggles, and an Exit button. Apply
// lands here in step 5. Purely presentational — all state lives in ParametersTab.

function Toggle({ checked, onChange, label, title }) {
  return (
    <label
      title={title}
      style={{ display: 'flex', alignItems: 'center', gap: 4, color: colors.text, fontSize: 12 }}
    >
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        style={{ accentColor: colors.accent }}
      />
      {label}
    </label>
  );
}

export default function ParamCompareBar({
  filename,
  counts,
  parseErrors = 0,
  differsOnly,
  onToggleDiffers,
  applyStatus = null,
  onExit,
  children,
}) {
  const { t } = useTranslation();
  const c = counts || {};
  // Row focus now lives in the sidebar (functional priority groups with per-group
  // counts), so the bar just states the fleet-wide totals.
  const summary = [];
  summary.push(`${c.differ || 0} ${t('settings.parameters.cmp_differ', 'differ')}`);
  if (c.fileOnly) summary.push(`${c.fileOnly} ${t('settings.parameters.cmp_file_only', 'file-only')}`);
  if (c.vehicleOnly) {
    summary.push(`${c.vehicleOnly} ${t('settings.parameters.cmp_vehicle_only', 'on vehicle, not in file')}`);
  }
  if (c.duplicates) summary.push(`${c.duplicates} ${t('settings.parameters.cmp_duplicates', 'duplicate')}`);
  if (c.readOnly) summary.push(`${c.readOnly} ${t('settings.parameters.cmp_read_only', 'read-only')}`);
  if (parseErrors) summary.push(`${parseErrors} ${t('settings.parameters.cmp_malformed', 'malformed lines')}`);

  return (
    <div
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: 12,
        flexWrap: 'wrap',
        padding: '7px 10px',
        background: 'rgba(0, 210, 255, 0.10)',
        borderBottom: `1px solid ${colors.border}`,
        fontSize: 12,
      }}
    >
      <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6 }}>
        <span style={{ color: colors.accent }}>
          {t('settings.parameters.comparing', 'Comparing')}
        </span>
        <strong style={{ color: colors.textBright, fontFamily: 'monospace' }}>{filename}</strong>
      </span>
      <span style={{ color: colors.textDim }}>{summary.join(' · ')}</span>
      {applyStatus && (
        <span style={{ color: applyStatus.fail > 0 ? colors.warning : colors.success }}>
          {applyStatus.ok} {t('settings.parameters.cmp_applied', 'applied')}
          {applyStatus.fail > 0
            ? `, ${applyStatus.fail} ${t('settings.parameters.cmp_failed', 'failed')}`
            : ''}
        </span>
      )}
      <span style={{ flex: 1 }} />
      <Toggle
        checked={differsOnly}
        onChange={onToggleDiffers}
        label={t('settings.parameters.differs_only', 'Differs only')}
      />
      {children}
      <button
        onClick={onExit}
        title={t('settings.parameters.exit_compare', 'Exit compare')}
        style={{
          padding: '4px 10px',
          background: colors.surface,
          color: colors.text,
          border: `1px solid ${colors.border}`,
          borderRadius: 4,
          cursor: 'pointer',
        }}
      >
        {t('settings.parameters.exit_compare', 'Exit compare')}
      </button>
    </div>
  );
}
