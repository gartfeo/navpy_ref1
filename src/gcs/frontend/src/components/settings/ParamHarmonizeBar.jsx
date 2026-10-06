import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';

// Control bar for "Sync UAVs" (harmonize) mode: the divergence summary, the
// "skip UAV-specific" toggle, an all-loaded warning, the Harmonize button (passed
// as children so ParametersTab owns the armed long-press), and Exit. Purely
// presentational.

export default function ParamHarmonizeBar({
  counts,
  allLoaded = true,
  loadedCount,
  totalCount,
  skipUavSpecific,
  onToggleSkip,
  applyStatus = null,
  onExit,
  children,
}) {
  const { t } = useTranslation();
  const c = counts || {};
  const summary = [];
  summary.push(`${c.divergent || 0} ${t('settings.parameters.hz_divergent', 'differ across UAVs')}`);
  if (c.skippedDivergent) {
    summary.push(`${c.skippedDivergent} ${t('settings.parameters.cmp_skipped', 'identity/calib skipped')}`);
  }
  if (c.noMajority) summary.push(`${c.noMajority} ${t('settings.parameters.hz_no_majority', 'no majority')}`);
  if (c.partial) summary.push(`${c.partial} ${t('settings.parameters.hz_partial', 'partial')}`);
  if (c.typeMismatch) summary.push(`${c.typeMismatch} ${t('settings.parameters.hz_type_mismatch', 'type mismatch')}`);
  if (c.readOnly) summary.push(`${c.readOnly} ${t('settings.parameters.cmp_read_only', 'read-only')}`);

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
        <span style={{ color: colors.accent }}>{t('settings.parameters.hz_title', 'Sync UAVs')}</span>
      </span>
      <span style={{ color: colors.textDim }}>{summary.join(' · ')}</span>
      {!allLoaded && (
        <span
          title={t('settings.parameters.hz_wait_title', 'Sync waits for every connected UAV so the fleet comparison is complete')}
          style={{ color: colors.warning }}
        >
          {t('settings.parameters.hz_waiting', 'waiting for UAV snapshots')}
          {typeof loadedCount === 'number' && typeof totalCount === 'number'
            ? ` (${loadedCount}/${totalCount})` : ''}
        </span>
      )}
      {applyStatus && (
        <span style={{ color: applyStatus.fail > 0 ? colors.warning : colors.success }}>
          {applyStatus.ok} {t('settings.parameters.cmp_applied', 'applied')}
          {applyStatus.fail > 0
            ? `, ${applyStatus.fail} ${t('settings.parameters.cmp_failed', 'failed')}`
            : ''}
        </span>
      )}
      <span style={{ flex: 1 }} />
      <label style={{ display: 'flex', alignItems: 'center', gap: 4, color: colors.text, fontSize: 12 }}>
        <input
          type="checkbox"
          checked={skipUavSpecific}
          onChange={(e) => onToggleSkip(e.target.checked)}
          style={{ accentColor: colors.accent }}
        />
        {t('settings.parameters.skip_uav_specific', 'Skip UAV-specific')}
      </label>
      {children}
      <button
        onClick={onExit}
        title={t('settings.parameters.exit_sync', 'Exit sync')}
        style={{
          padding: '4px 10px',
          background: colors.surface,
          color: colors.text,
          border: `1px solid ${colors.border}`,
          borderRadius: 4,
          cursor: 'pointer',
        }}
      >
        {t('settings.parameters.exit_sync', 'Exit sync')}
      </button>
    </div>
  );
}
