import React from 'react';
import { useTranslation } from 'react-i18next';
import { formatValueForDisplay } from './paramTree';
import { colors } from '../../styles';

// Read-only-until-clicked value cell for the "Sync UAVs" (harmonize) grid. Shows
// each UAV's CURRENT value. Sync colors (green winner / red outlier) only render
// once the row is SELECTED — an unselected row shows plain values, since nothing
// will be written for it. Clicking a present cell in an unselected row picks that
// value as the winner AND selects the row (colors appear); clicking the cell
// that's already the shown winner of a selected row unselects it instead of
// re-picking ("click again to unselect") — see VirtualParamGrid's onPick
// composition. Never shows a draft.

export function HarmonizeValueCell({ rec, cell, state, error = null, width, onPick, isBase = false }) {
  const { t } = useTranslation();
  const loaded = cell ? cell.loaded : false;
  const present = cell ? cell.present : false;
  const isWinner = state ? state.isWinner : false;
  const isOutlier = state ? state.isOutlier : false;

  let text;
  if (!loaded) text = '…';
  else if (!present) text = '—';
  else text = rec ? formatValueForDisplay(rec) : '—';

  let bg = 'transparent';
  let color = present ? colors.text : colors.textDim;
  if (error) { bg = 'rgba(244, 67, 54, 0.14)'; color = colors.error; } else if (isWinner) { bg = 'rgba(76, 175, 80, 0.20)'; color = colors.success; } else if (isOutlier) { bg = 'rgba(244, 67, 54, 0.16)'; color = colors.error; }

  let title;
  if (present && rec) {
    title = isBase
      ? t('settings.parameters.hz_base_cell', 'base — the value the other UAVs sync to')
      : isWinner
        ? t('settings.parameters.hz_winner_cell', 'winner — this value is kept')
        : isOutlier
          ? t('settings.parameters.hz_outlier_cell', 'differs — will be set to the winner')
          : t('settings.parameters.hz_pick_cell', 'click to make this the winner');
  } else {
    title = loaded
      ? t('settings.parameters.not_present', 'not present on this vehicle')
      : t('settings.parameters.loading_cell', 'loading…');
  }
  if (error) title += `\n${t('settings.parameters.apply_error', 'apply failed')}: ${error}`;

  const clickable = present && onPick;
  return (
    <div
      title={title}
      onClick={clickable ? () => onPick(rec.value) : undefined}
      style={{
        width,
        flex: '0 0 auto',
        padding: '0 6px',
        display: 'flex',
        alignItems: 'center',
        overflow: 'hidden',
        whiteSpace: 'nowrap',
        fontFamily: 'monospace',
        fontSize: 12,
        color,
        fontWeight: isBase ? 600 : isWinner ? 500 : 400,
        gap: 4,
        background: bg,
        border: `1px solid ${error ? colors.error : isBase ? colors.success : 'transparent'}`,
        cursor: clickable ? 'pointer' : 'default',
      }}
    >
      {isBase && (
        // Anchor = the base value the other UAVs sync to. Distinguishes the chosen
        // source cell from other cells that merely share the same (green) value.
        <svg
          width="11" height="11" viewBox="0 0 24 24" aria-hidden="true"
          fill="none" stroke={colors.success} strokeWidth="2.2"
          strokeLinecap="round" strokeLinejoin="round" style={{ flex: '0 0 auto' }}
        >
          <circle cx="12" cy="5" r="2.5" />
          <line x1="12" y1="22" x2="12" y2="7.5" />
          <path d="M5 12a7 7 0 0 0 14 0" />
          <line x1="3" y1="12" x2="5.5" y2="12" />
          <line x1="18.5" y1="12" x2="21" y2="12" />
        </svg>
      )}
      <span style={{ overflow: 'hidden', textOverflow: 'ellipsis' }}>{text}</span>
    </div>
  );
}
