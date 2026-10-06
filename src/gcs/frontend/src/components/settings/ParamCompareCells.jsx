import React from 'react';
import { useTranslation } from 'react-i18next';
import { formatValueForDisplay } from './paramTree';
import { colors } from '../../styles';

// Read-only presentational cells for the Parameters compare view. The compare
// value cell shows the vehicle's CURRENT value (never a draft) so the displayed
// number always agrees with the compare model; the File cell shows the loaded
// file's value; the name tags surface the skip/keep/file-only classification.

function fmtFileValue(v) {
  if (typeof v !== 'number' || !Number.isFinite(v)) return String(v);
  if (Number.isInteger(v)) return String(v);
  return Number(v.toPrecision(6)).toString();
}

// Icon-only classification badges (14px inline SVGs) rendered in the Name cell.
// Text pills were replaced by icons because the Name column is narrow (220px):
// a param name plus TWO full-text pills (e.g. a row that's both identity/calib
// AND no-majority) squeezed the name to zero width and hid it entirely. Icons
// leave the name fully visible; the label + explanation live in the hover
// tooltip and the count in the bar. currentColor drives the icon color.
const TAG_ICON = {
  lock: (
    <svg width="13" height="13" viewBox="0 0 16 16" aria-hidden="true" fill="none">
      <rect x="3.5" y="7" width="9" height="6.5" rx="1.2" fill="currentColor" />
      <path d="M5 7 V5 a3 3 0 0 1 6 0 V7" fill="none" stroke="currentColor" strokeWidth="1.4" />
    </svg>
  ),
  plug: (
    <svg width="13" height="13" viewBox="0 0 16 16" aria-hidden="true" fill="none"
      stroke="currentColor" strokeWidth="1.4" strokeLinecap="round">
      <line x1="6" y1="2" x2="6" y2="5" />
      <line x1="10" y1="2" x2="10" y2="5" />
      <path d="M4 5 h8 v2 a4 4 0 0 1 -8 0 z" />
      <line x1="8" y1="11" x2="8" y2="14" />
    </svg>
  ),
  file: (
    <svg width="13" height="13" viewBox="0 0 16 16" aria-hidden="true" fill="none"
      stroke="currentColor" strokeWidth="1.3" strokeLinejoin="round">
      <path d="M4 2 h5 l3 3 v9 h-8 z" />
      <path d="M9 2 v3 h3" />
    </svg>
  ),
  neq: (
    <svg width="13" height="13" viewBox="0 0 16 16" aria-hidden="true" fill="none"
      stroke="currentColor" strokeWidth="1.6" strokeLinecap="round">
      <line x1="3.5" y1="6.5" x2="12.5" y2="6.5" />
      <line x1="3.5" y1="10" x2="12.5" y2="10" />
      <line x1="12" y1="3.5" x2="4" y2="13" />
    </svg>
  ),
  warn: (
    <svg width="13" height="13" viewBox="0 0 16 16" aria-hidden="true" fill="none"
      stroke="currentColor" strokeWidth="1.4" strokeLinejoin="round" strokeLinecap="round">
      <path d="M8 2.5 L14.5 13.5 H1.5 Z" />
      <line x1="8" y1="6.5" x2="8" y2="9.5" />
      <circle cx="8" cy="11.6" r="0.7" fill="currentColor" stroke="none" />
    </svg>
  ),
  half: (
    <svg width="13" height="13" viewBox="0 0 16 16" aria-hidden="true" fill="none"
      stroke="currentColor" strokeWidth="1.4">
      <circle cx="8" cy="8" r="5.5" />
      <path d="M8 2.5 a5.5 5.5 0 0 1 0 11 z" fill="currentColor" stroke="none" />
    </svg>
  ),
};

/**
 * Small classification badges rendered inside the Name cell (icon-only, hover
 * for the label + reason):
 *   identity/calib (lock)   — skip-list match, excluded from the default push
 *   bus/port (plug)         — keep-list match (looks hardware-ish but is shared)
 *   file only (document)    — present in the file, absent on the vehicle(s)
 * Harmonize-only (explains why the row's checkbox is disabled — otherwise an
 * untagged disabled checkbox reads as an unexplained "read-only" row):
 *   no majority (≠)         — the UAVs' values are all distinct (or an even
 *                             split); disabled until you click a cell to pick one
 *   type mismatch (warning) — the param's stored type differs across UAVs
 *   partial (half circle)   — not present on every loaded UAV; can still have a
 *                             winner, just never auto-selected
 */
export function CompareNameTags({ row, harmonizeMode = false }) {
  const { t } = useTranslation();
  if (!row) return null;
  // Read-only (view-only firmware) rows carry ONLY the lock badge — the row is
  // never writable, so the no-majority / type-mismatch / partial tags (which
  // explain a disabled-but-harmonizable row) would be misleading here.
  if (row.readOnly) {
    const label = t('settings.parameters.tag_read_only', 'read-only');
    return (
      <span style={{ display: 'inline-flex', alignItems: 'center', gap: 5, flex: '0 0 auto' }}>
        <span
          title={`${label} — ${t('settings.parameters.tag_read_only_title', 'Firmware-maintained — view only, cannot be written')}`}
          aria-label={label}
          style={{ display: 'inline-flex', alignItems: 'center', color: colors.textDim }}
        >
          {TAG_ICON.lock}
        </span>
      </span>
    );
  }
  const tags = [];
  if (row.safetyTag === 'identity-cal-skip') {
    tags.push({
      key: 'skip', icon: 'lock', color: colors.textDim,
      label: t('settings.parameters.tag_identity', 'identity/calib'),
      title: t('settings.parameters.tag_identity_title',
        'Per-board calibration/identity — excluded from the default push'),
    });
  } else if (row.safetyTag === 'bus-port-keep') {
    tags.push({
      key: 'keep', icon: 'plug', color: colors.success,
      label: t('settings.parameters.tag_busport', 'bus/port'),
      title: t('settings.parameters.tag_busport_title',
        'Wiring/protocol config — shared across identical airframes, kept'),
    });
  }
  if (row.status === 'file-only') {
    tags.push({
      key: 'fileonly', icon: 'file', color: colors.accent,
      label: t('settings.parameters.tag_file_only', 'file only'),
      title: t('settings.parameters.tag_file_only_title',
        'In the file but not present on the vehicle(s) — cannot be written'),
    });
  }
  if (harmonizeMode) {
    if (row.typeMismatch) {
      tags.push({
        key: 'typemismatch', icon: 'warn', color: colors.warning,
        label: t('settings.parameters.tag_type_mismatch', 'type mismatch'),
        title: t('settings.parameters.tag_type_mismatch_title',
          'Stored type differs across UAVs — click a cell to pick the winner'),
      });
    } else if (row.defaultWinner === null) {
      tags.push({
        key: 'nomajority', icon: 'neq', color: colors.warning,
        label: t('settings.parameters.tag_no_majority', 'no majority'),
        title: t('settings.parameters.tag_no_majority_title',
          "UAVs disagree with no majority value — click a cell to pick the winner"),
      });
    }
    if (row.partial) {
      tags.push({
        key: 'partial', icon: 'half', color: colors.textDim,
        label: t('settings.parameters.tag_partial', 'partial'),
        title: t('settings.parameters.tag_partial_title',
          'Not present on every connected UAV — never auto-selected'),
      });
    }
  }
  if (tags.length === 0) return null;
  return (
    <span style={{ display: 'inline-flex', alignItems: 'center', gap: 5, flex: '0 0 auto' }}>
      {tags.map((tag) => (
        <span
          key={tag.key}
          title={`${tag.label} — ${tag.title}`}
          aria-label={tag.label}
          style={{ display: 'inline-flex', alignItems: 'center', color: tag.color }}
        >
          {TAG_ICON[tag.icon]}
        </span>
      ))}
    </span>
  );
}

/**
 * The File column cell — the loaded file's value for this parameter. A trailing
 * "~" marks rows whose value the autopilot would store differently (int
 * truncate/clamp) on at least one vehicle.
 */
export function FileValueCell({ row, width }) {
  const { t } = useTranslation();
  const anyCoerced = row && row.perVehicle
    && Object.values(row.perVehicle).some((c) => c && c.coerced);
  const title = row
    ? `${t('settings.parameters.file', 'File')}: ${row.fileValue}`
      + (anyCoerced ? ` (${t('settings.parameters.adjusted_to_fit', 'adjusted to fit on write')})` : '')
    : '';
  return (
    <div
      title={title}
      style={{
        width,
        flex: '0 0 auto',
        padding: '0 6px',
        display: 'flex',
        alignItems: 'center',
        gap: 3,
        overflow: 'hidden',
        whiteSpace: 'nowrap',
        fontFamily: 'monospace',
        fontSize: 12,
        color: colors.accent,
        borderRight: `1px solid ${colors.border}`,
      }}
    >
      <span style={{ overflow: 'hidden', textOverflow: 'ellipsis' }}>
        {row ? fmtFileValue(row.fileValue) : ''}
      </span>
      {anyCoerced && <span style={{ color: colors.textDim }}>~</span>}
    </div>
  );
}

/**
 * Read-only per-vehicle value cell for compare mode. Shows the vehicle's current
 * value, highlighted when it differs from the file; em-dash when the param is
 * absent on that vehicle; dim ellipsis while its snapshot is still loading.
 */
export function CompareValueCell({ rec, cell, error = null, width }) {
  const { t } = useTranslation();
  const loaded = cell ? cell.loaded : false;
  const present = cell ? cell.present : false;
  const differs = cell ? cell.differs : false;

  let text;
  let color = colors.text;
  if (!loaded) { text = '…'; color = colors.textDim; }
  else if (!present) { text = '—'; color = colors.textDim; }
  else { text = rec ? formatValueForDisplay(rec) : '—'; color = differs ? colors.warning : colors.text; }

  let title = present && rec
    ? `${t('settings.parameters.current', 'current')}: ${rec.value}`
    : (loaded
      ? t('settings.parameters.not_present', 'not present on this vehicle')
      : t('settings.parameters.loading_cell', 'loading…'));
  if (error) title += `\n${t('settings.parameters.apply_error', 'apply failed')}: ${error}`;

  return (
    <div
      title={title}
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
        color: error ? colors.error : color,
        border: `1px solid ${error ? colors.error : 'transparent'}`,
        background: error
          ? 'rgba(244, 67, 54, 0.14)'
          : (differs ? 'rgba(255, 152, 0, 0.16)' : 'transparent'),
      }}
    >
      <span style={{ overflow: 'hidden', textOverflow: 'ellipsis' }}>{text}</span>
    </div>
  );
}
