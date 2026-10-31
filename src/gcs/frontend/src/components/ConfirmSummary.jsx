import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import { timerColorFor } from './TaskConfirmCard';
import { URGENT_LEFT_SEC } from '../hooks/useConfirmAlert';

/** One line, 30 px plus a 4 px gap: todo 69's 34 px budget for this summary. */
const CHIP_HEIGHT_PX = 30;
const CHIP_GAP_PX = 4;
// Width budget in the 320 px hosts (268 px inside): count 28, badge <= 72,
// "+N" <= 24, time <= 40, 5 gaps of 5, the words in the rest (>= 79).
const BADGE_MAX_PX = 72;
const MORE_MAX_PX = 24;
const TIME_MAX_PX = 40;
const GAP_PX = 5;

/**
 * The pending-confirm summary chip (todo 67, gcs_confirm_cue.md decision 1):
 * the first item of ConfirmCardList's pinned part while any round is
 * pending. Bell and count; the badge of the UAV whose time is shown, "+N"
 * for the others; that round's time; then the short words for what the GCS
 * sends at zero (or the sound-off note). Only the words shrink. A tap (or
 * Enter/Space: it is a button) brings that UAV's card into view and unlocks
 * the sound; it never approves or denies.
 *
 * `summary` is pendingSummary(); `label` confirmZeroLabel() for the shown
 * round; `names` the pending UAVs' names in summary order.
 */
export default function ConfirmSummary({ summary, label, vehicle, names, soundState, onActivate }) {
  const { t } = useTranslation();
  const more = summary.count - 1;
  const urgent = summary.secondsLeft <= URGENT_LEFT_SEC;
  const soundOff = soundState === 'suspended' ? t('task.soundOff')
    : soundState === 'unavailable' ? t('task.noSound') : null;
  const words = soundOff ?? t(label.chipKey);
  const full = `${t(label.cardKey, { time: label.time })} — ${names.join(', ')}${soundOff ? ` — ${soundOff}` : ''}`;
  return (
    <button
      type="button"
      data-testid="confirm-summary"
      data-shown={summary.shownSysId}
      onClick={onActivate}
      title={full}
      aria-label={full}
      style={{
        display: 'flex', alignItems: 'center', gap: GAP_PX, width: '100%', height: CHIP_HEIGHT_PX,
        marginBottom: CHIP_GAP_PX, padding: '0 8px', boxSizing: 'border-box', borderRadius: 6,
        border: `2px solid ${urgent ? colors.error : colors.warning}`,
        background: urgent ? 'rgba(244, 67, 54, 0.13)' : 'rgba(255, 152, 0, 0.13)',
        color: colors.textBright, fontSize: 12, fontWeight: 700, whiteSpace: 'nowrap',
        cursor: 'pointer', touchAction: 'manipulation', textAlign: 'left',
      }}
    >
      <span style={{ flexShrink: 0 }} aria-hidden="true">&#128276; {summary.count}</span>
      <span style={{ flexShrink: 0, maxWidth: BADGE_MAX_PX, overflow: 'hidden', textOverflow: 'ellipsis' }}>
        <UavBadge name={vehicle.name} index={vehicle.index} />
      </span>
      {more > 0 && (
        <span data-testid="confirm-summary-more" style={{ flexShrink: 0, maxWidth: MORE_MAX_PX, color: colors.textDim, fontSize: 11 }}>
          +{more}
        </span>
      )}
      <span
        data-testid="confirm-summary-time"
        style={{ flexShrink: 0, maxWidth: TIME_MAX_PX, color: timerColorFor(summary.secondsLeft), fontVariantNumeric: 'tabular-nums' }}
      >
        {label.time}
      </span>
      <span
        data-testid="confirm-summary-words"
        style={{ flex: '1 1 auto', minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', fontWeight: 600, color: soundOff ? colors.warning : colors.textBright }}
      >
        {words}
      </span>
    </button>
  );
}
