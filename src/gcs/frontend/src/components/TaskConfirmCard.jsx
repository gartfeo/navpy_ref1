import React, { useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import {
  isPending, canCancel, confirmRoundKey, confirmWaitSec, confirmDeadlineMs, msLeft, secondsLeft,
  confirmZeroLabel, CONFIRM_PHASE,
} from '../hooks/taskConfirmationState';
import { confirmNow } from '../utils/confirmClock';
import { LONG_PRESS_MS } from '../utils/armAction';
import useLongPress from '../hooks/useLongPress';

/** Countdown colour steps (s left): green above, amber above, red at or below. */
const TIMER_AMBER_ABOVE_SEC = 5;
const TIMER_GREEN_ABOVE_SEC = 10;

/** The countdown colour for `seconds` left; the summary chip uses it too. */
export function timerColorFor(seconds) {
  if (seconds > TIMER_GREEN_ABOVE_SEC) return colors.success;
  return seconds > TIMER_AMBER_ABOVE_SEC ? colors.warning : colors.error;
}

/** Every control on a decided row: the app's 32 px map-button size (todo 69). */
const ROW_CONTROL_PX = 32;
/** The opened still of a decided row (ConfirmStill), smaller than a pending card's 160 px. */
const OPEN_STILL_MAX_PX = 120;

/**
 * Confirmation card for a task confirm request.
 *
 * While the card is PENDING it is the full card: the countdown +
 * Approve/Deny, and it auto-fires the timeout action (per the per-vehicle AAS
 * params). Once DECIDED it is a compact row (ConfirmCardList pins it, so
 * Cancel task never scrolls away): APPROVED shows "Approval sent", a Still
 * button and Cancel task; DENIED/CANCELED show the outcome and auto-dismiss
 * (handled by the owning hook). The auto-timeout never runs once decided.
 *
 * timeoutSec and autoApprove come from the per-vehicle AAS confirmed values
 * (AAS_NAV_CWT and AAS_NAV_CM_FL) and drive the auto-fire. The defensive
 * fallback to DEFAULT_CONFIRM_WAIT_SEC = 30 fires only when the prop is
 * non-finite or negative -- explicit `0` is honored as "immediate timeout".
 *
 * The countdown label names what the GCS sends at zero (todo 68,
 * gcs_confirm_cue.md decision 5) from `labelPolicy` -- the RAW cached
 * { cmFl, cwt }, undefined when not loaded -- so a vehicle whose params are
 * not loaded never reads as a definite policy. Without it (a card alone) the
 * label uses the auto-fire props.
 */
export default function TaskConfirmCard({
  sysId, entry, vehicleName, vehicleIndex,
  onApprove, onDeny, onCancel, autoApprove, timeoutSec,
  // CONF-03 (D-18/D-19/D-20): true when this popup's (sysId, entry.taskId)
  // was locally marked forced by the "Ask me anyway" override -- no wire
  // field, the GCS already knows which POI it forced. A forced popup is
  // marked gate-overridden and its approve requires press-and-hold, like the
  // override button itself; a normal popup keeps instant approve.
  forced = false,
  // Camera panel action for a PENDING card, or null when not offered (that
  // UAV has no stream, or RC is on): { viewing, onView }. It only moves the
  // video; it never approves, denies or pauses the countdown.
  videoAction = null,
  // Decided row only: whether this round's still is open (ConfirmCardList
  // shows it), and the toggle, or null when there is no still to show.
  stillOpen = false,
  onToggleStill = null,
  // Pending card only (ConfirmCardList passes them): the raw cached policy
  // for the label, the list's shared `now` (so the card and the summary chip
  // show the same second), whether the fired latch holds for this round, and
  // the latch callback, called from the auto-fire path.
  labelPolicy = null,
  now = null,
  fired = false,
  onAutoFire = null,
}) {
  const { t } = useTranslation();
  const pending = isPending(entry);
  // Identity of the confirm round on screen (a re-ask of the same POI is a
  // new round with the same task id).
  const roundKey = confirmRoundKey(entry);
  const { handlers: approveHoldHandlers, progress: approveProgress } = useLongPress({
    onComplete: () => onApprove(sysId),
    duration: LONG_PRESS_MS,
    enabled: pending && forced,
    // Reset the hold per ROUND: a press-and-hold started against the previous
    // round must not carry over and approve the new one without a fresh hold.
    resetKey: roundKey,
  });
  const effectiveTimeout = confirmWaitSec(timeoutSec).sec;
  const deadlineMs = confirmDeadlineMs(entry, timeoutSec);
  // Only re-renders a standalone card each second; the render reads the clock.
  const [, setRemaining] = useState(0);

  // Guards against double-firing the auto-action within ONE confirm round.
  // Reset it the moment the round changes: if the previous round auto-fired,
  // a stale `true` here would make the new round's countdown fire nothing and
  // leave the card pending forever. Done during render so the guard is
  // already clear when the countdown effect below runs for the new round.
  const firedRef = useRef(false);
  const firedRoundRef = useRef(roundKey);
  // The latch callback is read at fire time, so a new callback identity
  // never re-runs the countdown effect.
  const onAutoFireRef = useRef(onAutoFire);
  onAutoFireRef.current = onAutoFire;
  // Hosted by the list, the label reads the list's `now`: the tick then only
  // decides the fire and does not re-render the card for its own display.
  const hostedRef = useRef(now != null);
  hostedRef.current = now != null;
  if (firedRoundRef.current !== roundKey) {
    firedRoundRef.current = roundKey;
    firedRef.current = false;
  }

  useEffect(() => {
    // Decided cards never count down or auto-fire.
    if (!pending) return undefined;
    let interval = null;
    const fire = () => {
      if (firedRef.current) return;
      firedRef.current = true;
      if (interval) clearInterval(interval);
      if (autoApprove) onApprove(sysId);
      else onDeny(sysId);
      onAutoFireRef.current?.(sysId, firedRoundRef.current);
    };
    const tick = () => {
      const r = msLeft(deadlineMs, confirmNow()) / 1000;
      if (!hostedRef.current) setRemaining(r);
      if (r <= 0) fire();
    };
    tick();
    if (!firedRef.current) interval = setInterval(tick, 1000);
    return () => { if (interval) clearInterval(interval); };
  }, [pending, sysId, deadlineMs, autoApprove, onApprove, onDeny]);

  if (!pending) {
    return (
      <DecidedRow
        sysId={sysId} entry={entry} vehicleName={vehicleName} vehicleIndex={vehicleIndex}
        onCancel={onCancel} stillOpen={stillOpen} onToggleStill={onToggleStill}
      />
    );
  }

  // The label's second: the list's shared sample when given, else the clock
  // at this render (the card's own tick re-renders it each second). A fired
  // round stays at 0 even if a longer wait or a clock step back gives it time.
  const clockNow = now ?? confirmNow();
  const shownSeconds = fired ? 0 : secondsLeft(deadlineMs, clockNow);
  const barFraction = fired || effectiveTimeout <= 0 ? 0 : msLeft(deadlineMs, clockNow) / 1000 / effectiveTimeout;
  const phase = fired ? CONFIRM_PHASE.FIRED : shownSeconds > 0 ? CONFIRM_PHASE.LIVE : CONFIRM_PHASE.DUE;
  const policy = labelPolicy ?? { cmFl: autoApprove, cwt: timeoutSec };
  const label = confirmZeroLabel({ phase, cmFl: policy.cmFl, cwt: policy.cwt, seconds: shownSeconds });
  const timerColor = timerColorFor(shownSeconds);

  return (
    <div style={{
      marginBottom: 12,
      padding: 10,
      background: colors.surface,
      borderRadius: 6,
      border: `2px solid ${colors.warning}`,
      animation: 'taskConfirmPulse 1.5s ease-in-out infinite',
    }}>
      <style>{`
        @keyframes taskConfirmPulse {
          0%, 100% { box-shadow: 0 0 4px ${colors.warning}40; }
          50% { box-shadow: 0 0 12px ${colors.warning}80; }
        }
      `}</style>

      {/* Header */}
      {/* The countdown words wrap under the title rather than shrink it. */}
      <div style={{ display: 'flex', flexWrap: 'wrap', justifyContent: 'space-between', alignItems: 'center', gap: '2px 6px', marginBottom: 6 }}>
        <span style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 13, fontWeight: 700, color: colors.warning }}>
          <UavBadge name={vehicleName} index={vehicleIndex} /> {t('task.title', { id: entry.taskId })}
        </span>
        <span
          data-testid={`confirm-countdown-${sysId}`}
          data-time={label.time}
          data-label={label.cardKey}
          style={{ marginLeft: 'auto', color: timerColor, fontSize: 13, fontWeight: 700, fontVariantNumeric: 'tabular-nums' }}
        >
          {t(label.cardKey, { time: label.time })}
        </span>
      </div>

      {/* CONF-03 gate-overridden mark (D-20): shown only for the locally
          forced (sysId, taskId) so the operator judges the image with the
          right context -- it may be small/unstable. */}
      {forced && (
        <div style={{
          marginBottom: 6,
          padding: '2px 6px',
          background: 'rgba(255, 152, 0, 0.15)',
          borderRadius: 3,
          fontSize: 10,
          fontWeight: 700,
          color: colors.warning,
        }}>
          {t('task.gateOverridden')}
        </div>
      )}

      <div style={{ fontSize: 11, color: colors.textDim, marginBottom: 6 }}>
        <PoiLocation entry={entry} />
      </div>

      {/* Image */}
      {entry.imageB64 ? (
        <img
          src={`data:image/jpeg;base64,${entry.imageB64}`}
          alt="Detection"
          style={{
            width: '100%',
            borderRadius: 4,
            marginBottom: 8,
            maxHeight: 160,
            objectFit: 'contain',
            background: '#000',
          }}
        />
      ) : (
        <div style={{
          width: '100%',
          height: 60,
          borderRadius: 4,
          marginBottom: 8,
          background: colors.bgLighter,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: colors.textDim,
          fontSize: 11,
        }}>
          {t('task.waitingForImage')}
        </div>
      )}

      {/* View camera: neutral, beside the still and away from Approve/Deny. */}
      {videoAction && (
        <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 12, flexWrap: 'wrap' }}>
          <button
            type="button"
            onClick={videoAction.viewing ? undefined : videoAction.onView}
            disabled={videoAction.viewing}
            aria-pressed={videoAction.viewing}
            style={{
              minHeight: 44, padding: '0 12px', borderRadius: 4, fontSize: 14, fontWeight: 600,
              cursor: videoAction.viewing ? 'default' : 'pointer', touchAction: 'manipulation',
              border: `1px solid ${colors.border}`, background: colors.bgLighter, color: colors.textBright,
            }}
          >
            {videoAction.viewing ? t('task.viewing') : t('task.viewCamera')}
          </button>
          <span style={{ fontSize: 11, color: colors.textDim }}>{t('task.viewCameraNote')}</span>
        </div>
      )}

      {/* Timer bar */}
      <div style={{
        height: 3,
        background: colors.bgLight,
        borderRadius: 2,
        marginBottom: 8,
        overflow: 'hidden',
      }}>
        <div style={{
          height: '100%',
          width: `${barFraction * 100}%`,
          background: timerColor,
          transition: 'width 1s linear, background-color 0.5s',
        }} />
      </div>

      {/* Actions */}
      <div style={{ display: 'flex', gap: 8 }}>
        {forced ? (
          // CONF-03 (D-19): a forced (gate-overridden) popup's approve
          // requires press-and-hold, mirroring the "Ask me anyway" button
          // itself -- no accidental approve of a degraded image.
          <button
            type="button"
            {...approveHoldHandlers}
            style={{
              flex: 1, padding: '8px 0', color: '#fff',
              border: 'none', borderRadius: 4, fontWeight: 700, fontSize: 13, cursor: 'pointer',
              touchAction: 'manipulation',
              background: approveProgress > 0
                ? `linear-gradient(to right, ${colors.success} ${approveProgress * 100}%, ${colors.warning} ${approveProgress * 100}%)`
                : colors.warning,
            }}
          >
            {t('task.holdToApprove')}
          </button>
        ) : (
          <button
            onClick={() => onApprove(sysId)}
            style={{
              flex: 1, padding: '8px 0', background: colors.success, color: '#fff',
              border: 'none', borderRadius: 4, fontWeight: 700, fontSize: 13, cursor: 'pointer',
            }}
          >
            {t('task.approve')}
          </button>
        )}
        <button
          onClick={() => onDeny(sysId)}
          style={{
            flex: 1, padding: '8px 0', background: colors.error, color: '#fff',
            border: 'none', borderRadius: 4, fontWeight: 700, fontSize: 13, cursor: 'pointer',
          }}
        >
          {t('task.deny')}
        </button>
      </div>
    </div>
  );
}

/** The POI's position line: coordinates, a map link and the altitude. */
function PoiLocation({ entry }) {
  const { t } = useTranslation();
  // The companion sends no position when it has not geolocated the POI.
  if (entry.lat == null || entry.lon == null) {
    return <>{t('task.positionUnknown')}{entry.alt != null && <span> — {Math.round(entry.alt)}m</span>}</>;
  }
  return (
    <>
      {entry.lat.toFixed(6)}, {entry.lon.toFixed(6)}
      {' '}
      <a
        href={`https://www.google.com/maps?q=${entry.lat},${entry.lon}`}
        target="_blank"
        rel="noopener noreferrer"
        style={{ color: colors.accent, textDecoration: 'none' }}
      >{t('task.map')}</a>
      {entry.alt != null && <span> — {Math.round(entry.alt)}m</span>}
    </>
  );
}

const DECIDED_ROW = {
  approved: { labelKey: 'task.approvalSent', color: colors.success },
  denied: { labelKey: 'task.denied', color: colors.error },
  canceled: { labelKey: 'task.cancellationRequested', color: colors.warning },
};

/**
 * A decided round as one 32 px row: UAV badge and outcome, then for an
 * approved round the Still toggle and Cancel task. The row never grows (the
 * still opens elsewhere), so pinned rows keep their places. A long label
 * shortens; Cancel task wraps onto two lines inside its 32 px rather than
 * leave the row (the Armenian label is wider than the row's free width); a
 * UAV name too long for the row is cut off, never Cancel task.
 */
function DecidedRow({ sysId, entry, vehicleName, vehicleIndex, onCancel, stillOpen, onToggleStill }) {
  const { t } = useTranslation();
  const ui = DECIDED_ROW[entry.status];
  const label = ui ? t(ui.labelKey) : entry.status;
  const approved = canCancel(entry);
  return (
    <div
      data-testid={`confirm-row-${sysId}`}
      style={{ display: 'flex', alignItems: 'center', gap: 6, height: ROW_CONTROL_PX, marginBottom: 4 }}
    >
      <span title={vehicleName} style={{ flex: '0 1 auto', minWidth: 0, overflow: 'hidden', whiteSpace: 'nowrap' }}>
        <UavBadge name={vehicleName} index={vehicleIndex} />
      </span>
      <span
        title={label}
        style={{
          flex: '1 1 0', minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap',
          color: ui?.color ?? colors.textDim, fontSize: 12, fontWeight: 700,
        }}
      >
        {label}
      </span>
      {approved && onToggleStill && (
        <button
          type="button"
          onClick={onToggleStill}
          aria-pressed={stillOpen}
          aria-label={t(stillOpen ? 'task.hideStill' : 'task.showStill')}
          title={t(stillOpen ? 'task.hideStill' : 'task.showStill')}
          style={{
            flex: '0 0 auto', width: ROW_CONTROL_PX, height: ROW_CONTROL_PX, boxSizing: 'border-box', padding: 0,
            display: 'flex', alignItems: 'center', justifyContent: 'center', borderRadius: 6,
            border: `1px solid ${stillOpen ? colors.accent : colors.border}`,
            background: stillOpen ? 'rgba(0, 210, 255, 0.2)' : colors.bgLighter,
            color: stillOpen ? colors.accent : colors.textBright,
            cursor: 'pointer', touchAction: 'manipulation',
          }}
        >
          <svg width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
            <rect x="1.5" y="3" width="13" height="10" rx="1.5" stroke="currentColor" strokeWidth="1.3" />
            <path d="M3.5 11l3-3.5 2.2 2.4L10.5 8l2 3" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" />
          </svg>
        </button>
      )}
      {approved && (
        <button
          type="button"
          onClick={() => onCancel(sysId)}
          style={{
            flex: '0 1 auto', height: ROW_CONTROL_PX, boxSizing: 'border-box', padding: '0 12px',
            background: 'transparent', color: colors.error, border: `1px solid ${colors.error}`,
            borderRadius: 4, fontWeight: 700, fontSize: 13, lineHeight: '14px', whiteSpace: 'normal',
            textAlign: 'center', cursor: 'pointer', touchAction: 'manipulation',
          }}
        >
          {t('task.cancelTask')}
        </button>
      )}
    </div>
  );
}

/**
 * The opened still of an approved round, at the top of ConfirmCardList's
 * scrolling part: its UAV badge and POI line, then the picture.
 */
export function ConfirmStill({ entry, vehicleName, vehicleIndex }) {
  const { t } = useTranslation();
  return (
    <div data-testid="confirm-still" style={{ marginBottom: 12 }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 4, fontSize: 12, fontWeight: 700, color: colors.textBright }}>
        <UavBadge name={vehicleName} index={vehicleIndex} /> {t('task.title', { id: entry.taskId })}
      </div>
      <div style={{ fontSize: 11, color: colors.textDim, marginBottom: 6 }}>
        <PoiLocation entry={entry} />
      </div>
      <img
        src={`data:image/jpeg;base64,${entry.imageB64}`}
        alt="Detection"
        style={{
          display: 'block', width: '100%', maxHeight: OPEN_STILL_MAX_PX,
          objectFit: 'contain', borderRadius: 4, background: '#000',
        }}
      />
    </div>
  );
}
