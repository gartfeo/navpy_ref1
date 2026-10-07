import React, { useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import { isPending, canCancel } from '../hooks/taskConfirmationState';
import { LONG_PRESS_MS } from '../utils/armAction';
import useLongPress from '../hooks/useLongPress';

const DEFAULT_TIMEOUT_SEC = 30;

/**
 * Confirmation card for a task confirm request.
 *
 * While the card is PENDING it shows the countdown + Approve/Deny and auto-
 * fires the timeout action (per the per-vehicle AAS params). Once DECIDED it
 * persists for review: APPROVED shows approval sent with a "Cancel task"
 * control; DENIED/CANCELED show the outcome and auto-dismiss (handled by the
 * owning hook). The auto-timeout never runs once decided.
 *
 * timeoutSec and autoApprove come from the per-vehicle AAS confirmed values
 * (AAS_NAV_CWT and AAS_NAV_CM_FL). The defensive fallback to
 * DEFAULT_TIMEOUT_SEC = 30 fires only when the prop is non-finite or negative
 * -- explicit `0` is honored as "immediate timeout".
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
}) {
  const { t } = useTranslation();
  const pending = isPending(entry);
  // Identity of the confirm round on screen. The same POI can be asked
  // again (a bounded re-ask), so the task id alone does NOT change between
  // rounds -- the round uid is what makes this a different question. Rounds
  // from a sender without a uid fall back to receivedAt, which a re-ask also
  // refreshes.
  const roundKey = `${entry.taskId}:${entry.roundUid ?? entry.receivedAt ?? ''}`;
  const { handlers: approveHoldHandlers, progress: approveProgress } = useLongPress({
    onComplete: () => onApprove(sysId),
    duration: LONG_PRESS_MS,
    enabled: pending && forced,
    // Reset the hold per ROUND: a press-and-hold started against the previous
    // round must not carry over and approve the new one without a fresh hold.
    resetKey: roundKey,
  });
  const effectiveTimeout = Number.isFinite(timeoutSec) && timeoutSec >= 0 ? timeoutSec : DEFAULT_TIMEOUT_SEC;
  const [remaining, setRemaining] = useState(() => {
    const elapsed = (Date.now() - entry.receivedAt) / 1000;
    return Math.max(0, effectiveTimeout - elapsed);
  });

  // Guards against double-firing the auto-action within ONE confirm round.
  // Reset it the moment the round changes: if the previous round auto-fired,
  // a stale `true` here would make the new round's countdown fire nothing and
  // leave the card pending forever. Done during render so the guard is
  // already clear when the countdown effect below runs for the new round.
  const firedRef = useRef(false);
  const firedRoundRef = useRef(roundKey);
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
    };
    const tick = () => {
      const elapsed = (Date.now() - entry.receivedAt) / 1000;
      const r = Math.max(0, effectiveTimeout - elapsed);
      setRemaining(r);
      if (r <= 0) fire();
    };
    tick();
    if (!firedRef.current) interval = setInterval(tick, 1000);
    return () => { if (interval) clearInterval(interval); };
  }, [pending, sysId, entry.receivedAt, autoApprove, onApprove, onDeny, effectiveTimeout]);

  const DECIDED_UI = {
    approved: { label: t('task.approved'), color: colors.success, sub: t('task.approvalSent') },
    denied: { label: t('task.denied'), color: colors.error, sub: null },
    canceled: { label: t('task.cancellationRequested'), color: colors.warning, sub: null },
  };
  const decided = DECIDED_UI[entry.status] || null;
  const accent = pending ? colors.warning : (decided?.color || colors.border);
  const timerColor = remaining > 10 ? colors.success : remaining > 5 ? colors.warning : colors.error;
  const mapsUrl = `https://www.google.com/maps?q=${entry.lat},${entry.lon}`;

  return (
    <div style={{
      marginBottom: 12,
      padding: 10,
      background: colors.surface,
      borderRadius: 6,
      border: `2px solid ${accent}`,
      ...(pending ? { animation: 'taskConfirmPulse 1.5s ease-in-out infinite' } : {}),
    }}>
      {pending && (
        <style>{`
          @keyframes taskConfirmPulse {
            0%, 100% { box-shadow: 0 0 4px ${colors.warning}40; }
            50% { box-shadow: 0 0 12px ${colors.warning}80; }
          }
        `}</style>
      )}

      {/* Header */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
        <span style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 13, fontWeight: 700, color: accent }}>
          <UavBadge name={vehicleName} index={vehicleIndex} /> {t('task.title', { id: entry.taskId })}
        </span>
        {pending ? (
          <span style={{ color: timerColor, fontSize: 13, fontWeight: 700, fontVariantNumeric: 'tabular-nums' }}>
            {Math.ceil(remaining)}s
          </span>
        ) : (
          <span style={{ color: decided?.color, fontSize: 12, fontWeight: 700 }}>
            {decided?.label}{decided?.sub ? ` · ${decided.sub}` : ''}
          </span>
        )}
      </div>

      {/* CONF-03 gate-overridden mark (D-20): shown only for the locally
          forced (sysId, taskId) so the operator judges the image with the
          right context -- it may be small/unstable. */}
      {pending && forced && (
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

      {/* Location */}
      <div style={{ fontSize: 11, color: colors.textDim, marginBottom: 6 }}>
        {entry.lat.toFixed(6)}, {entry.lon.toFixed(6)}
        {' '}
        <a
          href={mapsUrl}
          target="_blank"
          rel="noopener noreferrer"
          style={{ color: colors.accent, textDecoration: 'none' }}
        >{t('task.map')}</a>
        {entry.alt != null && (
          <span> — {Math.round(entry.alt)}m</span>
        )}
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
      ) : pending ? (
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
      ) : null}

      {/* Timer bar (pending only) */}
      {pending && (
        <div style={{
          height: 3,
          background: colors.bgLight,
          borderRadius: 2,
          marginBottom: 8,
          overflow: 'hidden',
        }}>
          <div style={{
            height: '100%',
            width: effectiveTimeout > 0 ? `${(remaining / effectiveTimeout) * 100}%` : '0%',
            background: timerColor,
            transition: 'width 1s linear, background-color 0.5s',
          }} />
        </div>
      )}

      {/* Actions */}
      {pending ? (
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
      ) : canCancel(entry) ? (
        <button
          onClick={() => onCancel(sysId)}
          style={{
            width: '100%', padding: '8px 0', background: 'transparent', color: colors.error,
            border: `1px solid ${colors.error}`, borderRadius: 4, fontWeight: 700, fontSize: 13,
            cursor: 'pointer',
          }}
        >
          {t('task.cancelTask')}
        </button>
      ) : null}
    </div>
  );
}
