import React, { useCallback, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { LONG_PRESS_MS } from '../../utils/armAction';
import useLongPress from '../../hooks/useLongPress';
import { resolveStartButtonMode } from '../../utils/startButtonMode';

export function ForceLaunchButton({ sysId, busy, ready, onLaunch }) {
  const { t } = useTranslation();
  const handleForce = useCallback(() => {
    if (window.confirm(t('launch.forceLaunchConfirm', { sysId }))) onLaunch(sysId, { force: true });
  }, [sysId, onLaunch, t]);

  const { handlers, progress } = useLongPress({
    onComplete: handleForce,
    duration: LONG_PRESS_MS,
    enabled: !ready && !busy,
  });

  const disabled = busy || (!ready && progress === 0);
  return (
    <button
      onClick={ready && !busy ? () => onLaunch(sysId) : undefined}
      disabled={busy}
      {...(!ready && !busy ? handlers : {})}
      style={(() => {
        const base = {
          width: '100%',
          marginTop: 4,
          padding: '6px 0',
          // Ready = green ("go" via click); not-ready = amber ("hold to force").
          // Both are actionable, so both get a solid colour (matching
          // ForceStartButton) rather than a muted/transparent surface look.
          // Only the in-flight (busy) state is dimmed.
          background: busy ? colors.surface : (ready ? colors.success : colors.warning),
          color: busy ? colors.textDim : '#000',
          border: 'none',
          borderRadius: 4,
          fontWeight: 700,
          fontSize: 11,
          cursor: busy ? 'wait' : 'pointer',
          opacity: busy ? 0.5 : 1,
          touchAction: 'manipulation',
        };
        if (progress > 0) {
          // Long-press fill: green grows over the amber force colour.
          base.background = `linear-gradient(to right, ${colors.success} ${progress * 100}%, ${colors.warning} ${progress * 100}%)`;
          base.opacity = 1;
          base.color = '#000';
        }
        return base;
      })()}
    >
      {busy ? t('launch.launchingBtn') : (!ready ? t('launch.holdToForce') : t('launch.launchBtn'))}
    </button>
  );
}

export function LauncherStatus({ connected, checking, onRetry, simulatorRunning, host, port, inline }) {
  const { t } = useTranslation();
  const label = simulatorRunning
    ? (connected ? t('launcher.simConnected') : t('launcher.simDisconnected'))
    : (connected ? `${host}:${port} ${t('launcher.connected')}` : `${host}:${port} ${t('launcher.disconnected')}`);

  // `inline` renders a compact tinted status badge (part of the launch panel,
  // sitting just above the START button) rather than a standalone grey card.
  const containerStyle = inline ? {
    display: 'flex',
    alignItems: 'center',
    gap: 8,
    marginTop: 10,
    marginBottom: 2,
    padding: '6px 10px',
    background: connected ? 'rgba(76, 175, 80, 0.12)' : 'rgba(244, 67, 54, 0.12)',
    border: `1px solid ${connected ? colors.success : colors.error}`,
    borderRadius: 4,
    fontSize: 12,
  } : {
    display: 'flex',
    alignItems: 'center',
    gap: 8,
    marginTop: 12,
    padding: '8px 10px',
    background: colors.surface,
    borderRadius: 4,
    border: `1px solid ${colors.border}`,
    fontSize: 12,
  };

  return (
    <div style={containerStyle}>
      <span style={{
        display: 'inline-block',
        width: 8,
        height: 8,
        borderRadius: '50%',
        background: connected ? colors.success : colors.error,
        flexShrink: 0,
      }} />
      <span style={{ color: connected ? colors.success : colors.error, flex: 1 }}>
        {checking ? t('launcher.checking') : label}
      </span>
      {!connected && !checking && (
        <button
          onClick={onRetry}
          style={{
            padding: '2px 8px',
            background: 'transparent',
            color: colors.accent,
            border: `1px solid ${colors.accent}`,
            borderRadius: 3,
            fontSize: 11,
            cursor: 'pointer',
          }}
        >
          {t('launcher.retry')}
        </button>
      )}
    </div>
  );
}

export function ForceStartButton({ label, busyLabel, armLabel, ready, disabled, onStart, onArm, armReady = false, large }) {
  const { t } = useTranslation();
  const onStartRef = useRef(onStart);
  onStartRef.current = onStart;
  const onArmRef = useRef(onArm);
  onArmRef.current = onArm;

  // Single source of truth for the button's interaction (kept in a pure,
  // unit-tested helper). `armReady` gates the ready path behind a hold so a
  // physical container launch can never start on a single stray click.
  const mode = resolveStartButtonMode({ armReady, ready, disabled });

  const handleComplete = useCallback(() => {
    if (mode.holdMode === 'arm') {
      // Ready + hold-to-arm: request confirmation, never launch directly.
      onArmRef.current?.();
    } else if (mode.holdMode === 'force') {
      // Not-ready force path: explicit confirm, then force-launch.
      if (window.confirm(t('monitor.forceStartConfirm'))) onStartRef.current?.({ force: true });
    }
  }, [mode.holdMode, t]);

  const { handlers, progress } = useLongPress({
    onComplete: handleComplete,
    duration: LONG_PRESS_MS,
    enabled: mode.holdEnabled,
    // Reset the hold if the mode flips mid-press (e.g. ready→not-ready), so an
    // "arm" gesture can never complete as a "force" launch (or vice versa).
    resetKey: mode.holdMode,
  });

  const showBusy = disabled && !!busyLabel;
  const arming = mode.holdMode === 'arm';   // ready, gated behind hold-to-arm
  const btnColor = !ready && !disabled ? colors.warning : colors.success;
  // The hold-to-arm path is already "ready" (green), so a solid-green track
  // would hide the green progress fill; use a muted (dark) green track instead
  // so the brighter green fill is visible as it grows.
  const armTrack = 'rgba(76, 175, 80, 0.28)';
  const track = arming ? armTrack : btnColor;
  const base = {
    width: '100%',
    padding: large ? '14px 0' : '10px 0',
    background: track,
    // White label throughout: readable on the dark-green track AND the green
    // fill (green-on-green would be invisible). Amber force path keeps black.
    color: !ready && !disabled ? '#000' : '#fff',
    border: arming ? `1px solid ${colors.success}` : 'none',
    borderRadius: 6,
    fontWeight: 700,
    fontSize: large ? 15 : 13,
    cursor: disabled ? 'not-allowed' : 'pointer',
    opacity: disabled ? 0.5 : 1,
    marginTop: 8,
    marginBottom: !ready && !disabled ? 0 : (large ? 12 : 8),
    touchAction: 'manipulation',
  };
  if (progress > 0) {
    base.background = `linear-gradient(to right, ${colors.success} ${progress * 100}%, ${track} ${progress * 100}%)`;
  }

  return (
    <>
      <button
        onClick={mode.clickToStart ? onStart : undefined}
        disabled={disabled}
        {...(mode.holdEnabled ? handlers : {})}
        style={base}
      >
        {showBusy ? busyLabel : (arming ? (armLabel || label) : label)}
      </button>
      {!ready && !disabled && (
        <div style={{
          marginTop: 4, marginBottom: large ? 12 : 8,
          padding: '6px 8px',
          background: 'rgba(244, 67, 54, 0.15)',
          border: `1px solid ${colors.error}`,
          borderRadius: 4, fontSize: 11,
          color: colors.error, textAlign: 'center',
        }}>
          {t('monitor.notAllReady')}
        </div>
      )}
    </>
  );
}

// Primary, full-width plan call-to-action. Used for "Start Planning" when no
// plan exists yet — the genuine main action at that stage.
export function EditButton({ label, onClick }) {
  return (
    <button
      onClick={onClick}
      style={{
        width: '100%',
        marginTop: 12,
        padding: '8px 0',
        background: colors.accent,
        color: '#000',
        border: 'none',
        borderRadius: 4,
        fontSize: 12,
        fontWeight: 700,
        cursor: 'pointer',
      }}
    >
      {label}
    </button>
  );
}

// Low-emphasis, inline plan-edit affordance for the section header. Once a plan
// already exists, editing it is a secondary action, so it lives here as a small
// pencil link rather than a full-width button in the action column — out of the
// way of START / START MISSION but always one click away.
export function EditPlanLink({ label, onClick }) {
  return (
    <button
      type="button"
      onClick={onClick}
      aria-label={label}
      title={label}
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        gap: 5,
        background: 'transparent',
        color: colors.accent,
        border: 'none',
        padding: '2px 2px',
        fontSize: 12,
        fontWeight: 600,
        cursor: 'pointer',
        flexShrink: 0,
      }}
    >
      <svg width="13" height="13" viewBox="0 0 16 16" fill="none" aria-hidden="true" style={{ flexShrink: 0 }}>
        <path d="M10.8 2.6l2.6 2.6M2.5 13.5l2.9-.6 7.2-7.2-2.3-2.3-7.2 7.2-.6 2.9z" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
      {label}
    </button>
  );
}
