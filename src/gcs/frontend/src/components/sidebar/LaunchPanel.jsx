import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import UavBadge from '../UavBadge';
import { orderVehiclesByLaunch } from '../../utils/launchSequence';
import { computeLaunchReadiness } from '../../utils/prearmChecks';

const STATE_DISPLAY = {
  idle:      { icon: '\u25CB', key: 'launch.waiting',    color: colors.textDim },
  queued:    { icon: '\u23F3', key: 'launch.queued',      color: colors.textDim },
  arming:    { icon: '\u23F3', key: 'launch.arming',      color: colors.warning },
  armed:     { icon: '\u2713', key: 'launch.armed',       color: colors.accent },
  launching: { icon: '\uD83D\uDE80', key: 'launch.launching', color: colors.warning },
  launched:  { icon: '\u2713', key: 'launch.launched',    color: colors.success },
  airborne:  { icon: '\u2713', key: 'launch.airborne',    color: colors.success },
  failed:    { icon: '\u2717', key: 'launch.failed',      color: colors.error },
  error:     { icon: '\u2717', key: 'launch.error',       color: colors.error },
};

/**
 * Per-vehicle display: the live launch state once launching has begun,
 * otherwise the pre-launch readiness (so the same list is shown before and
 * after pressing START).
 */
function getDisplay(launchStates, v, gates) {
  const state = launchStates?.[v.sys_id]?.state;
  if (state) return STATE_DISPLAY[state] || STATE_DISPLAY.idle;
  if (v.mode === 'AUTO') return STATE_DISPLAY.airborne;  // already flying
  const ready = computeLaunchReadiness(v, gates).ready;
  return ready
    ? { icon: '\u25CF', key: 'launch.ready', color: colors.success }
    : { icon: '\u25CF', key: 'launch.notReady', color: colors.warning };
}

export default function LaunchPanel({ vehicleList, launchStates, onAbort, onTriggerVehicle, launchOrder, gates, launcherStatus }) {
  const { t } = useTranslation();
  // Show vehicles in the operator-defined launch order so the live sequence
  // matches the launch order.
  const orderedVehicles = orderVehiclesByLaunch(vehicleList, launchOrder);
  const anyActive = vehicleList.some((v) => {
    const s = launchStates?.[v.sys_id]?.state;
    return s === 'queued' || s === 'arming' || s === 'armed' || s === 'launching';
  });

  return (
    <div style={{ marginTop: 12 }}>
      {/* Launch-sequence list: readiness before START, live states after.
          Kept visible throughout so the panel doesn't disappear. */}
      <div style={{
        fontSize: 11, color: colors.textDim, marginBottom: 6,
        textTransform: 'uppercase', letterSpacing: 1,
      }}>
        {t('monitor.launchSequence')}
      </div>
      {orderedVehicles.map((v, i) => {
        const display = getDisplay(launchStates, v, gates);
        const ls = launchStates?.[v.sys_id];
        const state = ls?.state;
        const canTrigger = !state || state === 'idle' || state === 'failed' || state === 'error';
        const hideTrigger = state === 'airborne' || state === 'launched' || state === 'queued';

        return (
          <div key={v.sys_id} style={{ marginBottom: 6 }}>
            <div
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: 8,
                fontSize: 13,
                color: colors.text,
              }}
            >
              <span style={{ color: colors.textDim, fontVariantNumeric: 'tabular-nums', minWidth: 16 }}>
                {i + 1}.
              </span>
              <span style={{ color: display.color }}>{display.icon}</span>
              <span style={{ flex: 1 }}><UavBadge name={v.name} index={i} /></span>
              <span style={{ color: display.color, fontSize: 11 }}>
                {t(display.key)}
              </span>
              {!hideTrigger && onTriggerVehicle && (
                <button
                  onClick={() => onTriggerVehicle?.(v.sys_id)}
                  disabled={!canTrigger}
                  style={{
                    padding: '2px 10px',
                    background: canTrigger ? colors.warning : colors.surface,
                    color: canTrigger ? '#000' : colors.textDim,
                    border: 'none',
                    borderRadius: 4,
                    fontWeight: 700,
                    fontSize: 11,
                    cursor: canTrigger ? 'pointer' : 'not-allowed',
                    opacity: canTrigger ? 1 : 0.5,
                  }}
                >
                  {t('launch.launchBtn')}
                </button>
              )}
            </div>
            {ls?.error && (
              <div style={{ color: colors.error, fontSize: 11, marginLeft: 22 }}>
                {ls.error}
              </div>
            )}
          </div>
        );
      })}
      {/* Launcher (ESP32/container) connection status — after the UAV list,
          just above the START button. */}
      {launcherStatus}
      {anyActive && onAbort && (
        <button
          onClick={onAbort}
          style={{
            width: '100%',
            padding: '8px 0',
            marginTop: 8,
            background: colors.error,
            color: '#fff',
            border: 'none',
            borderRadius: 4,
            fontWeight: 700,
            fontSize: 12,
            cursor: 'pointer',
          }}
        >
          {t('launch.abortLaunch')}
        </button>
      )}
    </div>
  );
}
