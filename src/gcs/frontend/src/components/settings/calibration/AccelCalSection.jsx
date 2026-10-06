import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../../styles';
import {
  ACCEL_CAL_POSITIONS,
  ACCEL_CAL_POS_KEY_BY_CODE,
} from '../../../utils/accelCal';

/**
 * Accelerometer / level calibration section.
 *
 * Per connected vehicle: a quick "Level" button (board-level trim) and a full
 * 6-position accel-cal wizard that shows the autopilot's current orientation
 * prompt and a "next position" confirm. State + actions come from useAccelCal.
 *
 * Re-authored from feat/uav-accel-cal's standalone CalibrationTab.jsx (202
 * lines, 466 commits behind dev, 7 conflicts vs dev — all in stale
 * SettingsModal/App.jsx/locale wiring that this file discards entirely) into
 * the unified Calibration tab shell (INTEG-04, D-04). Only this component's
 * own logic is ported; the shell (Task 1) owns all tab wiring.
 */
export default function AccelCalSection({ vehicleList, accelCal, onOpenConnect, onClose }) {
  const { t } = useTranslation();
  const vehicles = vehicleList || [];
  const cal = accelCal || {};
  const calByVehicle = cal.calByVehicle || {};

  if (vehicles.length === 0) {
    return (
      <div style={{
        padding: '12px 16px', marginBottom: 12,
        background: 'rgba(255,255,255,0.05)', borderRadius: 4,
        color: colors.textDim, fontSize: 13, textAlign: 'center',
        display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 10,
      }}>
        <span>{t('calibration.noUavs')}</span>
        <button
          onClick={() => { onClose?.(); onOpenConnect?.(); }}
          style={{
            padding: '4px 12px', fontSize: 12, fontWeight: 600,
            background: colors.accent, color: '#000',
            border: 'none', borderRadius: 4, cursor: 'pointer',
          }}
        >
          {t('connection.connect')}
        </button>
      </div>
    );
  }

  return (
    <>
      <div style={{ color: colors.textDim, fontSize: 12, marginBottom: 12 }}>
        {t('calibration.intro')}
      </div>
      <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
        {vehicles.map((v, i) => (
          <VehicleCalCard
            key={v.sys_id}
            vehicle={v}
            color={zoneColorsSolid[i % zoneColorsSolid.length]}
            state={calByVehicle[v.sys_id]}
            onStartLevel={() => cal.startLevel?.(v.sys_id)}
            onStartFull={() => cal.startFullCal?.(v.sys_id)}
            onConfirmPos={() => cal.confirmPosition?.(v.sys_id)}
            onDismiss={() => cal.dismiss?.(v.sys_id)}
          />
        ))}
      </div>
    </>
  );
}

function VehicleCalCard({ vehicle, color, state, onStartLevel, onStartFull, onConfirmPos, onDismiss }) {
  const { t } = useTranslation();
  const name = vehicle.name || `UAV ${vehicle.sys_id}`;

  return (
    <div style={{
      border: `1px solid ${colors.border}`,
      borderRadius: 6,
      padding: 12,
      background: colors.bg,
    }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 10 }}>
        <span style={{ width: 8, height: 8, borderRadius: '50%', background: color, flexShrink: 0 }} />
        <span style={{ color: colors.textBright, fontSize: 13, fontWeight: 700 }}>{name}</span>
      </div>
      <CalBody
        state={state}
        onStartLevel={onStartLevel}
        onStartFull={onStartFull}
        onConfirmPos={onConfirmPos}
        onDismiss={onDismiss}
        t={t}
      />
    </div>
  );
}

function CalBody({ state, onStartLevel, onStartFull, onConfirmPos, onDismiss, t }) {
  // Idle — offer the two calibration flows.
  if (!state) {
    return (
      <div style={{ display: 'flex', gap: 8 }}>
        <CalButton onClick={onStartLevel} title={t('calibration.levelDesc')}>
          {t('calibration.levelBtn')}
        </CalButton>
        <CalButton onClick={onStartFull} primary title={t('calibration.fullDesc')}>
          {t('calibration.fullBtn')}
        </CalButton>
      </div>
    );
  }

  // Final result — success/failed banner + dismiss.
  if (state.result) {
    const ok = state.result === 'success';
    return (
      <div>
        <div style={{
          padding: '8px 10px', borderRadius: 4, marginBottom: 8,
          background: ok ? 'rgba(76,175,80,0.15)' : 'rgba(244,67,54,0.15)',
          border: `1px solid ${ok ? colors.success : colors.error}`,
          color: ok ? colors.success : colors.error,
          fontSize: 13, fontWeight: 600,
        }}>
          {ok ? t('calibration.resultSuccess') : t('calibration.resultFailed')}
        </div>
        {state.prompt && (
          <div style={{ color: colors.textDim, fontSize: 11, marginBottom: 8 }}>{state.prompt}</div>
        )}
        <CalButton onClick={onDismiss}>{t('calibration.done')}</CalButton>
      </div>
    );
  }

  // Quick level cal in flight — no interactive prompts, just confirmation.
  if (state.mode === 'level') {
    return (
      <div>
        <div style={{ color: colors.text, fontSize: 13, marginBottom: 8 }}>
          {state.busy ? t('calibration.starting') : t('calibration.levelSent')}
        </div>
        <CalButton onClick={onDismiss}>{t('calibration.done')}</CalButton>
      </div>
    );
  }

  // Full 6-position wizard.
  const posKey = state.step != null ? ACCEL_CAL_POS_KEY_BY_CODE[state.step] : null;
  const posIndex = state.step != null
    ? ACCEL_CAL_POSITIONS.findIndex((p) => p.code === state.step)
    : -1;
  const awaiting = state.busy || state.step == null;

  return (
    <div>
      {awaiting ? (
        <div style={{ color: colors.textDim, fontSize: 13, marginBottom: 10 }}>
          {state.busy ? t('calibration.starting') : t('calibration.waiting')}
        </div>
      ) : (
        <div style={{ marginBottom: 10 }}>
          {posIndex >= 0 && (
            <div style={{ color: colors.accent, fontSize: 11, fontWeight: 600, marginBottom: 4 }}>
              {t('calibration.positionProgress', { n: posIndex + 1, total: ACCEL_CAL_POSITIONS.length })}
            </div>
          )}
          <div style={{ color: colors.textDim, fontSize: 12 }}>{t('calibration.placePrefix')}</div>
          <div style={{ color: colors.textBright, fontSize: 15, fontWeight: 700 }}>
            {posKey ? t(`calibration.positions.${posKey}`) : (state.prompt || t('calibration.waiting'))}
          </div>
        </div>
      )}
      <div style={{ display: 'flex', gap: 8 }}>
        <CalButton onClick={onConfirmPos} primary disabled={awaiting}>
          {t('calibration.nextBtn')}
        </CalButton>
        <CalButton onClick={onDismiss}>{t('calibration.cancel')}</CalButton>
      </div>
    </div>
  );
}

function CalButton({ children, onClick, primary, disabled, title }) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      title={title}
      style={{
        flex: '0 1 auto',
        padding: '7px 14px',
        fontSize: 13,
        fontWeight: 600,
        borderRadius: 4,
        cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.4 : 1,
        border: primary ? 'none' : `1px solid ${colors.border}`,
        background: primary ? colors.accent : 'transparent',
        color: primary ? '#000' : colors.text,
      }}
    >
      {children}
    </button>
  );
}
