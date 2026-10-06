import React, { useEffect, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../../styles';
import ConfirmModal from '../../ConfirmModal.jsx';
import { SettingsSection } from '../SettingsField.jsx';
import { statusKey, MAG_CAL_STATUS } from '../../../utils/compassCal';

const FAILED_STATUSES = new Set([
  MAG_CAL_STATUS.FAILED,
  MAG_CAL_STATUS.BAD_ORIENTATION,
  MAG_CAL_STATUS.BAD_RADIUS,
]);

function barColor(status) {
  if (status === MAG_CAL_STATUS.SUCCESS) return colors.success;
  if (FAILED_STATUSES.has(status)) return colors.error;
  return colors.accent;
}

function CompassProgressRow({ t, id, compass }) {
  const pct = Math.max(0, Math.min(100, compass.pct ?? 0));
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 10, fontSize: 12 }}>
      <span style={{ color: colors.textDim, minWidth: 78 }}>
        {t('calibration.compass', { id })}
      </span>
      <div style={{ flex: 1, height: 8, background: colors.bg, borderRadius: 4, overflow: 'hidden' }}>
        <div style={{ width: `${pct}%`, height: '100%', background: barColor(compass.status), transition: 'width 0.2s' }} />
      </div>
      <span style={{ color: colors.text, minWidth: 36, textAlign: 'right' }}>{pct}%</span>
      <span style={{ color: barColor(compass.status), minWidth: 90 }}>
        {t(`calibration.status.${statusKey(compass.status)}`)}
      </span>
    </div>
  );
}

const btnBase = {
  padding: '5px 14px', fontSize: 13, borderRadius: 4,
  border: 'none', cursor: 'pointer', fontWeight: 600,
};

function VehicleCalCard({ t, vehicle, state, accepted, onStart, onCancel, onAccept, onReboot }) {
  const status = state?.status || 'idle';
  const compasses = state?.compasses || {};
  const compassIds = Object.keys(compasses).sort((a, b) => a - b);
  const isRunning = status === 'running';
  const isSuccess = status === 'success';
  const isFailed = status === 'failed';

  return (
    <div style={{
      border: `1px solid ${colors.border}`, borderRadius: 6,
      padding: 12, marginBottom: 10, background: colors.bg,
    }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 8 }}>
        <span style={{ color: colors.textBright, fontWeight: 600, fontSize: 14 }}>
          {vehicle.name || `UAV ${vehicle.sys_id}`}
          <span style={{ color: colors.textDim, fontWeight: 400, marginLeft: 6, fontSize: 12 }}>
            (sysid {vehicle.sys_id})
          </span>
        </span>
        <span style={{
          fontSize: 12, fontWeight: 600,
          color: isSuccess ? colors.success : isFailed ? colors.error : isRunning ? colors.accent : colors.textDim,
        }}>
          {t(`calibration.state.${status}`)}
        </span>
      </div>

      {compassIds.length > 0 && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 6, marginBottom: 10 }}>
          {compassIds.map((id) => (
            <CompassProgressRow key={id} t={t} id={id} compass={compasses[id]} />
          ))}
        </div>
      )}

      {/* Onboard mag calibration only progresses while the vehicle is
          physically rotated through orientations — a stationary vehicle
          reports 0%/Step 1 indefinitely, which is correct autopilot
          behavior, not a stalled connection. Keep this hint visible for the
          whole running duration (not just before the first progress tick,
          which arrives almost instantly and would otherwise hide the
          guidance the moment it's most needed). */}
      {isRunning && (
        <div style={{ color: colors.textDim, fontSize: 12, marginBottom: 10 }}>
          {t('calibration.startingHint')}
        </div>
      )}

      {isSuccess && accepted && (
        <div style={{
          color: colors.warning, fontSize: 12, marginBottom: 10,
          padding: '6px 8px', border: `1px solid ${colors.warning}`, borderRadius: 4,
        }}>
          {t('calibration.rebootRequired')}
        </div>
      )}

      <div style={{ display: 'flex', gap: 8 }}>
        {(status === 'idle' || isFailed || status === 'cancelled') && (
          <button style={{ ...btnBase, background: colors.accent, color: '#000' }} onClick={() => onStart(vehicle.sys_id)}>
            {t('calibration.start')}
          </button>
        )}
        {isRunning && (
          <button style={{ ...btnBase, background: colors.warning, color: '#000' }} onClick={() => onCancel(vehicle.sys_id)}>
            {t('calibration.cancel')}
          </button>
        )}
        {isSuccess && !accepted && (
          <button style={{ ...btnBase, background: colors.success, color: '#000' }} onClick={() => onAccept(vehicle.sys_id)}>
            {t('calibration.accept')}
          </button>
        )}
        {isSuccess && accepted && (
          <button style={{ ...btnBase, background: colors.primary, color: '#fff' }} onClick={() => onReboot(vehicle.sys_id)}>
            {t('calibration.reboot')}
          </button>
        )}
      </div>
    </div>
  );
}

/**
 * Per-vehicle onboard compass (magnetometer) calibration: start/cancel,
 * live per-compass progress via compass_cal_listener -> WS -> useCompassCal,
 * accept, then an explicit reboot confirmation (T-1-08).
 *
 * Ported from feat/uav-compass-cal's standalone CalibrationTab.jsx (198
 * lines, 1 conflict vs dev on ConfirmModal.jsx) into the unified Calibration
 * tab shell (INTEG-04, D-04). ConfirmModal itself needed no changes — dev's
 * generic `tone`/`confirmLabel` props already cover the reboot-confirmation
 * capability the branch used its own `accent` prop for.
 */
export default function CompassCalSection({ vehicleList, compassCal }) {
  const { t } = useTranslation();
  const [accepted, setAccepted] = useState({});  // sysId -> true
  const [rebootPoi, setRebootPoi] = useState(null);

  const vehicles = vehicleList || [];
  const cal = compassCal?.calByVehicle || {};

  // Drop the "accepted" flag for any vehicle no longer in a success state
  // (re-run, cancel, failure, disconnect) so it can't skip the Accept step.
  useEffect(() => {
    setAccepted((prev) => {
      let changed = false;
      const next = {};
      for (const k of Object.keys(prev)) {
        if (cal[k]?.status === 'success') next[k] = prev[k];
        else changed = true;
      }
      return changed ? next : prev;
    });
  }, [cal]);

  const handleAccept = (sysId) => {
    setAccepted((prev) => ({ ...prev, [sysId]: true }));
    compassCal?.accept(sysId);
  };

  const handleStart = (sysId) => {
    setAccepted((prev) => {
      if (!prev[sysId]) return prev;
      const next = { ...prev };
      delete next[sysId];
      return next;
    });
    compassCal?.start(sysId);
  };

  const confirmReboot = () => {
    if (rebootPoi != null) compassCal?.reboot(rebootPoi);
    setRebootPoi(null);
  };

  return (
    <SettingsSection title={t('calibration.title')}>
      <div style={{ color: colors.textDim, fontSize: 12, marginBottom: 12 }}>
        {t('calibration.description')}
      </div>

      {vehicles.length === 0 ? (
        <div style={{ color: colors.textDim, fontSize: 13 }}>{t('calibration.noVehicles')}</div>
      ) : (
        vehicles.map((v) => (
          <VehicleCalCard
            key={v.sys_id}
            t={t}
            vehicle={v}
            state={cal[v.sys_id]}
            accepted={!!accepted[v.sys_id]}
            onStart={handleStart}
            onCancel={(sysId) => compassCal?.cancel(sysId)}
            onAccept={handleAccept}
            onReboot={(sysId) => setRebootPoi(sysId)}
          />
        ))
      )}

      {rebootPoi != null && (
        <ConfirmModal
          title={t('calibration.rebootConfirmTitle')}
          message={t('calibration.rebootConfirmMessage')}
          tone="caution"
          confirmLabel={t('calibration.reboot')}
          onConfirm={confirmReboot}
          onCancel={() => setRebootPoi(null)}
        />
      )}
    </SettingsSection>
  );
}
