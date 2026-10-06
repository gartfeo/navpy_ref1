import React, { useState, useEffect, useCallback } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { SettingsField, SettingsSection, inputStyle } from './SettingsField.jsx';
import LaunchSequenceSection from './LaunchSequenceSection.jsx';
import LaunchTuningSection, { hasNonDefaultTuning } from './LaunchTuningSection.jsx';

export default function LaunchTab({ draft, setDraft, vehicleList, simMode, devMode }) {
  const { t } = useTranslation();
  const l = draft.launch || {};
  const set = (k, v) => setDraft({ ...draft, launch: { ...l, [k]: v } });

  const [esp32Status, setEsp32Status] = useState(null);
  const [checking, setChecking] = useState(false);
  const [simRunning, setSimRunning] = useState(false);
  const [simBusy, setSimBusy] = useState(false);

  const isContainer = l.launch_type === 'container';

  const checkEsp32 = useCallback(() => {
    setChecking(true);
    fetch('/api/control/launch/esp32-status')
      .then((r) => r.ok ? r.json() : null)
      .then((data) => {
        setEsp32Status(data);
        if (data) setSimRunning(!!data.simulator_running);
      })
      .catch(() => setEsp32Status(null))
      .finally(() => setChecking(false));
  }, []);

  // Check status on mount when container mode
  useEffect(() => {
    if (isContainer) checkEsp32();
  }, [isContainer, checkEsp32]);

  // Also poll sim status on mount
  useEffect(() => {
    if (!simMode) return;
    fetch('/api/control/launch/esp32-sim/status')
      .then((r) => r.ok ? r.json() : null)
      .then((data) => { if (data) setSimRunning(data.running); })
      .catch(() => {});
  }, [simMode]);

  const toggleSim = useCallback(async () => {
    setSimBusy(true);
    try {
      const endpoint = simRunning
        ? '/api/control/launch/esp32-sim/stop'
        : `/api/control/launch/esp32-sim/start?port=${l.esp32_port}`;
      const res = await fetch(endpoint, { method: 'POST' });
      if (res.ok) {
        setSimRunning(!simRunning);
        // Refresh ESP32 status after toggling
        setTimeout(checkEsp32, 500);
      } else {
        const err = await res.json().catch(() => null);
        alert(err?.detail || `Failed (HTTP ${res.status})`);
      }
    } catch (e) {
      alert(`Error: ${e.message}`);
    } finally {
      setSimBusy(false);
    }
  }, [simRunning, checkEsp32]);

  const channelMap = l.default_channel_map || {};

  const setChannel = (sysId, ch) => {
    const next = { ...channelMap };
    if (ch === '' || ch == null) {
      delete next[String(sysId)];
    } else {
      next[String(sysId)] = Number(ch);
    }
    set('default_channel_map', next);
  };

  return (
    <>
      <SettingsSection title={t('launchTab.launchType')}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13 }}>
          <span style={{ color: colors.text, minWidth: 80 }}>{t('launchTab.launchTypeLabel')}</span>
          <select
            value={l.launch_type || 'bungee'}
            onChange={(e) => set('launch_type', e.target.value)}
            style={{ ...inputStyle, cursor: 'pointer' }}
          >
            <option value="bungee">{t('launchTab.bungee')}</option>
            <option value="container">{t('launchTab.container')}</option>
          </select>
        </div>
        <div style={{ fontSize: 11, color: colors.textDim, marginTop: 4 }}>
          {isContainer
            ? t('launchTab.containerDesc')
            : t('launchTab.bungeeDesc')}
        </div>
      </SettingsSection>

      {isContainer && (
        <>
          <SettingsSection title={t('launchTab.esp32Connection')}>
            <SettingsField label={t('launchTab.host')} info={t('launchTab.hostInfo')} value={l.esp32_host} onChange={(v) => set('esp32_host', v)} type="text" />
            <SettingsField label={t('launchTab.port')} info={t('launchTab.portInfo')} value={l.esp32_port} onChange={(v) => set('esp32_port', Math.max(1, Math.round(v)))} step={1} />

            <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginTop: 4 }}>
              <button
                onClick={checkEsp32}
                disabled={checking}
                style={{
                  padding: '4px 12px',
                  fontSize: 12,
                  background: 'transparent',
                  border: `1px solid ${colors.border}`,
                  borderRadius: 4,
                  color: colors.text,
                  cursor: checking ? 'wait' : 'pointer',
                }}
              >
                {checking ? t('launchTab.checkingConnection') : t('launchTab.testConnection')}
              </button>
              {esp32Status && (
                <span style={{
                  fontSize: 12,
                  color: esp32Status.reachable ? colors.success : colors.error,
                  fontWeight: 600,
                }}>
                  {esp32Status.reachable ? t('launchTab.esp32Connected') : (esp32Status.reason || t('launchTab.unreachable'))}
                </span>
              )}
            </div>
          </SettingsSection>

          {simMode && (
            <SettingsSection title={t('launchTab.launchSimulator')}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
                <button
                  onClick={toggleSim}
                  disabled={simBusy}
                  style={{
                    padding: '6px 16px',
                    fontSize: 12,
                    fontWeight: 600,
                    background: simRunning ? colors.error : colors.success,
                    color: '#fff',
                    border: 'none',
                    borderRadius: 4,
                    cursor: simBusy ? 'wait' : 'pointer',
                    opacity: simBusy ? 0.6 : 1,
                  }}
                >
                  {simBusy ? '...' : simRunning ? t('launchTab.stopSimulator') : t('launchTab.startSimulator')}
                </button>
                <span style={{
                  fontSize: 12,
                  color: simRunning ? colors.success : colors.textDim,
                }}>
                  {simRunning ? t('launchTab.running', { port: l.esp32_port }) : t('launchTab.stopped')}
                </span>
              </div>
              <div style={{ fontSize: 11, color: colors.textDim, marginTop: 4 }}>
                {t('launchTab.simulatorHelp')}
              </div>
            </SettingsSection>
          )}

          <SettingsSection title={t('launchTab.timeouts')}>
            <SettingsField label={t('launchTab.altitudeThreshold')} info={t('launchTab.altitudeThresholdInfo')} value={l.altitude_threshold_m} onChange={(v) => set('altitude_threshold_m', v)} step={1} />
            <SettingsField label={t('launchTab.armTimeout')} info={t('launchTab.armTimeoutInfo')} value={l.arm_timeout_s} onChange={(v) => set('arm_timeout_s', v)} step={1} min={1} />
            <SettingsField label={t('launchTab.altitudeTimeout')} info={t('launchTab.altitudeTimeoutInfo')} value={l.altitude_timeout_s} onChange={(v) => set('altitude_timeout_s', v)} step={5} min={5} />
          </SettingsSection>
        </>
      )}

      {/* Launch sequence (+ ESP32 channel mapping) — operator-facing, drag to
          reorder the launch sequence. Always available. */}
      <LaunchSequenceSection
        vehicleList={vehicleList}
        launchOrder={l.launch_order}
        onOrderChange={(order) => set('launch_order', order)}
        isContainer={isContainer}
        channelMap={channelMap}
        onChannelChange={setChannel}
      />

      {/* Auto preflight calibration — operator-facing, default off. When on,
          START runs gyro+baro cal on disarmed vehicles before arming. */}
      <SettingsSection title={t('launchTab.preflightCal')}>
        <SettingsField
          type="checkbox"
          label={t('launchTab.autoPreflightCal')}
          info={t('launchTab.autoPreflightCalInfo')}
          value={l.auto_preflight_cal ?? false}
          onChange={(v) => set('auto_preflight_cal', v)}
          description={t('launchTab.autoPreflightCalHint')}
        />
        {l.auto_preflight_cal && (
          <SettingsField
            label={t('launchTab.preflightCalSettle')}
            info={t('launchTab.preflightCalSettleInfo')}
            value={l.preflight_cal_settle_s ?? 2.0}
            onChange={(v) => set('preflight_cal_settle_s', v)}
            step={0.5}
            min={0}
          />
        )}
      </SettingsSection>

      {/* Advanced tuning — editable only in DEV mode. Backend always applies
          saved values, so warn when non-default values are active but hidden. */}
      {devMode ? (
        <LaunchTuningSection l={l} set={set} isContainer={isContainer} />
      ) : (
        hasNonDefaultTuning(l) && (
          <div style={{
            fontSize: 11,
            color: colors.warning,
            border: `1px solid ${colors.warning}`,
            borderRadius: 4,
            padding: '6px 8px',
            marginBottom: 16,
          }}>
            {t('launchTab.tuningActiveNote')}
          </div>
        )
      )}
    </>
  );
}
