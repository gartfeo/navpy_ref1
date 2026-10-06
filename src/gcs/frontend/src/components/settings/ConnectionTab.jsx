import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { SettingsField, SettingsSection, inputStyle } from './SettingsField.jsx';
import { flushDiagnosticEvents } from '../../utils/diagnostics';

export default function ConnectionTab({ draft, setDraft, simMode }) {
  const { t } = useTranslation();
  const c = draft.connection || {};
  const set = (k, v) => setDraft({ ...draft, connection: { ...c, [k]: v } });

  const sim = draft.simulation || {};
  const presets = sim.sitl_presets || [];

  const setPreset = (idx, val) => {
    const next = [...presets];
    next[idx] = val;
    setDraft({ ...draft, simulation: { ...sim, sitl_presets: next } });
  };

  const addPreset = () => {
    const last = presets[presets.length - 1] || 'udp:0.0.0.0:14560';
    const m = last.match(/^(.+:)(\d+)$/);
    const next = m ? m[1] + (parseInt(m[2], 10) + 10) : last;
    setDraft({ ...draft, simulation: { ...sim, sitl_presets: [...presets, next] } });
  };

  const removePreset = (idx) => {
    setDraft({ ...draft, simulation: { ...sim, sitl_presets: presets.filter((_, i) => i !== idx) } });
  };

  const exportDiagnostics = async () => {
    await flushDiagnosticEvents();
    try {
      const response = await fetch('/api/diagnostics/export');
      if (!response.ok) return;
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      const anchor = document.createElement('a');
      anchor.href = url;
      const disposition = response.headers.get('Content-Disposition') || '';
      anchor.download = disposition.match(/filename="?([^";]+)"?/)?.[1] || 'gcs-diagnostics.zip';
      anchor.click();
      URL.revokeObjectURL(url);
    } catch {
      // Export failure is non-fatal; the operator can retry.
    }
  };

  return (
    <>
      <SettingsSection title={t('connectionTab.title')}>
        <SettingsField label={t('connectionTab.defaultDevice')} value={c.default_device} onChange={(v) => set('default_device', v)} type="text" />
        <SettingsField label={t('connectionTab.autoConnect')} description={t('connectionTab.autoConnectHelp')} value={c.auto_connect} onChange={(v) => set('auto_connect', v)} type="checkbox" />
        <SettingsField label={t('connectionTab.heartbeatTimeout')} value={c.heartbeat_timeout_s} onChange={(v) => set('heartbeat_timeout_s', v)} step={0.5} />
        <SettingsField label={t('connectionTab.telemetryRate')} value={c.telemetry_rate_hz} onChange={(v) => set('telemetry_rate_hz', Math.max(1, Math.round(v)))} step={1} />
        <SettingsField label={t('connectionTab.wsBroadcast')} value={c.ws_broadcast_interval_s} onChange={(v) => set('ws_broadcast_interval_s', v)} step={0.05} />
        <SettingsField label={t('connectionTab.reconnectBase')} value={c.reconnect_base_s} onChange={(v) => set('reconnect_base_s', v)} step={0.5} />
        <SettingsField label={t('connectionTab.reconnectMax')} value={c.reconnect_max_s} onChange={(v) => set('reconnect_max_s', v)} step={1} />
      </SettingsSection>

      <SettingsSection title={t('connectionTab.diagnostics')}>
        <div style={{ fontSize: 12, color: colors.textDim }}>
          {t('connectionTab.diagnosticsHelp')}
        </div>
        <button
          type="button"
          onClick={exportDiagnostics}
          style={{
            alignSelf: 'flex-start', padding: '6px 12px', fontSize: 12,
            background: 'transparent', border: `1px solid ${colors.accent}`,
            borderRadius: 4, color: colors.accent, cursor: 'pointer',
          }}
        >
          {t('connectionTab.exportDiagnostics')}
        </button>
      </SettingsSection>

      {simMode && (
        <SettingsSection title={t('connectionTab.sitlConnections')}>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
            {presets.map((preset, i) => (
              <div key={i} style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13 }}>
                <span style={{ color: colors.textDim, minWidth: 50, flexShrink: 0 }}>{t('connectionTab.id', { index: i + 1 })}</span>
                <input
                  type="text"
                  value={preset}
                  onChange={(e) => setPreset(i, e.target.value)}
                  style={{ ...inputStyle, flex: 1, maxWidth: 240 }}
                />
                {presets.length > 1 && (
                  <button
                    onClick={() => removePreset(i)}
                    style={{
                      background: 'transparent',
                      border: `1px solid ${colors.border}`,
                      borderRadius: 4,
                      color: colors.textDim,
                      fontSize: 11,
                      padding: '2px 8px',
                      cursor: 'pointer',
                    }}
                  >
                    {t('connectionTab.remove')}
                  </button>
                )}
              </div>
            ))}
          </div>
          <button
            onClick={addPreset}
            style={{
              alignSelf: 'flex-start',
              padding: '4px 12px',
              fontSize: 12,
              background: 'transparent',
              border: `1px solid ${colors.accent}`,
              borderRadius: 4,
              color: colors.accent,
              cursor: 'pointer',
            }}
          >
            {t('connectionTab.add')}
          </button>
          <div style={{ fontSize: 11, color: colors.textDim }}>
            {t('connectionTab.sitlHelp')}
          </div>
        </SettingsSection>
      )}
    </>
  );
}
