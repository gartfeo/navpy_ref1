import React, { useState, useEffect, useCallback } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../../styles';
import { SettingsSection } from '../SettingsField.jsx';
import { useTelemetryStore } from '../../../hooks/useTelemetryStore';

// Display window for the RC bars. Real PWM lives in ~1000–2000 µs; the wider
// window keeps captured extremes (and the occasional 980/2010) inside the bar.
const DISPLAY_MIN = 900;
const DISPLAY_MAX = 2100;
// A channel must move at least this far (max-min, µs) to count as calibrated;
// guards against writing a flat min==max range for a stick that never moved.
const MOVED_THRESHOLD_US = 20;

function toPct(pwm) {
  const p = (pwm - DISPLAY_MIN) / (DISPLAY_MAX - DISPLAY_MIN);
  return Math.max(0, Math.min(1, p)) * 100;
}

/**
 * Per-vehicle radio (RC transmitter) calibration, mirroring Mission Planner:
 * live PWM bars per channel, capture min/max while the operator sweeps the
 * sticks, capture the resting center as trim, toggle per-channel reverse, then
 * write RCn_MIN/MAX/TRIM/REVERSED. Requires a disarmed vehicle.
 *
 * Ported from feat/uav-radio-cal's standalone CalibrationTab.jsx (383 lines,
 * 0 conflicts vs dev) into the unified Calibration tab shell (INTEG-04, D-04).
 */
export default function RadioCalSection({ vehicleList, sendCommand }) {
  const { t } = useTranslation();
  const vehicles = vehicleList || [];
  const [selected, setSelected] = useState(() => vehicles[0]?.sys_id ?? null);

  // Keep the selection valid as vehicles connect/disconnect.
  useEffect(() => {
    if ((selected == null || !vehicles.some((v) => v.sys_id === selected)) && vehicles.length) {
      setSelected(vehicles[0].sys_id);
    } else if (selected != null && vehicles.length === 0) {
      setSelected(null);
    }
  }, [vehicles, selected]);

  const vehicle = useTelemetryStore((s) => (selected != null ? s.getVehicles()[selected] : null));
  const live = vehicle?.rc_channels?.channels || [];
  const armed = !!vehicle?.armed;

  const [phase, setPhase] = useState('idle');   // 'idle' | 'capturing'
  const [captured, setCaptured] = useState({});  // ch -> {min, max}
  const [trims, setTrims] = useState({});         // ch -> pwm
  const [reversed, setReversed] = useState({});   // ch -> bool
  const [status, setStatus] = useState(null);     // {key, fallback, options}
  const [saving, setSaving] = useState(false);

  // Reset the session whenever the operator switches vehicle.
  useEffect(() => {
    setPhase('idle');
    setCaptured({});
    setTrims({});
    setReversed({});
    setStatus(null);
  }, [selected]);

  // Accumulate min/max from live input while capturing. Runs each telemetry
  // frame (live is a fresh array per WS update); functional update avoids
  // needing `captured` in the dependency list.
  useEffect(() => {
    if (phase !== 'capturing' || live.length === 0) return;
    setCaptured((prev) => {
      const next = { ...prev };
      for (let i = 0; i < live.length; i++) {
        const v = live[i];
        if (v == null) continue;
        const ch = i + 1;
        const cur = next[ch] || { min: v, max: v };
        next[ch] = { min: Math.min(cur.min, v), max: Math.max(cur.max, v) };
      }
      return next;
    });
  }, [phase, live]);

  const start = useCallback(() => {
    const cap = {};
    const tr = {};
    for (let i = 0; i < live.length; i++) {
      const v = live[i];
      if (v == null) continue;
      cap[i + 1] = { min: v, max: v };
      tr[i + 1] = v;
    }
    setCaptured(cap);
    setTrims(tr);
    setReversed({});
    setStatus(null);
    setPhase('capturing');
  }, [live]);

  const captureCenter = useCallback(() => {
    const tr = {};
    for (let i = 0; i < live.length; i++) {
      const v = live[i];
      if (v != null) tr[i + 1] = v;
    }
    setTrims((prev) => ({ ...prev, ...tr }));
    setStatus({ key: 'calibrationTab.status.centerCaptured', fallback: 'Center captured.' });
  }, [live]);

  const cancel = useCallback(() => {
    setPhase('idle');
    setCaptured({});
    setTrims({});
    setReversed({});
    setStatus(null);
  }, []);

  const toggleReversed = useCallback((ch) => {
    setReversed((prev) => ({ ...prev, [ch]: !prev[ch] }));
  }, []);

  const save = useCallback(async () => {
    const channels = {};
    Object.entries(captured).forEach(([ch, mm]) => {
      if (mm.max - mm.min < MOVED_THRESHOLD_US) return; // never moved -> skip
      const trim = trims[ch] != null ? trims[ch] : Math.round((mm.min + mm.max) / 2);
      channels[ch] = {
        min: Math.round(mm.min),
        max: Math.round(mm.max),
        trim: Math.round(trim),
        reversed: !!reversed[ch],
      };
    });

    if (Object.keys(channels).length === 0) {
      setStatus({ key: 'calibrationTab.status.noChannels', fallback: 'No channels moved — sweep all sticks and switches to their extremes first.' });
      return;
    }

    setSaving(true);
    setStatus({ key: 'calibrationTab.status.saving', fallback: 'Saving…' });
    const res = await sendCommand('rc_cal_save', [selected], { channels });
    setSaving(false);

    const r = res?.results?.[String(selected)];
    if (!res || !r) {
      setStatus({ key: 'calibrationTab.status.error', fallback: 'Save failed — no response from backend.' });
      return;
    }
    if (r.status === 'refused_armed') {
      setStatus({ key: 'calibrationTab.status.armedRefused', fallback: 'Vehicle is armed — calibration refused.' });
      return;
    }
    if (r.status === 'not_connected') {
      setStatus({ key: 'calibrationTab.status.notConnected', fallback: 'Vehicle not connected.' });
      return;
    }
    if (r.status === 'partial') {
      setStatus({ key: 'calibrationTab.status.partial', fallback: 'Some parameters failed to write. Check the vehicle link and retry.' });
      return;
    }
    const count = Object.keys(r.params || {}).length;
    setStatus({ key: 'calibrationTab.status.saved', fallback: 'Saved {{count}} parameters.', options: { count } });
    setPhase('idle');
  }, [captured, trims, reversed, sendCommand, selected]);

  const renderStatus = () => {
    if (!status) return null;
    const text = t(status.key, { defaultValue: status.fallback, ...(status.options || {}) });
    return <div style={{ fontSize: 12, color: colors.textDim, marginTop: 8 }}>{text}</div>;
  };

  if (vehicles.length === 0) {
    return (
      <SettingsSection title={t('calibrationTab.title')}>
        <div style={{ fontSize: 13, color: colors.textDim }}>
          {t('calibrationTab.noVehicles')}
        </div>
      </SettingsSection>
    );
  }

  const channelCount = live.length;
  const canCalibrate = !armed && channelCount > 0;

  return (
    <SettingsSection title={t('calibrationTab.title')}>
      {/* Vehicle selector */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, flexWrap: 'wrap', marginBottom: 4 }}>
        <span style={{ fontSize: 12, color: colors.textDim }}>{t('calibrationTab.selectVehicle')}</span>
        {vehicles.map((v) => {
          const active = v.sys_id === selected;
          return (
            <button
              key={v.sys_id}
              onClick={() => setSelected(v.sys_id)}
              style={{
                padding: '4px 12px',
                fontSize: 12,
                borderRadius: 4,
                border: `1px solid ${active ? colors.accent : colors.border}`,
                background: active ? colors.accent : 'transparent',
                color: active ? '#000' : colors.textDim,
                cursor: 'pointer',
                fontWeight: active ? 600 : 400,
              }}
            >
              {v.name || `UAV ${v.sys_id}`}
            </button>
          );
        })}
      </div>

      {/* Armed warning */}
      {armed && (
        <div style={{
          padding: '8px 12px',
          borderRadius: 4,
          border: `1px solid ${colors.error}`,
          background: 'rgba(244,67,54,0.12)',
          color: colors.error,
          fontSize: 12,
          fontWeight: 600,
        }}>
          {t('calibrationTab.armedWarning')}
        </div>
      )}

      {/* Waiting for RC */}
      {!armed && channelCount === 0 && (
        <div style={{ fontSize: 13, color: colors.textDim }}>
          {t('calibrationTab.waitingRc')}
        </div>
      )}

      {/* Channel bars */}
      {channelCount > 0 && (
        <>
          {phase === 'capturing' && (
            <div style={{ fontSize: 12, color: colors.warning, marginBottom: 2 }}>
              {t('calibrationTab.instructions')}
            </div>
          )}
          <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
            {live.map((value, i) => {
              const ch = i + 1;
              const mm = captured[ch];
              const trim = trims[ch];
              return (
                <ChannelBar
                  key={ch}
                  ch={ch}
                  value={value}
                  min={mm?.min}
                  max={mm?.max}
                  trim={trim}
                  reversed={!!reversed[ch]}
                  onToggleReversed={() => toggleReversed(ch)}
                  reversedLabel={t('calibrationTab.reversed')}
                  channelLabel={t('calibrationTab.channel', { n: ch })}
                  showReverse={phase === 'capturing'}
                />
              );
            })}
          </div>
        </>
      )}

      {/* Controls */}
      <div style={{ display: 'flex', gap: 8, marginTop: 10, flexWrap: 'wrap' }}>
        {phase === 'idle' ? (
          <button
            onClick={start}
            disabled={!canCalibrate}
            style={primaryBtn(canCalibrate)}
          >
            {t('calibrationTab.start')}
          </button>
        ) : (
          <>
            <button onClick={captureCenter} style={secondaryBtn(true)}>
              {t('calibrationTab.captureCenter')}
            </button>
            <button
              onClick={save}
              disabled={!canCalibrate || saving}
              style={primaryBtn(canCalibrate && !saving)}
            >
              {saving ? t('calibrationTab.saving') : t('calibrationTab.save')}
            </button>
            <button onClick={cancel} style={secondaryBtn(true)}>
              {t('calibrationTab.cancel')}
            </button>
          </>
        )}
      </div>

      {renderStatus()}
    </SettingsSection>
  );
}

function primaryBtn(enabled) {
  return {
    padding: '6px 16px',
    fontSize: 13,
    fontWeight: 600,
    borderRadius: 4,
    border: 'none',
    background: enabled ? colors.accent : colors.border,
    color: enabled ? '#000' : colors.textDim,
    cursor: enabled ? 'pointer' : 'not-allowed',
  };
}

function secondaryBtn(enabled) {
  return {
    padding: '6px 16px',
    fontSize: 13,
    borderRadius: 4,
    border: `1px solid ${colors.border}`,
    background: 'transparent',
    color: colors.text,
    cursor: enabled ? 'pointer' : 'not-allowed',
  };
}

function Tick({ pct, color, width = 2 }) {
  return (
    <div style={{
      position: 'absolute',
      left: `${pct}%`,
      top: 0,
      bottom: 0,
      width,
      marginLeft: -width / 2,
      background: color,
    }} />
  );
}

function ChannelBar({ ch, value, min, max, trim, reversed, onToggleReversed, reversedLabel, channelLabel, showReverse }) {
  const hasValue = value != null;
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 12 }}>
      <span style={{ color: colors.textDim, width: 52, flexShrink: 0 }}>{channelLabel}</span>
      <div style={{
        position: 'relative',
        flex: 1,
        height: 16,
        background: colors.bg,
        border: `1px solid ${colors.border}`,
        borderRadius: 3,
        overflow: 'hidden',
      }}>
        {/* captured min/max region */}
        {min != null && max != null && (
          <div style={{
            position: 'absolute',
            left: `${toPct(min)}%`,
            width: `${Math.max(0, toPct(max) - toPct(min))}%`,
            top: 0,
            bottom: 0,
            background: 'rgba(0,210,255,0.14)',
          }} />
        )}
        {min != null && <Tick pct={toPct(min)} color={colors.accent} />}
        {max != null && <Tick pct={toPct(max)} color={colors.accent} />}
        {trim != null && <Tick pct={toPct(trim)} color={colors.warning} />}
        {/* live marker */}
        {hasValue && <Tick pct={toPct(value)} color={colors.textBright} width={3} />}
      </div>
      <span style={{ color: colors.text, width: 38, textAlign: 'right', flexShrink: 0, fontVariantNumeric: 'tabular-nums' }}>
        {hasValue ? value : '--'}
      </span>
      {showReverse && (
        <label style={{ display: 'flex', alignItems: 'center', gap: 4, color: colors.textDim, cursor: 'pointer', flexShrink: 0 }}>
          <input
            type="checkbox"
            checked={reversed}
            onChange={onToggleReversed}
            style={{ accentColor: colors.accent, cursor: 'pointer' }}
          />
          {reversedLabel}
        </label>
      )}
    </div>
  );
}
