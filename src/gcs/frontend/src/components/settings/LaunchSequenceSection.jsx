import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { SettingsSection, inputStyle } from './SettingsField.jsx';
import { orderVehiclesByLaunch, resolveChannelMap } from '../../utils/launchSequence';

/**
 * Operator-facing launch-sequence editor merged with ESP32 channel mapping.
 * Use the up/down buttons to set the launch order (persisted as
 * launch.launch_order); pick each vehicle's relay channel (container only).
 * Not DEV-gated.
 */
export default function LaunchSequenceSection({
  vehicleList, launchOrder, onOrderChange, isContainer, channelMap, onChannelChange,
}) {
  const { t } = useTranslation();

  if (!vehicleList || vehicleList.length === 0) return null;

  const ordered = orderVehiclesByLaunch(vehicleList, launchOrder);
  // Channel each "Auto" vehicle will actually trigger (matches the backend).
  const resolvedChannels = resolveChannelMap(channelMap, ordered.map((v) => v.sys_id));

  const move = (from, to) => {
    if (to < 0 || to >= ordered.length || from === to) return;
    const ids = ordered.map((v) => v.sys_id);
    const [moved] = ids.splice(from, 1);
    ids.splice(to, 0, moved);
    onOrderChange(ids);
  };

  const btnStyle = (enabled) => ({
    padding: '2px 8px',
    fontSize: 12,
    lineHeight: 1,
    background: 'transparent',
    border: `1px solid ${colors.border}`,
    borderRadius: 4,
    color: enabled ? colors.text : colors.textDim,
    cursor: enabled ? 'pointer' : 'not-allowed',
    opacity: enabled ? 1 : 0.4,
  });

  return (
    <SettingsSection title={t('launchTab.sequence')}>
      <div style={{ fontSize: 11, color: colors.textDim, marginBottom: 4 }}>
        {t('launchTab.sequenceHelp')}
      </div>
      {ordered.map((v, i) => (
        <div
          key={v.sys_id}
          style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 13, padding: '3px 4px' }}
        >
          <span style={{ color: colors.textDim, minWidth: 16, fontVariantNumeric: 'tabular-nums' }}>
            {i + 1}.
          </span>
          <span style={{ color: colors.text, flex: 1 }}>
            {v.name} (ID {v.sys_id})
          </span>
          <button
            type="button"
            onClick={() => move(i, i - 1)}
            disabled={i === 0}
            title={t('launchTab.moveUp')}
            style={btnStyle(i !== 0)}
          >
            {'↑'}
          </button>
          <button
            type="button"
            onClick={() => move(i, i + 1)}
            disabled={i === ordered.length - 1}
            title={t('launchTab.moveDown')}
            style={btnStyle(i !== ordered.length - 1)}
          >
            {'↓'}
          </button>
          {isContainer && (
            <select
              value={channelMap?.[String(v.sys_id)] ?? ''}
              onChange={(e) => onChannelChange(v.sys_id, e.target.value)}
              style={{ ...inputStyle, maxWidth: 110, cursor: 'pointer' }}
            >
              <option value="">
                {resolvedChannels[v.sys_id] != null
                  ? t('launchTab.autoChannelAssigned', { ch: resolvedChannels[v.sys_id] })
                  : t('launchTab.autoChannel')}
              </option>
              {[1, 2, 3, 4, 5, 6].map((ch) => (
                <option key={ch} value={ch}>{t('launchTab.channel', { ch })}</option>
              ))}
            </select>
          )}
        </div>
      ))}
    </SettingsSection>
  );
}
