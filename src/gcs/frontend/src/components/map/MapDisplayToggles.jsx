import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';

export default function MapDisplayToggles({
  phase,
  searchPattern,
  showZones,
  setShowZones,
  showTracks,
  setShowTracks,
  showLaunchZone,
  setShowLaunchZone,
}) {
  const { t } = useTranslation();
  const planning = phase === 'PLANNING';

  const showZoneToggle = planning && searchPattern !== 'corridor';
  const showTrackToggle = planning;
  const showLzToggle = planning;

  if (!showZoneToggle && !showTrackToggle && !showLzToggle) return null;

  return (
    <div style={{
      position: 'absolute',
      top: 12,
      left: 12,
      display: 'flex',
      alignItems: 'center',
      gap: 10,
      background: 'rgba(22, 33, 62, 0.85)',
      border: `1px solid ${colors.border}`,
      borderRadius: 6,
      padding: '4px 10px',
      backdropFilter: 'blur(4px)',
      zIndex: 10,
    }}>
      {showZoneToggle && <Toggle checked={showZones} onChange={setShowZones} label={t('mapToggles.zones')} />}
      {showTrackToggle && <Toggle checked={showTracks} onChange={setShowTracks} label={t('mapToggles.tracks')} />}
      {showLzToggle && <Toggle checked={showLaunchZone} onChange={setShowLaunchZone} label={t('mapToggles.launchZone')} />}
    </div>
  );
}

function Toggle({ checked, onChange, label }) {
  return (
    <label style={{
      display: 'flex',
      alignItems: 'center',
      gap: 5,
      cursor: 'pointer',
      color: colors.text,
      fontSize: 12,
      userSelect: 'none',
    }}>
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        style={{ accentColor: colors.accent }}
      />
      {label}
    </label>
  );
}
