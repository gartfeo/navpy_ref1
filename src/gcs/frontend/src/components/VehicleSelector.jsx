import React, { useEffect, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, setColors } from '../styles';

const pillBase = {
  height: 36,
  padding: '0 14px',
  borderRadius: 18,
  fontSize: 12,
  fontWeight: 600,
  cursor: 'pointer',
  border: '2px solid transparent',
  touchAction: 'manipulation',
  color: colors.textBright,
  pointerEvents: 'auto',
};

export default function VehicleSelector({ vehicleList, selectedSysId, onSelect }) {
  const { t } = useTranslation();
  if (!vehicleList || vehicleList.length === 0) return null;

  const noSelection = selectedSysId == null;
  const [blinkOn, setBlinkOn] = useState(true);

  useEffect(() => {
    if (!noSelection) return;
    const id = setInterval(() => setBlinkOn((v) => !v), 600);
    return () => clearInterval(id);
  }, [noSelection]);

  return (
    <div style={{
      position: 'absolute',
      bottom: 24,
      left: '50%',
      transform: 'translateX(-50%)',
      zIndex: 10,
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      gap: 6,
      pointerEvents: 'none',
    }}>
      {noSelection && (
        <span style={{
          fontSize: 11,
          fontWeight: 600,
          color: colors.warning,
          textShadow: '0 1px 3px rgba(0,0,0,0.8)',
        }}>
          {t('vehicleSelector.selectUav')}
        </span>
      )}
      <div style={{ display: 'flex', gap: 6 }}>
        {vehicleList.map((v, i) => {
          const baseColor = setColors[i % setColors.length];
          const isSelected = v.sys_id === selectedSysId;
          return (
            <button
              key={v.sys_id}
              onClick={() => onSelect(v.sys_id)}
              style={{
                ...pillBase,
                background: isSelected
                  ? baseColor
                  : `${baseColor}66`,
                borderColor: isSelected ? '#fff'
                  : noSelection && blinkOn ? colors.warning : 'transparent',
              }}
            >
              {v.name || `UAV ${v.sys_id}`}
            </button>
          );
        })}
      </div>
    </div>
  );
}
