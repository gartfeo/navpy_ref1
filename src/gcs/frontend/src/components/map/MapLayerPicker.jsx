import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import MapIconBtn from '../MapIconBtn';
import { BASE_LAYERS, isBaseLayerAvailable } from './utils/baseLayers';

/** Base map switch: a layers button that opens one choice per base layer. */
export default function MapLayerPicker({ value, onChange, hasIon }) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);

  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
      {open && (
        <div role="group" aria-label={t('mapLayers.title')} style={{
          display: 'flex',
          gap: 4,
          background: 'rgba(22, 33, 62, 0.85)',
          border: `1px solid ${colors.border}`,
          borderRadius: 6,
          padding: 3,
          backdropFilter: 'blur(4px)',
        }}>
          {BASE_LAYERS.map((layer) => {
            const available = isBaseLayerAvailable(layer.id, hasIon);
            const active = layer.id === value;
            return (
              <button
                key={layer.id}
                type="button"
                aria-pressed={active}
                disabled={!available}
                title={available ? t(layer.labelKey) : t('mapLayers.needsIon')}
                onClick={() => { onChange(layer.id); setOpen(false); }}
                style={{
                  background: active ? 'rgba(0, 210, 255, 0.2)' : 'transparent',
                  color: !available ? colors.textDim : (active ? colors.accent : colors.textBright),
                  border: `1px solid ${active ? colors.accent : 'transparent'}`,
                  borderRadius: 4,
                  padding: '4px 8px',
                  fontSize: 12,
                  cursor: available ? 'pointer' : 'not-allowed',
                }}
              >
                {t(layer.labelKey)}
              </button>
            );
          })}
        </div>
      )}
      <MapIconBtn title={t('mapLayers.title')} active={open} onClick={() => setOpen((o) => !o)}>
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <path d="M7 1.5 L12.5 4.5 L7 7.5 L1.5 4.5 Z" stroke="currentColor" strokeWidth="1.3" strokeLinejoin="round"/>
          <path d="M1.5 7.2 L7 10.2 L12.5 7.2" stroke="currentColor" strokeWidth="1.3" strokeLinejoin="round"/>
          <path d="M1.5 9.8 L7 12.8 L12.5 9.8" stroke="currentColor" strokeWidth="1.3" strokeLinejoin="round"/>
        </svg>
      </MapIconBtn>
    </div>
  );
}
