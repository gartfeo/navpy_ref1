import React, { useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { SettingsSection } from './SettingsField.jsx';
import { DELIVERY_HUB_TYPES } from '../map/constants/deliveryHubIcons.js';

const cellStyle = {
  padding: '4px 6px',
  fontSize: 12,
  color: colors.textBright,
  borderBottom: `1px solid ${colors.border}`,
};

const inputCellStyle = {
  ...cellStyle,
  padding: 0,
};

const cellInputStyle = {
  width: '100%',
  background: 'transparent',
  border: 'none',
  color: colors.textBright,
  fontSize: 12,
  padding: '4px 6px',
  outline: 'none',
};

export default function DefaultDeliveryHubsTab({ draft, setDraft, onStartPlacing }) {
  const { t } = useTranslation();
  const deliveryHubs = draft.default_delivery_hubs || [];
  const fileRef = useRef(null);

  const setDeliveryHubs = (newDeliveryHubs) => {
    setDraft({ ...draft, default_delivery_hubs: newDeliveryHubs });
  };

  const updateDeliveryHub = (index, field, value) => {
    const updated = deliveryHubs.map((o, i) =>
      i === index ? { ...o, [field]: value } : o
    );
    setDeliveryHubs(updated);
  };

  const addDeliveryHub = () => {
    setDeliveryHubs([...deliveryHubs, { name: '', type: 'other', lat: 0, lon: 0 }]);
  };

  const removeDeliveryHub = (index) => {
    setDeliveryHubs(deliveryHubs.filter((_, i) => i !== index));
  };

  const handleImportCsv = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      const text = ev.target.result;
      const lines = text.split(/\r?\n/).filter((l) => l.trim());
      if (lines.length < 2) return;
      const headers = lines[0].split(',').map((h) => h.trim().toLowerCase());
      const nameIdx = headers.indexOf('name');
      const typeIdx = headers.indexOf('type');
      const latIdx = headers.indexOf('lat');
      const lonIdx = headers.indexOf('lon');
      if (nameIdx < 0 || latIdx < 0 || lonIdx < 0) return;
      const imported = [];
      for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',').map((c) => c.trim());
        if (cols.length <= Math.max(nameIdx, latIdx, lonIdx)) continue;
        const t = typeIdx >= 0 ? cols[typeIdx].toLowerCase() : 'other';
        imported.push({
          name: cols[nameIdx],
          type: DELIVERY_HUB_TYPES.includes(t) ? t : 'other',
          lat: parseFloat(cols[latIdx]),
          lon: parseFloat(cols[lonIdx]),
        });
      }
      if (imported.length > 0) {
        setDeliveryHubs([...deliveryHubs, ...imported]);
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  };

  return (
    <>
      <SettingsSection title={t('defaultDeliveryHubsTab.title')}>
        <p style={{ color: colors.textDim, fontSize: 12, marginTop: 0 }}>
          {t('defaultDeliveryHubsTab.description')}
        </p>
        <div style={{ marginBottom: 8, display: 'flex', gap: 8 }}>
          <button
            onClick={addDeliveryHub}
            style={{
              padding: '4px 12px', fontSize: 12,
              background: colors.surface, border: `1px solid ${colors.border}`,
              borderRadius: 4, color: colors.textBright, cursor: 'pointer',
            }}
          >
            {t('defaultDeliveryHubsTab.add')}
          </button>
          <button
            onClick={() => fileRef.current?.click()}
            style={{
              padding: '4px 12px', fontSize: 12,
              background: colors.surface, border: `1px solid ${colors.border}`,
              borderRadius: 4, color: colors.textBright, cursor: 'pointer',
            }}
          >
            {t('defaultDeliveryHubsTab.importCsv')}
          </button>
          {onStartPlacing && (
            <button
              onClick={onStartPlacing}
              style={{
                padding: '4px 12px', fontSize: 12,
                background: colors.surface, border: `1px solid ${colors.border}`,
                borderRadius: 4, color: colors.textBright, cursor: 'pointer',
              }}
            >
              {t('defaultDeliveryHubsTab.placeOnMap')}
            </button>
          )}
          <input
            ref={fileRef}
            type="file"
            accept=".csv"
            style={{ display: 'none' }}
            onChange={handleImportCsv}
          />
          {deliveryHubs.length > 0 && (
            <button
              onClick={() => { if (window.confirm(t('defaultDeliveryHubsTab.clearConfirm'))) setDeliveryHubs([]); }}
              style={{
                padding: '4px 12px', fontSize: 12, marginLeft: 'auto',
                background: 'transparent', border: `1px solid ${colors.border}`,
                borderRadius: 4, color: colors.warning, cursor: 'pointer',
              }}
            >
              {t('defaultDeliveryHubsTab.clearAll')}
            </button>
          )}
        </div>

        {deliveryHubs.length === 0 ? (
          <div style={{ color: colors.textDim, fontSize: 12, padding: 8 }}>
            {t('defaultDeliveryHubsTab.noHubs')}
          </div>
        ) : (
          <div style={{ overflowX: 'auto' }}>
            <table style={{ width: '100%', borderCollapse: 'collapse' }}>
              <thead>
                <tr>
                  {[t('defaultDeliveryHubsTab.name'), t('defaultDeliveryHubsTab.type'), t('defaultDeliveryHubsTab.lat'), t('defaultDeliveryHubsTab.lon'), ''].map((h) => (
                    <th key={h} style={{
                      ...cellStyle, textAlign: 'left', fontWeight: 600,
                      color: colors.textDim, fontSize: 11,
                    }}>{h}</th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {deliveryHubs.map((location, i) => (
                  <tr key={i}>
                    <td style={inputCellStyle}>
                      <input
                        value={location.name}
                        onChange={(e) => updateDeliveryHub(i, 'name', e.target.value)}
                        style={cellInputStyle}
                        placeholder={t('defaultDeliveryHubsTab.name')}
                      />
                    </td>
                    <td style={inputCellStyle}>
                      <select
                        value={location.type}
                        onChange={(e) => updateDeliveryHub(i, 'type', e.target.value)}
                        style={{ ...cellInputStyle, cursor: 'pointer', background: colors.surface }}
                      >
                        {DELIVERY_HUB_TYPES.map((deliveryHubType) => (
                          <option key={deliveryHubType} value={deliveryHubType} style={{ background: colors.surface, color: colors.textBright }}>{t('deliveryHub.types.' + deliveryHubType)}</option>
                        ))}
                      </select>
                    </td>
                    <td style={inputCellStyle}>
                      <input
                        type="number"
                        value={location.lat}
                        step={0.0001}
                        onChange={(e) => updateDeliveryHub(i, 'lat', parseFloat(e.target.value) || 0)}
                        style={{ ...cellInputStyle, width: 100 }}
                      />
                    </td>
                    <td style={inputCellStyle}>
                      <input
                        type="number"
                        value={location.lon}
                        step={0.0001}
                        onChange={(e) => updateDeliveryHub(i, 'lon', parseFloat(e.target.value) || 0)}
                        style={{ ...cellInputStyle, width: 100 }}
                      />
                    </td>
                    <td style={cellStyle}>
                      <button
                        onClick={() => removeDeliveryHub(i)}
                        style={{
                          background: 'none', border: 'none',
                          color: colors.error, cursor: 'pointer',
                          fontSize: 14, padding: '0 4px',
                        }}
                        title={t('defaultDeliveryHubsTab.remove')}
                      >&times;</button>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}

        <div style={{ color: colors.textDim, fontSize: 11, marginTop: 8 }}>
          {t('defaultDeliveryHubsTab.csvFormat', { types: DELIVERY_HUB_TYPES.join(', ') })}
        </div>
      </SettingsSection>
    </>
  );
}
