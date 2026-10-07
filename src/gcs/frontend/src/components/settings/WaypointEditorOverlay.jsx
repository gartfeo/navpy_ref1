import React, { useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../styles';
import { inputStyle } from './SettingsField.jsx';
import MiniWaypointPath from './MiniWaypointPath.jsx';
import BitmaskInput from './BitmaskInput.jsx';

export default function WaypointEditorOverlay({ vehicleMissions, vehicleList, selectedVehicles, poisMap, navLastWpMap, onPerVehicleNavLastWpChange, vehicleParams, onToggle, onDownload, downloading, onPerVehicleTargChange, onDiscard, onApply, onClose }) {
  const { t } = useTranslation();
  const selIds = [...(selectedVehicles || [])];
  const backdropMouseDown = useRef(false);
  return (
    <div
      style={{
        position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.7)',
        display: 'flex', alignItems: 'center', justifyContent: 'center', zIndex: 10001,
      }}
      onMouseDown={(e) => { backdropMouseDown.current = e.target === e.currentPoi; }}
      onClick={(e) => { if (e.target === e.currentPoi && backdropMouseDown.current) onClose(); }}
    >
      <div style={{
        width: 800, maxHeight: '80vh', background: colors.bgLight,
        border: `1px solid ${colors.border}`, borderRadius: 8,
        display: 'flex', flexDirection: 'column', overflow: 'hidden',
      }}>
        <div style={{
          padding: '10px 16px', borderBottom: `1px solid ${colors.border}`,
          display: 'flex', alignItems: 'center', position: 'relative',
        }}>
          <span style={{ fontSize: 14, fontWeight: 700, color: colors.textBright }}>{t('vehicle.simulatedPoiSelection')}</span>
          <span style={{ position: 'absolute', left: '50%', transform: 'translateX(-50%)', fontSize: 10, color: colors.textDim }}>{t('vehicle.scrollZoomDragPan')}</span>
          <button title={t('settings.close')} onClick={onClose} style={{ marginLeft: 'auto', background: 'none', border: 'none', color: colors.textDim, cursor: 'pointer', fontSize: 18, padding: '2px 6px' }}>&times;</button>
        </div>
        <div style={{ flex: 1, overflowY: 'auto', padding: 16, display: 'flex', flexDirection: 'column', gap: 12 }}>
          <MiniWaypointPath
            vehicleMissions={vehicleMissions}
            vehicleList={vehicleList}
            selectedVehicles={selectedVehicles}
            poisMap={poisMap}
            navLastWpMap={navLastWpMap}
            onToggle={onToggle}
            onDownload={onDownload}
            downloading={downloading}
            tall
          />
          {(() => {
            const cols = selIds.map((sid) => {
              const vi = vehicleList.findIndex((v) => v.sys_id === sid);
              return {
                sid,
                clr: zoneColorsSolid[(vi >= 0 ? vi : 0) % zoneColorsSolid.length],
                name: vehicleList.find((v) => v.sys_id === sid)?.name || `UAV ${sid}`,
                lastWp: navLastWpMap?.[sid] ?? 0,
                targMask: poisMap[sid] || 0,
              };
            });
            const cellInput = { ...inputStyle, fontSize: 12, padding: '2px 4px', width: 80, boxSizing: 'border-box', textAlign: 'left' };
            return (
              <div style={{ display: 'inline-grid', gridTemplateColumns: `80px repeat(${cols.length}, 90px)`, gap: '4px 8px', alignItems: 'center', fontSize: 12, margin: '0 auto', transform: 'translateX(-44px)' }}>
                <span />
                {cols.map((c) => (
                  <span key={c.sid} style={{ display: 'flex', alignItems: 'center', gap: 3, justifyContent: 'center' }}>
                    <span style={{ width: 7, height: 7, borderRadius: '50%', background: c.clr, flexShrink: 0 }} />
                    <span style={{ color: c.clr, fontWeight: 600, fontSize: 11 }}>{c.name}</span>
                  </span>
                ))}
                <span style={{ color: colors.textDim, fontWeight: 600, fontSize: 10 }}>{t('vehicle.detectAfter')}</span>
                {cols.map((c) => (
                  <input
                    key={c.sid}
                    type="number"
                    value={c.lastWp}
                    onChange={(e) => onPerVehicleNavLastWpChange(c.sid, Math.max(0, Math.round(Number(e.target.value))))}
                    min={0} step={1}
                    style={cellInput}
                  />
                ))}
                <span style={{ color: colors.textDim, fontWeight: 600, fontSize: 10 }}>{t('planningSidebar.simPoiWps')}</span>
                {cols.map((c) => (
                  <BitmaskInput
                    key={c.sid}
                    value={c.targMask}
                    onChange={(mask) => onPerVehicleTargChange(c.sid, mask)}
                    style={cellInput}
                  />
                ))}
              </div>
            );
          })()}
        </div>
        <div style={{
          padding: '8px 16px', borderTop: `1px solid ${colors.border}`,
          display: 'flex', justifyContent: 'flex-end', gap: 8,
        }}>
          <button
            onClick={() => { onDiscard(); onClose(); }}
            style={{
              padding: '5px 14px', fontSize: 12,
              background: 'transparent', border: `1px solid ${colors.border}`,
              borderRadius: 4, color: colors.error, cursor: 'pointer',
            }}
          >
            {t('settings.discard')}
          </button>
          <button
            onClick={onApply}
            style={{
              padding: '5px 14px', fontSize: 12, fontWeight: 600,
              background: colors.accent, border: 'none',
              borderRadius: 4, color: '#000', cursor: 'pointer',
            }}
          >
            {t('settings.apply')}
          </button>
        </div>
      </div>
    </div>
  );
}
