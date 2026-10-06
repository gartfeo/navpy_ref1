import React, { useState, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import VisionTab from './settings/VisionTab.jsx';
import FlightTab from './settings/FlightTab.jsx';
import ConnectionTab from './settings/ConnectionTab.jsx';
import MapTab from './settings/MapTab.jsx';
import AasTab from './settings/AasTab.jsx';
import FallbackDeliveryLocationsTab from './settings/FallbackDeliveryLocationsTab.jsx';
import LaunchTab from './settings/LaunchTab.jsx';
import ParametersTab from './settings/ParametersTab.jsx';
import FailsafeTab from './settings/FailsafeTab.jsx';
import CalibrationTab from './settings/CalibrationTab.jsx';

    const TABS = ['UAV', 'Parameters', 'Failsafe', 'Vision', 'Flight', 'Connection', 'Map', 'FallbackDeliveryLocations', 'Launch', 'Calibration'];

const TAB_COMPONENTS = {
  Vision: VisionTab,
  Flight: FlightTab,
  Connection: ConnectionTab,
  Map: MapTab,
  UAV: AasTab,
  Parameters: ParametersTab,
  Failsafe: FailsafeTab,
  FallbackDeliveryLocations: FallbackDeliveryLocationsTab,
  Launch: LaunchTab,
  Calibration: CalibrationTab,
};

export default function SettingsModal({ settings, onSave, onReset, onClose, onOpenConnect, vehicleList, initialTab, onVehicleTargWpsChange, onVehicleNavLastWpChange, onStartPlacingFallbackLocation, aasParams, fullParams, sendCommand, compassCal, accelCal }) {
  const { t, i18n } = useTranslation();
  const TAB_LABELS = {
    UAV: t('settings.tabs.uav'),
    Parameters: t('settings.tabs.parameters'),
    Failsafe: t('settings.tabs.failsafe'),
    Vision: t('settings.tabs.vision'),
    Flight: t('settings.tabs.flight'),
    Connection: t('settings.tabs.connection'),
    Map: t('settings.tabs.map'),
    FallbackDeliveryLocations: t('settings.tabs.fallbackDeliveryLocations'),
    Launch: t('settings.tabs.launch'),
    Calibration: t('settings.tabs.calibration'),
  };
  const [draft, setDraft] = useState(settings || {});
  const [activeTab, setActiveTab] = useState(initialTab || TABS[0]);
  const [vehicleMissions, setVehicleMissions] = useState({});
  const preSaveRef = useRef(null);
  const preResetRef = useRef(null);
  const backdropMouseDown = useRef(false);

  useEffect(() => {
    if (settings) setDraft(settings);
  }, [settings]);

  const simMode = draft?.simulation?.sim_mode ?? false;
  const devMode = draft?.simulation?.dev_mode ?? false;
  const TabComponent = TAB_COMPONENTS[activeTab];

  const toggleSimMode = () => {
    const next = !simMode;
    setDraft((prev) => ({
      ...prev,
      simulation: {
        ...(prev.simulation || {}),
        sim_mode: next,
        // turning off demo also turns off dev
        ...(!next && { dev_mode: false }),
      },
    }));
  };

  const toggleDevMode = () => {
    setDraft((prev) => ({
      ...prev,
      simulation: { ...(prev.simulation || {}), dev_mode: !devMode },
    }));
  };

  return (
    <div
      style={{
        position: 'fixed',
        inset: 0,
        background: 'rgba(0,0,0,0.6)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 9999,
      }}
      onMouseDown={(e) => { backdropMouseDown.current = e.target === e.currentTarget; }}
      onClick={(e) => { if (e.target === e.currentTarget && backdropMouseDown.current) onClose(); }}
    >
      <div
        style={{
          width: activeTab === 'Parameters' ? 'min(1200px, 95vw)' : 700,
          height: activeTab === 'Parameters' ? '85vh' : undefined,
          maxHeight: '85vh',
          background: colors.bgLight,
          border: `1px solid ${colors.border}`,
          borderRadius: 8,
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden',
        }}
      >
        {/* Header */}
        <div style={{
          padding: '12px 16px',
          borderBottom: `1px solid ${colors.border}`,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
        }}>
          <span style={{ fontSize: 16, fontWeight: 700, color: colors.textBright }}>{t('settings.title')}</span>
          <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
            <select
              value={i18n.language}
              onChange={(e) => i18n.changeLanguage(e.target.value)}
              style={{
                padding: '3px 8px',
                fontSize: 12,
                borderRadius: 4,
                border: `1px solid ${colors.border}`,
                background: 'transparent',
                color: colors.textDim,
                cursor: 'pointer',
              }}
            >
              <option value="en">EN</option>
              <option value="hy">{'\u0540\u0545'}</option>
            </select>
            <button
              onClick={toggleSimMode}
              style={{
                padding: '3px 10px',
                fontSize: 12,
                fontWeight: 700,
                borderRadius: 4,
                border: `1px solid ${simMode ? colors.warning : colors.border}`,
                background: simMode ? colors.warning : 'transparent',
                color: simMode ? '#000' : colors.textDim,
                cursor: 'pointer',
              }}
            >
              DEMO
            </button>
            {simMode && (
              <button
                onClick={toggleDevMode}
                style={{
                  padding: '3px 10px',
                  fontSize: 12,
                  fontWeight: 700,
                  borderRadius: 4,
                  border: `1px solid ${devMode ? colors.error : colors.border}`,
                  background: devMode ? colors.error : 'transparent',
                  color: devMode ? '#fff' : colors.textDim,
                  cursor: 'pointer',
                }}
              >
                DEV
              </button>
            )}
            <button
              onClick={onClose}
              style={{
                background: 'none', border: 'none', color: colors.textDim,
                cursor: 'pointer', fontSize: 18, padding: '2px 6px',
              }}
            >
              &times;
            </button>
          </div>
        </div>

        {/* Tabs */}
        <div style={{
          display: 'flex',
          flexWrap: 'wrap',
          gap: 0,
          borderBottom: `1px solid ${colors.border}`,
          padding: '0 16px',
        }}>
          {TABS.map((tab) => (
            <button
              key={tab}
              onClick={() => setActiveTab(tab)}
              style={{
                flexShrink: 0,
                whiteSpace: 'nowrap',
                padding: '8px 14px',
                fontSize: 13,
                fontWeight: activeTab === tab ? 600 : 400,
                color: activeTab === tab ? colors.accent : colors.textDim,
                background: 'none',
                border: 'none',
                borderBottom: activeTab === tab ? `2px solid ${colors.accent}` : '2px solid transparent',
                cursor: 'pointer',
              }}
            >
              {TAB_LABELS[tab] || tab}
            </button>
          ))}
        </div>

        {/* Content */}
        <div style={{
          flex: 1,
          overflow: activeTab === 'Parameters' ? 'hidden' : 'auto',
          padding: activeTab === 'Parameters' ? '8px 12px 8px 12px' : 16,
          display: 'flex',
          flexDirection: 'column',
          minHeight: 0,
        }}>
          {TabComponent && <TabComponent draft={draft} setDraft={setDraft} vehicleList={vehicleList} onOpenConnect={onOpenConnect} onClose={onClose} vehicleMissions={vehicleMissions} setVehicleMissions={setVehicleMissions} simMode={simMode} devMode={devMode} onVehicleTargWpsChange={onVehicleTargWpsChange} onVehicleNavLastWpChange={onVehicleNavLastWpChange} onStartPlacing={activeTab === 'FallbackDeliveryLocations' ? onStartPlacingFallbackLocation : undefined} preSaveRef={preSaveRef} preResetRef={preResetRef} aasParams={aasParams} fullParams={fullParams} sendCommand={sendCommand} compassCal={compassCal} accelCal={accelCal} />}
        </div>

        {/* Footer buttons */}
        <div style={{
          padding: '10px 16px',
          borderTop: `1px solid ${colors.border}`,
          display: 'flex',
          justifyContent: 'space-between',
        }}>
          <button
            onClick={() => { if (preResetRef.current) preResetRef.current(); onReset(); }}
            style={{
              padding: '6px 14px', fontSize: 13,
              background: 'transparent', border: `1px solid ${colors.border}`,
              borderRadius: 4, color: colors.text, cursor: 'pointer',
            }}
          >
            {t('settings.reset')}
          </button>
          <div style={{ display: 'flex', gap: 8 }}>
            <button
              onClick={onClose}
              style={{
                padding: '6px 14px', fontSize: 13,
                background: 'transparent', border: `1px solid ${colors.border}`,
                borderRadius: 4, color: colors.text, cursor: 'pointer',
              }}
            >
              {t('settings.cancel')}
            </button>
            <button
              onClick={async () => { if (preSaveRef.current) await preSaveRef.current(); onSave(draft); }}
              style={{
                padding: '6px 14px', fontSize: 13,
                background: colors.accent, border: 'none',
                borderRadius: 4, color: '#000', fontWeight: 600, cursor: 'pointer',
              }}
            >
              {t('settings.apply')}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
