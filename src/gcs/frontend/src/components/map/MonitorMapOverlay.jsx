import React from 'react';
import { useTranslation } from 'react-i18next';
import MapIconBtn from '../MapIconBtn';
import { colors } from '../../styles';

/**
 * Monitor-mode map overlay: RC control toggle, camera follow selector,
 * vision/coverage/trail toggles and clears. Pure view — no hooks or state.
 */
export default function MonitorMapOverlay({
  vehicleList,
  followSysId, onFollowChange,
  onToggleManualControl,
  showVision, onToggleVision,
  showCoverage, onToggleCoverage,
  onClearCoverage,
  showTrails, onToggleTrails,
  onClearTrails,
  onEditPlan, planEntry,
}) {
  const { t } = useTranslation();
  const planLabel = planEntry?.isEdit ? t('monitor.editPlan') : t('monitor.startPlanning');
  return (
    <>
      {/* Top-left bar: RC toggle */}
      <div style={{
        position: 'absolute',
        top: 12,
        left: 12,
        zIndex: 10,
        display: 'flex',
        gap: 4,
        pointerEvents: 'auto',
      }}>
        <MapIconBtn
          title={t('monitorOverlay.enterRc')}
          onClick={onToggleManualControl}
        >
          <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
            <rect x="2" y="5" width="12" height="9" rx="2" stroke="currentColor" strokeWidth="1.3" fill="none"/>
            <line x1="8" y1="5" x2="8" y2="1.5" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round"/>
            <circle cx="8" cy="1.5" r="0.8" fill="currentColor"/>
            <circle cx="5.5" cy="9.5" r="1.6" stroke="currentColor" strokeWidth="1"/>
            <circle cx="5.5" cy="9.5" r="0.5" fill="currentColor"/>
            <circle cx="10.5" cy="9.5" r="1.6" stroke="currentColor" strokeWidth="1"/>
            <circle cx="10.5" cy="9.5" r="0.5" fill="currentColor"/>
          </svg>
        </MapIconBtn>
      </div>
      {/* Top-right bar: camera follow + display toggles */}
      <div style={{
        position: 'absolute',
        top: 12,
        right: 12,
        zIndex: 10,
        display: 'flex',
        alignItems: 'center',
        gap: 6,
        pointerEvents: 'auto',
      }}>
        {/* Plan entry — moved off the sidebar UAV-status header so a long
            translated label (e.g. Armenian) can never crowd E-STOP. The wide
            map absorbs the label, and it sits with the plan drawn beneath it. */}
        {planEntry?.show && (
          <button
            type="button"
            onClick={onEditPlan}
            title={planLabel}
            aria-label={planLabel}
            style={{
              height: 32,
              display: 'inline-flex',
              alignItems: 'center',
              gap: 6,
              padding: '0 12px',
              borderRadius: 6,
              fontSize: 12,
              fontWeight: 600,
              cursor: 'pointer',
              border: `1px solid ${colors.accent}`,
              background: 'rgba(22, 33, 62, 0.85)',
              color: colors.accent,
              backdropFilter: 'blur(4px)',
              whiteSpace: 'nowrap',
            }}
          >
            <svg width="13" height="13" viewBox="0 0 16 16" fill="none" aria-hidden="true" style={{ flexShrink: 0 }}>
              <path d="M10.8 2.6l2.6 2.6M2.5 13.5l2.9-.6 7.2-7.2-2.3-2.3-7.2 7.2-.6 2.9z" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
            {planLabel}
          </button>
        )}
        {/* Camera follow — combobox */}
        <select
          value={followSysId ?? ''}
          onChange={(e) => onFollowChange(e.target.value === '' ? null : Number(e.target.value))}
          style={{
            height: 32,
            padding: '0 8px',
            borderRadius: 6,
            fontSize: 12,
            fontWeight: 600,
            cursor: 'pointer',
            border: `1px solid ${colors.border}`,
            background: 'rgba(22, 33, 62, 0.85)',
            color: colors.textBright,
            backdropFilter: 'blur(4px)',
            outline: 'none',
          }}
        >
          <option value="">{t('monitorOverlay.free')}</option>
          {vehicleList.map((v) => (
            <option key={v.sys_id} value={v.sys_id}>
              {v.name || `UAV ${v.sys_id}`}
            </option>
          ))}
        </select>
        {/* Vision toggle */}
        <MapIconBtn
          title={t('monitorOverlay.toggleVision')}
          onClick={onToggleVision}
          active={showVision}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <circle cx="7" cy="2" r="1.5" stroke="currentColor" strokeWidth="1.2"/>
            <path d="M5.5 3.5L2 13h10L8.5 3.5" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" fill={showVision ? 'currentColor' : 'none'} fillOpacity="0.2"/>
          </svg>
        </MapIconBtn>
        {/* Coverage toggle */}
        <MapIconBtn
          title={t('monitorOverlay.toggleCoverage')}
          onClick={onToggleCoverage}
          active={showCoverage}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <path d="M2 11L4 3l3 1 4-2 1 9H2z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" fill={showCoverage ? 'currentColor' : 'none'} fillOpacity="0.2"/>
            <path d="M4 6h6M3 8.5h8" stroke="currentColor" strokeWidth="0.8" strokeDasharray="1.5 1.5" opacity="0.6"/>
          </svg>
        </MapIconBtn>
        {/* Clear coverage */}
        <MapIconBtn
          title={t('monitorOverlay.clearCoverage')}
          onClick={onClearCoverage}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <line x1="3" y1="3" x2="11" y2="11" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round"/>
            <line x1="11" y1="3" x2="3" y2="11" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round"/>
          </svg>
        </MapIconBtn>
        {/* Flight path trace toggle */}
        <MapIconBtn
          title={t('monitorOverlay.toggleTrails')}
          onClick={onToggleTrails}
          active={showTrails}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <path d="M1.5 12C4 12 3 6 6.5 6S9 2 12.5 2" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round" strokeDasharray={showTrails ? 'none' : '1.5 1.5'}/>
            <circle cx="12.5" cy="2" r="1.3" fill="currentColor"/>
          </svg>
        </MapIconBtn>
        {/* Clear flight path trace */}
        <MapIconBtn
          title={t('monitorOverlay.clearTrails')}
          onClick={onClearTrails}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <path d="M1.5 12C4 12 3 6 6.5 6" stroke="currentColor" strokeWidth="1.2" strokeLinecap="round" opacity="0.5"/>
            <line x1="8" y1="3" x2="12.5" y2="7.5" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round"/>
            <line x1="12.5" y1="3" x2="8" y2="7.5" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round"/>
          </svg>
        </MapIconBtn>
      </div>
    </>
  );
}
