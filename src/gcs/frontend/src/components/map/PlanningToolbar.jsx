import React from 'react';
import { useTranslation } from 'react-i18next';
import MapIconBtn from '../MapIconBtn';
import { colors, setColors } from '../../styles';
import { DELIVERY_HUB_TYPES, getDeliveryHubIcon } from './constants/deliveryHubIcons.js';

export default function PlanningToolbar({
  isDrawing,
  onStartDraw,
  onStopDraw,
  searchPattern,
  polygon,
  launchPoint,
  corridorPoints,
  placingCorridor,
  onToggleCorridor,
  effectiveSets,
  activeSetIndex,
  setActiveSetIndex,
  placingDeliveryHub,
  onToggleDeliveryHub,
  placingDeliveryHubType,
  setPlacingDeliveryHubType,
  fenceEnabled,
  onToggleFence,
  observedFenceMode,
  observedFenceAutoenableMode,
  fenceMapSource,
  placingExclusion,
  onToggleExclusion,
  onUndo,
  canUndo,
  onClearAll,
  onLoadPolygon,
  onSavePolygon,
}) {
  const { t } = useTranslation();
  // The observed mode, plus the raw FENCE_AUTOENABLE value when the whole
  // fleet agrees on one. The value is the only trigger fact we have: the
  // wording stays mode-neutral because e.g. 3 is ONLY_WHEN_ARMED and 2 is
  // ENABLE_DISABLE_FLOOR_ONLY, not "at takeoff".
  const observedSuffix = observedFenceMode && observedFenceMode !== 'none'
    ? t(`planningToolbar.fenceObserved.${observedFenceMode}`)
      + (observedFenceAutoenableMode != null ? ` (FENCE_AUTOENABLE=${observedFenceAutoenableMode})` : '')
    : null;
  return (
    <div style={{
      position: 'absolute',
      top: 12,
      right: 12,
      display: 'flex',
      flexDirection: 'column',
      // Right-hugging, so a wider child (the fence source label) no longer
      // stretches the icon rows and their anchored popups.
      alignItems: 'flex-end',
      gap: 4,
      zIndex: 10,
    }}>
      {/* Draw zone toggle — hidden in corridor mode */}
      {searchPattern !== 'corridor' && (
        <MapIconBtn
          title={isDrawing ? t('planningToolbar.finishDrawing') : t('planningToolbar.drawZone')}
          onClick={isDrawing ? onStopDraw : onStartDraw}
          active={isDrawing}
          disabled={false}
        >
          <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
            <path d="M3 12L2 5l4-3h4l4 3-1 7H3z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" fill="none"/>
            <circle cx="3" cy="12" r="1.5" fill="currentColor"/>
            <circle cx="2" cy="5" r="1.5" fill="currentColor"/>
            <circle cx="6" cy="2" r="1.5" fill="currentColor"/>
            <circle cx="10" cy="2" r="1.5" fill="currentColor"/>
            <circle cx="14" cy="5" r="1.5" fill="currentColor"/>
            <circle cx="13" cy="12" r="1.5" fill="currentColor"/>
          </svg>
        </MapIconBtn>
      )}
      <div style={{ height: 4 }} />

      {/* Corridor placement toggle — single icon + set selector submenu */}
      <div style={{ position: 'relative' }}>
        <MapIconBtn
          title={placingCorridor ? t('planningToolbar.stopCorridor') : (launchPoint ? t('planningToolbar.addCorridor') : t('planningToolbar.drawCorridor'))}
          onClick={onToggleCorridor}
          active={placingCorridor}
          disabled={searchPattern !== 'corridor' && polygon.length < 3}
          color={effectiveSets > 1 ? setColors[activeSetIndex % setColors.length] : undefined}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <polyline points="1,2 12,5 2,8 13,12" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round" strokeLinejoin="round" fill="none"/>
          </svg>
        </MapIconBtn>
        {placingCorridor && effectiveSets > 1 && searchPattern !== 'corridor' && (
          <div style={{
            position: 'absolute',
            right: 36,
            top: 0,
            display: 'flex',
            flexDirection: 'column',
            gap: 1,
            background: 'rgba(22, 33, 62, 0.95)',
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
            padding: 4,
            backdropFilter: 'blur(4px)',
            whiteSpace: 'nowrap',
          }}>
            {Array.from({ length: effectiveSets }, (_, si) => {
              const color = setColors[si % setColors.length];
              const isSelected = activeSetIndex === si;
              return (
                <button
                  key={si}
                  onClick={() => setActiveSetIndex(si)}
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    gap: 6,
                    padding: '3px 8px 3px 4px',
                    background: isSelected ? 'rgba(0, 210, 255, 0.2)' : 'transparent',
                    border: isSelected ? `1px solid ${colors.accent}` : '1px solid transparent',
                    borderRadius: 4,
                    cursor: 'pointer',
                    color: isSelected ? colors.accent : colors.textBright,
                    fontSize: 11,
                  }}
                >
                  <span style={{
                    width: 10, height: 10, borderRadius: '50%',
                    background: color, flexShrink: 0,
                  }} />
                  {t('planningToolbar.set', { index: si + 1 })}
                </button>
              );
            })}
          </div>
        )}
      </div>
      {/* Geofence toggle — shield icon; enables/derives the inclusion fence.
          With the planner fence off, the tooltip reports what the connected
          vehicles hold (unknown / mixed / auto-enable / on), so the operator is
          never shown a blank shield for a fleet that is actually fenced. */}
      <MapIconBtn
        title={fenceEnabled
          ? t('planningToolbar.disableFence')
          : (observedSuffix
            ? `${t('planningToolbar.enableFence')} — ${observedSuffix}`
            : t('planningToolbar.enableFence'))}
        onClick={onToggleFence}
        active={fenceEnabled}
        disabled={polygon.length < 3}
        color={fenceEnabled ? '#ff9800' : undefined}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <path d="M7 1l5 2v4c0 3-2.2 5-5 6-2.8-1-5-3-5-6V3l5-2z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" fill={fenceEnabled ? 'currentColor' : 'none'} fillOpacity={fenceEnabled ? 0.25 : 0}/>
        </svg>
      </MapIconBtn>
      {/* Which ring the map is drawing. The planner preview and the ring read
          back from the vehicles look identical on the map, and they can differ
          (an unsent planner edit while the vehicles still hold the old ring),
          so the source has to be named. Display only — the map's own render
          selection is unchanged. */}
      {fenceMapSource && (
        <div style={{
          fontSize: 11,
          lineHeight: 1.35,
          textAlign: 'right',
          maxWidth: 148,
          padding: '3px 6px',
          // Opaque panel in the toolbar's own colour: the label sits on live
          // satellite imagery, where translucent small text is unreadable.
          background: colors.bgLight,
          border: `1px solid ${colors.border}`,
          borderRadius: 6,
          color: fenceMapSource === 'none' ? colors.textDim : colors.warning,
        }}>
          {t(`planningToolbar.fenceSource.${fenceMapSource}`)}
        </div>
      )}

      {/* Keep-out (exclusion) drawing — no-entry icon; click vertices, double-click to finish */}
      <MapIconBtn
        title={placingExclusion ? t('planningToolbar.stopKeepOut') : t('planningToolbar.drawKeepOut')}
        onClick={onToggleExclusion}
        active={placingExclusion}
        color={placingExclusion ? '#ff3b30' : undefined}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <circle cx="7" cy="7" r="5.5" stroke="currentColor" strokeWidth="1.4" fill={placingExclusion ? 'currentColor' : 'none'} fillOpacity={placingExclusion ? 0.2 : 0}/>
          <line x1="3.1" y1="3.1" x2="10.9" y2="10.9" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
        </svg>
      </MapIconBtn>

      {/* delivery hub placement — location icon + type selector (opens left) */}
      <div style={{ position: 'relative' }}>
        <MapIconBtn
          title={placingDeliveryHub ? t('planningToolbar.stopDeliveryHub') : t('planningToolbar.placeDeliveryHub')}
          onClick={onToggleDeliveryHub}
          active={placingDeliveryHub}
        >
          <img src={getDeliveryHubIcon('other')} width={14} height={14} alt="" />
        </MapIconBtn>
        {placingDeliveryHub && (
          <div style={{
            position: 'absolute',
            right: 36,
            top: 0,
            display: 'flex',
            flexDirection: 'column',
            gap: 1,
            background: 'rgba(22, 33, 62, 0.95)',
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
            padding: 4,
            backdropFilter: 'blur(4px)',
            whiteSpace: 'nowrap',
          }}>
            {DELIVERY_HUB_TYPES.map((deliveryHubType) => (
              <button
                key={deliveryHubType}
                onClick={() => setPlacingDeliveryHubType(deliveryHubType)}
                title={t('deliveryHub.types.' + deliveryHubType)}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: 6,
                  padding: '3px 8px 3px 4px',
                  background: deliveryHubType === placingDeliveryHubType ? 'rgba(0, 210, 255, 0.2)' : 'transparent',
                  border: deliveryHubType === placingDeliveryHubType ? `1px solid ${colors.accent}` : '1px solid transparent',
                  borderRadius: 4,
                  cursor: 'pointer',
                  color: deliveryHubType === placingDeliveryHubType ? colors.accent : colors.textBright,
                  fontSize: 11,
                  maxWidth: 120,
                }}
              >
                <img src={getDeliveryHubIcon(deliveryHubType)} width={16} height={16} alt="" style={{ flexShrink: 0 }} />
                <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                  {t('deliveryHub.types.' + deliveryHubType)}
                </span>
              </button>
            ))}
          </div>
        )}
      </div>
      {/* Undo */}
      <MapIconBtn
        title={t('planningToolbar.undo')}
        onClick={onUndo}
        disabled={!canUndo}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <path d="M3 5h5a3 3 0 1 1 0 6H7" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M5 3L3 5l2 2" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      </MapIconBtn>
      {/* Clear all */}
      <MapIconBtn
        title={t('planningToolbar.clearAll')}
        onClick={onClearAll}
        disabled={polygon.length === 0 && !launchPoint && corridorPoints.length === 0}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <path d="M3 3l8 8M11 3l-8 8" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
        </svg>
      </MapIconBtn>

      <div style={{ height: 4 }} />

      {/* Load */}
      <MapIconBtn
        title={t('planningToolbar.loadPlan')}
        onClick={onLoadPolygon}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <path d="M1 3v8a1 1 0 0 0 1 1h10a1 1 0 0 0 1-1V5a1 1 0 0 0-1-1H7L5.5 2.5A1 1 0 0 0 4.8 2H2a1 1 0 0 0-1 1z" stroke="currentColor" strokeWidth="1.3" strokeLinejoin="round"/>
        </svg>
      </MapIconBtn>
      {/* Save */}
      <MapIconBtn
        title={t('planningToolbar.savePlan')}
        onClick={onSavePolygon}
        disabled={polygon.length < 3 && !launchPoint}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
          <path d="M2 1h8l3 3v8a1 1 0 0 1-1 1H2a1 1 0 0 1-1-1V2a1 1 0 0 1 1-1z" stroke="currentColor" strokeWidth="1.3" strokeLinejoin="round"/>
          <path d="M4 1v4h5V1" stroke="currentColor" strokeWidth="1.2"/>
          <rect x="3.5" y="8" width="7" height="4" rx="0.5" stroke="currentColor" strokeWidth="1.1"/>
        </svg>
      </MapIconBtn>
    </div>
  );
}
