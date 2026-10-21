import React, { useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import MapIconBtn from '../MapIconBtn';
import TrailsControl from './TrailsControl';
import { colors } from '../../styles';
import useElementSize from '../../hooks/useElementSize';
import { DEFAULT_TOP_CONTROLS_BOTTOM_PX, PANEL_MARGIN_PX } from '../../utils/vehicleWindowLayout';

const NO_SIZE = { width: 0, height: DEFAULT_TOP_CONTROLS_BOTTOM_PX - PANEL_MARGIN_PX };

/**
 * Monitor-mode map overlay: RC control toggle, plan entry, vision toggle,
 * the Trails control (trail and camera-mark history), and the vehicle video
 * toggle at the right end, above the window's corner. The top-right controls wrap on a narrow pane;
 * their bottom edge is reported so the vehicle window stays below them.
 */
export default function MonitorMapOverlay({
  onTopControlsBottom,
  onToggleManualControl,
  showVision, onToggleVision,
  trails,
  onEditPlan, planEntry,
  videoOpen, onToggleVideo,
}) {
  const { t } = useTranslation();
  const planLabel = planEntry?.isEdit ? t('monitor.editPlan') : t('monitor.startPlanning');
  const topRightRef = useRef(null);
  const topRight = useElementSize(topRightRef, NO_SIZE);
  useEffect(() => {
    onTopControlsBottom?.(PANEL_MARGIN_PX + topRight.height);
  }, [topRight.height, onTopControlsBottom]);
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
      {/* Top-right bar: plan entry + display toggles. Wraps on a
          narrow pane instead of running under the RC button. */}
      <div ref={topRightRef} data-testid="monitor-top-right" style={{
        position: 'absolute',
        top: 12,
        right: 12,
        zIndex: 10,
        display: 'flex',
        flexWrap: 'wrap',
        justifyContent: 'flex-end',
        alignItems: 'center',
        gap: 6,
        maxWidth: 'calc(100% - 68px)',
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
        {/* Trails: flight trail and camera marks history, one control */}
        <TrailsControl {...trails} />
        {/* Vehicle video: opens and closes the vehicle window below. A stable
            name with the pressed state; the tooltip says what a tap does. */}
        <MapIconBtn
          title={videoOpen ? t('monitorOverlay.hideVideo') : t('monitorOverlay.showVideo')}
          aria-label={t('monitorOverlay.video')}
          aria-pressed={videoOpen}
          onClick={onToggleVideo}
          active={videoOpen}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none" aria-hidden="true">
            <rect x="1" y="4" width="9" height="7" rx="1.2" stroke="currentColor" strokeWidth="1.3"/>
            <path d="M10 6.5l3-1.8v5.6l-3-1.8z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round"/>
          </svg>
        </MapIconBtn>
      </div>
    </>
  );
}
