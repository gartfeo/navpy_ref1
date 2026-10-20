import React, { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../styles';
import { PANEL_MARGIN_PX } from '../../utils/vehicleWindowLayout';

/** Touch target for every control inside the panel (handheld rule). */
const TARGET_PX = 44;
/** Panel width on a wide map pane. */
const PANEL_MAX_WIDTH_PX = 320;
/** Gap between the bar button and the panel under it. */
const PANEL_GAP_PX = 6;

/**
 * Panel bounds inside the map pane: right-aligned under the button, never
 * past the pane's left or bottom margin, so a narrow or short pane scrolls
 * the panel instead of cutting it off.
 */
export function panelBounds(buttonRect, paneRect) {
  return {
    width: Math.max(0, Math.min(PANEL_MAX_WIDTH_PX, buttonRect.right - paneRect.left - PANEL_MARGIN_PX)),
    maxHeight: Math.max(0, paneRect.bottom - buttonRect.bottom - PANEL_GAP_PX - PANEL_MARGIN_PX),
  };
}

const panelBtn = {
  minHeight: TARGET_PX,
  padding: '0 12px',
  borderRadius: 6,
  fontSize: 13,
  cursor: 'pointer',
  border: `1px solid ${colors.border}`,
  background: colors.bgLighter,
  color: colors.textBright,
};

const checkRow = {
  minHeight: TARGET_PX,
  display: 'inline-flex',
  alignItems: 'center',
  gap: 6,
  padding: '0 10px',
  borderRadius: 6,
  border: `1px solid ${colors.border}`,
  fontSize: 13,
  cursor: 'pointer',
  color: colors.text,
};

function Check({ checked, onChange, children }) {
  return (
    <label style={checkRow}>
      <input type="checkbox" checked={checked} onChange={onChange} />
      {children}
    </label>
  );
}

/**
 * The one Trails control (docs/design/gcs_trails_marks_control.md): a 32 px
 * bar button that opens a panel with the two history layers, one checkbox
 * per UAV, fleet-wide Pause / Resume recording and Clear all. Unchecking only
 * hides; recording goes on. Pure view: state lives in useDisplaySettings.
 */
export default function TrailsControl({
  vehicleList = [],
  showTrails, onToggleTrails,
  showMarks, onToggleMarks,
  hiddenSysIds, onToggleUav, onShowAllUavs, onHideAllUavs,
  pausedAt, onTogglePause,
  onClearAll,
}) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);
  const rootRef = useRef(null);
  const buttonRef = useRef(null);
  const paused = pausedAt != null;
  const [bounds, setBounds] = useState(null);

  // The bar sits in the map pane, which is its offset parent.
  useLayoutEffect(() => {
    if (!open) return undefined;
    const measure = () => {
      const button = buttonRef.current;
      const pane = rootRef.current?.parentElement?.offsetParent;
      if (!button || !pane) return;
      setBounds(panelBounds(button.getBoundingClientRect(), pane.getBoundingClientRect()));
    };
    measure();
    // The map pane can resize without the window (split view, sidebars).
    const pane = rootRef.current?.parentElement?.offsetParent;
    if (typeof ResizeObserver !== 'undefined' && pane) {
      const ro = new ResizeObserver(measure);
      ro.observe(pane);
      return () => ro.disconnect();
    }
    window.addEventListener('resize', measure);
    return () => window.removeEventListener('resize', measure);
  }, [open]);

  useEffect(() => {
    if (!open) return undefined;
    const onKey = (e) => {
      if (e.key === 'Escape') {
        setOpen(false);
        buttonRef.current?.focus();
      }
    };
    const onPointer = (e) => {
      if (rootRef.current && !rootRef.current.contains(e.target)) setOpen(false);
    };
    document.addEventListener('keydown', onKey);
    document.addEventListener('pointerdown', onPointer);
    return () => {
      document.removeEventListener('keydown', onKey);
      document.removeEventListener('pointerdown', onPointer);
    };
  }, [open]);

  return (
    <div ref={rootRef} style={{ position: 'relative' }}>
      <button
        ref={buttonRef}
        type="button"
        onClick={() => setOpen((o) => !o)}
        aria-label={t('trailsPanel.title')}
        aria-expanded={open}
        aria-haspopup="true"
        title={t('trailsPanel.title')}
        style={{
          height: 32,
          display: 'inline-flex',
          alignItems: 'center',
          gap: 6,
          padding: '0 8px',
          borderRadius: 6,
          fontSize: 12,
          fontWeight: 600,
          cursor: 'pointer',
          border: `1px solid ${open ? colors.accent : colors.border}`,
          background: 'rgba(22, 33, 62, 0.85)',
          color: open ? colors.accent : colors.textBright,
          backdropFilter: 'blur(4px)',
          whiteSpace: 'nowrap',
        }}
      >
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none" aria-hidden="true">
          <path d="M1.5 12C4 12 3 6 6.5 6S9 2 12.5 2" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round"/>
          <circle cx="12.5" cy="2" r="1.3" fill="currentColor"/>
        </svg>
        {t('trailsPanel.button')}
        {paused && (
          <span
            data-testid="trails-paused-badge"
            style={{
              fontSize: 11,
              padding: '1px 6px',
              borderRadius: 4,
              border: `1px solid ${colors.warning}`,
              color: colors.warning,
            }}
          >
            {t('trailsPanel.paused')}
          </span>
        )}
        <svg width="10" height="10" viewBox="0 0 10 10" fill="none" aria-hidden="true">
          <path d="M2 3.5l3 3 3-3" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      </button>
      {open && (
        <div
          role="group"
          aria-label={t('trailsPanel.title')}
          data-testid="trails-panel"
          style={{
            position: 'absolute',
            top: 32 + PANEL_GAP_PX,
            right: 0,
            width: bounds ? bounds.width : PANEL_MAX_WIDTH_PX,
            maxHeight: bounds ? bounds.maxHeight : undefined,
            boxSizing: 'border-box',
            overflowY: 'auto',
            padding: 12,
            borderRadius: 8,
            border: `1px solid ${colors.border}`,
            background: colors.bgLight,
            color: colors.text,
            display: 'flex',
            flexDirection: 'column',
            gap: 8,
          }}
        >
          <div style={{ fontSize: 14, fontWeight: 600, color: colors.textBright }}>
            {t('trailsPanel.title')}
          </div>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
            <Check checked={!!showTrails} onChange={onToggleTrails}>{t('trailsPanel.flightTrail')}</Check>
            <Check checked={!!showMarks} onChange={onToggleMarks}>{t('trailsPanel.cameraMarks')}</Check>
          </div>
          <div style={{ fontSize: 12, color: colors.textDim }}>{t('trailsPanel.uavs')}</div>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
            {vehicleList.map((v, i) => (
              <Check
                key={v.sys_id}
                checked={!hiddenSysIds?.has(v.sys_id)}
                onChange={() => onToggleUav(v.sys_id)}
              >
                <span
                  aria-hidden="true"
                  style={{
                    width: 10,
                    height: 10,
                    borderRadius: 2,
                    background: zoneColorsSolid[i % zoneColorsSolid.length],
                  }}
                />
                {v.name || `UAV ${v.sys_id}`}
              </Check>
            ))}
          </div>
          <div style={{ display: 'flex', gap: 6 }}>
            <button type="button" style={panelBtn} onClick={onShowAllUavs}>{t('trailsPanel.all')}</button>
            <button type="button" style={panelBtn} onClick={onHideAllUavs}>{t('trailsPanel.none')}</button>
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between', gap: 6 }}>
            <button type="button" style={panelBtn} onClick={onTogglePause}>
              {paused ? t('trailsPanel.resume') : t('trailsPanel.pause')}
            </button>
            <button type="button" style={panelBtn} onClick={onClearAll}>{t('trailsPanel.clearAll')}</button>
          </div>
          {paused && (
            <div style={{ fontSize: 12, color: colors.warning }}>
              {t('trailsPanel.notRecordingSince', { time: new Date(pausedAt).toLocaleTimeString() })}
            </div>
          )}
        </div>
      )}
    </div>
  );
}
