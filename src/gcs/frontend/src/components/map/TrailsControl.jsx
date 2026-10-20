import React, { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../styles';
import { PANEL_MARGIN_PX } from '../../utils/vehicleWindowLayout';

/** Height of every control inside the panel: compact, like the map bar. */
const CONTROL_PX = 28;
/** Panel width on a wide map pane. */
const PANEL_MAX_WIDTH_PX = 196;
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

/** The map bar's toggle look (MapIconBtn): cyan tint and border when on. */
function toggleStyle(on) {
  return {
    height: CONTROL_PX,
    minWidth: CONTROL_PX,
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 6,
    padding: '0 8px',
    borderRadius: 6,
    fontSize: 12,
    cursor: 'pointer',
    border: `1px solid ${on ? colors.accent : colors.border}`,
    background: on ? 'rgba(0, 210, 255, 0.2)' : 'rgba(22, 33, 62, 0.85)',
    color: on ? colors.accent : colors.textBright,
  };
}

function Toggle({ on, onClick, label, children }) {
  return (
    <button type="button" aria-pressed={!!on} aria-label={label} title={label} onClick={onClick} style={toggleStyle(on)}>
      {children}
    </button>
  );
}

function IconBtn({ label, onClick, on = false, children }) {
  return (
    <button type="button" aria-label={label} title={label} onClick={onClick} style={{ ...toggleStyle(on), padding: 0 }}>
      {children}
    </button>
  );
}

const sep = { borderTop: `1px solid ${colors.border}`, margin: '6px 0' };
const row = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: 4 };

/**
 * The one Trails control (docs/design/gcs_trails_marks_control.md): a 32 px
 * bar button that opens a compact panel of map-bar toggles: the two history
 * layers, one toggle per UAV, fleet-wide Pause / Resume recording and Clear all. Unchecking only
 * hides; recording goes on. Pure view: state lives in useDisplaySettings.
 */
export default function TrailsControl({
  vehicleList = [],
  showTrails, onToggleTrails,
  showMarks, onToggleMarks,
  hiddenSysIds, onToggleUav,
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
            padding: 8,
            borderRadius: 8,
            border: `1px solid ${colors.border}`,
            background: colors.bgLight,
            color: colors.text,
          }}
        >
          <div style={row}>
            <Toggle on={showTrails} onClick={onToggleTrails} label={t('trailsPanel.flightTrail')}>
              <TrailIcon />{t('trailsPanel.flightTrail')}
            </Toggle>
            <Toggle on={showMarks} onClick={onToggleMarks} label={t('trailsPanel.cameraMarks')}>
              <MarksIcon />{t('trailsPanel.cameraMarks')}
            </Toggle>
          </div>
          {vehicleList.length > 0 && (
            <>
              <div style={sep} />
              <div style={row}>
                {vehicleList.map((v, i) => (
                  <Toggle
                    key={v.sys_id}
                    on={!hiddenSysIds?.has(v.sys_id)}
                    onClick={() => onToggleUav(v.sys_id)}
                    label={v.name || `UAV ${v.sys_id}`}
                  >
                    <span
                      aria-hidden="true"
                      style={{
                        width: 8,
                        height: 8,
                        borderRadius: '50%',
                        background: zoneColorsSolid[i % zoneColorsSolid.length],
                      }}
                    />
                    {v.name || v.sys_id}
                  </Toggle>
                ))}
              </div>
            </>
          )}
          <div style={sep} />
          <div style={row}>
            <IconBtn
              on={paused}
              onClick={onTogglePause}
              label={paused ? t('trailsPanel.resume') : t('trailsPanel.pause')}
            >
              {paused ? <PlayIcon /> : <PauseIcon />}
            </IconBtn>
            <IconBtn onClick={onClearAll} label={t('trailsPanel.clearAll')}><TrashIcon /></IconBtn>
            <span style={{ marginLeft: 4, fontSize: 12, color: paused ? colors.warning : colors.textDim }}>
              {paused
                ? t('trailsPanel.pausedSince', { time: formatClock(pausedAt) })
                : t('trailsPanel.recording')}
            </span>
          </div>
        </div>
      )}
    </div>
  );
}

/** Pause time as HH:MM, the operator's clock. */
function formatClock(ms) {
  return new Date(ms).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

const icon = { width: 14, height: 14, viewBox: '0 0 14 14', fill: 'none', 'aria-hidden': true };

function TrailIcon() {
  return (
    <svg {...icon}>
      <path d="M1.5 12C4 12 3 6 6.5 6S9 2 12.5 2" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round"/>
    </svg>
  );
}

function MarksIcon() {
  return (
    <svg {...icon}>
      <rect x="1.5" y="4" width="11" height="8" rx="1.5" stroke="currentColor" strokeWidth="1.2"/>
      <circle cx="7" cy="8" r="2" stroke="currentColor" strokeWidth="1.2"/>
      <path d="M5 4l1-1.5h2L9 4" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round"/>
    </svg>
  );
}

function PauseIcon() {
  return (
    <svg {...icon}>
      <path d="M5 3v8M9 3v8" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round"/>
    </svg>
  );
}

function PlayIcon() {
  return (
    <svg {...icon}>
      <path d="M4.5 2.5v9l7-4.5z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round"/>
    </svg>
  );
}

function TrashIcon() {
  return (
    <svg {...icon}>
      <path d="M2.5 4h9M5.5 4V2.5h3V4M3.5 4l.6 7.5h5.8l.6-7.5" stroke="currentColor" strokeWidth="1.2" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  );
}
