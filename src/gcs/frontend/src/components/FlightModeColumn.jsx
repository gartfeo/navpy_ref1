import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import { PICKER_MODES } from '../utils/flightModes';
import { getArmAction, LONG_PRESS_MS } from '../utils/armAction';
import { RC_RAISED_Z_INDEX } from '../utils/appLayout';
import useLongPress from '../hooks/useLongPress';

/** Arm button border/text color when disarmed. Red when prearm fails, green otherwise. */
export const armBtnColor = (prearmOk) =>
  prearmOk === false ? colors.error : colors.success;

/** The column's inset from the map's top-left corner, and its buttons' width (App keeps RC overlays clear of them). */
export const MODE_COLUMN_INSET_PX = 12;
export const MODE_BUTTON_WIDTH_PX = 80;

const btnBase = {
  minHeight: 32,
  width: MODE_BUTTON_WIDTH_PX,
  padding: '4px 8px',
  borderRadius: 6,
  fontSize: 12,
  fontWeight: 600,
  cursor: 'pointer',
  border: `1px solid ${colors.border}`,
  background: 'rgba(22, 33, 62, 0.85)',
  color: colors.textBright,
  backdropFilter: 'blur(4px)',
  touchAction: 'manipulation',
  textAlign: 'center',
};

const activeBtn = {
  ...btnBase,
  background: colors.accent,
  color: '#000',
  borderColor: colors.accent,
};

const dividerStyle = {
  width: '80%',
  height: 1,
  background: colors.border,
  alignSelf: 'center',
};

/** The picker list keeps this much of the pane below it. */
const PICKER_LIST_MARGIN_PX = 12;

const dropdownStyle = {
  position: 'absolute',
  left: '100%',
  top: 0,
  marginLeft: 4,
  boxSizing: 'border-box',
  overflowY: 'auto',
  display: 'flex',
  flexDirection: 'column',
  gap: 2,
  background: 'rgba(22, 33, 62, 0.95)',
  border: `1px solid ${colors.border}`,
  borderRadius: 8,
  padding: 4,
  backdropFilter: 'blur(4px)',
  zIndex: 20,
  whiteSpace: 'nowrap',
};

const disabledBtn = {
  ...btnBase,
  opacity: 0.4,
  cursor: 'not-allowed',
};

/** ARM button border/text color based on pre-arm severity. */
export const armSeverityColor = (severity) => {
  if (severity === 'fail') return colors.error;
  if (severity === 'warn') return colors.warning;
  return colors.success;
};

export const formatPrearmWarning = (warning, t) => {
  if (warning && typeof warning === 'object') {
    if (typeof warning.key === 'string') {
      return t(warning.key, warning);
    }
    return '';
  }
  if (typeof warning === 'string' && warning.startsWith('prearm.')) {
    return t(warning);
  }
  return warning == null ? '' : String(warning);
};

/**
 * RC flight-mode column: RC, the mode picker, ARM (docs/design/gcs_rc_physical_sticks.md).
 * `paneHeight` caps the picker list; `onHeight` gets the column's height, which the
 * warnings and the hint below ARM change.
 */
export default function FlightModeColumn({ currentMode, armed, prearmOk, prearmSeverity, prearmWarnings, altRel, sysId, onSetMode, onArmDisarm, onToggleManualControl, listRaised = false, paneHeight, onHeight }) {
  const { t } = useTranslation();
  // The UAV and mode the list was opened over; it shows, and takes a pick, only
  // while both still hold, so a pick can never go to another UAV or over a mode
  // the operator did not see.
  const [openedFor, setOpenedFor] = useState(null);
  const columnRef = useRef(null);
  const pickerRef = useRef(null);
  const hasPoi = sysId != null;
  const pickerOpen = hasPoi && openedFor != null
    && openedFor.sysId === sysId && openedFor.mode === currentMode;
  const closePicker = useCallback(() => setOpenedFor(null), []);
  const togglePicker = () => setOpenedFor(pickerOpen ? null : { sysId, mode: currentMode });

  // Drop a list whose UAV or mode went away, so it does not come back with them.
  useEffect(() => { if (!pickerOpen) closePicker(); }, [pickerOpen, closePicker]);

  useEffect(() => {
    if (!pickerOpen) return undefined;
    // Not consumed: a press on a stick still moves the stick.
    const onPointerDown = (e) => {
      if (!columnRef.current?.contains(e.target)) closePicker();
    };
    const onKeyDown = (e) => { if (e.key === 'Escape') closePicker(); };
    document.addEventListener('pointerdown', onPointerDown, true);
    document.addEventListener('keydown', onKeyDown);
    return () => {
      document.removeEventListener('pointerdown', onPointerDown, true);
      document.removeEventListener('keydown', onKeyDown);
    };
  }, [pickerOpen, closePicker]);

  useEffect(() => {
    const el = columnRef.current;
    if (!el || !onHeight || typeof ResizeObserver === 'undefined') return undefined;
    const measure = () => onHeight(el.offsetHeight);
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(el);
    return () => { observer.disconnect(); onHeight(null); };
  }, [onHeight]);

  const { requiresLongPress, hint, force } = getArmAction({ armed, prearmSeverity, altRel });

  const longPressComplete = useCallback(() => {
    if (!hasPoi) return;
    const msg = armed
      ? t('flightMode.forceDisarm', { sysId })
      : (force ? t('flightMode.forceArm', { sysId }) : t('flightMode.armVehicle', { sysId }));
    if (!window.confirm(msg)) return;
    if (armed) {
      onArmDisarm(sysId, false, { force: true });
    } else {
      onArmDisarm(sysId, true, force ? { force: true } : undefined);
    }
  }, [hasPoi, armed, sysId, force, onArmDisarm, t]);

  const { handlers, progress } = useLongPress({
    onComplete: longPressComplete,
    duration: LONG_PRESS_MS,
    enabled: hasPoi && requiresLongPress,
  });

  const handleSelect = useCallback((mode) => {
    closePicker();
    if (!pickerOpen || mode === currentMode) return;
    onSetMode(sysId, mode);
  }, [pickerOpen, currentMode, sysId, onSetMode, closePicker]);

  const isKnownMode = hasPoi && PICKER_MODES.includes(currentMode);
  const pickerTop = MODE_COLUMN_INSET_PX + (pickerRef.current?.offsetTop ?? 0);
  const listMaxHeight = paneHeight == null ? undefined
    : Math.max(0, paneHeight - pickerTop - PICKER_LIST_MARGIN_PX);

  return (
    <div ref={columnRef} style={{
      position: 'absolute',
      top: MODE_COLUMN_INSET_PX,
      left: MODE_COLUMN_INSET_PX,
      width: MODE_BUTTON_WIDTH_PX,
      // Over the RC confirm list: its mode picker holds flight commands and opens
      // towards the list and over the speed tape (7); the list is at 9 unless
      // raised on a phone.
      zIndex: listRaised ? RC_RAISED_Z_INDEX + 1 : 10,
      display: 'flex',
      flexDirection: 'column',
      gap: 4,
      pointerEvents: 'auto',
    }}>
      {/* RC control toggle — same width as mode buttons */}
      <button
        onClick={onToggleManualControl}
        style={{
          ...activeBtn,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: 4,
        }}
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none" style={{ marginTop: -4 }}>
          <rect x="2" y="5" width="12" height="9" rx="2" stroke="currentColor" strokeWidth="1.3" fill="none"/>
          <line x1="8" y1="5" x2="8" y2="1.5" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round"/>
          <circle cx="8" cy="1.5" r="0.8" fill="currentColor"/>
          <circle cx="5.5" cy="9.5" r="1.6" stroke="currentColor" strokeWidth="1"/>
          <circle cx="5.5" cy="9.5" r="0.5" fill="currentColor"/>
          <circle cx="10.5" cy="9.5" r="1.6" stroke="currentColor" strokeWidth="1"/>
          <circle cx="10.5" cy="9.5" r="0.5" fill="currentColor"/>
        </svg>
        {t('flightMode.rc')}
      </button>

      <div style={dividerStyle} />

      {/* Mode picker: the current mode; a tap opens every mode to the right */}
      <div ref={pickerRef} style={{ position: 'relative' }}>
        <button
          disabled={!hasPoi}
          aria-haspopup="listbox"
          aria-expanded={pickerOpen}
          onClick={() => hasPoi && togglePicker()}
          style={{
            ...(hasPoi ? btnBase : disabledBtn),
            color: !hasPoi ? colors.textDim : isKnownMode ? colors.accent : colors.textBright,
            borderColor: isKnownMode ? colors.accent : colors.border,
          }}
        >
          {currentMode || t('flightMode.mode')}
          <span style={{ marginLeft: 3, fontSize: 8 }}>{pickerOpen ? '◀' : '▶'}</span>
        </button>
        {pickerOpen && (
          <div role="listbox" style={{ ...dropdownStyle, maxHeight: listMaxHeight }}>
            {PICKER_MODES.map((mode) => (
              <button
                key={mode}
                role="option"
                aria-selected={mode === currentMode}
                onClick={() => handleSelect(mode)}
                style={{
                  ...btnBase,
                  width: 'auto',
                  minWidth: 72,
                  flexShrink: 0,
                  ...(mode === currentMode
                    ? { background: colors.accent, color: '#000', borderColor: colors.accent }
                    : {}),
                }}
              >
                {mode}
              </button>
            ))}
          </div>
        )}
      </div>

      <div style={dividerStyle} />

      {/* ARM / DISARM toggle */}
      <button
        disabled={!hasPoi}
        onClick={() => {
          if (!hasPoi || requiresLongPress) return;
          if (armed) {
            onArmDisarm(sysId, false);
          } else {
            onArmDisarm(sysId, true);
          }
        }}
        {...handlers}
        style={(() => {
          const base = { ...(!hasPoi ? disabledBtn : btnBase), minHeight: 32, fontWeight: 700 };
          if (hasPoi && armed) {
            const airborne = requiresLongPress;
            const armColor = airborne ? colors.error : colors.success;
            Object.assign(base, { background: armColor, color: '#000', borderColor: armColor });
          } else if (hasPoi) {
            Object.assign(base, { color: armSeverityColor(prearmSeverity), borderColor: armSeverityColor(prearmSeverity) });
          }
          if (progress > 0) {
            const fillColor = armed ? colors.error : armSeverityColor(prearmSeverity);
            base.background = `linear-gradient(to right, ${fillColor} ${progress * 100}%, rgba(22,33,62,0.85) ${progress * 100}%)`;
            base.color = colors.textBright;
          }
          return base;
        })()}
      >
        {armed ? t('flightMode.disarm') : t('flightMode.arm')}
      </button>
      {/* Pre-arm warnings */}
      {hasPoi && !armed && prearmWarnings && prearmWarnings.length > 0 && (
        <div style={{
          fontFamily: 'monospace',
          fontSize: 10,
          lineHeight: '14px',
          color: armSeverityColor(prearmSeverity),
          maxWidth: MODE_BUTTON_WIDTH_PX,
          overflow: 'hidden',
        }}>
          {prearmWarnings.slice(0, 3).map((w, i) => (
            <div key={i} style={{ whiteSpace: 'normal', overflowWrap: 'break-word', wordBreak: 'normal' }}>{formatPrearmWarning(w, t)}</div>
          ))}
        </div>
      )}
      {hint && (
        <div style={{ fontSize: 10, color: armed ? colors.error : armSeverityColor(prearmSeverity), textAlign: 'center', maxWidth: MODE_BUTTON_WIDTH_PX }}>
          {t(hint)}
        </div>
      )}
    </div>
  );
}
