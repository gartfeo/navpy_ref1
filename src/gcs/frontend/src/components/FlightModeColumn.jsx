import React, { useCallback, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import { QUICK_MODES, OTHER_MODES } from '../utils/flightModes';
import { getArmAction, LONG_PRESS_MS } from '../utils/armAction';
import useLongPress from '../hooks/useLongPress';

/** Arm button border/text color when disarmed. Red when prearm fails, green otherwise. */
export const armBtnColor = (prearmOk) =>
  prearmOk === false ? colors.error : colors.success;

const btnBase = {
  minHeight: 32,
  width: 80,
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

const dropdownStyle = {
  position: 'absolute',
  left: '100%',
  top: 0,
  marginLeft: 4,
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

export default function FlightModeColumn({ currentMode, armed, prearmOk, prearmSeverity, prearmWarnings, altRel, sysId, onSetMode, onArmDisarm, onToggleManualControl }) {
  const { t } = useTranslation();
  const [showOther, setShowOther] = useState(false);
  const hasTarget = sysId != null;

  const { requiresLongPress, hint, force } = getArmAction({ armed, prearmSeverity, altRel });

  const longPressComplete = useCallback(() => {
    if (!hasTarget) return;
    const msg = armed
      ? t('flightMode.forceDisarm', { sysId })
      : (force ? t('flightMode.forceArm', { sysId }) : t('flightMode.armVehicle', { sysId }));
    if (!window.confirm(msg)) return;
    if (armed) {
      onArmDisarm(sysId, false, { force: true });
    } else {
      onArmDisarm(sysId, true, force ? { force: true } : undefined);
    }
  }, [hasTarget, armed, sysId, force, onArmDisarm, t]);

  const { handlers, progress } = useLongPress({
    onComplete: longPressComplete,
    duration: LONG_PRESS_MS,
    enabled: hasTarget && requiresLongPress,
  });

  const handleSelect = useCallback((mode) => {
    if (!hasTarget || mode === currentMode) return;
    onSetMode(sysId, mode);
    setShowOther(false);
  }, [hasTarget, currentMode, sysId, onSetMode]);

  const isOtherActive = hasTarget && OTHER_MODES.includes(currentMode);

  return (
    <div style={{
      position: 'absolute',
      top: 12,
      left: 12,
      zIndex: 10,
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

      {/* Quick mode buttons */}
      {QUICK_MODES.map((mode) => (
        <button
          key={mode}
          disabled={!hasTarget}
          onClick={() => handleSelect(mode)}
          style={!hasTarget ? disabledBtn
            : mode === currentMode ? activeBtn : btnBase}
        >
          {mode}
        </button>
      ))}

      <div style={dividerStyle} />

      {/* "Other" button with right-side dropdown */}
      <div style={{ position: 'relative' }}>
        <button
          disabled={!hasTarget}
          onClick={() => hasTarget && setShowOther((v) => !v)}
          style={{
            ...(hasTarget ? btnBase : disabledBtn),
            fontSize: 11,
            color: !hasTarget ? colors.textDim
              : isOtherActive ? colors.accent : colors.textDim,
            borderColor: isOtherActive ? colors.accent : colors.border,
          }}
        >
          {isOtherActive ? currentMode : t('flightMode.other')}
          <span style={{ marginLeft: 3, fontSize: 8 }}>{showOther ? '◀' : '▶'}</span>
        </button>
        {showOther && hasTarget && (
          <div style={dropdownStyle}>
            {OTHER_MODES.map((mode) => (
              <button
                key={mode}
                onClick={() => handleSelect(mode)}
                style={{
                  ...btnBase,
                  width: 'auto',
                  minWidth: 72,
                  minHeight: 28,
                  padding: '3px 8px',
                  fontSize: 11,
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
        disabled={!hasTarget}
        onClick={() => {
          if (!hasTarget || requiresLongPress) return;
          if (armed) {
            onArmDisarm(sysId, false);
          } else {
            onArmDisarm(sysId, true);
          }
        }}
        {...handlers}
        style={(() => {
          const base = { ...(!hasTarget ? disabledBtn : btnBase), minHeight: 32, fontWeight: 700 };
          if (hasTarget && armed) {
            const airborne = requiresLongPress;
            const armColor = airborne ? colors.error : colors.success;
            Object.assign(base, { background: armColor, color: '#000', borderColor: armColor });
          } else if (hasTarget) {
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
      {hasTarget && !armed && prearmWarnings && prearmWarnings.length > 0 && (
        <div style={{
          fontFamily: 'monospace',
          fontSize: 10,
          lineHeight: '14px',
          color: armSeverityColor(prearmSeverity),
          maxWidth: 120,
          overflow: 'hidden',
        }}>
          {prearmWarnings.slice(0, 3).map((w, i) => (
            <div key={i} style={{ whiteSpace: 'normal', overflowWrap: 'break-word', wordBreak: 'normal' }}>{formatPrearmWarning(w, t)}</div>
          ))}
        </div>
      )}
      {hint && (
        <div style={{ fontSize: 10, color: armed ? colors.error : armSeverityColor(prearmSeverity), textAlign: 'center', maxWidth: 120 }}>
          {t(hint)}
        </div>
      )}
    </div>
  );
}
