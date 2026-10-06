import React, { useState, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import ConfirmModal from './ConfirmModal';

// Host-specific sizing for the toggle button; the menu itself is identical.
const TOGGLE_SIZE = {
  sidebar: { padding: '4px 12px', fontSize: 12, caret: 10 },
  topbar: { padding: '6px 16px', fontSize: 13, caret: 11 },
};

/**
 * E-STOP split-button dropdown, shared by its two hosts: the MonitoringSidebar
 * UAV Status header (primary) and the TopBar (fallback, shown only when the
 * sidebar instance is not mounted — planning phase or manual control).
 *
 * The button opens a scope menu — Stop ALL, or a single UAV (colored UAV
 * badge, then a red "Stop" label) — and every choice is gated by the danger
 * ConfirmModal before onEstop fires: undefined = ALL, [sysId] = one.
 */
export default function EstopDropdown({ vehicleList, onEstop, size = 'sidebar' }) {
  const { t } = useTranslation();
  const [showEstop, setShowEstop] = useState(false);
  // null = stop ALL; otherwise { sysId, name } for a single-UAV E-STOP.
  const [estopTarget, setEstopTarget] = useState(null);
  const [showEstopMenu, setShowEstopMenu] = useState(false);
  const estopRef = useRef(null);

  // Close the E-STOP scope menu on click-outside (ref wraps the split button
  // + menu so a row click isn't swallowed before it fires).
  useEffect(() => {
    if (!showEstopMenu) return undefined;
    const handler = (e) => {
      if (estopRef.current && !estopRef.current.contains(e.target)) {
        setShowEstopMenu(false);
      }
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [showEstopMenu]);

  const openEstop = (target) => {
    setEstopTarget(target);
    setShowEstopMenu(false);
    setShowEstop(true);
  };
  // Every row is a stop action, so every row reads red/bold.
  const estopMenuItemStyle = {
    display: 'block',
    width: '100%',
    textAlign: 'left',
    background: 'transparent',
    border: 'none',
    borderBottom: `1px solid ${colors.border}`,
    color: colors.error,
    fontWeight: 700,
    fontSize: 13,
    padding: '8px 12px',
    cursor: 'pointer',
  };
  const toggle = TOGGLE_SIZE[size] || TOGGLE_SIZE.sidebar;

  return (
    <div style={{ position: 'relative' }} ref={estopRef}>
      <button
        type="button"
        onClick={() => setShowEstopMenu((v) => !v)}
        aria-haspopup="menu"
        aria-expanded={showEstopMenu}
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: 6,
          background: colors.error,
          color: '#fff',
          border: 'none',
          borderRadius: 4,
          padding: toggle.padding,
          fontWeight: 700,
          fontSize: toggle.fontSize,
          cursor: 'pointer',
        }}
      >
        {t('topBar.estop')} <span style={{ fontSize: toggle.caret }}>▾</span>
      </button>

      {showEstopMenu && (
        <div
          role="menu"
          style={{
            position: 'absolute',
            top: '100%',
            right: 0,
            marginTop: 4,
            background: colors.bgLight,
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
            boxShadow: '0 8px 24px rgba(0,0,0,0.4)',
            zIndex: 1000,
            minWidth: 180,
            overflow: 'hidden',
          }}
        >
          {/* Inline styles can't express :hover — rows need a fill on
              hover/press to read as buttons, not plain text. */}
          <style>{`
            .estopMenuItem:hover { background: rgba(244, 67, 54, 0.15); }
            .estopMenuItem:active { background: rgba(244, 67, 54, 0.3); }
          `}</style>
          <button className="estopMenuItem" onClick={() => openEstop(null)} style={estopMenuItemStyle}>
            {t('topBar.stopAll')}
          </button>
          {(vehicleList || []).map((v, i) => {
            const name = v.name || `UAV ${v.sys_id}`;
            return (
              <button
                key={v.sys_id}
                className="estopMenuItem"
                onClick={() => openEstop({ sysId: v.sys_id, name })}
                style={{ ...estopMenuItemStyle, display: 'flex', alignItems: 'center', gap: 6 }}
              >
                <UavBadge name={name} index={i} />
                <span>{t('topBar.stop')}</span>
              </button>
            );
          })}
        </div>
      )}

      {showEstop && (
        <ConfirmModal
          title={t('topBar.emergencyStop')}
          message={estopTarget
            ? t('topBar.disarmOne', { name: estopTarget.name })
            : t('topBar.disarmAll')}
          onConfirm={() => {
            onEstop(estopTarget ? [estopTarget.sysId] : undefined);
            setShowEstop(false);
            setEstopTarget(null);
          }}
          onCancel={() => { setShowEstop(false); setEstopTarget(null); }}
        />
      )}
    </div>
  );
}
