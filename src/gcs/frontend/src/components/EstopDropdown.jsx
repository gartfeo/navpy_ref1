import React, { useState, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import ConfirmModal from './ConfirmModal';

/** The app's 32 px control size (todo 69; larger waits for todo 94's gloved check). */
const TOGGLE_HEIGHT_PX = 32;
/** Gap between the button and its menu, and the least room the menu leaves under it. */
const MENU_GAP_PX = 4;
const MENU_SCREEN_MARGIN_PX = 8;

/**
 * E-STOP split-button dropdown. Its one host is the TopBar, in every phase
 * (planning, monitor, RC), so it is never inside a scroll (todo 69).
 *
 * The button opens a scope menu — Stop ALL, or a single UAV (colored UAV
 * badge, then a red "Stop" label) — and every choice is gated by the danger
 * ConfirmModal before onEstop fires: undefined = ALL, [sysId] = one.
 */
export default function EstopDropdown({ vehicleList, onEstop }) {
  const { t } = useTranslation();
  const [showEstop, setShowEstop] = useState(false);
  // null = stop ALL; otherwise { sysId, name } for a single-UAV E-STOP.
  const [estopTarget, setEstopTarget] = useState(null);
  const [showEstopMenu, setShowEstopMenu] = useState(false);
  // The menu's height cap, measured when it opens and on resize: the app never
  // scrolls, so a long fleet's rows scroll inside the menu instead of running
  // off screen.
  const [menuMaxHeight, setMenuMaxHeight] = useState(null);
  const estopRef = useRef(null);

  const measureMenu = () => {
    const bottom = estopRef.current ? estopRef.current.getBoundingClientRect().bottom : 0;
    setMenuMaxHeight(Math.max(0, window.innerHeight - bottom - MENU_GAP_PX - MENU_SCREEN_MARGIN_PX));
  };

  // While open: close the E-STOP scope menu on click-outside (ref wraps the
  // split button + menu so a row click isn't swallowed before it fires), and
  // re-measure its cap when the screen size changes.
  useEffect(() => {
    if (!showEstopMenu) return undefined;
    const handler = (e) => {
      if (estopRef.current && !estopRef.current.contains(e.target)) {
        setShowEstopMenu(false);
      }
    };
    document.addEventListener('mousedown', handler);
    window.addEventListener('resize', measureMenu);
    return () => {
      document.removeEventListener('mousedown', handler);
      window.removeEventListener('resize', measureMenu);
    };
  }, [showEstopMenu]);

  const toggleMenu = () => {
    measureMenu();
    setShowEstopMenu((v) => !v);
  };

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

  return (
    <div style={{ position: 'relative' }} ref={estopRef}>
      <button
        type="button"
        onClick={toggleMenu}
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
          height: TOGGLE_HEIGHT_PX,
          boxSizing: 'border-box',
          padding: '0 16px',
          fontWeight: 700,
          fontSize: 13,
          cursor: 'pointer',
          touchAction: 'manipulation',
        }}
      >
        {t('topBar.estop')} <span style={{ fontSize: 11 }}>▾</span>
      </button>

      {showEstopMenu && (
        <div
          role="menu"
          style={{
            position: 'absolute',
            top: '100%',
            right: 0,
            marginTop: MENU_GAP_PX,
            maxHeight: menuMaxHeight ?? undefined,
            background: colors.bgLight,
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
            boxShadow: '0 8px 24px rgba(0,0,0,0.4)',
            zIndex: 1000,
            minWidth: 180,
            overflowX: 'hidden',
            overflowY: 'auto',
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
