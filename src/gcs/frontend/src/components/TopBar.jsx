import React, { useRef, useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, topBarHeight } from '../styles';
import ConnectionPanel from './ConnectionPanel';
import EstopDropdown from './EstopDropdown';
import { PHASES } from '../hooks/useMissionState';
import useFullscreen from '../hooks/useFullscreen';
import { APP_BRANCH, branchColor } from '../utils/appBranch';

const iconButtonStyle = {
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  width: 32,
  height: 32,
  background: 'transparent',
  border: `1px solid ${colors.border}`,
  borderRadius: 4,
  color: colors.textDim,
  cursor: 'pointer',
  padding: 0,
};

export default function TopBar({
  connected,
  phase,
  vehicleList,
  seenIds,
  onScan,
  onConnect,
  onDisconnect,
  onDownloadPlan,
  downloadingSysIds,
  showConn,
  setShowConn,
  onOpenSettings,
  defaultDevice,
  onDeviceUsed,
  onAvailableVehiclesChange,
  autoScan,
  devMode,
  onEstop,
  manualControlEnabled,
}) {
  const { t } = useTranslation();
  const dropdownRef = useRef(null);
  const fullscreen = useFullscreen();

  // Close dropdown on click-outside
  useEffect(() => {
    if (!showConn) return;
    const handler = (e) => {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target)) {
        setShowConn(false);
      }
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [showConn]);

  const uavCount = vehicleList?.length || 0;

  // Fallback E-STOP host: the primary dropdown lives in the MonitoringSidebar
  // UAV Status header, but that sidebar is unmounted during the planning phase
  // and while manual control hides the sidebar. In those states — with
  // vehicles still connected (possibly armed) — the TopBar must keep E-STOP
  // reachable, so it renders the same shared dropdown only then.
  const showEstopFallback = uavCount > 0
    && !!onEstop
    && (phase !== PHASES.MONITOR || manualControlEnabled);

  return (
    <>
      <div
        style={{
          height: topBarHeight,
          background: colors.bg,
          borderBottom: `1px solid ${colors.border}`,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '0 16px',
          color: colors.text,
          flexShrink: 0,
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <img src="/aas-logo.svg" alt="AAS" style={{ height: 28 }} />
          <span style={{ fontWeight: 700, fontSize: 16, color: colors.textBright }}>
            {t('app.gcs')}
          </span>
          {/* Branch badge is a dev/instance-distinguishing aid — hidden for
              operators in non-dev mode. */}
          {APP_BRANCH && devMode && (() => {
            const bc = branchColor(APP_BRANCH);
            return (
              <span
                title={APP_BRANCH}
                style={{
                  display: 'inline-flex',
                  alignItems: 'center',
                  gap: 5,
                  maxWidth: 240,
                  fontSize: 11,
                  padding: '2px 8px',
                  borderRadius: 10,
                  background: bc.bg,
                  color: bc.fg,
                  border: `1px solid ${bc.border}`,
                  fontWeight: 600,
                  fontFamily: 'monospace',
                }}
              >
                <span style={{
                  width: 7,
                  height: 7,
                  borderRadius: '50%',
                  background: bc.dot,
                  flex: '0 0 auto',
                }} />
                <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                  {APP_BRANCH}
                </span>
              </span>
            );
          })()}
          {phase === 'PLANNING' && (
            <span
              style={{
                fontSize: 12,
                padding: '2px 8px',
                borderRadius: 4,
                background: colors.surfaceLight,
                color: colors.accent,
                fontWeight: 600,
              }}
            >
              {t('app.planning')}
            </span>
          )}
        </div>

        <div style={{ display: 'flex', alignItems: 'center', gap: 16 }}>
          {/* Connection dropdown toggle */}
          <div style={{ position: 'relative' }} ref={dropdownRef}>
            <button
              onClick={() => setShowConn((v) => !v)}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: 6,
                fontSize: 13,
                background: showConn ? colors.surfaceLight : 'transparent',
                border: `1px solid ${showConn ? colors.accent : colors.border}`,
                borderRadius: 4,
                padding: '4px 10px',
                color: colors.text,
                cursor: 'pointer',
              }}
            >
              <span
                style={{
                  width: 8,
                  height: 8,
                  borderRadius: '50%',
                  background: uavCount > 0 ? colors.success : colors.error,
                }}
              />
              {uavCount > 0 ? t('topBar.uavCount', { count: uavCount }) : t('topBar.noUavs')}
            </button>

            {showConn && (
              <div
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
                }}
              >
                <ConnectionPanel
                  vehicleList={vehicleList || []}
                  seenIds={seenIds}
                  onScan={onScan}
                  onConnect={onConnect}
                  onDisconnect={onDisconnect}
                  onDownloadPlan={onDownloadPlan}
                  downloadingSysIds={downloadingSysIds}
                  defaultDevice={defaultDevice}
                  onDeviceUsed={onDeviceUsed}
                  onAvailableVehiclesChange={onAvailableVehiclesChange}
                  autoScan={autoScan}
                />
              </div>
            )}
          </div>

          {showEstopFallback && (
            <EstopDropdown vehicleList={vehicleList} onEstop={onEstop} size="topbar" />
          )}

          {/* Fullscreen toggle: on the handheld this hides the Android bars */}
          {fullscreen.supported && (
            <button
              onClick={fullscreen.toggle}
              style={iconButtonStyle}
              title={t(fullscreen.active ? 'topBar.exitFullscreen' : 'topBar.fullscreen')}
              aria-pressed={fullscreen.active}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path
                  d={fullscreen.active
                    ? 'M5.5 1.5v4h-4M14.5 5.5h-4v-4M10.5 14.5v-4h4M1.5 10.5h4v4'
                    : 'M1.5 5.5v-4h4M10.5 1.5h4v4M14.5 10.5v4h-4M5.5 14.5h-4v-4'}
                  stroke="currentColor" strokeWidth="1.2" strokeLinecap="round" strokeLinejoin="round"
                />
              </svg>
            </button>
          )}

          {/* Settings gear */}
          <button
            onClick={onOpenSettings}
            style={iconButtonStyle}
            title={t('topBar.settings')}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
              <path d="M6.7 1h2.6l.4 1.8.9.4 1.6-.8 1.8 1.8-.8 1.6.4.9 1.8.4v2.6l-1.8.4-.4.9.8 1.6-1.8 1.8-1.6-.8-.9.4-.4 1.8H6.7l-.4-1.8-.9-.4-1.6.8-1.8-1.8.8-1.6-.4-.9L.6 9.3V6.7l1.8-.4.4-.9-.8-1.6L3.8 2l1.6.8.9-.4L6.7 1z"
                stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" fill="none"/>
              <circle cx="8" cy="8" r="2" stroke="currentColor" strokeWidth="1.2" fill="none"/>
            </svg>
          </button>
        </div>
      </div>
    </>
  );
}
