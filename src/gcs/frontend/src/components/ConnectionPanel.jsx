import React, { useState, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import { useTelemetryStore } from '../hooks/useTelemetryStore';
import { DEFAULT_GCS_DEVICE, resolvePanelDevice } from '../utils/connectionDevice';

export default function ConnectionPanel({
  vehicleList,
  seenIds,
  onScan,
  onConnect,
  onDisconnect,
  onDownloadPlan,
  downloadingSysIds,
  defaultDevice,
  onDeviceUsed,
  onAvailableVehiclesChange,
  autoScan,
}) {
  const { t } = useTranslation();
  // Subscribe for live telemetry (link_ok, battery)
  const liveVehicleList = useTelemetryStore(s => s.getVehicleList());
  if (liveVehicleList.length > 0) vehicleList = liveVehicleList;
  const liveSeenIds = useTelemetryStore(s => s.getSeenIds());
  seenIds = liveSeenIds;
  const [device, setDevice] = useState(defaultDevice || '');

  // Sync from settings when it loads; do not overwrite an operator-edited device.
  const userEditedDeviceRef = useRef(false);
  useEffect(() => {
    setDevice((prev) => resolvePanelDevice(prev, defaultDevice, userEditedDeviceRef.current));
  }, [defaultDevice]);
  const [scanning, setScanning] = useState(false);
  const [scanError, setScanError] = useState(null);
  const [downloadingPlan, setDownloadingPlan] = useState(false);
  const [connecting, setConnecting] = useState(new Set());
  // Available = scanned but not yet connected
  const [available, setAvailable] = useState([]);
  // Collapse the connected-vehicle list into a single section (open by default).
  const [connectedCollapsed, setConnectedCollapsed] = useState(false);
  // Two-click confirm for Disconnect All (drops every UAV at once).
  const [confirmAll, setConfirmAll] = useState(false);
  useEffect(() => {
    if (!confirmAll) return undefined;
    const id = setTimeout(() => setConfirmAll(false), 3000);
    return () => clearTimeout(id);
  }, [confirmAll]);

  const connectedIds = new Set(vehicleList.map((v) => v.sys_id));

  // Clear from connecting once vehicle appears in vehicleList via telemetry
  useEffect(() => {
    setConnecting((prev) => {
      const next = new Set(prev);
      let changed = false;
      for (const id of prev) {
        if (connectedIds.has(id)) { next.delete(id); changed = true; }
      }
      return changed ? next : prev;
    });
  }, [vehicleList]);

  const handleScan = async () => {
    if (!device.trim()) return;
    setScanning(true);
    setScanError(null);
    const result = await onScan(device.trim());
    setScanning(false);
    if (result?.error) {
      setScanError(result.error);
    } else if (result?.vehicles) {
      // Persist the device string on successful scan
      if (result.found > 0 && onDeviceUsed) onDeviceUsed(device.trim());
      // Replace available list with fresh scan results (clears stale entries)
      const dev = device.trim();
      setAvailable((prev) => {
        const otherDevice = prev.filter((a) => a.device !== dev);
        const fresh = result.vehicles
          .filter((v) => !connectedIds.has(v.sys_id))
          .map((v) => ({ ...v, device: dev }));
        return [...otherDevice, ...fresh];
      });
      if (result.found === 0) {
        setScanError(t('connection.noVehiclesFound'));
      }
    }
  };

  // Auto-trigger scan when opened from sidebar
  const autoScannedRef = useRef(false);
  useEffect(() => {
    if (autoScan && !autoScannedRef.current && device.trim()) {
      autoScannedRef.current = true;
      handleScan();
    }
  }, [autoScan, device]);

  const handleConnect = async (av) => {
    setConnecting((prev) => new Set(prev).add(av.sys_id));
    const result = await onConnect(av.device, av.sys_id, av.name);
    if (result?.error) {
      // Only clear on error — success is cleared by useEffect when telemetry arrives
      setConnecting((prev) => { const s = new Set(prev); s.delete(av.sys_id); return s; });
    }
    // Don't remove from available here — connectedIds filter handles it once telemetry arrives
  };

  const handleDisconnect = async (sysId) => {
    const v = vehicleList.find((v) => v.sys_id === sysId);
    await onDisconnect(sysId);
    // Move back to available (was just connected so likely still alive)
    if (v) {
      setAvailable((prev) => {
        if (prev.some((a) => a.sys_id === sysId)) return prev;
        return [...prev, { sys_id: sysId, name: v.name, device }];
      });
    }
  };

  const handleDownloadPlan = async () => {
    setDownloadingPlan(true);
    await onDownloadPlan();
    setDownloadingPlan(false);
  };

  // Filter available: must not be connected/connecting, and must have a recent heartbeat
  const seen = seenIds ? new Set(seenIds) : null;
  const availableFiltered = available.filter((a) =>
    !connectedIds.has(a.sys_id) && !connecting.has(a.sys_id) && (!seen || seen.has(a.sys_id)),
  );
  const availableSignature = availableFiltered
    .map((a) => `${a.sys_id}:${a.name || ''}:${a.device || ''}`)
    .join('|');
  useEffect(() => {
    onAvailableVehiclesChange?.(availableFiltered);
  }, [onAvailableVehiclesChange, availableSignature]);
  useEffect(() => (
    () => onAvailableVehiclesChange?.([])
  ), [onAvailableVehiclesChange]);
  // Vehicles currently connecting (not yet in vehicleList)
  const connectingList = available.filter((a) => connecting.has(a.sys_id) && !connectedIds.has(a.sys_id));

  return (
    <div style={{ padding: 12, minWidth: 280 }}>
      <div style={{ color: colors.textDim, fontSize: 11, fontWeight: 600, marginBottom: 8 }}>
        {t('connection.mavlinkConnection')}
      </div>
      <div style={{ display: 'flex', gap: 6, marginBottom: 8 }}>
        <input
          type="text"
          value={device}
          onChange={(e) => {
            userEditedDeviceRef.current = true;
            setDevice(e.target.value);
            setScanError(null);
          }}
          placeholder={defaultDevice || DEFAULT_GCS_DEVICE}
          style={inputStyle}
          onKeyDown={(e) => e.key === 'Enter' && handleScan()}
        />
        <button
          onClick={handleScan}
          disabled={scanning || !device.trim()}
          style={{
            ...btnStyle,
            background: colors.accent,
            color: '#000',
            fontWeight: 700,
            opacity: scanning ? 0.5 : 1,
            cursor: scanning ? 'wait' : 'pointer',
            minWidth: 50,
          }}
        >
          {scanning ? t('connection.scanning') : t('connection.connect')}
        </button>
      </div>
      {scanError && (
        <div style={{ color: colors.error, fontSize: 11, marginBottom: 6 }}>{scanError}</div>
      )}

      {/* Connected + Connecting vehicles */}
      {(vehicleList.length > 0 || connectingList.length > 0) && (
        <div style={{ borderTop: `1px solid ${colors.border}`, paddingTop: 8, marginTop: 4 }}>
          <button
            type="button"
            onClick={() => setConnectedCollapsed((c) => !c)}
            aria-expanded={!connectedCollapsed}
            style={{ display: 'flex', alignItems: 'center', gap: 6, width: '100%', padding: 0, marginBottom: 6, background: 'none', border: 'none', cursor: 'pointer', textAlign: 'left', color: colors.textDim, fontSize: 11, fontWeight: 600 }}
          >
            <span style={{ fontSize: 9, width: 9, flex: '0 0 auto', transition: 'transform 0.15s ease', transform: connectedCollapsed ? 'none' : 'rotate(90deg)' }}>▶</span>
            {t('connection.connected', { count: vehicleList.length })}{connectingList.length > 0 ? ` + ${connectingList.length} ${t('connection.connecting')}` : ''}
          </button>
          {!connectedCollapsed && (<>
          {connectingList.map((av) => {
            const isDl = downloadingSysIds?.has(av.sys_id);
            return (
              <div
                key={av.sys_id}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                  padding: '4px 6px',
                  marginBottom: 4,
                  background: colors.surface,
                  borderRadius: 4,
                  border: `1px solid ${colors.border}`,
                  opacity: 0.6,
                }}
              >
                <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                  <span
                    style={{
                      width: 7,
                      height: 7,
                      borderRadius: '50%',
                      background: isDl ? colors.accent : colors.warning,
                      flexShrink: 0,
                    }}
                  />
                  <span style={{ color: colors.textBright, fontSize: 12, fontWeight: 600 }}>
                    {av.name}
                  </span>
                  <span style={{ color: isDl ? colors.accent : colors.warning, fontSize: 10 }}>
                    {isDl ? t('connection.downloading') : t('connection.connectingStatus')}
                  </span>
                </div>
              </div>
            );
          })}
          {vehicleList.map((v) => {
            // Backend probe (is_probing) counts too, so auto-connected vehicles
            // still show "Downloading…" here even though the frontend never drove
            // the download.
            const isDl = downloadingSysIds?.has(v.sys_id) || v.is_probing;
            return (
              <div
                key={v.sys_id}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'space-between',
                  padding: '4px 6px',
                  marginBottom: 4,
                  background: colors.surface,
                  borderRadius: 4,
                  border: `1px solid ${colors.border}`,
                }}
              >
                <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                  <span
                    style={{
                      width: 7,
                      height: 7,
                      borderRadius: '50%',
                      background: isDl ? colors.accent : (v.link_ok ? colors.success : colors.error),
                      flexShrink: 0,
                    }}
                  />
                  <span style={{ color: colors.textBright, fontSize: 12, fontWeight: 600 }}>
                    {v.name}
                  </span>
                  {isDl ? (
                    <span style={{ color: colors.accent, fontSize: 10 }}>
                      {t('connection.downloading')}
                      {Array.isArray(v.mission_download_progress) ? ` ${v.mission_download_progress[0]}/${v.mission_download_progress[1]}` : ''}
                    </span>
                  ) : (
                    <span style={{ color: colors.textDim, fontSize: 11 }}>
                      {v.battery != null ? `${v.battery}%` : '--'}
                    </span>
                  )}
                </div>
                <button
                  onClick={() => handleDisconnect(v.sys_id)}
                  title={t('connection.disconnect')}
                  style={{
                    background: 'none',
                    border: 'none',
                    color: colors.textDim,
                    cursor: 'pointer',
                    padding: '0 2px',
                    fontSize: 14,
                    lineHeight: 1,
                  }}
                >
                  ×
                </button>
              </div>
            );
          })}
          </>)}
          <div style={{ display: 'flex', gap: 6, marginTop: 6 }}>
            <button
              onClick={handleDownloadPlan}
              disabled={downloadingPlan || downloadingSysIds?.size > 0 || vehicleList.length === 0}
              style={{
                flex: 1,
                padding: '4px 0',
                background: 'none',
                color: (downloadingPlan || downloadingSysIds?.size > 0) ? colors.textDim : colors.accent,
                border: `1px solid ${colors.border}`,
                borderRadius: 4,
                fontSize: 11,
                fontWeight: 600,
                cursor: (downloadingPlan || downloadingSysIds?.size > 0) ? 'wait' : 'pointer',
                opacity: (downloadingPlan || downloadingSysIds?.size > 0) ? 0.5 : 1,
              }}
            >
              {(downloadingPlan || downloadingSysIds?.size > 0) ? t('connection.downloading') : t('connection.downloadPlan')}
            </button>
            <button
              onClick={async () => {
                if (confirmAll) {
                  setConfirmAll(false);
                  for (const v of vehicleList) await handleDisconnect(v.sys_id);
                } else {
                  setConfirmAll(true);
                }
              }}
              disabled={vehicleList.length === 0}
              style={{
                flex: 1,
                padding: '4px 0',
                background: confirmAll ? colors.error : 'none',
                color: confirmAll ? '#fff' : colors.error,
                border: `1px solid ${confirmAll ? colors.error : colors.border}`,
                borderRadius: 4,
                fontSize: 11,
                fontWeight: 600,
                cursor: 'pointer',
              }}
            >
              {confirmAll ? t('connection.confirmQ') : t('connection.disconnectAll')}
            </button>
          </div>
        </div>
      )}

      {/* Available (scanned but not connected) vehicles */}
      {availableFiltered.length > 0 && (
        <div style={{ borderTop: `1px solid ${colors.border}`, paddingTop: 8, marginTop: 4 }}>
          <div style={{ color: colors.textDim, fontSize: 11, fontWeight: 600, marginBottom: 6 }}>
            {t('connection.available', { count: availableFiltered.length })}
          </div>
          {availableFiltered.map((av) => (
            <div
              key={av.sys_id}
              style={{
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'space-between',
                padding: '4px 6px',
                marginBottom: 4,
                background: 'rgba(15,52,96,0.4)',
                borderRadius: 4,
                border: `1px solid ${colors.border}`,
              }}
            >
              <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                <span
                  style={{
                    width: 7,
                    height: 7,
                    borderRadius: '50%',
                    background: colors.textDim,
                    flexShrink: 0,
                  }}
                />
                <span style={{ color: colors.textDim, fontSize: 12, fontWeight: 600 }}>
                  {av.name}
                </span>
              </div>
              <button
                onClick={() => handleConnect(av)}
                disabled={connecting.has(av.sys_id)}
                style={{
                  background: 'none',
                  border: `1px solid ${colors.border}`,
                  borderRadius: 3,
                  color: colors.accent,
                  cursor: connecting.has(av.sys_id) ? 'wait' : 'pointer',
                  padding: '1px 8px',
                  fontSize: 10,
                  fontWeight: 600,
                  opacity: connecting.has(av.sys_id) ? 0.5 : 1,
                }}
              >
                {connecting.has(av.sys_id) ? '...' : t('connection.connect')}
              </button>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

const inputStyle = {
  flex: 1,
  background: colors.surface,
  color: colors.textBright,
  border: `1px solid ${colors.border}`,
  borderRadius: 4,
  padding: '4px 8px',
  fontSize: 12,
  boxSizing: 'border-box',
  outline: 'none',
};

const btnStyle = {
  border: 'none',
  borderRadius: 4,
  padding: '4px 10px',
  fontSize: 13,
};
