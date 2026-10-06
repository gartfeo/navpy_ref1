import React, { useState, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';

export function navpyLogViewerVisibility(instance, showLogs) {
  const hasLogs = instance?.log?.length > 0;
  return {
    linkVisible: hasLogs,
    panelVisible: hasLogs && showLogs,
  };
}

function downloadLogs(sysId) {
  fetch(`/api/logs/download/${sysId}`)
    .then((res) => {
      if (!res.ok) throw new Error(`No logs (${res.status})`);
      return res.blob();
    })
    .then((blob) => {
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `uav_${sysId}_logs.zip`;
      a.click();
      URL.revokeObjectURL(url);
    })
    .catch(() => {});
}

export default function NavpySimButton({ sysId, instance, busy, connection, onToggle }) {
  const { t } = useTranslation();
  const isRunning = instance?.running;
  const [showLogs, setShowLogs] = useState(false);
  const logEndRef = useRef(null);

  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, showLogs);

  useEffect(() => {
    if (panelVisible && logEndRef.current) {
      logEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [panelVisible, instance?.log?.length]);

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: 4 }}>
        <button
          onClick={() => { if (isRunning) setShowLogs(false); onToggle(); }}
          disabled={busy}
          style={{
            padding: '4px 8px',
            background: 'transparent',
            color: isRunning ? colors.error : colors.success,
            border: `1px solid ${isRunning ? colors.error : colors.success}`,
            borderRadius: 4,
            fontWeight: 600,
            fontSize: 10,
            cursor: busy ? 'wait' : 'pointer',
            opacity: busy ? 0.5 : 1,
          }}
        >
          {busy ? '...' : isRunning ? t('navpy.stopNavpy') : t('navpy.startNavpy')}
        </button>
        {connection && (
          <span style={{ fontSize: 9, color: colors.textDim }}>{connection}</span>
        )}
        {linkVisible && (
          <span
            onClick={() => setShowLogs(prev => !prev)}
            style={{
              color: colors.accent,
              fontSize: 10,
              cursor: 'pointer',
              userSelect: 'none',
              marginLeft: 'auto',
            }}
          >
            {showLogs ? t('navpy.hideLogs') : t('navpy.logs')}
          </span>
        )}
      </div>
      {panelVisible && (
        <div style={{
          marginTop: 4,
          border: `1px solid ${colors.border}`,
          borderRadius: 3,
          overflow: 'hidden',
        }}>
          <div style={{
            maxHeight: 200,
            overflowY: 'auto',
            padding: 6,
            background: '#0d1520',
            fontSize: 11,
            fontFamily: 'monospace',
            color: colors.text,
            wordBreak: 'break-all',
          }}>
            {instance.log.map((line, i) => (
              <div key={i}>{line}</div>
            ))}
            <div ref={logEndRef} />
          </div>
          <button
            onClick={() => downloadLogs(sysId)}
            style={{
              width: '100%',
              padding: '4px 0',
              background: colors.surface,
              color: colors.accent,
              border: 'none',
              borderTop: `1px solid ${colors.border}`,
              fontSize: 10,
              fontWeight: 600,
              cursor: 'pointer',
            }}
          >
            {t('navpy.downloadLogs')}
          </button>
        </div>
      )}
    </div>
  );
}
