import React, { useEffect, useLayoutEffect, useRef, useState, useSyncExternalStore } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import messageLogStore from '../stores/messageLogStore';
import { useTelemetryStore, useVehicleList } from '../hooks/useTelemetryStore';
import { backendNowMs } from '../utils/backendClock';
import {
  MESSAGE_FILTERS,
  filterEntries,
  formatExport,
  formatTime,
  namedValueRows,
  newestKey,
  severityTone,
} from '../utils/messageLog';

const TONE_COLOR = { error: colors.error, warning: colors.warning, info: colors.text };

// Within this many pixels of the bottom counts as "at the newest message":
// new rows then keep the list scrolled down, as in Mission Planner.
const STICK_TO_BOTTOM_PX = 24;

function useEntries(sysId) {
  return useSyncExternalStore(messageLogStore.subscribe, () => messageLogStore.getEntries(sysId));
}

function downloadText(filename, text) {
  const url = URL.createObjectURL(new Blob([text], { type: 'text/plain' }));
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function NamedValues({ namedValues }) {
  const { t } = useTranslation();
  // Re-render with each telemetry message so ages advance on the backend clock.
  useTelemetryStore((s) => s.getSnapshot());
  const rows = namedValueRows(namedValues, backendNowMs());
  if (rows.length === 0) return null;
  return (
    <div data-testid="messages-named-values" style={{
      display: 'flex', flexWrap: 'wrap', gap: 6, padding: '6px 10px',
      borderBottom: `1px solid ${colors.border}`,
    }}>
      {rows.map((r) => (
        <span
          key={r.name}
          title={t('messages.valueTitle', { source: r.source, age: r.age != null ? r.age.toFixed(0) : '—' })}
          style={{
            fontSize: 11, padding: '1px 6px', borderRadius: 3, fontFamily: 'monospace',
            background: colors.surface,
            color: r.stale ? colors.warning : colors.textBright,
            border: `1px solid ${r.stale ? colors.warning : colors.border}`,
          }}
        >
          {r.name} {typeof r.value === 'number' ? Number(r.value.toFixed(3)) : String(r.value)}
          {r.stale && ` · ${t('messages.staleFor', { age: r.age.toFixed(0) })}`}
        </span>
      ))}
    </div>
  );
}

/**
 * Per-UAV messages history, like Mission Planner's Messages tab: the UAV's
 * own autopilot and companion STATUSTEXT and named-value changes (for
 * example the watchdog's NAVWD), newest at the bottom. A non-modal panel at
 * the map's bottom left, so the map stays usable while it is open.
 */
export default function MessagesWindow({ sysId, onSelect, onClose }) {
  const { t } = useTranslation();
  const vehicleList = useVehicleList();
  const vehicle = useTelemetryStore((s) => s.getVehicles()[sysId]);
  const entries = useEntries(sysId);
  // Messages broadcast while the socket was down are not replayed: load the
  // backlog again on every (re)connect, as well as on open and UAV switch.
  const connected = useTelemetryStore((s) => s.isConnected());
  const [filter, setFilter] = useState('all');
  const [loadError, setLoadError] = useState(false);
  const listRef = useRef(null);
  const stickRef = useRef(true);

  useEffect(() => {
    if (!connected) return undefined;
    let cancelled = false;
    setLoadError(false);
    stickRef.current = true;
    fetch(`/api/vehicles/${sysId}/messages`)
      .then((r) => (r.ok ? r.json() : Promise.reject(new Error(String(r.status)))))
      .then((body) => { if (!cancelled) messageLogStore.loadBacklog(sysId, body.entries, body.epoch); })
      .catch(() => { if (!cancelled) setLoadError(true); });
    return () => { cancelled = true; };
  }, [sysId, connected]);

  const shown = filterEntries(entries, filter);
  // The newest row's identity, not the count: at capacity a new row evicts
  // the oldest and the count stays the same.
  const newest = newestKey(shown);

  useLayoutEffect(() => {
    const el = listRef.current;
    if (el && stickRef.current) el.scrollTop = el.scrollHeight;
  }, [newest, filter, sysId]);

  const onScroll = () => {
    const el = listRef.current;
    if (!el) return;
    stickRef.current = el.scrollHeight - el.scrollTop - el.clientHeight <= STICK_TO_BOTTOM_PX;
  };

  const name = vehicle?.name || `UAV ${sysId}`;
  const sorted = [...vehicleList].sort((a, b) => a.sys_id - b.sys_id);

  return (
    <div
      role="dialog"
      aria-label={t('messages.title')}
      data-testid="messages-window"
      style={{
        position: 'fixed', left: 12, bottom: 64, zIndex: 30,
        width: 'min(620px, calc(100vw - 24px))', height: 'min(45vh, 420px)',
        display: 'flex', flexDirection: 'column',
        background: 'rgba(22, 33, 62, 0.96)', border: `1px solid ${colors.border}`,
        borderRadius: 8, boxShadow: '0 4px 16px rgba(0,0,0,0.5)', overflow: 'hidden',
      }}
    >
      <div style={{
        display: 'flex', alignItems: 'center', gap: 6, padding: '6px 10px',
        borderBottom: `1px solid ${colors.border}`, flexWrap: 'wrap',
      }}>
        <span style={{ fontSize: 13, fontWeight: 700, color: colors.textBright, marginRight: 4 }}>
          {t('messages.title')}
        </span>
        {sorted.map((v, i) => (
          <button
            key={v.sys_id}
            type="button"
            aria-pressed={v.sys_id === sysId}
            onClick={() => onSelect(v.sys_id)}
            style={{
              padding: 2, borderRadius: 4, cursor: 'pointer', background: 'none',
              border: `1px solid ${v.sys_id === sysId ? colors.accent : 'transparent'}`,
            }}
          >
            <UavBadge name={v.name} index={i} />
          </button>
        ))}
        <span style={{ flex: 1 }} />
        <select
          aria-label={t('messages.filter')}
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          style={{
            fontSize: 11, background: colors.surface, color: colors.text,
            border: `1px solid ${colors.border}`, borderRadius: 4, padding: '2px 4px',
          }}
        >
          {MESSAGE_FILTERS.map((f) => (
            <option key={f} value={f}>{t(`messages.filters.${f}`)}</option>
          ))}
        </select>
        <button
          type="button"
          disabled={entries.length === 0}
          onClick={() => downloadText(`${name.replace(/\s+/g, '_')}_messages.txt`, formatExport(entries, name))}
          style={{
            fontSize: 11, padding: '2px 8px', borderRadius: 4, cursor: entries.length ? 'pointer' : 'default',
            background: 'none', color: entries.length ? colors.accent : colors.textDim,
            border: `1px solid ${entries.length ? colors.accent : colors.border}`,
          }}
        >
          {t('messages.export')}
        </button>
        <button
          type="button"
          onClick={onClose}
          aria-label={t('settings.close')}
          style={{ background: 'none', border: 'none', color: colors.textDim, cursor: 'pointer', fontSize: 18, padding: '0 4px' }}
        >
          &times;
        </button>
      </div>
      <NamedValues namedValues={vehicle?.named_values} />
      <div
        ref={listRef}
        onScroll={onScroll}
        data-testid="messages-list"
        style={{ flex: 1, overflowY: 'auto', padding: '4px 10px', fontFamily: 'monospace', fontSize: 11 }}
      >
        {shown.length === 0 && (
          <div style={{ color: colors.textDim, padding: '8px 0' }}>
            {loadError ? t('messages.loadFailed') : t('messages.empty')}
          </div>
        )}
        {shown.map((e) => (
          <div key={`${e.epoch}:${e.seq}`} style={{ display: 'flex', gap: 8, lineHeight: '17px', color: TONE_COLOR[severityTone(e)] }}>
            <span style={{ color: colors.textDim, flex: '0 0 auto' }}>{formatTime(e.ts)}</span>
            <span style={{ color: colors.textDim, flex: '0 0 9ch' }}>{t(`messages.source.${e.source}`, { defaultValue: e.source })}</span>
            <span style={{ flex: '0 0 7ch', fontWeight: 600 }}>{e.label}</span>
            <span style={{ wordBreak: 'break-word' }}>
              {e.text}
              {e.incomplete && (
                <span title={t('messages.incompleteTitle')} style={{ color: colors.warning }}>
                  {` [${t('messages.incomplete')}]`}
                </span>
              )}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}
