import React, { useState, useCallback } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import UavBadge from './UavBadge';
import ConfirmModal from './ConfirmModal';
import { gpsFixLabel, haccLabel } from './hud/StatusBarIndicator';
import { useTelemetryStore, useVehicleList } from '../hooks/useTelemetryStore';
import { launchGatesFromSettings } from '../utils/prearmChecks';
import { computeVehicleReadiness, computeFleetParamSync } from '../utils/preflightReadiness';
import { remediationFor, topIssue } from '../utils/preflightRemediation';

const BASE_ROWS = ['prearm', 'companion', 'mission', 'gps', 'throttle', 'battery', 'ekf', 'rc', 'sensors'];

const STATUS_COLOR = {
  go: colors.success,
  warn: colors.warning,
  nogo: colors.error,
  na: colors.textDim,
};

function GoBadge({ overall }) {
  const { t } = useTranslation();
  const label = overall === 'nogo' ? t('preflight.noGo')
    : overall === 'warn' ? t('preflight.caution')
    : overall === 'na' ? t('preflight.unknown')
    : t('preflight.go');
  return (
    <span style={{
      padding: '1px 8px', fontSize: 11, fontWeight: 700, borderRadius: 4,
      letterSpacing: 0.5, color: '#000', background: STATUS_COLOR[overall] || colors.textDim,
    }}>
      {label}
    </span>
  );
}

function prearmText(t, w) {
  if (typeof w === 'object') return t(w.key, w);
  if (typeof w === 'string' && w.startsWith('prearm.')) return t(w);
  return w;
}

function checkValue(t, check) {
  if (!check) return '—';
  const { key, status, detail } = check;
  if (key === 'companion') {
    // Short cell status; the full "start / connect NavPy" guidance lives in the
    // action strip + hover remediation. Long sidebar-badge wording would wrap
    // badly in the narrow grid column.
    if (status === 'go') return t('preflight.ccActive');
    if (status === 'warn') return t('preflight.ccChecking');
    if (status === 'nogo') return t('preflight.ccDisconnected');
    return '—';
  }
  if (key === 'mission') {
    if (status === 'go') return t('preflight.missionUploaded');
    if (status === 'nogo') return t('preflight.missionMissing');
    return '—';
  }
  if (key === 'throttle') {
    if (status === 'go') return t('preflight.ok');
    if (status === 'nogo') return t('preflight.throttleHigh');
    return '—';
  }
  if (key === 'airspeed') {
    if (status === 'na') return t('preflight.noPitot');
    const r = detail.reading;
    if (r == null) return t('preflight.unhealthy');
    return `${r.toFixed(1)} m/s${status === 'warn' ? ' ⚠' : ''}`;
  }
  if (key === 'gps') {
    if (status === 'nogo' && (detail.fix == null || detail.fix < 3)) return t('preflight.noFix');
    const parts = [gpsFixLabel(detail.fix)];
    if (detail.sats != null) parts.push(t('preflight.sats', { count: detail.sats }));
    if (detail.hacc != null) parts.push(haccLabel(detail.hacc));
    return parts.join(' · ');
  }
  if (key === 'ekf') {
    if (status === 'na') return '—';
    if (detail?.reason === 'uninitialized') return t('preflight.ekfUninit');
    return t('preflight.ekfVar', { value: (detail?.maxVariance ?? 0).toFixed(2) });
  }
  if (key === 'battery') {
    const parts = [];
    if (detail.pct != null) parts.push(`${Math.round(detail.pct)}%`);
    if (detail.volt != null) parts.push(`${detail.volt.toFixed(1)}V`);
    return parts.length ? parts.join(' · ') : '—';
  }
  if (key === 'rc') {
    if (status === 'go') return t('preflight.linkOk');
    if (status === 'nogo') return t('preflight.linkFail');
    return '—';
  }
  if (key === 'sensors') {
    if (status === 'na') return '—';
    if (status === 'go') return t('preflight.ok');
    return (detail.failed || []).map((s) => t(`preflight.sensorNames.${s}`)).join(', ');
  }
  if (key === 'prearm') {
    if (status === 'go') return t('preflight.ok');
    const msgs = [...(detail.warnings || []), ...(detail.advisories || [])].map((w) => prearmText(t, w));
    return msgs.length ? msgs.join('; ') : t('preflight.checking');
  }
  return '';
}

async function sendPreflightCal(sysIds, pitotCovered) {
  const res = await fetch('/api/control/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      command: 'preflight_cal',
      sys_ids: sysIds,
      ...(pitotCovered ? { params: { pitot_covered: true } } : {}),
    }),
  });
  if (!res.ok) throw new Error(`cal failed: HTTP ${res.status}`);
  return res.json();
}

// Definite pitot — used for the airspeed grid ROW (display only).
const hasPitot = (v) => v.airspeed_present === true;
// Might have a pitot (present OR not-yet-reported) — used for the SAFETY gate:
// only a definite "no pitot" (=== false) may skip the covered-pitot confirm,
// matching the backend's conservative _pitot_baro_ok gate.
const mayHavePitot = (v) => v.airspeed_present !== false;

const cellBase = {
  padding: '6px 8px', borderBottom: `1px solid ${colors.border}`, fontSize: 12,
  display: 'flex', alignItems: 'center', minWidth: 0,
};

/** Amber confirmation shown before any cal that would re-zero a pitot vehicle. */
function PitotConfirm({ onConfirm, onCancel }) {
  const { t } = useTranslation();
  return (
    <div
      style={{ position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.6)', display: 'flex',
        alignItems: 'center', justifyContent: 'center', zIndex: 10000 }}
      onClick={(e) => { if (e.target === e.currentTarget) onCancel(); }}
    >
      <div style={{ background: colors.bgLight, border: `2px solid ${colors.warning}`,
        borderRadius: 8, padding: 24, maxWidth: 360, textAlign: 'center' }}>
        <div style={{ color: colors.warning, fontSize: 16, fontWeight: 700, marginBottom: 10 }}>
          {t('preflight.airspeedConfirmTitle')}
        </div>
        <div style={{ color: colors.text, fontSize: 13, lineHeight: 1.6, marginBottom: 18 }}>
          {t('preflight.airspeedConfirmMsg')}
        </div>
        <div style={{ display: 'flex', gap: 10, justifyContent: 'center' }}>
          <button onClick={onCancel} style={{ padding: '7px 16px', fontSize: 13, background: 'transparent',
            border: `1px solid ${colors.border}`, borderRadius: 4, color: colors.text, cursor: 'pointer' }}>
            {t('settings.cancel')}
          </button>
          <button onClick={onConfirm} style={{ padding: '7px 16px', fontSize: 13, background: colors.warning,
            border: 'none', borderRadius: 4, color: '#000', fontWeight: 700, cursor: 'pointer' }}>
            {t('preflight.airspeedConfirmOk')}
          </button>
        </div>
      </div>
    </div>
  );
}

/**
 * Fleet-wide "params not synced" advisory row (CONF-01, D-01..D-06). Never
 * rendered when the check is 'go'/'na' — this row is purely additive and does
 * not participate in the per-vehicle GO/NO-GO grid or its overall verdict
 * (D-03: advisory only, never blocking). Expanding shows the per-param diff
 * from `paramSync.detail.rows` (each `distinctValues` group names which UAVs
 * hold which value); the link hands off to the existing Sync UAVs /
 * Harmonize UI (`ParamHarmonizeBar`, reached via Settings > Parameters) so no
 * new one-click writer is introduced (D-05).
 */
function ParamSyncWarningRow({ paramSync, vehicleList, onOpenParamSync }) {
  const { t } = useTranslation();
  const [expanded, setExpanded] = useState(false);
  if (!paramSync || paramSync.status !== 'warn') return null;

  const nameForSysId = (sid) => {
    const v = vehicleList.find((vv) => vv.sys_id === sid);
    return v ? v.name : `#${sid}`;
  };

  return (
    <div style={{ marginBottom: 14, border: `1px solid ${colors.warning}`, borderRadius: 4, overflow: 'hidden' }}>
      <button
        type="button"
        onClick={() => setExpanded((e) => !e)}
        aria-expanded={expanded}
        style={{ width: '100%', display: 'flex', alignItems: 'center', justifyContent: 'space-between',
          gap: 8, padding: '8px 10px', background: `${colors.warning}1f`, border: 'none',
          cursor: 'pointer', textAlign: 'left' }}
      >
        <span style={{ color: colors.warning, fontSize: 12, fontWeight: 700 }}>
          {t('preflight.paramSync.title', { count: paramSync.detail.divergent })}
        </span>
        <span style={{ color: colors.textDim, fontSize: 11 }}>{expanded ? '▲' : '▼'}</span>
      </button>
      {expanded && (
        <div style={{ padding: '8px 10px', fontSize: 12, color: colors.text }}>
          {paramSync.detail.rows.map((row) => (
            <div key={row.name} style={{ marginBottom: 6, lineHeight: 1.5 }}>
              <span style={{ fontWeight: 600 }}>{row.name}</span>
              {': '}
              {row.distinctValues.map((group, i) => (
                <span key={String(group.value)}>
                  {i > 0 ? '; ' : ''}
                  <span style={{ color: colors.textDim }}>{String(group.value)}</span>
                  {' → '}
                  {group.sysIds.map(nameForSysId).join(', ')}
                </span>
              ))}
            </div>
          ))}
          {onOpenParamSync && (
            <button
              type="button"
              onClick={onOpenParamSync}
              style={{ marginTop: 2, fontSize: 11, fontWeight: 600, color: colors.accent,
                background: 'transparent', border: `1px solid ${colors.accent}`, borderRadius: 4,
                padding: '3px 8px', cursor: 'pointer' }}
            >
              {t('preflight.paramSync.syncLink')} ↗
            </button>
          )}
        </div>
      )}
    </div>
  );
}

function ReadinessBody({ settings, onReboot, fullParams, onOpenParamSync }) {
  const { t } = useTranslation();
  useTelemetryStore((s) => s.getSnapshot());  // live telemetry
  const vehicleList = useVehicleList();
  const gates = launchGatesFromSettings(settings);

  const [fleetSent, setFleetSent] = useState(false);
  const [pendingCal, setPendingCal] = useState(null); // {sysIds} | null
  const [tip, setTip] = useState(null); // {text, left, top} | null
  const [rebootTarget, setRebootTarget] = useState(null); // {sysId, name} | null

  const doCal = useCallback(async ({ sysIds, pitotCovered }) => {
    setPendingCal(null);
    try {
      await sendPreflightCal(sysIds, pitotCovered);
      // Only report success on an actual OK response — a failed send must not
      // show the green "Sent" state for a safety-adjacent action.
      setFleetSent(true);
      setTimeout(() => setFleetSent(false), 2500);
    } catch { /* failure surfaced by backend log; UI stays un-"Sent" */ }
  }, []);

  // One fleet action: if any target has (or might have) a pitot, confirm the
  // covered-pitot precondition before sending (baro re-zeros airspeed); only a
  // fleet of definite no-pitot vehicles sends directly.
  const requestCal = useCallback((sysIds) => {
    if (!sysIds.length) return;
    const touchesPitot = vehicleList.some((v) => sysIds.includes(v.sys_id) && mayHavePitot(v));
    if (touchesPitot) setPendingCal({ sysIds });
    else doCal({ sysIds, pitotCovered: false });
  }, [vehicleList, doCal]);

  const openLink = (url) => { if (url) window.open(url, '_blank', 'noopener,noreferrer'); };

  if (vehicleList.length === 0) {
    return <div style={{ color: colors.textDim, fontSize: 13 }}>{t('preflight.noVehicles')}</div>;
  }

  const rows = vehicleList.map((v) => {
    const linkDown = !v.link_ok;
    const { checks, overall } = computeVehicleReadiness(v, gates);
    return {
      v, linkDown, overall,
      byKey: Object.fromEntries(checks.map((c) => [c.key, c])),
      issue: linkDown ? null : topIssue(checks),
    };
  });
  // Only vehicles that need something show up in the action strip — a fully
  // ready fleet shows nothing above the grid.
  const problemRows = rows.filter((r) => r.linkDown || r.issue);
  const anyPitot = rows.some((r) => hasPitot(r.v));
  const gridRows = anyPitot ? [...BASE_ROWS, 'airspeed'] : BASE_ROWS;
  const disarmedIds = vehicleList.filter((v) => v.link_ok && !v.armed).map((v) => v.sys_id);

  // Fleet-wide "params not synced" advisory (CONF-01) — computed over the
  // already-fetched per-vehicle param snapshots (D-06); purely additive, never
  // folds into the per-vehicle GO/NO-GO grid above.
  const paramSync = computeFleetParamSync({
    snapshotsByVehicle: fullParams?.snapshotsByVehicle,
    sysIds: vehicleList.map((v) => v.sys_id),
  });

  const showTip = (e, hintKey) => {
    const el = e.currentTarget;
    setTip({ text: t(hintKey), left: el.offsetLeft, top: el.offsetTop + el.offsetHeight + 2 });
  };

  return (
    <>
      {/* Action-needed strip: only vehicles with a problem, nothing shown when
          the whole fleet is ready. Cal-type fixes point at the one fleet
          "Preflight cal" action below rather than duplicating a button here;
          link-type fixes (compass/accel/radio — outside that batch action)
          get their own handoff chip since they're genuinely separate actions. */}
      {problemRows.length > 0 && (
        <>
          <div style={{ fontSize: 10, color: colors.textDim, letterSpacing: 1, textTransform: 'uppercase', marginBottom: 6 }}>
            {t('preflight.actionNeeded')}
          </div>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: 8, marginBottom: 14 }}>
            {problemRows.map(({ v, linkDown, overall, issue }) => (
              <span key={v.sys_id} style={{ display: 'inline-flex', alignItems: 'center', gap: 6,
                padding: '4px 10px', borderRadius: 4, fontSize: 12,
                background: `${STATUS_COLOR[overall] || colors.textDim}1f`,
                color: STATUS_COLOR[overall] || colors.textDim }}>
                <UavBadge name={v.name} index={vehicleList.indexOf(v)} />
                <span style={{ color: colors.text }}>
                  {linkDown ? t('preflight.linkDown') : t(issue.hintKey)}
                </span>
                {!linkDown && issue.action?.type === 'link' && (
                  <button type="button" onClick={() => openLink(issue.action.url)}
                    style={{ fontSize: 11, fontWeight: 600, color: colors.accent, background: 'transparent',
                      border: `1px solid ${colors.accent}`, borderRadius: 4, padding: '1px 6px', cursor: 'pointer' }}>
                    {t(`preflight.calLink.${issue.action.tool}`)} ↗
                  </button>
                )}
              </span>
            ))}
          </div>
        </>
      )}

      <ParamSyncWarningRow paramSync={paramSync} vehicleList={vehicleList} onOpenParamSync={onOpenParamSync} />

      {/* Detail grid (hover a red/amber cell for how to fix) */}
      <div style={{ fontSize: 10, color: colors.textDim, letterSpacing: 1, textTransform: 'uppercase', marginBottom: 6 }}>
        {t('preflight.detail')}
      </div>

      <div style={{ overflowX: 'auto', border: `1px solid ${colors.border}`, borderRadius: 4 }}>
        <div style={{ display: 'grid', position: 'relative', minWidth: 'fit-content',
          gridTemplateColumns: `140px repeat(${rows.length}, minmax(130px, 1fr))` }}
          onMouseLeave={() => setTip(null)}>

          <div style={{ ...cellBase, background: colors.bgLighter, position: 'sticky', left: 0, zIndex: 1,
            borderRight: `1px solid ${colors.border}` }} />
          {rows.map(({ v, linkDown, overall }) => (
            <div key={`h-${v.sys_id}`} style={{ ...cellBase, background: colors.bgLighter, gap: 6 }}>
              <UavBadge name={v.name} index={vehicleList.indexOf(v)} />
              {linkDown ? <span style={{ color: colors.textDim, fontSize: 11 }}>{t('preflight.linkDown')}</span> : <GoBadge overall={overall} />}
            </div>
          ))}

          {gridRows.map((key, ri) => {
            // Start on bgLight (not bgLighter) so the first row reads as
            // distinct from the header row directly above it.
            const bg = ri % 2 === 0 ? colors.bgLight : colors.bgLighter;
            return (
              <React.Fragment key={key}>
                <div style={{ ...cellBase, background: bg, color: colors.textDim, fontWeight: 600,
                  position: 'sticky', left: 0, zIndex: 1, borderRight: `1px solid ${colors.border}` }}>
                  {t(`preflight.${key}`)}
                </div>
                {rows.map(({ v, linkDown, byKey }) => {
                  const check = byKey[key];
                  const status = linkDown ? 'na' : (check?.status || 'na');
                  const rem = linkDown ? null : remediationFor(check);
                  return (
                    <div key={`${key}-${v.sys_id}`}
                      onMouseEnter={rem ? (e) => showTip(e, rem.hintKey) : undefined}
                      style={{ ...cellBase, background: bg, wordBreak: 'break-word',
                        color: STATUS_COLOR[status] || colors.text,
                        textDecoration: rem ? 'underline dotted' : 'none', textUnderlineOffset: 2,
                        cursor: rem ? 'help' : 'default' }}>
                      {linkDown ? '—' : checkValue(t, check)}
                    </div>
                  );
                })}
              </React.Fragment>
            );
          })}

          {/* Per-UAV maintenance actions (moved here from the vehicle status
              card): reboot the autopilot, gated by the same caution confirm.
              Armed vehicles are disabled in the UI and refused by the backend. */}
          {onReboot && (() => {
            const bg = gridRows.length % 2 === 0 ? colors.bgLight : colors.bgLighter;
            return (
              <React.Fragment key="actions">
                <div style={{ ...cellBase, background: bg, color: colors.textDim, fontWeight: 600,
                  position: 'sticky', left: 0, zIndex: 1, borderRight: `1px solid ${colors.border}` }}>
                  {t('preflight.actions')}
                </div>
                {rows.map(({ v, linkDown }) => {
                  const blocked = linkDown || v.armed;
                  return (
                    <div key={`actions-${v.sys_id}`} style={{ ...cellBase, background: bg }}>
                      <button
                        type="button"
                        onClick={() => setRebootTarget({ sysId: v.sys_id, name: v.name })}
                        disabled={blocked}
                        title={linkDown ? t('preflight.linkDown') : v.armed ? t('reboot.armedBlocked') : t('reboot.tooltip')}
                        style={{ fontSize: 11, fontWeight: 600, padding: '2px 8px', borderRadius: 4,
                          background: 'transparent',
                          color: blocked ? colors.textDim : colors.warning,
                          border: `1px solid ${blocked ? colors.border : colors.warning}`,
                          cursor: blocked ? 'not-allowed' : 'pointer', opacity: blocked ? 0.5 : 1 }}
                      >
                        {t('reboot.button')}
                      </button>
                    </div>
                  );
                })}
              </React.Fragment>
            );
          })()}

          {tip && (
            <div style={{ position: 'absolute', left: Math.max(4, tip.left), top: tip.top, maxWidth: 240,
              background: '#0b0b16', border: `1px solid ${colors.accent}`, borderRadius: 6, padding: '8px 10px',
              fontSize: 11, color: colors.text, lineHeight: 1.5, zIndex: 5, pointerEvents: 'none',
              boxShadow: '0 6px 18px rgba(0,0,0,.5)' }}>
              {tip.text}
            </div>
          )}
        </div>
      </div>

      {/* One action for the whole fleet: calibrate every disarmed vehicle at once. */}
      <button
        type="button"
        disabled={!disarmedIds.length}
        onClick={() => requestCal(disarmedIds)}
        style={{ width: '100%', marginTop: 12, padding: '9px 0', fontSize: 13, fontWeight: 700,
          background: fleetSent ? colors.success : colors.accent, color: '#000',
          border: 'none', borderRadius: 4,
          cursor: disarmedIds.length ? 'pointer' : 'not-allowed', opacity: disarmedIds.length ? 1 : 0.5 }}
      >
        {fleetSent ? t('preflight.sent') : t('preflight.calibrate')}
      </button>
      <div style={{ fontSize: 10, color: colors.textDim, textAlign: 'center', marginTop: 4 }}>
        {t('preflight.calibrationDesc')} · {t('preflight.calAllHint')}
      </div>

      {pendingCal && (
        <PitotConfirm
          onConfirm={() => doCal({ ...pendingCal, pitotCovered: true })}
          onCancel={() => setPendingCal(null)}
        />
      )}

      {rebootTarget && (
        <ConfirmModal
          title={t('reboot.confirmTitle')}
          message={t('reboot.confirmMessage', { name: rebootTarget.name })}
          confirmLabel={t('confirm.confirmReboot')}
          tone="caution"
          onConfirm={() => {
            onReboot(rebootTarget.sysId);
            setRebootTarget(null);
          }}
          onCancel={() => setRebootTarget(null)}
        />
      )}
    </>
  );
}

/**
 * One-click Preflight readiness overlay: a checks × vehicles grid (pre-arm,
 * companion, mission, GPS, throttle, battery, EKF, RC, sensors, airspeed) with
 * each vehicle's name + overall GO/CAUTION/NO-GO combined in its header cell. An
 * "action needed" strip above the grid lists only the vehicles with a problem (a
 * fully ready fleet shows nothing there); hovering a red/amber cell in the grid
 * gives the same fix. The overall verdict is reconciled with the launch gate, so
 * a green header means the vehicle is launch-ready; the overlay still does not
 * itself gate launch. Calibration (gyro + baro, and
 * airspeed zero on pitot vehicles behind a covered-pitot confirm) runs here as
 * one fleet-wide action; it also runs automatically on START when enabled in
 * Launch settings. Per-UAV maintenance actions (autopilot reboot behind a
 * caution confirm) live in an Actions row at the bottom of the grid. A
 * fleet-wide (not per-vehicle) "params not synced" advisory row sits above
 * the grid when the connected UAVs disagree on a shared param (CONF-01,
 * D-01..D-06) — expandable to the per-param diff, with a link to the
 * existing Sync UAVs / Harmonize UI (`onOpenParamSync`); it is purely
 * advisory and never changes the grid's GO/NO-GO verdicts.
 */
export default function PreflightReadiness({ settings, onClose, onReboot, fullParams, onOpenParamSync }) {
  const { t } = useTranslation();
  return (
    <div
      style={{ position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.6)', display: 'flex',
        alignItems: 'center', justifyContent: 'center', zIndex: 9999 }}
      onClick={(e) => { if (e.target === e.currentTarget) onClose(); }}
    >
      <div style={{ width: 'min(920px, 95vw)', maxHeight: '85vh', background: colors.bgLight,
        border: `1px solid ${colors.border}`, borderRadius: 8, display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>
        <div style={{ padding: '12px 16px', borderBottom: `1px solid ${colors.border}`,
          display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
          <span style={{ fontSize: 16, fontWeight: 700, color: colors.textBright }}>{t('preflight.overlayTitle')}</span>
          <button onClick={onClose} aria-label={t('settings.close')}
            style={{ background: 'none', border: 'none', color: colors.textDim, cursor: 'pointer', fontSize: 18, padding: '2px 6px' }}>
            &times;
          </button>
        </div>
        <div style={{ padding: 16, overflow: 'auto' }}>
          <ReadinessBody settings={settings} onReboot={onReboot} fullParams={fullParams} onOpenParamSync={onOpenParamSync} />
        </div>
      </div>
    </div>
  );
}
