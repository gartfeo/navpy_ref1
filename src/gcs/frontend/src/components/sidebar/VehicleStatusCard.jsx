import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import UavBadge from '../UavBadge';
import { gpsFixLabel, gpsColor, haccLabel, linkBars, linkBarColor } from '../hud/StatusBarIndicator';
import { batteryColor } from '../hud/BatteryIndicator';
import { computeMissionStatus } from '../../utils/missionStatus';
import { isInitializing } from '../../utils/loadStage';
import { hasPendingConfirm } from '../../hooks/taskConfirmationState';
import { computeLaunchReadiness, visibleAdvisories } from '../../utils/prearmChecks';
import { interpolatedDistance } from '../../utils/missionProgress';
import MissionStatusRow from './MissionStatusRow';
import LoadStageRow from './LoadStageRow';
import { BatteryIcon, SatelliteIcon, SignalIcon } from './StatusIcons';
import NavpySimButton from './NavpySimButton';
import { ForceLaunchButton } from './LaunchControls';
import { MONITOR_PHASES } from '../../constants/monitorPhases';
import { LONG_PRESS_MS } from '../../utils/armAction';
import useLongPress from '../../hooks/useLongPress';

function CompanionBadge({ status }) {
  const { t } = useTranslation();
  // Tri-state companion liveness. "checking" is a transient gap in the 1 Hz
  // heartbeat on the shared link (ordinary packet loss) — shown amber/verifying,
  // NOT a red fault — so brief loss during flight doesn't flap the badge to
  // "disconnected". Only sustained silence ("down") is red. Unknown/missing
  // status falls back to "down" (matches the prior default).
  const variants = {
    ok: { bg: colors.success, title: t('vehicle.ccActive') },
    checking: { bg: colors.warning, title: t('vehicle.ccChecking') },
    down: { bg: colors.error, title: t('vehicle.ccDown') },
  };
  const v = variants[status] || variants.down;
  return (
    <span
      title={v.title}
      style={{
        display: 'inline-block',
        padding: '1px 4px',
        fontSize: 9,
        fontWeight: 700,
        borderRadius: 3,
        background: v.bg,
        color: '#fff',
        lineHeight: '14px',
        opacity: 0.9,
      }}
    >
      {t('vehicle.cc')}
    </span>
  );
}

// CONF-03 recognition-gate blocked state (D-15/D-16/D-17) + the "Ask me
// anyway" one-shot override (D-18/D-19/D-20). Rendered only while `blocked`
// is present (live, per-UAV, source-filtered by the backend). The override
// button requires press-and-hold, like the launch override, so a slip of the
// mouse can never bypass the recognition gate by accident.
function ConfirmBlockedRow({ blocked, sysId, onForceConfirm }) {
  const { t } = useTranslation();
  const canForce = typeof onForceConfirm === 'function' && blocked.task_id != null;
  const { handlers, progress } = useLongPress({
    onComplete: () => onForceConfirm(sysId, blocked.task_id),
    duration: LONG_PRESS_MS,
    enabled: canForce,
    resetKey: blocked.task_id,
  });
  const reasonLabel = blocked.reason === 'pixels'
    ? t('vehicle.confirmBlockedPixels', { detail: blocked.detail })
    : t('vehicle.confirmBlockedZoom');

  return (
    <div style={{
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      gap: 6,
      marginTop: 4,
      padding: '3px 6px',
      background: 'rgba(255, 152, 0, 0.10)',
      borderRadius: 3,
      fontSize: 10,
      lineHeight: '14px',
      color: colors.warning,
    }}>
      <span>{reasonLabel}</span>
      {canForce && (
        <button
          type="button"
          {...handlers}
          title={t('vehicle.holdToOverride')}
          style={{
            flex: '0 0 auto',
            padding: '2px 6px',
            border: `1px solid ${colors.warning}`,
            borderRadius: 3,
            fontSize: 9,
            fontWeight: 700,
            color: progress > 0 ? '#000' : colors.warning,
            background: progress > 0
              ? `linear-gradient(to right, ${colors.warning} ${progress * 100}%, transparent ${progress * 100}%)`
              : 'transparent',
            cursor: 'pointer',
            touchAction: 'manipulation',
          }}
        >
          {t('vehicle.askMeAnyway')}
        </button>
      )}
    </div>
  );
}

export default function VehicleStatusCard({
  vehicle: v, index: i, zoneDistance: zd, wpOffset,
  assignments, pendingConfirms, onForceConfirm, isContainer, phase, launchStates,
  onLaunch, gates, simMode, devMode, navpyStatus, navpyBusy, navpyConn,
  onToggleNavpy, onFlyToLocation, loadStage,
}) {
  const { t } = useTranslation();
  const linkDown = !v.link_ok;
  const gpsFix = linkDown ? null : v.gps_fix;
  const gpsHacc = linkDown ? null : v.gps_hacc;
  const battery = linkDown ? null : v.battery;

  const mp = v.mission_progress || 0;
  const trackIdx = mp - wpOffset;
  const totalKm = zd ? zd.total / 1000 : 0;
  const inAuto = v.mode === 'AUTO';

  const passedKm = inAuto && trackIdx >= 0 ? interpolatedDistance(v, zd, wpOffset) / 1000 : 0;
  const pct = totalKm > 0 ? Math.min(passedKm / totalKm, 1) : 0;

  let statusText = '';
  if (inAuto && totalKm > 0) statusText = `${passedKm.toFixed(1)} / ${totalKm.toFixed(1)} km`;
  else if (!inAuto && totalKm > 0) statusText = `0.0 / ${totalKm.toFixed(1)} km`;

  const vAssignment = Object.values(assignments || {}).find((e) => e.receiverId === v.sys_id) || null;
  const readiness = computeLaunchReadiness(v, gates);
  // Non-blocking advisories (e.g. arming checks disabled) are dismissible so they
  // don't nag. Dismissal is per-card and lasts while this card stays mounted.
  const [dismissed, setDismissed] = useState(() => new Set());
  const shownAdvisories = visibleAdvisories(readiness.advisories, dismissed);
  // While a UAV is still coming online, keep the card to identity + connection
  // progress + the load-stage badge, and hide the premature/contradictory bits
  // (readiness warnings like "mission not uploaded" while it's still downloading,
  // the force-launch button before it's ready, and the stale on-ground
  // mode/speed/alt row). They all return the moment it reaches ready.
  const initializing = isInitializing(loadStage);
  const canBungeeLaunch = !initializing && !isContainer && !v.armed && (phase === MONITOR_PHASES.PRE_LAUNCH || phase === MONITOR_PHASES.IN_FLIGHT);
  const ls = launchStates?.[v.sys_id];
  const launchBusy = ls?.state === 'arming' || ls?.state === 'launching';

  return (
    <div
      title={v.armed ? t('flightMode.armed') : t('flightMode.disarmed')}
      style={{
        marginBottom: 12,
        padding: 8,
        background: colors.surface,
        borderRadius: 4,
        // Border colour-codes armed state (red = armed) so it reads at a glance
        // without an extra tag crowding the header row.
        border: `1px solid ${v.armed ? colors.error : colors.border}`,
      }}
    >
      {/* Row 1: Identity + RTK + connectivity + power */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 4 }}>
        <span style={{ color: colors.textBright, fontSize: 13, fontWeight: 600, display: 'flex', alignItems: 'center', gap: 4 }}>
          <UavBadge name={v.name} index={i} />
          <CompanionBadge status={v.companion_status} />
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: 4, fontSize: 11 }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: 2 }}>
            <SatelliteIcon color={gpsColor(gpsFix)} />
            <span style={{ color: gpsColor(gpsFix), fontWeight: 600 }}>
              {gpsFixLabel(gpsFix)}
            </span>
            <span style={{ color: colors.textDim }}>{haccLabel(gpsHacc)}</span>
          </span>
          <SignalIcon quality={v.link_quality} ok={v.link_ok} />
          <span style={{ color: linkBarColor(v.link_ok ? linkBars(v.link_quality) : 0), fontWeight: 600, width: '3.5ch', textAlign: 'right' }}>
            {v.link_ok ? `${v.link_quality ?? 0}%` : '--%'}
          </span>
          <BatteryIcon color={batteryColor(battery)} level={battery} />
          <span style={{ color: batteryColor(battery), fontWeight: 600, width: '3.5ch', textAlign: 'right' }}>
            {battery != null ? `${Math.round(battery)}%` : '--'}
          </span>
        </span>
      </div>
      {/* Row 2: Navigation — flight mode + armed state + speed/alt.
          Hidden while initializing (stale zeros on the ground). */}
      {!initializing && (
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 4, fontSize: 11 }}>
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <span style={{ color: colors.accent, fontWeight: 600 }}>{v.mode || '--'}</span>
          </span>
          <span style={{ display: 'flex', alignItems: 'center', gap: 8, color: colors.textDim }}>
            <span>{v.ground_speed != null ? `${v.ground_speed.toFixed(1)} m/s` : ''}</span>
            <span>{v.alt_rel != null ? `${Math.round(v.alt_rel)} m` : ''}</span>
          </span>
        </div>
      )}
      {/* Row 3: while the UAV is still coming online, trace its load stage
          (connecting → downloading); once ready, the mission status takes over. */}
      {loadStage && loadStage !== 'ready' ? (
        <LoadStageRow stage={loadStage} progress={v.mission_download_progress} />
      ) : (
        <MissionStatusRow
          vehicle={v}
          missionStatus={computeMissionStatus(v, vAssignment, wpOffset, hasPendingConfirm(pendingConfirms || {}, v.sys_id))}
          assignment={vAssignment}
          onFlyToLocation={onFlyToLocation}
          searchProgress={totalKm > 0 ? { pct: pct * 100, statusText: statusText } : null}
        />
      )}
      {!initializing && v.confirm_blocked && (
        <ConfirmBlockedRow
          blocked={v.confirm_blocked}
          sysId={v.sys_id}
          onForceConfirm={onForceConfirm}
        />
      )}
      {!initializing && readiness.issues.length > 0 && (
        <div style={{
          marginTop: 4,
          padding: '3px 6px',
          background: 'rgba(244, 67, 54, 0.12)',
          borderRadius: 3,
          fontSize: 10,
          lineHeight: '14px',
          color: colors.error,
        }}>
          {readiness.issues.map((w, wi) => <div key={wi}>{typeof w === 'object' ? t(w.key, w) : (typeof w === 'string' && w.startsWith('prearm.') ? t(w) : w)}</div>)}
        </div>
      )}
      {!initializing && shownAdvisories.length > 0 && (
        <div style={{ marginTop: 4 }}>
          {shownAdvisories.map((a, ai) => {
            const key = typeof a === 'object' ? a.key : a;
            const label = typeof a === 'object' ? t(a.key, a) : t(a);
            return (
              <div
                key={ai}
                style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center',
                  gap: 6,
                  padding: '2px 6px',
                  background: 'rgba(255, 152, 0, 0.10)',
                  borderRadius: 3,
                  fontSize: 10,
                  lineHeight: '14px',
                  color: colors.warning,
                }}
              >
                <span>{label}</span>
                <button
                  type="button"
                  aria-label={t('prearm.dismiss')}
                  title={t('prearm.dismiss')}
                  onClick={() => setDismissed((prev) => new Set(prev).add(key))}
                  style={{
                    background: 'none',
                    border: 'none',
                    color: colors.warning,
                    cursor: 'pointer',
                    padding: 0,
                    fontSize: 12,
                    lineHeight: '12px',
                  }}
                >
                  ✕
                </button>
              </div>
            );
          })}
        </div>
      )}
      {canBungeeLaunch && (
        <ForceLaunchButton
          sysId={v.sys_id}
          busy={launchBusy}
          ready={readiness.ready}
          onLaunch={onLaunch}
        />
      )}
      {devMode && !v.pending && (
        <NavpySimButton
          sysId={v.sys_id}
          instance={navpyStatus[v.sys_id]}
          busy={navpyBusy[v.sys_id]}
          connection={navpyConn}
          onToggle={onToggleNavpy}
        />
      )}
    </div>
  );
}

