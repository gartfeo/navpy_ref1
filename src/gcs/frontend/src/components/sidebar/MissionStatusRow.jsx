import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { flatDist } from '../../utils/geo';

/**
 * Format distance: "X.X km" if >= 1000m, "XXX m" otherwise.
 */
function formatDist(meters) {
  return meters >= 1000
    ? `${(meters / 1000).toFixed(1)} km`
    : `${Math.round(meters)} m`;
}

/**
 * Colored pill badge matching existing AssignmentRow style.
 */
function Badge({ label, color }) {
  return (
    <span style={{
      padding: '1px 5px',
      borderRadius: 3,
      background: color,
      color: '#fff',
      fontWeight: 700,
      fontSize: 9,
      textTransform: 'uppercase',
      flexShrink: 0,
    }}>
      {label}
    </span>
  );
}

/**
 * Task info: "#{taskId} {taskType}"
 */
function TaskLabel({ assignment }) {
  if (!assignment) return null;
  return (
    <span style={{ color: colors.textBright, fontWeight: 600, flexShrink: 0 }}>
      #{assignment.taskId} {assignment.taskType}
    </span>
  );
}

/**
 * Clickable coordinates that fly to the assignment location.
 */
function ClickableCoords({ assignment, onFlyToLocation }) {
  if (!assignment) return null;
  return (
    <span
      onClick={() => onFlyToLocation?.(assignment.lat, assignment.lon)}
      style={{
        color: colors.accent,
        cursor: 'pointer',
        marginLeft: 'auto',
        flexShrink: 0,
      }}
    >
      {assignment.lat.toFixed(5)}, {assignment.lon.toFixed(5)}
    </span>
  );
}

/**
 * Thin progress bar matching WPProgress style from MonitoringSidebar.
 */
function ProgressBar({ pct }) {
  return (
    <div style={{ height: 4, background: colors.bgLight, borderRadius: 2, overflow: 'hidden', marginTop: 4 }}>
      <div style={{
        height: '100%',
        width: `${Math.round(Math.min(pct, 100))}%`,
        background: colors.accent,
        transition: 'width 0.5s',
      }} />
    </div>
  );
}

/**
 * Status-specific content renderers keyed by status id.
 * Each returns the inline elements (after the badge) and optional extra row.
 */
const STATUS_RENDERERS = {
  idle: (_v, _a, _fly, searchProgress) => ({
    inline: searchProgress
      ? <span style={{ color: colors.textDim, marginLeft: 'auto' }}>{searchProgress.statusText}</span>
      : null,
    extra: null,
  }),
  en_route: (_v, _a, _fly, searchProgress) => ({
    inline: searchProgress
      ? <span style={{ color: colors.textDim, marginLeft: 'auto' }}>{searchProgress.statusText}</span>
      : null,
    extra: null,
  }),

  searching: (_v, _a, _fly, searchProgress) => ({
    inline: searchProgress
      ? <span style={{ color: colors.textDim, marginLeft: 'auto' }}>{searchProgress.statusText}</span>
      : null,
    extra: searchProgress
      ? <ProgressBar pct={searchProgress.pct} />
      : null,
  }),

  waiting: (_v, assignment, onFlyToLocation) => ({
    inline: (
      <>
        <TaskLabel assignment={assignment} />
        <ClickableCoords assignment={assignment} onFlyToLocation={onFlyToLocation} />
      </>
    ),
    extra: null,
  }),

  assigned: (_v, assignment, onFlyToLocation) => ({
    inline: (
      <>
        <TaskLabel assignment={assignment} />
        <ClickableCoords assignment={assignment} onFlyToLocation={onFlyToLocation} />
      </>
    ),
    extra: null,
  }),

  approaching: (vehicle, assignment, onFlyToLocation) => {
    let distLabel = null;
    if (vehicle && assignment && vehicle.lat != null && vehicle.lon != null) {
      const d = flatDist(
        { lat: vehicle.lat, lon: vehicle.lon },
        { lat: assignment.lat, lon: assignment.lon },
      );
      distLabel = <span style={{ color: colors.textDim }}>{formatDist(d)}</span>;
    }
    return {
      inline: (
        <>
          <TaskLabel assignment={assignment} />
          {distLabel}
          <ClickableCoords assignment={assignment} onFlyToLocation={onFlyToLocation} />
        </>
      ),
      extra: null,
    };
  },

  confirming: (_v, assignment, onFlyToLocation) => ({
    inline: (
      <>
        <TaskLabel assignment={assignment} />
        <ClickableCoords assignment={assignment} onFlyToLocation={onFlyToLocation} />
      </>
    ),
    extra: null,
  }),

  confirmed: (_v, assignment, onFlyToLocation) => ({
    inline: (
      <>
        <TaskLabel assignment={assignment} />
        <ClickableCoords assignment={assignment} onFlyToLocation={onFlyToLocation} />
      </>
    ),
    extra: null,
  }),

  // NavPy phases and outcomes (docs/design/gcs_mission_phase.md); the outcome
  // reason row and the "Last: ..." line are drawn below for every status.
  homing: () => ({ inline: null, extra: null }),
  recovering: () => ({ inline: null, extra: null }),
  docked: () => ({ inline: null, extra: null }),
  dock_failed: () => ({ inline: null, extra: null }),
};

const LAST_KEYS = {
  docked: 'missionStatus.lastDocked',
  dock_failed: 'missionStatus.lastDockFailed',
};

/** Why the dock failed, under a DOCK FAILED (SIM) badge. */
function ReasonRow({ outcome }) {
  const { t } = useTranslation();
  if (!outcome?.reasonKey) return null;
  return (
    <div style={{
      marginTop: 4, padding: '3px 6px', borderRadius: 3, fontSize: 10,
      background: 'rgba(244,67,54,.12)', color: colors.error,
    }}>
      {t('missionStatus.reason', { reason: t(outcome.reasonKey) })}
    </div>
  );
}

/** A held outcome while NavPy has moved on, e.g. searching after a dock. */
function LastOutcome({ last }) {
  const { t } = useTranslation();
  if (!last) return null;
  return (
    <div style={{ color: colors.textDim, fontSize: 10, marginTop: 4 }}>
      {t('missionStatus.last', { outcome: t(LAST_KEYS[last.kind]) })}
    </div>
  );
}

/**
 * MissionStatusRow renders Row 3 of the UAV status card:
 * always-visible mission lifecycle status with contextual info.
 *
 * Pure display component -- no state, no effects.
 */
export default function MissionStatusRow({ vehicle, missionStatus, assignment, onFlyToLocation, searchProgress }) {
  const { t } = useTranslation();
  const renderer = STATUS_RENDERERS[missionStatus.status];
  if (!renderer) return null;

  const { inline, extra } = renderer(vehicle, assignment, onFlyToLocation, searchProgress);

  const stale = !!missionStatus.stale;
  return (
    <div style={stale ? { opacity: 0.55 } : undefined}>
      <div style={{ display: 'flex', gap: 6, fontSize: 10, alignItems: 'center' }}>
        <Badge label={t(missionStatus.labelKey)} color={missionStatus.color} />
        {inline}
        {stale && <span style={{ color: colors.textDim }}>{t('missionStatus.lastHeard')}</span>}
      </div>
      {extra}
      <ReasonRow outcome={missionStatus.outcome} />
      <LastOutcome last={missionStatus.last} />
    </div>
  );
}
