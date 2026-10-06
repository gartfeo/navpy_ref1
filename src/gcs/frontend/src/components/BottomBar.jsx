import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors, bottomBarHeight, zoneColorsSolid } from '../styles';
import { PHASES } from '../hooks/useMissionState';
import { missionWpOffset, flatDist } from '../utils/geo';
import { useTelemetryStore } from '../hooks/useTelemetryStore';

const STAGE_LABELS = {
  clearing: 'uploadStages.clearing',
  uploading: 'uploadStages.uploading',
  verifying: 'uploadStages.verifying',
  reconnecting: 'uploadStages.reconnecting',
  complete: 'uploadStages.complete',
  failed: 'uploadStages.failed',
};

export default function BottomBar({
  phase,
  analysis,
  polygon,
  onUpload,
  onExitPlanning,
  onOpenConnect,
  vehicleList,
  plan,
  launchPoint,
  corridorPoints,
  searchPattern,
  uploading,
  uploadProgress,
  effectiveUavCount,
}) {
  const { t } = useTranslation();
  // Subscribe for live telemetry (link_ok, mission_progress, position)
  const liveVehicleList = useTelemetryStore(s => s.getVehicleList());
  if (liveVehicleList.length > 0) vehicleList = liveVehicleList;
  const hasPoly = polygon && polygon.length >= 3;
  const isCorridor = searchPattern === 'corridor';
  // Corridor mode: uploadable when there's a corridor path (launch + at least 1 waypoint)
  const canUpload = isCorridor
    ? (launchPoint && corridorPoints?.length > 0)
    : (hasPoly && analysis);

  const hasProgress = uploading && uploadProgress && Object.keys(uploadProgress).length > 0;

  return (
    <div
      style={{
        height: bottomBarHeight,
        background: colors.bg,
        borderTop: `1px solid ${colors.border}`,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        padding: '0 16px',
        color: colors.text,
        fontSize: 13,
        flexShrink: 0,
      }}
    >
      {phase === PHASES.PLANNING && (
        <>
          <div style={{ display: 'flex', gap: 12, alignItems: 'center', flex: 1, minWidth: 0 }}>
            {hasProgress ? (
              vehicleList.map((v, i) => {
                const p = uploadProgress[v.sys_id];
                return (
                  <UploadProgressItem
                    key={v.sys_id}
                    name={v.name || `UAV ${v.sys_id}`}
                    index={i}
                    progress={p}
                  />
                );
              })
            ) : isCorridor ? (
              corridorPoints?.length > 0 && plan?.zones?.length > 0 && (
                <>
                  <Stat label={t('bottomBar.dist')} value={`${plan.total_distance_km} km`} />
                  <Stat label={t('bottomBar.time')} value={`~${plan.estimated_time_min} min`} />
                  <Stat label={t('bottomBar.uavs')} value={plan.zones.length} />
                </>
              )
            ) : (
              analysis && (
                <>
                  <Stat label={t('bottomBar.area')} value={`${analysis.area_km2} km²`} />
                  <Stat label={t('bottomBar.strips')} value={analysis.strip_count} />
                  <Stat label={t('bottomBar.dist')} value={`${analysis.total_distance_km} km`} />
                  <Stat label={t('bottomBar.time')} value={`~${analysis.estimated_time_min} min`} />
                  <Stat label={t('bottomBar.uavs')} value={effectiveUavCount ?? analysis.required_uavs} />
                  <Stat label={t('bottomBar.launchZoneDistance')} value={analysis.launch_zone_buffer_m ? `${(analysis.launch_zone_buffer_m / 1000).toFixed(0)} km` : t('bottomBar.na')} />
                </>
              )
            )}
          </div>

          <div style={{ display: 'flex', gap: 8 }}>
            <BarButton onClick={onExitPlanning}>{t('bottomBar.exitPlan')}</BarButton>
            {vehicleList.length === 0 ? (
              <BarButton primary onClick={onOpenConnect}>
                {t('bottomBar.connect')}
              </BarButton>
            ) : (
              <BarButton
                primary
                onClick={onUpload}
                disabled={!canUpload || uploading}
              >
                {uploading ? t('bottomBar.uploading') : t('bottomBar.upload')}
              </BarButton>
            )}
          </div>
        </>
      )}

      {phase === PHASES.MONITOR && (
        <>
          <span>{t('bottomBar.uavsOnline', { count: vehicleList.filter((v) => v.link_ok).length })}</span>
          <span>
            {plan?.zones
              ? t('bottomBar.percentComplete', { percentage: Math.round(
                  _overallProgress(vehicleList, plan.zones, polygon, launchPoint, corridorPoints, searchPattern) * 100
                ) })
              : ''}
          </span>
        </>
      )}
    </div>
  );
}

function UploadProgressItem({ name, index, progress }) {
  const { t } = useTranslation();
  const color = zoneColorsSolid[index % zoneColorsSolid.length];
  const stage = progress?.stage;
  const wpSent = progress?.wp_sent;
  const wpTotal = progress?.wp_total;
  const done = progress?.done;
  const error = progress?.error;

  let label;
  if (error) {
    label = t('uploadStages.failed');
  } else if (done) {
    label = t('uploadStages.complete');
  } else if (stage === 'uploading' && wpSent != null && wpTotal) {
    label = t('uploadStages.wps', { sent: wpSent, total: wpTotal });
  } else {
    label = t(STAGE_LABELS[stage] || 'uploadStages.waiting');
  }

  const pct = (stage === 'uploading' && wpTotal)
    ? Math.round((wpSent || 0) / wpTotal * 100)
    : done ? 100 : stage === 'verifying' ? 95 : stage === 'clearing' ? 5 : 0;

  const barColor = error ? colors.error : done ? colors.success : color;

  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 6, minWidth: 0 }}>
      <div
        style={{
          width: 6,
          height: 6,
          borderRadius: '50%',
          background: color,
          flexShrink: 0,
        }}
      />
      <div style={{ minWidth: 0, flex: 1 }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
          <span style={{
            color: colors.textBright,
            fontSize: 11,
            fontWeight: 600,
            whiteSpace: 'nowrap',
            overflow: 'hidden',
            textOverflow: 'ellipsis',
          }}>
            {name}
          </span>
          <span style={{ color: error ? colors.error : colors.textDim, fontSize: 10, whiteSpace: 'nowrap' }}>
            {label}
          </span>
        </div>
        <div style={{
          height: 3,
          width: 80,
          background: colors.bgLight,
          borderRadius: 2,
          overflow: 'hidden',
          marginTop: 1,
        }}>
          <div style={{
            height: '100%',
            width: `${pct}%`,
            background: barColor,
            transition: 'width 0.3s',
          }} />
        </div>
      </div>
    </div>
  );
}

/** Compute overall mission progress as weighted average of distance flown per zone. */
function _overallProgress(vehicleList, zones, polygon, launchPoint, corridorPoints, searchPattern) {
  if (!zones || zones.length === 0 || vehicleList.length === 0) return 0;
  const wpOffset = missionWpOffset(
    searchPattern, polygon?.length || 0, corridorPoints?.length || 0, !!launchPoint,
  );
  let totalDist = 0;
  let passedDist = 0;
  zones.forEach((zone, i) => {
    if (!zone.track || zone.track.length < 2) return;
    const v = vehicleList[i];
    const cum = [0];
    for (let j = 1; j < zone.track.length; j++) {
      const a = zone.track[j - 1], b = zone.track[j];
      cum.push(cum[j - 1] + flatDist(a, b));
    }
    const zoneTotal = cum[cum.length - 1];
    totalDist += zoneTotal;
    if (!v) return;
    const trackIdx = (v.mission_progress || 0) - wpOffset;
    if (v.mode !== 'AUTO') {
      if (trackIdx >= zone.track.length - 1) passedDist += zoneTotal;
      return;
    }
    if (trackIdx <= 0) return;
    const prevIdx = Math.max(0, Math.min(trackIdx - 1, zone.track.length - 1));
    const nextIdx = Math.min(trackIdx, zone.track.length - 1);
    let dist = cum[prevIdx];
    if (prevIdx !== nextIdx && v.lat != null && v.lon != null) {
      const a = zone.track[prevIdx], b = zone.track[nextIdx];
      const dx = b.lon - a.lon, dy = b.lat - a.lat;
      const len2 = dx * dx + dy * dy;
      let t = len2 > 0 ? ((v.lon - a.lon) * dx + (v.lat - a.lat) * dy) / len2 : 0;
      t = Math.max(0, Math.min(1, t));
      dist += t * ((cum[nextIdx] || 0) - (cum[prevIdx] || 0));
    }
    passedDist += dist;
  });
  return totalDist > 0 ? Math.min(passedDist / totalDist, 1) : 0;
}

function Stat({ label, value }) {
  return (
    <span>
      <span style={{ color: colors.textDim }}>{label}:</span>{' '}
      <span style={{ color: colors.textBright, fontWeight: 600 }}>{value}</span>
    </span>
  );
}

function BarButton({ children, onClick, disabled, primary }) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      style={{
        background: primary ? colors.primary : colors.surface,
        color: disabled ? colors.textDim : colors.textBright,
        border: `1px solid ${colors.border}`,
        borderRadius: 4,
        padding: '4px 14px',
        fontSize: 13,
        fontWeight: primary ? 700 : 500,
        cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.5 : 1,
      }}
    >
      {children}
    </button>
  );
}
