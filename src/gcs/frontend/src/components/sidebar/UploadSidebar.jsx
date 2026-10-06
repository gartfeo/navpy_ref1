import React from 'react';
import { colors, zoneColorsSolid, zoneColorNames } from '../../styles';
import UavBadge from '../UavBadge';
import { SectionTitle, Divider, Label } from './SidebarPrimitives';

export default function UploadSidebar({
  analysis,
  plan,
  vehicleList,
  uploadProgress,
}) {
  const requiredUavs = analysis?.required_uavs || 0;
  const connectedCount = vehicleList.length;
  const hasEnough = connectedCount >= requiredUavs;

  return (
    <div style={{ padding: 16 }}>
      <SectionTitle>UPLOAD MISSION</SectionTitle>
      <Divider />

      {/* --- Connected Vehicles --- */}
      <Label style={{ marginBottom: 8 }}>Connected UAVs:</Label>
      {vehicleList.length === 0 ? (
        <div style={{ color: colors.textDim, fontSize: 13, marginBottom: 8 }}>
          No vehicles connected — use top bar to connect
        </div>
      ) : (
        vehicleList.map((v, i) => {
          const zone = plan?.zones?.[i];
          const progress = uploadProgress[v.sys_id];
          return (
            <div
              key={v.sys_id}
              style={{
                marginBottom: 12,
                padding: 8,
                background: colors.surface,
                borderRadius: 4,
                border: `1px solid ${colors.border}`,
              }}
            >
              <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
                <span style={{ color: colors.textBright, fontSize: 13, fontWeight: 600, display: 'flex', alignItems: 'center', gap: 4 }}>
                  <StatusDot ok={v.link_ok} /> <UavBadge name={v.name} index={i} />
                </span>
                <span style={{ color: colors.textDim, fontSize: 12 }}>
                  {v.battery != null ? `${v.battery}%` : '--'}
                </span>
              </div>
              {zone && (
                <div style={{
                  display: 'inline-block',
                  fontSize: 11,
                  fontWeight: 600,
                  marginTop: 4,
                  padding: '1px 8px',
                  borderRadius: 3,
                  background: zoneColorsSolid[i % zoneColorsSolid.length],
                  color: '#fff',
                }}>
                  {zoneColorNames[i % zoneColorNames.length]}
                </div>
              )}
              {progress && (
                <ProgressBar progress={progress.progress} done={progress.done} error={progress.error} />
              )}
            </div>
          );
        })
      )}

      <div
        style={{
          marginTop: 12,
          padding: 8,
          background: hasEnough ? colors.surface : 'rgba(255,152,0,0.15)',
          borderRadius: 4,
          border: `1px solid ${hasEnough ? colors.border : colors.warning}`,
          color: hasEnough ? colors.success : colors.warning,
          fontSize: 13,
        }}
      >
        {`Need ${requiredUavs}, found ${connectedCount}`}
      </div>
    </div>
  );
}


function StatusDot({ ok }) {
  return (
    <span
      style={{
        display: 'inline-block',
        width: 8,
        height: 8,
        borderRadius: '50%',
        background: ok ? colors.success : colors.error,
        marginRight: 4,
      }}
    />
  );
}

function ProgressBar({ progress, done, error }) {
  const pct = Math.round(progress * 100);
  return (
    <div style={{ marginTop: 4 }}>
      <div
        style={{
          height: 4,
          background: colors.bgLight,
          borderRadius: 2,
          overflow: 'hidden',
        }}
      >
        <div
          style={{
            height: '100%',
            width: `${pct}%`,
            background: error ? colors.error : done ? colors.success : colors.accent,
            transition: 'width 0.3s',
          }}
        />
      </div>
      <div style={{ fontSize: 10, color: error ? colors.error : colors.textDim, marginTop: 2 }}>
        {error || (done ? 'Done' : `${pct}%`)}
      </div>
    </div>
  );
}
