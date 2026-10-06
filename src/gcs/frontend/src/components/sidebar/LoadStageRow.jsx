import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { progressPct } from '../../utils/loadStage';

/**
 * Row 3 of a UAV status card WHILE the vehicle is still loading (connecting /
 * downloading its mission). Overrides the mission-status row until the vehicle is
 * ready, so the operator can trace a UAV coming online instead of seeing it pop
 * in already "done". Once ready, VehicleStatusCard renders MissionStatusRow
 * instead.
 *
 * Pure display — the stage is computed upstream (useLoadStages).
 */
const STAGE_META = {
  connecting: { color: colors.warning, labelKey: 'loadStage.connecting', hintKey: 'loadStage.connectingHint' },
  downloading: { color: colors.accent, labelKey: 'loadStage.downloading', hintKey: 'loadStage.downloadingHint' },
};

export default function LoadStageRow({ stage, progress }) {
  const { t } = useTranslation();
  const meta = STAGE_META[stage];
  if (!meta) return null;

  const pct = stage === 'downloading' ? progressPct(progress) : null;
  const hasCount = pct != null && Array.isArray(progress);

  return (
    <div>
      <div style={{ display: 'flex', gap: 6, fontSize: 10, alignItems: 'center' }}>
        <style>{`
          @keyframes loadStagePulse { 0%,100% { opacity: 1; } 50% { opacity: 0.25; } }
          @media (prefers-reduced-motion: reduce) { .loadStageDot { animation: none !important; } }
        `}</style>
        <span style={{
          padding: '1px 5px',
          borderRadius: 3,
          background: meta.color,
          color: '#fff',
          fontWeight: 700,
          fontSize: 9,
          textTransform: 'uppercase',
          flexShrink: 0,
        }}>
          {t(meta.labelKey)}
        </span>
        <span
          className="loadStageDot"
          style={{
            width: 6,
            height: 6,
            borderRadius: '50%',
            background: meta.color,
            flexShrink: 0,
            animation: 'loadStagePulse 1s ease-in-out infinite',
          }}
        />
        <span style={{ color: colors.textDim }}>
          {hasCount ? `${progress[0]}/${progress[1]} waypoints` : t(meta.hintKey)}
        </span>
      </div>
      {hasCount && (
        <div style={{ height: 3, background: colors.bgLight, borderRadius: 2, overflow: 'hidden', marginTop: 3 }}>
          <div style={{
            height: '100%',
            width: `${pct}%`,
            background: meta.color,
            transition: 'width 0.3s',
          }} />
        </div>
      )}
    </div>
  );
}
