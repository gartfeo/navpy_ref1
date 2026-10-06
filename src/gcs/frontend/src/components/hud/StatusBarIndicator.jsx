import React from 'react';
import { colors } from '../../styles';

const FIX_LABELS = {
  0: '--', 1: '--', 2: '2D', 3: '3D',
  4: 'DGPS', 5: 'RTK\u2009F', 6: 'RTK',
};

export const gpsFixLabel = (fix) => FIX_LABELS[fix] ?? '--';

export const gpsColor = (fix) => {
  if (fix == null || fix <= 1) return colors.error;
  if (fix === 2) return colors.warning;
  if (fix >= 5) return colors.accent;   // RTK float / fixed — cyan
  return colors.success;                // 3D / DGPS — green
};

/** Number of filled signal bars (0-4) from link quality 0-100. */
export const linkBars = (quality) => {
  if (quality == null || quality <= 0) return 0;
  if (quality >= 80) return 4;
  if (quality >= 55) return 3;
  if (quality >= 30) return 2;
  return 1;
};

export const linkBarColor = (bars) => {
  if (bars <= 0) return colors.error;
  if (bars <= 1) return colors.error;
  if (bars <= 2) return colors.warning;
  return colors.success;
};

/* --- SVG bar constants (used by SignalBars below) --- */
export const BAR_W = 3;
export const BAR_GAP = 1.5;
export const BAR_COUNT = 4;
export const BAR_MAX_H = 14;
export const BAR_MIN_H = 4;
export const SVG_W = BAR_COUNT * BAR_W + (BAR_COUNT - 1) * BAR_GAP;

/** Format horizontal accuracy for display. <1m → cm, ≥1m → m. */
export const haccLabel = (hacc) => {
  if (hacc == null) return '--';
  if (hacc < 1) return Math.round(hacc * 100) + ' cm';
  return hacc.toFixed(1) + ' m';
};

export default React.memo(function StatusBarIndicator({
  gpsFix, gpsHacc, linkOk, linkQuality,
}) {
  const fixLabel = gpsFixLabel(gpsFix);
  const fixClr = gpsColor(gpsFix);
  const filledBars = linkOk ? linkBars(linkQuality) : 0;
  const barFillColor = linkBarColor(filledBars);

  return (
    <div style={{
      fontFamily: '"Consolas", "Monaco", "Courier New", monospace',
      background: 'rgba(0, 0, 0, 0.45)',
      borderRadius: 6,
      padding: '6px 8px 4px',
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'flex-end',
    }}>
      {/* Signal bars + percentage row */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
        <span style={{
          fontSize: 12,
          fontWeight: 600,
          color: barFillColor,
          display: 'inline-block',
          width: '3.5ch',
          textAlign: 'right',
        }}>
          {linkOk ? `${linkQuality ?? 0}%` : '--%'}
        </span>
        <svg width={SVG_W} height={BAR_MAX_H} viewBox={`0 0 ${SVG_W} ${BAR_MAX_H}`}>
          {Array.from({ length: BAR_COUNT }, (_, i) => {
            const h = BAR_MIN_H + ((BAR_MAX_H - BAR_MIN_H) * i) / (BAR_COUNT - 1);
            const x = i * (BAR_W + BAR_GAP);
            const y = BAR_MAX_H - h;
            const filled = i < filledBars;
            return (
              <rect
                key={i}
                x={x} y={y}
                width={BAR_W} height={h}
                rx={0.5} ry={0.5}
                fill={filled ? barFillColor : 'rgba(255,255,255,0.15)'}
              />
            );
          })}
        </svg>
      </div>
      {/* GPS fix type + accuracy */}
      <div style={{ marginTop: 1, fontSize: 9 }}>
        <span style={{ color: fixClr, fontWeight: 600 }}>
          {fixLabel}
        </span>
        <span style={{ color: colors.textDim, marginLeft: 3 }}>
          {haccLabel(gpsHacc)}
        </span>
      </div>
    </div>
  );
});
