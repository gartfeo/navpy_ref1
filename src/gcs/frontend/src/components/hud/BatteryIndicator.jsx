import React from 'react';
import { colors } from '../../styles';

export const batteryColor = (pct) => {
  if (pct == null) return colors.textDim;
  if (pct > 50) return colors.success;
  if (pct >= 20) return colors.warning;
  return colors.error;
};

const BODY_W = 32;
const BODY_H = 14;
const BODY_R = 2;
const NUB_W = 2;
const NUB_H = 6;
const FILL_PAD = 2;
const SVG_W = BODY_W + NUB_W + 1;
const SVG_H = BODY_H;

export default React.memo(function BatteryIndicator({ battery, voltage, current }) {
  const fill = battery != null ? Math.max(0, Math.min(100, battery)) : 0;
  const fillW = (BODY_W - FILL_PAD * 2) * (fill / 100);
  const color = batteryColor(battery);
  const isLow = battery != null && battery < 20;
  const outlineColor = isLow ? colors.error : (battery != null ? colors.text : colors.textDim);
  const pctText = battery != null ? `${Math.round(battery)}%` : '--%';

  const vText = voltage != null ? `${voltage.toFixed(1)}V` : null;
  const aText = current != null ? `${current.toFixed(1)}A` : null;
  const hasVA = vText || aText;

  const vaStyle = {
    fontSize: 9,
    color: colors.text,
    display: 'inline-block',
    width: '4.5ch',
    textAlign: 'right',
  };

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'flex-end',
      fontFamily: '"Consolas", "Monaco", "Courier New", monospace',
      background: 'rgba(0, 0, 0, 0.45)',
      borderRadius: 6,
      padding: '6px 8px 4px',
    }}>
      {/* Percentage + battery icon row */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
        <span style={{
          fontSize: 12,
          fontWeight: 600,
          color,
          textAlign: 'right',
          display: 'inline-block',
          width: '3.5ch',
        }}>
          {pctText}
        </span>
        <svg width={SVG_W} height={SVG_H} viewBox={`0 0 ${SVG_W} ${SVG_H}`}>
          {/* Battery body outline */}
          <rect
            x={0.5} y={0.5}
            width={BODY_W - 1} height={BODY_H - 1}
            rx={BODY_R} ry={BODY_R}
            fill="none"
            stroke={outlineColor}
            strokeWidth={1}
          />
          {/* Terminal nub */}
          <rect
            x={BODY_W} y={(BODY_H - NUB_H) / 2}
            width={NUB_W} height={NUB_H}
            rx={1} ry={1}
            fill={outlineColor}
          />
          {/* Fill bar */}
          {battery != null && fillW > 0 && (
            <rect
              x={FILL_PAD} y={FILL_PAD}
              width={fillW} height={BODY_H - FILL_PAD * 2}
              rx={1} ry={1}
              fill={color}
            />
          )}
        </svg>
      </div>
      {/* Voltage / current line — fixed-width slots */}
      <div style={{
        marginTop: 1,
        fontSize: 9,
        visibility: hasVA ? 'visible' : 'hidden',
      }}>
        <span style={vaStyle}>{vText || '0.0V'}</span>
        <span style={{ fontSize: 9, color: colors.textDim }}>{'\u00b7'}</span>
        <span style={vaStyle}>{aText || '0.0A'}</span>
      </div>
    </div>
  );
});
