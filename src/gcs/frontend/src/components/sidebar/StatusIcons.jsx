import React from 'react';
import { linkBars, linkBarColor } from '../hud/StatusBarIndicator';

export function BatteryIcon({ color, level }) {
  const cells = 4;
  const filled = level == null ? 0 : Math.round((level / 100) * cells);
  const dim = 'rgba(255,255,255,0.15)';
  return (
    <svg width={16} height={9} viewBox="0 0 16 9" style={{ verticalAlign: 'middle' }}>
      <rect x={0.5} y={0.5} width={13} height={8} rx={1.5} ry={1.5}
        fill="none" stroke={color} strokeWidth={1} />
      <rect x={14} y={2.5} width={1.5} height={4} rx={0.5} ry={0.5} fill={color} />
      {Array.from({ length: cells }, (_, i) => (
        <rect key={i} x={1.5 + i * 3} y={1.5} width={2.5} height={6} rx={0.5}
          fill={i < filled ? color : dim} />
      ))}
    </svg>
  );
}

export function SatelliteIcon({ color }) {
  return (
    <svg width={11} height={11} viewBox="0 0 11 11" style={{ verticalAlign: 'middle' }}>
      <circle cx={5.5} cy={5.5} r={2} fill="none" stroke={color} strokeWidth={1} />
      <circle cx={5.5} cy={5.5} r={1} fill={color} />
      <line x1={5.5} y1={0} x2={5.5} y2={2} stroke={color} strokeWidth={0.8} />
      <line x1={5.5} y1={9} x2={5.5} y2={11} stroke={color} strokeWidth={0.8} />
      <line x1={0} y1={5.5} x2={2} y2={5.5} stroke={color} strokeWidth={0.8} />
      <line x1={9} y1={5.5} x2={11} y2={5.5} stroke={color} strokeWidth={0.8} />
    </svg>
  );
}

export function SignalIcon({ quality, ok }) {
  const filled = ok ? linkBars(quality) : 0;
  const barColor = linkBarColor(filled);
  const dim = 'rgba(255,255,255,0.15)';
  return (
    <svg width={13} height={10} viewBox="0 0 13 10" style={{ verticalAlign: 'middle' }}>
      {[0, 1, 2, 3].map((i) => {
        const h = 3 + i * 2.3;
        const x = i * 3.3;
        return (
          <rect key={i} x={x} y={10 - h} width={2.5} height={h} rx={0.5}
            fill={i < filled ? barColor : dim} />
        );
      })}
    </svg>
  );
}
