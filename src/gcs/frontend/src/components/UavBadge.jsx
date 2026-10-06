import React from 'react';
import { zoneColorsSolid } from '../styles';

/**
 * Colored UAV name badge — background matches the zone color for the vehicle index.
 */
export default function UavBadge({ name, index }) {
  const bg = zoneColorsSolid[(index ?? 0) % zoneColorsSolid.length];
  return (
    <span style={{
      display: 'inline-block',
      padding: '0 5px',
      borderRadius: 3,
      background: bg,
      color: '#fff',
      fontSize: 11,
      fontWeight: 700,
      lineHeight: '18px',
    }}>
      {name}
    </span>
  );
}
