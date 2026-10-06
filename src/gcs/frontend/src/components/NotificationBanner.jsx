import React from 'react';

/**
 * Toast-style notification banner positioned at bottom-center of the map area.
 * Renders nothing when text is null/undefined.
 */
export default function NotificationBanner({ text }) {
  if (!text) return null;
  return (
    <div style={{
      position: 'absolute',
      bottom: 60,
      left: '50%',
      transform: 'translateX(-50%)',
      background: 'rgba(0,0,0,0.75)',
      color: '#fff',
      fontFamily: 'monospace',
      fontSize: 14,
      padding: '6px 16px',
      borderRadius: 6,
      zIndex: 50,
      pointerEvents: 'none',
      whiteSpace: 'nowrap',
    }}>
      {text}
    </div>
  );
}
