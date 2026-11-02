import React from 'react';

/**
 * Toast-style notification banner positioned at bottom-center of the map area.
 * Renders nothing when text is null/undefined. `top` moves it there instead,
 * clear of what lies at the bottom. The text wraps on a narrow pane.
 */
export default function NotificationBanner({ text, top }) {
  if (!text) return null;
  return (
    <div style={{
      position: 'absolute',
      ...(top != null ? { top } : { bottom: 60 }),
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
      width: 'max-content',
      maxWidth: 'calc(100% - 24px)',
      boxSizing: 'border-box',
      textAlign: 'center',
    }}>
      {text}
    </div>
  );
}
