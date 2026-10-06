import React from 'react';

// Sizing preset identifiers, in the approved Small / Medium / Large order.
export const DOCK_CLASS_IDS = ['small', 'medium', 'large'];
const SYMBOL_WIDTHS = [8, 13, 18];

/** Dock-pad symbols distinguish presets; icon widths are not physical dimensions. */
export default function DockClassIcon({ id, size = 16 }) {
  const index = DOCK_CLASS_IDS.indexOf(id);
  if (index < 0) return null;
  const width = SYMBOL_WIDTHS[index];
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none"
    stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round"
    aria-hidden="true" style={{ flexShrink: 0 }}>
    <rect x={(24 - width) / 2} y="5" width={width} height="14" rx="2" />
    <path d="M10 9h2a3 3 0 0 1 0 6h-2z" />
  </svg>;
}
