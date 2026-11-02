import React from 'react';
import { createPortal } from 'react-dom';
import { colors } from '../../styles';

/**
 * The monitor sidebar's action in the panel's footer
 * (docs/design/gcs_handheld_layout.md). `slot` is the empty element App places
 * in the confirm list's scrolling part, with no box of its own: `pinned`, the
 * action stays in view on a short screen and over pending cards; not pinned, it
 * lies after the sidebar. Either way it keeps its state and hooks here. The
 * wrapper takes back the sidebar's 16 px bottom padding, so when the content
 * fits the action sits right after it. With no slot it renders in the flow.
 */
export default function SidebarFooter({ slot, pinned = true, children }) {
  if (!slot) return children;
  return createPortal(
    <div
      style={{
        background: colors.bgLight,
        padding: '0 16px 8px',
        marginTop: -16,
        ...(pinned && { position: 'sticky', bottom: 0 }),
      }}
    >
      {children}
    </div>,
    slot,
  );
}
