/**
 * Layout contract of the monitor-view camera panel (docs/design/
 * gcs_video_streams.md, section 5 "Touch and layout"). Pure numbers, so the
 * contract is testable without a browser; the component renders exactly these
 * sizes.
 *
 * The panel sits bottom-left of the map pane. The reset-view button holds the
 * bottom-right corner and the monitor controls the top-right, so the panel's
 * right edge always stops one margin before the reset-view column, and the
 * expanded panel stops one margin below the top controls.
 */

/** Touch target for every new or touched control. */
export const TOUCH_TARGET_PX = 44;
/** Distance from the pane edges and between neighbouring controls. */
export const PANEL_MARGIN_PX = 12;
/**
 * Height of the monitor map's top-right bar controls (map buttons, plan,
 * follow, FPV). They keep the existing 32 px map-button size so the bar stays
 * one consistent row (owner, 2026-10-18).
 */
export const MAP_CONTROL_PX = 32;
/** Size of the existing reset-view map button in the bottom-right corner. */
export const RESET_VIEW_PX = 32;
/** Below this map-pane width the badges show only the UAV number. */
export const NARROW_PANE_PX = 480;
/** Docked panel width when the pane is wide enough for it. */
export const DOCKED_WIDTH_PX = 432;
/** Header text size; the header background is opaque. */
export const HEADER_FONT_PX = 14;

export const HEADER_PADDING_PX = 6;
export const HEADER_GAP_PX = 4;
/** Badge with the full "UAV n" label (longer names are cut with an ellipsis). */
export const FULL_BADGE_PX = 72;
/** Badge with the number only (narrow pane). */
export const NUMBER_BADGE_PX = TOUCH_TARGET_PX;
/** The single menu button that holds the badges that do not fit. */
export const MENU_BUTTON_PX = TOUCH_TARGET_PX;
/** The "UAV n · stream name" title keeps at least this much, then ellipsizes. */
export const TITLE_MIN_PX = 88;
/** The live-state label (LIVE, CONNECTING, ...). */
export const STATE_PX = 96;

/** Space taken from the right of the pane: reset-view column plus margins. */
const RIGHT_RESERVE_PX = PANEL_MARGIN_PX + RESET_VIEW_PX + PANEL_MARGIN_PX;

/**
 * @param {object} p
 * @param {number} p.paneWidth - map pane width in px.
 * @param {number} p.paneHeight - map pane height in px.
 * @param {number} p.topControlsBottom - bottom edge of the top-right controls, px from the pane top.
 * @param {number} p.badgeCount - UAVs with a stream.
 * @param {number} [p.selectedIndex] - index of the selected UAV among them, -1 if none.
 * @param {boolean} [p.expanded]
 */
export function videoPanelLayout({
  paneWidth, paneHeight, topControlsBottom, badgeCount, selectedIndex = -1, expanded = false,
}) {
  const narrow = paneWidth < NARROW_PANE_PX;
  const maxWidth = Math.max(0, paneWidth - PANEL_MARGIN_PX - RIGHT_RESERVE_PX);
  const width = expanded ? maxWidth : Math.min(DOCKED_WIDTH_PX, maxWidth);
  // Bottom margin, the panel, a margin, then the top controls.
  const maxHeight = Math.max(0, paneHeight - PANEL_MARGIN_PX - PANEL_MARGIN_PX - topControlsBottom);

  const badgeWidth = narrow ? NUMBER_BADGE_PX : FULL_BADGE_PX;
  const badgeRoom = width - 2 * HEADER_PADDING_PX - TITLE_MIN_PX - STATE_PX - 2 * HEADER_GAP_PX;
  const fits = (n) => n * badgeWidth + Math.max(0, n - 1) * HEADER_GAP_PX;

  let visibleCount = badgeCount;
  let menu = false;
  if (fits(badgeCount) > badgeRoom) {
    menu = true;
    visibleCount = 0;
    while (visibleCount < badgeCount
      && fits(visibleCount + 1) + HEADER_GAP_PX + MENU_BUTTON_PX <= badgeRoom) {
      visibleCount += 1;
    }
  }

  // Badges keep the fleet order; the selected one is always among the visible.
  const visible = Array.from({ length: visibleCount }, (_, i) => i);
  if (menu && selectedIndex >= visibleCount && visibleCount > 0) {
    visible[visibleCount - 1] = selectedIndex;
  }
  const overflow = [];
  for (let i = 0; i < badgeCount; i++) if (!visible.includes(i)) overflow.push(i);

  return {
    narrow,
    width,
    maxHeight,
    badgeLabel: narrow ? 'number' : 'full',
    badgeWidth,
    visible,
    overflow,
  };
}

/** Fallback for the top controls' bottom edge before it is measured: one row. */
export const DEFAULT_TOP_CONTROLS_BOTTOM_PX = PANEL_MARGIN_PX + MAP_CONTROL_PX;
