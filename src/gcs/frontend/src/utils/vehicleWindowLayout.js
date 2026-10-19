/**
 * Layout contract of the monitor view's vehicle window and the swapped map
 * (docs/design/gcs_video_streams.md, section 5 "Touch and layout" and
 * "Vehicle window"). Pure numbers, so the contract is testable without a
 * browser; the components render exactly these sizes.
 *
 * The window sits top-right of the map pane, one margin below the top-right
 * action bar's measured bottom. The reset-view button holds the bottom-right
 * corner and the main map's credit line the bottom-left, so the window's
 * bottom stops one margin above both; the picture shrinks to fit. Swapped
 * (maximized), the window takes the pane below the action bar and the map
 * shrinks into a small box at the top right of the window's picture, under
 * its header, control and status rows and above the picture's own credit
 * line (the synthetic view has one, camera video has none).
 */

/** Touch target for every new or touched control. */
export const TOUCH_TARGET_PX = 44;
/** Distance from the pane edges and between neighbouring controls. */
export const PANEL_MARGIN_PX = 12;
/**
 * Height of the monitor map's top-right bar controls (map buttons, plan).
 * They keep the existing 32 px map-button size so the bar stays one
 * consistent row (owner, 2026-10-18).
 */
export const MAP_CONTROL_PX = 32;
/** Size of the existing reset-view map button in the bottom-right corner. */
export const RESET_VIEW_PX = 32;
/** Below this map-pane width the UAV buttons show only the icon and the number. */
export const NARROW_PANE_PX = 480;
/** Normal window width when the pane is wide enough for it. */
export const DOCKED_WIDTH_PX = 432;
/** Header text size; the header background is opaque. */
export const HEADER_FONT_PX = 14;
/** The window's border, between its edge and its rows and picture. */
export const WINDOW_BORDER_PX = 1;

export const HEADER_PADDING_PX = 6;
export const HEADER_GAP_PX = 4;
/** UAV button with the icon and the full "UAV n" label (longer names are cut with an ellipsis). */
export const FULL_BADGE_PX = 72;
/** UAV button with the icon and the number only (narrow pane). */
export const NUMBER_BADGE_PX = TOUCH_TARGET_PX;
/** The single menu button that holds the UAV buttons that do not fit. */
export const MENU_BUTTON_PX = TOUCH_TARGET_PX;
/** The "UAV n · stream name" title keeps at least this much, then ellipsizes. */
export const TITLE_MIN_PX = 88;
/** The live-state label (LIVE, CONNECTING, ...). */
export const STATE_PX = 96;
/** The swapped small map is at most this wide, and at most half the large window. */
export const SMALL_MAP_MAX_PX = 320;
/** Cesium's ion logo (Assets/Images/ion-credit.png, 138 px wide; it does not wrap) and the credit line's 5 px right padding (CesiumWidget.css). */
const ION_CREDIT_PX = 138 + 5;
/**
 * The small map is never narrower: its own credit line stays whole inside its
 * 1 px border, and its 16:9 height stays above the touch target. A pane with
 * less room above the picture's credit line offers no swap (canSwap false).
 */
export const SMALL_MAP_MIN_PX = ION_CREDIT_PX + 2 * WINDOW_BORDER_PX;
/** Inset of the picture's top-left tags (SYNTHETIC · not camera, NO FRAMES). */
export const PICTURE_TAG_INSET_PX = 6;

/** Space the reset-view button takes from the bottom of the pane, plus the margin above it. */
const RESET_VIEW_RESERVE_PX = PANEL_MARGIN_PX + RESET_VIEW_PX + PANEL_MARGIN_PX;

/** Picture aspect ratio (camera video, the synthetic view and the small map). */
export const PICTURE_ASPECT = 16 / 9;

function badgeLayout(width, narrow, badgeCount, selectedIndex) {
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

  // Buttons keep the fleet order; the selected one is always among the visible.
  const visible = Array.from({ length: visibleCount }, (_, i) => i);
  if (menu && selectedIndex >= visibleCount && visibleCount > 0) {
    visible[visibleCount - 1] = selectedIndex;
  }
  const overflow = [];
  for (let i = 0; i < badgeCount; i++) if (!visible.includes(i)) overflow.push(i);
  return { badgeLabel: narrow ? 'number' : 'full', badgeWidth, visible, overflow };
}

/** Width inside the window's border: the picture's width. */
const pictureWidth = (win) => win.width - 2 * WINDOW_BORDER_PX;

/**
 * The small map inside the large window's picture: 12 px under the window's
 * last row and in from its right edge, 16:9, at most min(320 px, half the
 * window) wide, and 12 px above the picture's credit line.
 */
function smallMapBox(win, paneHeight, windowChromePx, windowCreditsPx) {
  const top = win.top + WINDOW_BORDER_PX + windowChromePx + PANEL_MARGIN_PX;
  const bottomLimit = paneHeight - PANEL_MARGIN_PX - WINDOW_BORDER_PX - windowCreditsPx - PANEL_MARGIN_PX;
  const widthCap = Math.min(SMALL_MAP_MAX_PX, Math.floor(pictureWidth(win) / 2));
  const byHeight = Math.floor(Math.max(0, bottomLimit - top) * PICTURE_ASPECT);
  const width = Math.min(widthCap, byHeight);
  return {
    top,
    right: PANEL_MARGIN_PX + WINDOW_BORDER_PX + PANEL_MARGIN_PX,
    width,
    height: Math.round(width / PICTURE_ASPECT),
  };
}

/** Swapped: the picture's top-left tag wraps 12 px before the small map instead of running under it. */
const tagMaxWidth = (win, smallMap) =>
  Math.max(0, pictureWidth(win) - PICTURE_TAG_INSET_PX - PANEL_MARGIN_PX - smallMap.width - PANEL_MARGIN_PX);

/**
 * @param {object} p
 * @param {number} p.paneWidth - map pane width in px.
 * @param {number} p.paneHeight - map pane height in px.
 * @param {number} p.topControlsBottom - bottom edge of the top-right action bar, px from the pane top.
 * @param {number} p.badgeCount - UAVs in the fleet.
 * @param {number} [p.selectedIndex] - index of the window's UAV among them, -1 if none.
 * @param {boolean} [p.swapped] - the window is maximized and the map small.
 * @param {number} [p.mapCreditsPx] - height of the main map's credit line.
 * @param {number} [p.windowChromePx] - height of the window's rows above its picture.
 * @param {number} [p.windowCreditsPx] - height of the window picture's credit line (synthetic view).
 * @returns {{ narrow: boolean,
 *   window: { top: number, right: number, width: number, height: number|null, maxHeight: number },
 *   smallMap: { top: number, right: number, width: number, height: number } | null,
 *   canSwap: boolean,
 *   tagMaxWidth: number | null,
 *   badgeLabel: string, badgeWidth: number, visible: number[], overflow: number[] }}
 */
export function vehicleWindowLayout({
  paneWidth, paneHeight, topControlsBottom, badgeCount, selectedIndex = -1, swapped = false,
  mapCreditsPx = 0, windowChromePx = 0, windowCreditsPx = 0,
}) {
  const narrow = paneWidth < NARROW_PANE_PX;
  const top = topControlsBottom + PANEL_MARGIN_PX;

  // Reset view is hidden while swapped, so the window reaches the bottom margin.
  const swappedHeight = Math.max(0, paneHeight - top - PANEL_MARGIN_PX);
  const swappedWin = {
    top, right: PANEL_MARGIN_PX, width: Math.max(0, paneWidth - 2 * PANEL_MARGIN_PX),
    height: swappedHeight, maxHeight: swappedHeight,
  };
  const swappedMap = smallMapBox(swappedWin, paneHeight, windowChromePx, windowCreditsPx);
  // Too little room for a whole small map: no swap rather than a map that
  // covers a credit line, clips its own or reaches size 0.
  const canSwap = swappedMap.width >= SMALL_MAP_MIN_PX;

  let win;
  let smallMap = null;
  if (swapped && canSwap) {
    win = swappedWin;
    smallMap = swappedMap;
  } else {
    const bottomReserve = Math.max(RESET_VIEW_RESERVE_PX, mapCreditsPx + PANEL_MARGIN_PX);
    win = {
      top,
      right: PANEL_MARGIN_PX,
      width: Math.min(DOCKED_WIDTH_PX, Math.max(0, paneWidth - 2 * PANEL_MARGIN_PX)),
      height: null,
      maxHeight: Math.max(0, paneHeight - top - bottomReserve),
    };
  }

  return {
    narrow,
    window: win,
    smallMap,
    canSwap,
    tagMaxWidth: smallMap ? tagMaxWidth(win, smallMap) : null,
    ...badgeLayout(win.width, narrow, badgeCount, selectedIndex),
  };
}

/** Fallback for the action bar's bottom edge before it is measured: one row. */
export const DEFAULT_TOP_CONTROLS_BOTTOM_PX = PANEL_MARGIN_PX + MAP_CONTROL_PX;
