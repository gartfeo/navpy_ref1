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
 * its bar and warning rows and above the picture's own credit
 * line (the synthetic view has one, camera video has none). Every control is
 * the app's 32 px map button (docs/design/gcs_video_streams.md, v12).
 */

/** Distance from the pane edges and between neighbouring controls. */
export const PANEL_MARGIN_PX = 12;
/**
 * Height of the monitor map's top-right bar controls (map buttons, plan) and
 * of every control in the vehicle window: the app's 32 px map button
 * (owner, 2026-10-18 and 2026-10-20).
 */
export const MAP_CONTROL_PX = 32;
/** Size of the existing reset-view map button in the bottom-right corner. */
export const RESET_VIEW_PX = 32;
/** Normal window width when the pane is wide enough for it. */
export const DOCKED_WIDTH_PX = 320;
/** Header text size of the RC FPV tag. */
export const HEADER_FONT_PX = 14;
/** Text size in the vehicle window: the plan button's 12 px. */
export const WINDOW_FONT_PX = 12;
/** The window's border, between its edge and its rows and picture. */
export const WINDOW_BORDER_PX = 1;
/** Padding of the window's bar and gap between its controls. */
export const BAR_PADDING_PX = 4;
export const BAR_GAP_PX = 4;
/** The UAV chip is at most this wide; only its name ellipsizes. */
export const CHIP_MAX_PX = 160;
/** The swapped small map is at most this wide, and at most half the large window. */
export const SMALL_MAP_MAX_PX = 320;
/** Cesium's ion logo (Assets/Images/ion-credit.png, 138 px wide; it does not wrap) and the credit line's 5 px right padding (CesiumWidget.css). */
const ION_CREDIT_PX = 138 + 5;
/**
 * The small map is never narrower: its own credit line stays whole inside its
 * 1 px border. A pane with less room above the picture's credit line offers no
 * swap (canSwap false).
 */
export const SMALL_MAP_MIN_PX = ION_CREDIT_PX + 2 * WINDOW_BORDER_PX;
/** Inset of the picture's top-left tags (LIVE, SYNTHETIC · not camera, NO FRAMES). */
export const PICTURE_TAG_INSET_PX = 6;

/** Space the reset-view button takes from the bottom of the pane, plus the margin above it. */
const RESET_VIEW_RESERVE_PX = PANEL_MARGIN_PX + RESET_VIEW_PX + PANEL_MARGIN_PX;

/** Picture aspect ratio (camera video, the synthetic view and the small map). */
export const PICTURE_ASPECT = 16 / 9;
/**
 * The normal-size picture never gets shorter: the 16:9 height of the smallest
 * small map. With less room the picture is not drawn (pictureFits false).
 */
export const PICTURE_MIN_H_PX = Math.ceil(SMALL_MAP_MIN_PX / PICTURE_ASPECT);

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
 * @param {boolean} [p.swapped] - the window is maximized and the map small.
 * @param {number} [p.mapCreditsPx] - height of the main map's credit line.
 * @param {number|null} [p.windowChromePx] - height of the window's bar and warning rows
 *   above its picture (never the "too little room" row, which replaces the picture);
 *   null until measured, and then no picture at normal size (none mounts to be closed again).
 * @param {number} [p.windowCreditsPx] - height of the window picture's credit line (synthetic view).
 * @returns {{
 *   window: { top: number, right: number, width: number, height: number|null, maxHeight: number },
 *   smallMap: { top: number, right: number, width: number, height: number } | null,
 *   canSwap: boolean,
 *   pictureFits: boolean,
 *   tagMaxWidth: number | null }}
 */
export function vehicleWindowLayout({
  paneWidth, paneHeight, topControlsBottom, swapped = false,
  mapCreditsPx = 0, windowChromePx = 0, windowCreditsPx = 0,
}) {
  const top = topControlsBottom + PANEL_MARGIN_PX;

  // Reset view is hidden while swapped, so the window reaches the bottom margin.
  const swappedHeight = Math.max(0, paneHeight - top - PANEL_MARGIN_PX);
  const swappedWin = {
    top, right: PANEL_MARGIN_PX, width: Math.max(0, paneWidth - 2 * PANEL_MARGIN_PX),
    height: swappedHeight, maxHeight: swappedHeight,
  };
  const chromePx = windowChromePx ?? 0;
  const swappedMap = smallMapBox(swappedWin, paneHeight, chromePx, windowCreditsPx);
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

  // Normal size: the picture keeps its width and shrinks in height to the
  // window's limit, never below PICTURE_MIN_H_PX. Swapped: it fills the window.
  const pictureFits = smallMap != null
    || (windowChromePx != null && win.maxHeight - 2 * WINDOW_BORDER_PX - windowChromePx >= PICTURE_MIN_H_PX);

  return {
    window: win,
    smallMap,
    canSwap,
    pictureFits,
    tagMaxWidth: smallMap ? tagMaxWidth(win, smallMap) : null,
  };
}

/** Fallback for the action bar's bottom edge before it is measured: one row. */
export const DEFAULT_TOP_CONTROLS_BOTTOM_PX = PANEL_MARGIN_PX + MAP_CONTROL_PX;
