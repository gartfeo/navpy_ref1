/**
 * Handheld layout (docs/design/gcs_handheld_layout.md): the rule for the right
 * panel, and the boxes of the confirm list where it lies over the map. Pure
 * functions, so the contract is testable without a browser; App renders exactly
 * these.
 */
import { colors, sidebarWidth } from '../styles';
import {
  BAR_PADDING_PX, MAP_CONTROL_PX, PANEL_MARGIN_PX, PICTURE_ASPECT, RESET_VIEW_PX, WINDOW_BORDER_PX,
} from './vehicleWindowLayout';

/** The map pane the vehicle window's layout contract assumes. */
const MIN_MAP_PANE_PX = 375;
/** Below this the panel cannot dock: it lies over the map instead. */
export const DOCKED_MIN_VIEWPORT_PX = sidebarWidth + MIN_MAP_PANE_PX;

/** Over the map controls (10), under the notification banner (50) and the top bar's menus (1000). */
export const OVERLAY_Z_INDEX = 30;
/** A round outranks the map: over the map controls (10) and the vehicle window (9). */
export const COLLAPSED_LIST_Z_INDEX = 11;
/** Over the altitude tape, so a card's labels stay readable. The flight-mode column goes over it while it shows. */
export const RC_RAISED_Z_INDEX = 11;

/** Room the collapsed list must have under the vehicle window: a chip, a decided row and the start of a card. */
export const COLLAPSED_LIST_MIN_PX = 160;
/** The RC list narrows no further between the mode column and the stick. */
export const RC_LIST_MIN_WIDTH_PX = 260;

/**
 * @param {boolean} open - the panel's one flag (hooks/useSidePanel.js).
 * @param {number} viewportWidth
 * @returns {'docked'|'overlay'|'collapsed'}
 */
export function sidePanelMode(open, viewportWidth) {
  if (!open) return 'collapsed';
  return viewportWidth >= DOCKED_MIN_VIEWPORT_PX ? 'docked' : 'overlay';
}

/** Nothing persists: a reload never hides the panel on a wide screen. */
export function panelOpenAtStart(viewportWidth) {
  return viewportWidth >= DOCKED_MIN_VIEWPORT_PX;
}

/** The normal-size vehicle window's bottom with no warning row: its one-line bar over a whole 16:9 picture. */
function windowBottom(box) {
  const picture = (box.width - 2 * WINDOW_BORDER_PX) / PICTURE_ASPECT;
  return box.top + 2 * WINDOW_BORDER_PX + MAP_CONTROL_PX + 2 * BAR_PADDING_PX + picture;
}

/**
 * The confirm list over the map while the panel is collapsed, at the left, clear
 * of the 32 px column on the right (reset view, planning toolbar). It starts at
 * the box's top and grows down, like the docked list, so a round arriving or a
 * row leaving does not move the rows above it under a finger. The top is under
 * the top controls; under the vehicle window when the two cannot sit side by
 * side, unless that leaves too little room, as the cards win over the picture.
 * The box ends level with the reset view's reserve, clear of a credit line of up
 * to two lines.
 *
 * Warning rows and the credit line change with no tap, so neither sets the box:
 * a row makes the window longer, under the list.
 *
 * @param {object} p
 * @param {number} p.paneWidth
 * @param {number} p.paneHeight
 * @param {number} p.topControlsBottom - bottom edge of the top controls, px from the pane top.
 * @param {{top: number, width: number}|null} [p.windowBox] - the vehicle window when it
 *   shows at normal size; null when closed or maximized.
 * @returns {{left: number, top: number, width: number, height: number}}
 */
export function collapsedListBox({ paneWidth, paneHeight, topControlsBottom, windowBox = null }) {
  const margin = PANEL_MARGIN_PX;
  const width = Math.max(0, Math.min(sidebarWidth, paneWidth - 2 * margin - RESET_VIEW_PX - margin));
  const bottom = paneHeight - margin - RESET_VIEW_PX - margin;
  let top = topControlsBottom + margin;
  if (windowBox && paneWidth < margin + width + margin + windowBox.width + margin) {
    const underWindow = Math.ceil(windowBottom(windowBox)) + margin;
    if (bottom - underWindow >= COLLAPSED_LIST_MIN_PX) top = underWindow;
  }
  return { left: margin, top, width, height: Math.max(0, bottom - top) };
}

/**
 * The right panel's host style. One element in all three modes, only this
 * changes, so a card's auto-fire and the sidebars' state survive a toggle. No
 * transform or filter: a position: fixed modal inside still means the viewport.
 *
 * @param {object} p
 * @param {'docked'|'overlay'|'collapsed'} p.mode
 * @param {boolean} p.roundsExist - any task-confirm round, pending or decided.
 * @param {{left: number, top: number, width: number, height: number}} p.box - collapsedListBox.
 */
export function panelHostStyle({ mode, roundsExist, box }) {
  const panel = { background: colors.bgLight };
  if (mode === 'docked') {
    return {
      width: sidebarWidth,
      height: '100%',
      ...panel,
      borderLeft: `1px solid ${colors.border}`,
      flexShrink: 0,
    };
  }
  if (mode === 'overlay') {
    return {
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      width: sidebarWidth,
      maxWidth: '100%',
      zIndex: OVERLAY_Z_INDEX,
      ...panel,
      borderLeft: `1px solid ${colors.border}`,
      boxSizing: 'border-box',
    };
  }
  if (!roundsExist) return { display: 'none' };
  return {
    position: 'absolute',
    left: box.left,
    top: box.top,
    width: box.width,
    maxHeight: box.height,
    zIndex: COLLAPSED_LIST_Z_INDEX,
    ...panel,
    border: `1px solid ${colors.border}`,
    borderRadius: 6,
    boxSizing: 'border-box',
    overflow: 'hidden',
  };
}

/**
 * The RC confirm list on a narrow pane. App's placement stands while 320 px fit
 * between the mode column and the right stick (`style` is then null). With less
 * room the list narrows, down to RC_LIST_MIN_WIDTH_PX. With less than that it
 * moves above the sticks, below the "View held" banner's row, and the banner
 * takes the width. `heldRight` is the banner's right inset where App's does not hold.
 * `raised` also means the pane is too narrow to centre the FPV tag clear of the column.
 *
 * @param {object} p
 * @param {number} p.paneWidth
 * @param {number} p.paneHeight
 * @param {number} p.left - first free x right of the mode column.
 * @param {number} p.right - the list's right inset: left of the right stick, plus the gap. The sticks
 *   use one inset on both axes, so a raised list keeps the same room above the pane's bottom.
 * @param {number} p.top - App's top.
 * @param {number} p.bottom - App's bottom: above the UAV selector.
 * @param {number} p.gap
 * @param {number} p.heldBottom - bottom edge of the room kept for the "View held" banner, px from the pane top.
 * @returns {{style: object|null, heldRight: number|null, heightPx: number, raised: boolean}}
 */
export function rcListPlacement({ paneWidth, paneHeight, left, right, top, bottom, gap, heldBottom }) {
  const room = paneWidth - left - right;
  if (room >= sidebarWidth) {
    return { style: null, heldRight: null, heightPx: Math.max(0, paneHeight - top - bottom), raised: false };
  }
  if (room >= RC_LIST_MIN_WIDTH_PX) {
    return {
      style: { width: room },
      heldRight: right + room + gap,
      heightPx: Math.max(0, paneHeight - top - bottom),
      raised: false,
    };
  }
  const raisedTop = heldBottom + gap;
  const heightPx = Math.max(0, paneHeight - raisedTop - right);
  return {
    style: {
      top: raisedTop,
      left,
      right: PANEL_MARGIN_PX,
      width: 'auto',
      maxHeight: heightPx,
      zIndex: RC_RAISED_Z_INDEX,
    },
    heldRight: PANEL_MARGIN_PX,
    heightPx,
    raised: true,
  };
}
