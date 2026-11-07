/**
 * Where the shared first-person HUD (components/TelemetryHud.jsx) puts its
 * parts, for its hosts (docs/design/gcs_video_streams.md, section 5
 * "Synthetic view"): RC control over the main map, the vehicle window's
 * Synthetic picture maximized (full), and at normal size (compact). Pure, so
 * every contract is testable without a browser.
 *
 * One rule for every host: what the right panel shows, the HUD does not
 * repeat. Battery, link, flight mode and status texts show only while the
 * panel is hidden (`panelShown` false: collapsed, or the drawer closed).
 *
 * A layout: {
 *   attitude: {left, right, top, bottom, scale}|null,  // the indicator's box; scale: ladder and roll arc;
 *                                                  // a host sets null where the horizon would mislead
 *   heading: {top, height, width, left?}|null,     // `left` absent centres the tape
 *   corner: {top, right, status, mode, waypoint, numbers},  // top-right column; see TelemetryHud
 *   speed: {left, top, width, height}|null, altitude: {right, top, width, height}|null,  // tapes
 *   numbers: {top, left, right}|null,              // compact: speed and altitude as plain numbers
 *   strip: {bottom, maxMessages}|null,             // status texts
 * }
 */
import { HUD_TAPE_MAX_PX } from './appLayout';
import { PANEL_MARGIN_PX, PICTURE_TAG_INSET_PX } from './vehicleWindowLayout';

/** The heading tape's size where it has room (its 60 degree scale is drawn over this width). */
export const HEADING_TAPE_W_PX = 300;
/** Maximized: the narrowest centred heading tape beside the small map (half its 60 degree scale's width). */
export const HEADING_TAPE_MIN_W_PX = HEADING_TAPE_W_PX / 2;
/** Its height in every host: HeadingTape draws its scale labels down to about this, so a shorter canvas clips them. */
export const HEADING_TAPE_H_PX = 36;
/** The speed and altitude tapes' width over the main map. */
export const RC_TAPE_W_PX = 70;
/** RC: the HUD's top-row inset from the pane edges. */
const RC_EDGE_PX = 12;
/** RC: the status texts' bottom inset, above the sticks' centre line. */
const RC_STRIP_BOTTOM_PX = 60;
/** Most status texts at once where there is room (StatusStrip keeps the last ones). */
export const STRIP_MAX_MESSAGES = 5;

/** The tapes' width in the window picture: narrower, so the picture keeps its middle. */
export const PICTURE_TAPE_W_PX = 56;
/** Below this a tape's value box and its scale no longer both show: the tape is left out. */
export const PICTURE_TAPE_MIN_PX = 48;
/**
 * Narrowest centred attitude box beside the small map: about the normal-size
 * picture's own indicator (it is sized from min(width, height), 179 px there).
 * A narrower window draws it full width, under the small map, rather than tiny.
 */
export const PICTURE_ATTITUDE_MIN_W_PX = 180;
/**
 * Maximized: the tag's least width beside the top-right column, about its
 * one-line label; with less room left of the small map the column moves
 * below the small map instead (narrower, the tag printed a character per
 * line under the column, live check 2026-11-06).
 */
export const TAG_MIN_W_PX = 120;
/** Gap between HUD rows. */
export const HUD_GAP_PX = 4;
/** One line of the HUD's plain-text readouts (TelemetryHud READOUT_FONT_PX plus outline). */
export const READOUT_LINE_PX = 16;
/** Compact: the status texts get one line. */
const COMPACT_STRIP_MESSAGES = 1;
/**
 * Compact: the attitude scale (pitch ladder, roll arc and their labels) is
 * sized from its box; below this box height its labels are too small to read
 * and crowd the rows around it, so only the horizon and the wings show.
 */
export const ATTITUDE_SCALE_MIN_PX = 110;
/**
 * Compact: below this band height the horizon's wings and centre dot run
 * together, so the horizon fills the whole picture behind the readouts
 * instead (horizon and wings only), as it does behind the tapes when maximized.
 */
export const ATTITUDE_BAND_MIN_PX = 2 * READOUT_LINE_PX;

/**
 * RC control: the maximized layout over the main map, with RC's own items
 * (the flight-mode column shows the mode, so the HUD never repeats it). The
 * right panel is never on screen in RC (App hides it for the full-width map),
 * so the link, battery and status texts always show here.
 *
 * @param {object} p
 * @param {{top: number, height: number}} p.tapeBox - utils/appLayout.js hudTapeBox.
 * @param {{left: number, top: number, height: number}} p.speedBox - utils/appLayout.js speedTapeBox.
 * @param {{width: number, height: number}} p.corner - the top-right column's measured size.
 */
export function rcHudLayout({ tapeBox, speedBox, corner }) {
  // The altitude tape starts below the top-right column.
  const altTop = Math.max(tapeBox.top, RC_EDGE_PX + corner.height + HUD_GAP_PX);
  const altHeight = tapeBox.top + tapeBox.height - altTop;
  return {
    attitude: { left: 0, right: 0, top: 0, bottom: 0, scale: true },
    heading: { top: RC_EDGE_PX, width: HEADING_TAPE_W_PX, height: HEADING_TAPE_H_PX },
    corner: {
      top: RC_EDGE_PX, right: RC_EDGE_PX, status: true, mode: false, waypoint: 'full', numbers: false,
    },
    speed: { ...speedBox, width: RC_TAPE_W_PX },
    altitude: altHeight >= PICTURE_TAPE_MIN_PX
      ? { top: altTop, height: altHeight, right: RC_EDGE_PX, width: RC_TAPE_W_PX }
      : null,
    numbers: null,
    strip: { bottom: RC_STRIP_BOTTOM_PX, maxMessages: STRIP_MAX_MESSAGES },
  };
}

/**
 * The vehicle window's Synthetic picture: full when maximized (`smallMap`
 * given), compact at normal size.
 *
 * @param {object} p
 * @param {number} p.width - the picture's size.
 * @param {number} p.height
 * @param {number} p.tagHeight - the tag's measured height.
 * @param {{width: number, height: number}} p.corner - the top-right column's measured size.
 * @param {number} p.creditsPx - the picture's credit line height.
 * @param {{width: number, height: number}|null} p.smallMap - maximized: the small map,
 *   PANEL_MARGIN_PX in from the picture's top and right edges.
 * @param {boolean} p.panelShown - the right panel is open.
 * @returns {{ hud: object, tagMaxWidth: number|null }} - tagMaxWidth null until the picture is measured.
 */
export function pictureHudLayout(p) {
  return p.smallMap ? fullPictureLayout(p) : compactPictureLayout(p);
}

/**
 * Maximized: the full HUD. Top row: the tag at the left and the top-right
 * column (left of the small map; below it, across the picture, where that
 * room is under TAG_MIN_W_PX for the tag); the heading tape below that row,
 * centred on the picture; the tapes from there to the credit
 * line, the altitude tape below the small map; the status texts above the
 * credit line. The attitude indicator keeps the picture's centre (the
 * camera's) and is inset by the small map's width on both sides, so its
 * ladder and roll arc never run under the small map (down to
 * PICTURE_ATTITUDE_MIN_W_PX).
 */
function fullPictureLayout({ width, height, tagHeight, corner, creditsPx, smallMap, panelShown }) {
  const inset = PICTURE_TAG_INSET_PX;
  const rightOffset = PANEL_MARGIN_PX + smallMap.width + PANEL_MARGIN_PX;
  const beside = Math.max(0, width - inset - rightOffset);
  const tapesBottom = height - creditsPx - inset;
  // Too narrow beside the small map for the tag and the column: the column
  // and the heading tape go below the small map, across the picture, where
  // the column fits there above the credit line.
  const belowMapTop = PANEL_MARGIN_PX + smallMap.height + HUD_GAP_PX;
  const stacked = beside - corner.width - HUD_GAP_PX < TAG_MIN_W_PX
    && belowMapTop + corner.height <= tapesBottom;
  const tagMaxWidth = width > 0 ? Math.max(0, stacked ? beside : beside - corner.width - HUD_GAP_PX) : null;
  const cornerTop = stacked ? belowMapTop : inset;
  const cornerRight = stacked ? inset : rightOffset;

  const rowTop = (stacked
    ? Math.max(inset + tagHeight, cornerTop + corner.height)
    : inset + Math.max(tagHeight, corner.height)) + HUD_GAP_PX;
  // The heading tape is centred on the picture's centre, the camera's axis
  // (the owner, 2026-11-07: "compas yaw is not centered"), and ends before
  // the small map; with less than HEADING_TAPE_MIN_W_PX there, it goes below
  // the small map at its full width.
  const centreX = width / 2;
  const besideHalf = Math.min(centreX - inset, width - rightOffset - centreX);
  const besideFits = stacked || 2 * besideHalf >= HEADING_TAPE_MIN_W_PX;
  const headingTop = besideFits ? rowTop : Math.max(rowTop, belowMapTop);
  const headingWidth = Math.max(0, Math.min(
    HEADING_TAPE_W_PX, width - 2 * inset, stacked || !besideFits ? Infinity : 2 * besideHalf,
  ));
  // Left out, never cut by the credit line, where it no longer fits above it.
  const heading = headingTop + HEADING_TAPE_H_PX <= tapesBottom
    ? { top: headingTop, height: HEADING_TAPE_H_PX, width: headingWidth, left: centreX - headingWidth / 2 }
    : null;

  const tapesTop = heading ? headingTop + HEADING_TAPE_H_PX + HUD_GAP_PX : headingTop;
  const tape = (top) => {
    const h = Math.min(HUD_TAPE_MAX_PX, tapesBottom - top);
    return h >= PICTURE_TAPE_MIN_PX ? { top, height: h, width: PICTURE_TAPE_W_PX } : null;
  };
  const speed = tape(tapesTop);
  const altitude = tape(Math.max(tapesTop, PANEL_MARGIN_PX + smallMap.height + HUD_GAP_PX));
  const centred = width - 2 * rightOffset >= PICTURE_ATTITUDE_MIN_W_PX;

  return {
    tagMaxWidth,
    hud: {
      attitude: { left: centred ? rightOffset : 0, right: centred ? rightOffset : 0, top: 0, bottom: 0, scale: true },
      heading,
      corner: {
        top: cornerTop, right: cornerRight, status: !panelShown, mode: !panelShown, waypoint: 'full', numbers: false,
      },
      speed: speed && { ...speed, left: inset },
      altitude: altitude && { ...altitude, right: inset },
      numbers: null,
      strip: panelShown ? null : { bottom: creditsPx + inset, maxMessages: STRIP_MAX_MESSAGES },
    },
  };
}

/**
 * Normal size: the flight picture only. Top row: the tag and, at the right,
 * the next waypoint ("WP n · distance"). Below: the heading strip, then speed
 * (left) and altitude (right) as plain numbers, then one line of status texts
 * above the credit line (panel hidden only). The attitude indicator fills the
 * band between those rows and the two numbers, so nothing overlaps.
 *
 * When the height runs out, parts drop in a fixed order: the status texts,
 * then the heading strip; the attitude scale needs ATTITUDE_SCALE_MIN_PX of
 * band and shows only where it has it, and a band under ATTITUDE_BAND_MIN_PX
 * gives the horizon the whole picture behind the readouts. Speed and altitude never drop: with no
 * row left for them they take the waypoint's place in the top row (the next
 * waypoint stays marked and numbered on the route).
 */
function compactPictureLayout({ width, height, tagHeight, corner, creditsPx, panelShown }) {
  const inset = PICTURE_TAG_INSET_PX;
  const tagMaxWidth = width > 0 ? Math.max(0, width - 2 * inset - corner.width - HUD_GAP_PX) : null;

  const top0 = inset + Math.max(tagHeight, corner.height) + HUD_GAP_PX;
  const bottom = height - creditsPx - inset;
  const avail = bottom - top0;
  const headingRow = HEADING_TAPE_H_PX + HUD_GAP_PX;
  const stripRow = HUD_GAP_PX + READOUT_LINE_PX;

  let strip = !panelShown;
  let headingShown = true;
  const need = () => (headingShown ? headingRow : 0) + READOUT_LINE_PX + (strip ? stripRow : 0);
  if (need() > avail) strip = false;
  if (need() > avail) headingShown = false;
  const numbersRow = need() <= avail;

  const headingWidth = Math.min(HEADING_TAPE_W_PX, Math.max(0, width - 2 * inset));
  const numbersTop = top0 + (headingShown ? headingRow : 0);
  const bandTop = numbersRow ? numbersTop + READOUT_LINE_PX + HUD_GAP_PX : top0;
  const bandBottom = Math.max(bandTop, bottom - (strip ? stripRow : 0));

  return {
    tagMaxWidth,
    hud: {
      attitude: bandBottom - bandTop >= ATTITUDE_BAND_MIN_PX
        ? {
          left: 0,
          right: 0,
          top: bandTop,
          bottom: Math.max(0, height - bandBottom),
          scale: bandBottom - bandTop >= ATTITUDE_SCALE_MIN_PX,
        }
        : { left: 0, right: 0, top: 0, bottom: 0, scale: false },
      heading: headingShown
        ? { top: top0, height: HEADING_TAPE_H_PX, width: headingWidth, left: (width - headingWidth) / 2 }
        : null,
      corner: {
        top: inset,
        right: inset,
        status: false,
        mode: false,
        waypoint: numbersRow ? 'compact' : null,
        numbers: !numbersRow,
      },
      speed: null,
      altitude: null,
      numbers: numbersRow ? { top: numbersTop, left: inset, right: inset } : null,
      strip: strip ? { bottom: creditsPx + inset, maxMessages: COMPACT_STRIP_MESSAGES } : null,
    },
  };
}
