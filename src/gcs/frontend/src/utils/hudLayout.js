/**
 * Where the shared first-person HUD (components/TelemetryHud.jsx) puts its
 * parts, for its two hosts (docs/design/gcs_video_streams.md, section 5
 * "Synthetic view"): RC control over the main map, and the vehicle window's
 * Synthetic picture. Pure, so both contracts are testable without a browser.
 *
 * A layout: { attitude: {left, right}, heading: {top, height, width, left?}, status: {top, right},
 * speed: {left, top, width, height}|null, altitude: {right, top, width, height}|null,
 * strip: {bottom} }. The attitude indicator fills the host's height between its side
 * insets and is centred there; `heading.left` absent centres the tape; a null tape is not drawn.
 */
import { HUD_TAPE_MAX_PX } from './appLayout';
import { PANEL_MARGIN_PX, PICTURE_TAG_INSET_PX } from './vehicleWindowLayout';

/** The heading tape's size where it has room (its 60 degree scale is drawn over this width). */
export const HEADING_TAPE_W_PX = 300;
/** Its height in both hosts: HeadingTape draws its scale labels down to about this, so a shorter canvas clips them. */
export const HEADING_TAPE_H_PX = 36;
/** The speed and altitude tapes' width over the main map. */
export const RC_TAPE_W_PX = 70;
/** RC: the HUD's top-row inset from the pane edges. */
const RC_EDGE_PX = 12;
/** RC: the status texts' bottom inset, above the sticks' centre line. */
const RC_STRIP_BOTTOM_PX = 60;

/** The tapes' width in the window picture: narrower, so the 318 px picture keeps its middle. */
export const PICTURE_TAPE_W_PX = 56;
/** Below this a tape's value box and its scale no longer both show: the tape is left out. */
export const PICTURE_TAPE_MIN_PX = 48;
/**
 * Narrowest centred attitude box beside the small map: about the normal-size
 * picture's own indicator (it is sized from min(width, height), 179 px there).
 * A narrower window draws it full width, under the small map, rather than tiny.
 */
export const PICTURE_ATTITUDE_MIN_W_PX = 180;
/** Gap between HUD rows in the picture. */
const PICTURE_GAP_PX = 4;

/**
 * RC control: the HUD as it has always been placed over the main map.
 *
 * @param {{tapeBox: {top: number, height: number}, speedBox: {left: number, top: number, height: number}}} p
 *   from utils/appLayout.js hudTapeBox and speedTapeBox.
 */
export function rcHudLayout({ tapeBox, speedBox }) {
  return {
    attitude: { left: 0, right: 0 },
    heading: { top: RC_EDGE_PX, width: HEADING_TAPE_W_PX, height: HEADING_TAPE_H_PX },
    status: { top: RC_EDGE_PX, right: RC_EDGE_PX },
    speed: { ...speedBox, width: RC_TAPE_W_PX },
    altitude: { top: tapeBox.top, height: tapeBox.height, right: RC_EDGE_PX, width: RC_TAPE_W_PX },
    strip: { bottom: RC_STRIP_BOTTOM_PX },
  };
}

/**
 * The vehicle window's Synthetic picture. Top row: the picture's tag at the
 * left and the HUD's link and battery row at the right (left of the small map
 * when maximized); the heading tape below that row, centred in the room left
 * of the small map; the tapes from there to the credit line, the altitude tape
 * below the small map; the status texts above the credit line. Maximized, the
 * attitude indicator keeps the picture's centre (the camera's) and is inset
 * by the small map's width on both sides, so its ladder and roll arc, sized
 * from its box, never run under the small map (down to PICTURE_ATTITUDE_MIN_W_PX).
 *
 * @param {object} p
 * @param {number} p.width - the picture's size.
 * @param {number} p.height
 * @param {number} p.tagHeight - the tag's measured height.
 * @param {{width: number, height: number}} p.status - the link and battery row's measured size.
 * @param {number} p.creditsPx - the picture's credit line height.
 * @param {{width: number, height: number}|null} p.smallMap - maximized: the small map,
 *   PANEL_MARGIN_PX in from the picture's top and right edges.
 * @returns {{ hud: object, tagMaxWidth: number|null }} - tagMaxWidth null until the picture is measured.
 */
export function pictureHudLayout({ width, height, tagHeight, status, creditsPx, smallMap }) {
  const inset = PICTURE_TAG_INSET_PX;
  // Right edge of the free top area: the picture's, or left of the small map.
  const rightOffset = smallMap ? PANEL_MARGIN_PX + smallMap.width + PANEL_MARGIN_PX : inset;
  const tagMaxWidth = width > 0 ? Math.max(0, width - inset - rightOffset - status.width - PICTURE_GAP_PX) : null;

  const headingTop = inset + Math.max(tagHeight, status.height) + PICTURE_GAP_PX;
  const room = Math.max(0, width - inset - rightOffset);
  const headingWidth = Math.min(HEADING_TAPE_W_PX, room);
  const heading = {
    top: headingTop, height: HEADING_TAPE_H_PX, width: headingWidth, left: inset + (room - headingWidth) / 2,
  };

  const tapesTop = headingTop + HEADING_TAPE_H_PX + PICTURE_GAP_PX;
  const tapesBottom = height - creditsPx - inset;
  const tape = (top) => {
    const h = Math.min(HUD_TAPE_MAX_PX, tapesBottom - top);
    return h >= PICTURE_TAPE_MIN_PX ? { top, height: h, width: PICTURE_TAPE_W_PX } : null;
  };
  const speed = tape(tapesTop);
  const altitude = tape(smallMap
    ? Math.max(tapesTop, PANEL_MARGIN_PX + smallMap.height + PICTURE_GAP_PX)
    : tapesTop);

  return {
    tagMaxWidth,
    hud: {
      attitude: smallMap && width - 2 * rightOffset >= PICTURE_ATTITUDE_MIN_W_PX
        ? { left: rightOffset, right: rightOffset }
        : { left: 0, right: 0 },
      heading,
      status: { top: inset, right: rightOffset },
      speed: speed && { ...speed, left: inset },
      altitude: altitude && { ...altitude, right: inset },
      strip: { bottom: creditsPx + inset },
    },
  };
}
