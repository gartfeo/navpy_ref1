// Placement of the shared first-person HUD in RC control and in the Synthetic picture.
import { describe, it, expect } from 'vitest';
import {
  HEADING_TAPE_H_PX, HEADING_TAPE_W_PX, PICTURE_ATTITUDE_MIN_W_PX, PICTURE_HEADING_H_PX, PICTURE_TAPE_MIN_PX, PICTURE_TAPE_W_PX, RC_TAPE_W_PX,
  pictureHudLayout, rcHudLayout,
} from './hudLayout';
import { HUD_TAPE_MAX_PX } from './appLayout';
import { PANEL_MARGIN_PX, PICTURE_TAG_INSET_PX } from './vehicleWindowLayout';

const INSET = PICTURE_TAG_INSET_PX;
const STATUS = { width: 100, height: 26 };
// The normal-size picture: the docked window's 318 x 179 px.
const normal = (over = {}) => pictureHudLayout({
  width: 318, height: 179, tagHeight: 24, status: STATUS, creditsPx: 16, smallMap: null, ...over,
});

describe('rcHudLayout', () => {
  it("is RC control's placement as it was: 12 px top row, centred 300 x 36 heading tape, 70 px tapes, texts 60 px up", () => {
    const tapeBox = { top: 100, height: 200 };
    const speedBox = { left: 12, top: 120, height: 180 };
    expect(rcHudLayout({ tapeBox, speedBox })).toEqual({
      attitude: { left: 0, right: 0 },
      heading: { top: 12, width: HEADING_TAPE_W_PX, height: HEADING_TAPE_H_PX },
      status: { top: 12, right: 12 },
      speed: { left: 12, top: 120, height: 180, width: RC_TAPE_W_PX },
      altitude: { top: 100, height: 200, right: 12, width: RC_TAPE_W_PX },
      strip: { bottom: 60 },
    });
  });
});

describe('pictureHudLayout', () => {
  it('normal size: tag and status share the top row, heading tape below it, tapes down to the credit line', () => {
    const { hud, tagMaxWidth } = normal();
    expect(hud.status).toEqual({ top: INSET, right: INSET });
    // The tag wraps before the status row.
    expect(tagMaxWidth).toBe(318 - INSET - INSET - STATUS.width - 4);
    const headingTop = INSET + 26 + 4;
    expect(hud.heading).toEqual({ top: headingTop, height: PICTURE_HEADING_H_PX, width: 300, left: (318 - 300) / 2 });
    const tapesTop = headingTop + PICTURE_HEADING_H_PX + 4;
    const tapeH = 179 - 16 - INSET - tapesTop;
    expect(hud.speed).toEqual({ left: INSET, top: tapesTop, height: tapeH, width: PICTURE_TAPE_W_PX });
    expect(hud.altitude).toEqual({ right: INSET, top: tapesTop, height: tapeH, width: PICTURE_TAPE_W_PX });
    expect(hud.strip).toEqual({ bottom: 16 + INSET });
    expect(hud.attitude).toEqual({ left: 0, right: 0 });
  });

  it('a taller tag (it wrapped) moves the rows below it down', () => {
    expect(normal({ tagHeight: 40 }).hud.heading.top).toBe(INSET + 40 + 4);
  });

  it('too little height for a readable tape: the tapes are left out, never squeezed', () => {
    const { hud } = normal({ creditsPx: 60 });
    expect(179 - 60 - INSET - hud.heading.top - PICTURE_HEADING_H_PX - 4).toBeLessThan(PICTURE_TAPE_MIN_PX);
    expect(hud.speed).toBeNull();
    expect(hud.altitude).toBeNull();
  });

  it('a narrow picture shortens the heading tape to its width', () => {
    expect(normal({ width: 200 }).hud.heading).toMatchObject({ width: 200 - 2 * INSET, left: INSET });
  });

  it('maximized: status, tag and heading tape stay left of the small map, the altitude tape below it', () => {
    const smallMap = { width: 320, height: 180 };
    const { hud, tagMaxWidth } = pictureHudLayout({
      width: 1200, height: 700, tagHeight: 24, status: STATUS, creditsPx: 16, smallMap,
    });
    const right = PANEL_MARGIN_PX + smallMap.width + PANEL_MARGIN_PX;
    expect(hud.status.right).toBe(right);
    expect(tagMaxWidth).toBe(1200 - INSET - right - STATUS.width - 4);
    const room = 1200 - INSET - right;
    expect(hud.heading.left + hud.heading.width).toBeLessThanOrEqual(1200 - right);
    expect(hud.heading.left).toBe(INSET + (room - 300) / 2);
    expect(hud.altitude.top).toBe(PANEL_MARGIN_PX + smallMap.height + 4);
    expect(hud.altitude.height).toBe(HUD_TAPE_MAX_PX);
    expect(hud.speed.top).toBeLessThan(hud.altitude.top);
    // The attitude indicator keeps the picture's centre and ends left of the small map.
    expect(hud.attitude).toEqual({ left: right, right });
    expect(1200 - hud.attitude.right).toBeLessThanOrEqual(1200 - PANEL_MARGIN_PX - smallMap.width - PANEL_MARGIN_PX);
  });

  it('maximized in a narrow window: the attitude indicator full width rather than below its minimum', () => {
    const smallMap = { width: 175, height: 98 };
    const right = PANEL_MARGIN_PX + smallMap.width + PANEL_MARGIN_PX;
    const width = 2 * right + PICTURE_ATTITUDE_MIN_W_PX - 1;
    const at = (w) => pictureHudLayout({
      width: w, height: 400, tagHeight: 24, status: STATUS, creditsPx: 16, smallMap,
    }).hud.attitude;
    expect(at(width)).toEqual({ left: 0, right: 0 });
    expect(at(width + 1)).toEqual({ left: right, right });
  });

  it('not yet measured: no tag wrap width, no tapes', () => {
    const { hud, tagMaxWidth } = pictureHudLayout({
      width: 0, height: 0, tagHeight: 0, status: { width: 0, height: 0 }, creditsPx: 0, smallMap: null,
    });
    expect(tagMaxWidth).toBeNull();
    expect(hud.speed).toBeNull();
    expect(hud.altitude).toBeNull();
  });
});
