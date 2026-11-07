// Placement of the shared first-person HUD in RC control and in the Synthetic
// picture, maximized (full) and at normal size (compact).
import { describe, it, expect } from 'vitest';
import {
  ATTITUDE_BAND_MIN_PX, ATTITUDE_SCALE_MIN_PX, HEADING_TAPE_H_PX, HEADING_TAPE_MIN_W_PX, HEADING_TAPE_W_PX, HUD_GAP_PX, PICTURE_ATTITUDE_MIN_W_PX,
  PICTURE_TAPE_MIN_PX, PICTURE_TAPE_W_PX, RC_TAPE_W_PX, READOUT_LINE_PX, STRIP_MAX_MESSAGES, TAG_MIN_W_PX,
  pictureHudLayout, rcHudLayout,
} from './hudLayout';
import { HUD_TAPE_MAX_PX } from './appLayout';
import { PANEL_MARGIN_PX, PICTURE_MIN_H_PX, PICTURE_TAG_INSET_PX } from './vehicleWindowLayout';

const INSET = PICTURE_TAG_INSET_PX;
const CORNER = { width: 100, height: 26 };
const LINE_CORNER = { width: 90, height: READOUT_LINE_PX };
// The normal-size picture: the docked window's 318 x 179 px.
const normal = (over = {}) => pictureHudLayout({
  width: 318, height: 179, tagHeight: 24, corner: LINE_CORNER, creditsPx: 16, smallMap: null, panelShown: true, ...over,
});
const SMALL_MAP = { width: 320, height: 180 };
const maximized = (over = {}) => pictureHudLayout({
  width: 1200, height: 700, tagHeight: 24, corner: CORNER, creditsPx: 16, smallMap: SMALL_MAP, panelShown: true, ...over,
});

/**
 * The compact picture's rows, top to bottom, as [name, top, bottom]: they
 * must never overlap (the tag and the corner share the top row side by side,
 * kept apart by tagMaxWidth).
 */
function compactRows({ hud }, { height, creditsPx, tagHeight, corner }) {
  const rows = [['top row', INSET, INSET + Math.max(tagHeight, corner.height)]];
  if (hud.heading) rows.push(['heading', hud.heading.top, hud.heading.top + hud.heading.height]);
  if (hud.numbers) rows.push(['numbers', hud.numbers.top, hud.numbers.top + READOUT_LINE_PX]);
  // The horizon behind everything (no band left) is not a row.
  const behind = hud.attitude.top === 0 && hud.attitude.bottom === 0;
  if (!behind) rows.push(['attitude', hud.attitude.top, height - hud.attitude.bottom]);
  if (hud.strip) rows.push(['strip', height - hud.strip.bottom - READOUT_LINE_PX, height - hud.strip.bottom]);
  rows.push(['credits', height - creditsPx, height]);
  return rows;
}
function expectNoOverlap(rows) {
  for (let i = 1; i < rows.length; i++) {
    expect(rows[i][1], `${rows[i][0]} starts below ${rows[i - 1][0]}`).toBeGreaterThanOrEqual(rows[i - 1][2]);
  }
}

describe('rcHudLayout', () => {
  const tapeBox = { top: 100, height: 200 };
  const speedBox = { left: 12, top: 120, height: 180 };

  it("keeps RC control's placement: 12 px top row, centred 300 x 36 heading tape, 70 px tapes", () => {
    expect(rcHudLayout({ tapeBox, speedBox, corner: CORNER })).toEqual({
      attitude: { left: 0, right: 0, top: 0, bottom: 0, scale: true },
      heading: { top: 12, width: HEADING_TAPE_W_PX, height: HEADING_TAPE_H_PX },
      corner: { top: 12, right: 12, status: true, mode: false, waypoint: 'full', numbers: false },
      speed: { left: 12, top: 120, height: 180, width: RC_TAPE_W_PX },
      altitude: { top: 100, height: 200, right: 12, width: RC_TAPE_W_PX },
      numbers: null,
      strip: { bottom: 60, maxMessages: STRIP_MAX_MESSAGES },
    });
  });

  it('link, battery and status texts always (RC hides the right panel, live check 2026-11-06); never the mode (its column does)', () => {
    const hud = rcHudLayout({ tapeBox, speedBox, corner: CORNER });
    expect(hud.corner).toMatchObject({ status: true, mode: false, waypoint: 'full' });
    expect(hud.strip).not.toBeNull();
  });

  it('the altitude tape starts below the top-right column, never under it', () => {
    const tall = { width: 140, height: 120 };
    const hud = rcHudLayout({ tapeBox: { top: 54, height: 200 }, speedBox, corner: tall });
    expect(hud.altitude.top).toBe(12 + tall.height + HUD_GAP_PX);
    expect(hud.altitude.top + hud.altitude.height).toBe(54 + 200);
  });
});

describe('pictureHudLayout maximized (full)', () => {
  it('the full HUD: tapes, heading tape, attitude scale, the next waypoint with its time', () => {
    const { hud, tagMaxWidth } = maximized();
    const right = PANEL_MARGIN_PX + SMALL_MAP.width + PANEL_MARGIN_PX;
    expect(hud.corner).toMatchObject({ top: INSET, right, waypoint: 'full', numbers: false });
    expect(tagMaxWidth).toBe(1200 - INSET - right - CORNER.width - HUD_GAP_PX);
    // Centred on the picture (the camera's axis), clear of the small map.
    expect(hud.heading).toEqual({
      top: INSET + CORNER.height + HUD_GAP_PX, height: HEADING_TAPE_H_PX, width: 300, left: 600 - 150,
    });
    expect(hud.heading.left + hud.heading.width).toBeLessThanOrEqual(1200 - right);
    expect(hud.speed).toMatchObject({ left: INSET, width: PICTURE_TAPE_W_PX, height: HUD_TAPE_MAX_PX });
    expect(hud.altitude).toMatchObject({ right: INSET, top: PANEL_MARGIN_PX + SMALL_MAP.height + HUD_GAP_PX });
    expect(hud.attitude).toEqual({ left: right, right, top: 0, bottom: 0, scale: true });
    expect(hud.numbers).toBeNull();
  });

  it('panel open: no mode, link, battery or status texts; panel hidden: all of them', () => {
    expect(maximized().hud.corner).toMatchObject({ status: false, mode: false });
    expect(maximized().hud.strip).toBeNull();
    const hidden = maximized({ panelShown: false }).hud;
    expect(hidden.corner).toMatchObject({ status: true, mode: true });
    expect(hidden.strip).toEqual({ bottom: 16 + INSET, maxMessages: STRIP_MAX_MESSAGES });
  });

  it("the heading tape's centre is the picture's: narrowed to clear the small map, below it when too narrow (owner 2026-11-07)", () => {
    const right = PANEL_MARGIN_PX + SMALL_MAP.width + PANEL_MARGIN_PX;
    // The live check's 854 px maximized picture: 91 px from the centre to the small map.
    const narrowed = maximized({ width: 854 }).hud.heading;
    expect(narrowed.left + narrowed.width / 2).toBe(427);
    expect(narrowed.width).toBe(2 * (854 - right - 427));
    expect(narrowed.top).toBe(INSET + CORNER.height + HUD_GAP_PX);
    // Too narrow beside the small map: full width, below it, still centred.
    const width = 2 * right + HEADING_TAPE_MIN_W_PX - 2;
    const below = maximized({ width, corner: { width: 40, height: 16 } }).hud;
    expect(below.heading.width).toBe(HEADING_TAPE_W_PX);
    expect(below.heading.left + below.heading.width / 2).toBe(width / 2);
    expect(below.heading.top).toBeGreaterThanOrEqual(PANEL_MARGIN_PX + SMALL_MAP.height + HUD_GAP_PX);
    expect(below.altitude.top).toBeGreaterThanOrEqual(below.heading.top + HEADING_TAPE_H_PX);
  });

  it('the heading tape keeps the height its drawing needs (a shorter canvas clipped its labels, Codex r1)', () => {
    expect(maximized().hud.heading.height).toBe(HEADING_TAPE_H_PX);
    expect(HEADING_TAPE_H_PX).toBeGreaterThanOrEqual(36);
  });

  it('too little height for a readable tape: the tapes are left out, never squeezed', () => {
    const { hud } = maximized({ height: 140 });
    expect(hud.speed).toBeNull();
    expect(hud.altitude).toBeNull();
    expect(140 - 16 - INSET - (hud.heading.top + HEADING_TAPE_H_PX + HUD_GAP_PX)).toBeLessThan(PICTURE_TAPE_MIN_PX);
  });

  it('too narrow beside the small map for the tag: the column and heading tape go below the small map (live check 2026-11-06)', () => {
    // The live check's 454 px window with the next-waypoint text (198 px).
    const corner = { width: 198, height: 16 };
    const { hud, tagMaxWidth } = maximized({ width: 454, height: 394, corner, smallMap: SMALL_MAP });
    const right = PANEL_MARGIN_PX + SMALL_MAP.width + PANEL_MARGIN_PX;
    expect(tagMaxWidth).toBe(454 - INSET - right);
    const cornerTop = PANEL_MARGIN_PX + SMALL_MAP.height + HUD_GAP_PX;
    expect(hud.corner).toMatchObject({ top: cornerTop, right: INSET });
    expect(hud.heading.top).toBe(cornerTop + corner.height + HUD_GAP_PX);
    expect(hud.heading.width).toBe(300);
    // Below the column: the altitude tape and the speed tape.
    expect(hud.altitude.top).toBeGreaterThanOrEqual(hud.heading.top + HEADING_TAPE_H_PX);
    // Where the tag has its width, the column stays beside it.
    const wide = maximized({ width: INSET + right + corner.width + HUD_GAP_PX + TAG_MIN_W_PX, corner });
    expect(wide.hud.corner).toMatchObject({ top: INSET, right });
    expect(wide.tagMaxWidth).toBe(TAG_MIN_W_PX);
  });

  it('stacked in a short window: the heading tape is left out rather than cut by the credit line (Codex r3)', () => {
    const smallMap = { width: 299, height: 168 };
    const corner = { width: 198, height: 16 };
    // Room below the small map for the column, not for the heading tape too.
    const { hud } = maximized({ width: 600, height: 250, corner, creditsPx: 38, smallMap });
    const creditsTop = 250 - 38;
    expect(hud.corner.top).toBe(PANEL_MARGIN_PX + smallMap.height + HUD_GAP_PX);
    expect(hud.corner.top + corner.height).toBeLessThanOrEqual(creditsTop);
    expect(hud.heading).toBeNull();
    expect(hud.speed).toBeNull();
    expect(hud.altitude).toBeNull();
    // Codex's case: no room below the small map for the column either, so it
    // stays beside it; the picture's centre is under the small map and there
    // is no room below it, so the heading tape is left out, never off centre.
    const short = maximized({ width: 600, height: 230, corner, creditsPx: 38, smallMap }).hud;
    expect(short.corner.top).toBe(INSET);
    expect(short.heading).toBeNull();
  });

  it('in a narrow window: the attitude indicator full width rather than below its minimum', () => {
    const smallMap = { width: 175, height: 98 };
    const right = PANEL_MARGIN_PX + smallMap.width + PANEL_MARGIN_PX;
    const width = 2 * right + PICTURE_ATTITUDE_MIN_W_PX - 1;
    const at = (w) => maximized({ width: w, height: 400, smallMap }).hud.attitude;
    expect(at(width)).toMatchObject({ left: 0, right: 0 });
    expect(at(width + 1)).toMatchObject({ left: right, right });
  });
});

describe('pictureHudLayout normal size (compact)', () => {
  it('the flight picture only: heading strip, speed and altitude as numbers, the next waypoint at the top right', () => {
    const { hud, tagMaxWidth } = normal();
    expect(hud.corner).toEqual({
      top: INSET, right: INSET, status: false, mode: false, waypoint: 'compact', numbers: false,
    });
    expect(tagMaxWidth).toBe(318 - 2 * INSET - LINE_CORNER.width - HUD_GAP_PX);
    const top0 = INSET + 24 + HUD_GAP_PX;
    expect(hud.heading).toEqual({ top: top0, height: HEADING_TAPE_H_PX, width: 300, left: 9 });
    expect(hud.numbers).toEqual({ top: top0 + HEADING_TAPE_H_PX + HUD_GAP_PX, left: INSET, right: INSET });
    expect(hud.speed).toBeNull();
    expect(hud.altitude).toBeNull();
  });

  it('never shows link, battery or mode, whatever the panel; status texts on one line only with the panel hidden', () => {
    for (const panelShown of [true, false]) {
      expect(normal({ panelShown }).hud.corner).toMatchObject({ status: false, mode: false });
    }
    expect(normal().hud.strip).toBeNull();
    expect(normal({ panelShown: false }).hud.strip).toEqual({ bottom: 16 + INSET, maxMessages: 1 });
  });

  it('nothing overlaps at the normal size, with either credit line', () => {
    for (const creditsPx of [16, 38]) {
      for (const panelShown of [true, false]) {
        const p = { width: 318, height: 179, tagHeight: 24, corner: LINE_CORNER, creditsPx, smallMap: null, panelShown };
        expectNoOverlap(compactRows(pictureHudLayout(p), p));
      }
    }
  });

  it('the attitude scale only in a band tall enough to read it', () => {
    expect(normal().hud.attitude.scale).toBe(false);
    const tall = normal({ height: 400 }).hud.attitude;
    expect(400 - tall.bottom - tall.top).toBeGreaterThanOrEqual(ATTITUDE_SCALE_MIN_PX);
    expect(tall.scale).toBe(true);
  });

  it('as the height runs out: the status texts drop first, then the heading strip; speed and altitude stay', () => {
    const at = (height) => normal({ height, panelShown: false, creditsPx: 38 }).hud;
    const heights = [];
    for (let h = 179; h >= PICTURE_MIN_H_PX; h--) heights.push([h, at(h)]);
    const firstWithout = (pick) => heights.find(([, hud]) => !pick(hud))?.[0] ?? 0;
    const stripGone = firstWithout((hud) => hud.strip);
    const headingGone = firstWithout((hud) => hud.heading);
    expect(stripGone).toBeGreaterThan(0);
    expect(headingGone).toBeGreaterThan(0);
    expect(stripGone).toBeGreaterThan(headingGone);
    for (const [, hud] of heights) expect(hud.numbers != null || hud.corner.numbers).toBe(true);
  });

  it('at the smallest picture nothing overlaps: speed and altitude move into the top row', () => {
    for (const creditsPx of [16, 38]) {
      const p = {
        width: 318, height: PICTURE_MIN_H_PX, tagHeight: 24, corner: LINE_CORNER, creditsPx, smallMap: null, panelShown: false,
      };
      const layout = pictureHudLayout(p);
      expectNoOverlap(compactRows(layout, p));
      expect(layout.hud.heading).toBeNull();
      expect(layout.hud.strip).toBeNull();
      if (!layout.hud.numbers) expect(layout.hud.corner).toMatchObject({ numbers: true, waypoint: null });
    }
  });

  it('a band too short for the horizon: it fills the picture behind the readouts, never a sliver (Codex r1)', () => {
    // The smallest picture with two credit lines and a wrapped tag.
    const p = {
      width: 146, height: PICTURE_MIN_H_PX, tagHeight: 52, corner: LINE_CORNER, creditsPx: 38, smallMap: null, panelShown: false,
    };
    const { hud } = pictureHudLayout(p);
    expect(hud.attitude).toEqual({ left: 0, right: 0, top: 0, bottom: 0, scale: false });
    // Wherever the band is kept it is at least the minimum.
    for (let h = 179; h >= PICTURE_MIN_H_PX; h--) {
      const a = normal({ height: h, creditsPx: 38 }).hud.attitude;
      if (a.top !== 0) expect(h - a.bottom - a.top).toBeGreaterThanOrEqual(ATTITUDE_BAND_MIN_PX);
    }
  });

  it('a taller tag (it wrapped) moves the rows below it down', () => {
    expect(normal({ tagHeight: 40 }).hud.heading.top).toBe(INSET + 40 + HUD_GAP_PX);
  });

  it('not yet measured: no tag wrap width', () => {
    const { tagMaxWidth } = pictureHudLayout({
      width: 0, height: 0, tagHeight: 0, corner: { width: 0, height: 0 }, creditsPx: 0, smallMap: null, panelShown: true,
    });
    expect(tagMaxWidth).toBeNull();
  });
});
