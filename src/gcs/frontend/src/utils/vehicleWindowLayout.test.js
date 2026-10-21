// Layout contract for the vehicle window, the swapped map and the monitor map
// controls (docs/design/gcs_video_streams.md, Test plan "Layout"). jsdom does
// no layout, so this checks the numbers the components render with; the
// measured browser layout is the live-tester screenshot check.
import { describe, it, expect } from 'vitest';
import { sidebarWidth, topBarHeight, bottomBarHeight } from '../styles';
import {
  BAR_GAP_PX, BAR_PADDING_PX, CHIP_MAX_PX, DEFAULT_TOP_CONTROLS_BOTTOM_PX, DOCKED_WIDTH_PX,
  MAP_CONTROL_PX, PANEL_MARGIN_PX, PICTURE_ASPECT, PICTURE_MIN_H_PX, RESET_VIEW_PX, SMALL_MAP_MAX_PX,
  PICTURE_TAG_INSET_PX, SMALL_MAP_MIN_PX, WINDOW_BORDER_PX, vehicleWindowLayout,
} from './vehicleWindowLayout';

// The viewing bar on one line: the UAV group (the widest chip,
// Camera, Synthetic; its outline and dividers take no width), one gap,
// Maximize and the padding. Narrower, Maximize wraps first (flex-wrap).
const BAR_NEED_PX = CHIP_MAX_PX + 3 * MAP_CONTROL_PX + BAR_GAP_PX + 2 * BAR_PADDING_PX;

const DESKTOP = { width: 1920 - sidebarWidth, height: 1080 - topBarHeight - bottomBarHeight };
// A 375 px map pane (the fixed 320 px sidebar is outside this design).
const NARROW = { width: 375, height: 812 - topBarHeight - bottomBarHeight };
// One row of top controls on desktop; on the narrow pane they wrap to two.
const TOP_BOTTOM = { desktop: DEFAULT_TOP_CONTROLS_BOTTOM_PX, narrow: PANEL_MARGIN_PX + 2 * MAP_CONTROL_PX + 6 };
// Typical Cesium credit line heights.
const MAP_CREDITS = 26;
const WINDOW_CREDITS = 24;
// The window's bar above the picture (one line), plus a failure row.
const CHROME = MAP_CONTROL_PX + 2 * BAR_PADDING_PX;
const FAILURE_ROW = 2 * 24;

const rect = (x, y, w, h) => ({ x, y, w, h });
const overlap = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
const inside = (a, b) => a.x >= b.x && a.y >= b.y && a.x + a.w <= b.x + b.w && a.y + a.h <= b.y + b.h;

function windowRect(pane, layout) {
  const box = layout.window;
  const height = box.height ?? box.maxHeight;
  return rect(pane.width - box.right - box.width, box.top, box.width, height);
}

const smallMapRect = (pane, layout) => {
  const box = layout.smallMap;
  return rect(pane.width - box.right - box.width, box.top, box.width, box.height);
};

const layoutAt = (pane, topBottom, extra) => vehicleWindowLayout({
  paneWidth: pane.width, paneHeight: pane.height, topControlsBottom: topBottom,
  mapCreditsPx: MAP_CREDITS, ...extra,
});

describe.each([
  ['desktop', DESKTOP, TOP_BOTTOM.desktop],
  ['a 375 px map pane', NARROW, TOP_BOTTOM.narrow],
])('vehicle window layout at %s', (_name, pane, topBottom) => {
  const topControls = rect(0, 0, pane.width, topBottom);
  const resetView = rect(pane.width - PANEL_MARGIN_PX - RESET_VIEW_PX, pane.height - PANEL_MARGIN_PX - RESET_VIEW_PX, RESET_VIEW_PX, RESET_VIEW_PX);
  const mapCredits = rect(0, pane.height - MAP_CREDITS, pane.width, MAP_CREDITS);

  {
    it('normal: top right under the bar, the bar on one line, above the reset view', () => {
      const layout = layoutAt(pane, topBottom, { windowChromePx: CHROME });
      expect(layout.window.width - 2 * WINDOW_BORDER_PX).toBeGreaterThanOrEqual(BAR_NEED_PX);
      expect(layout.pictureFits).toBe(true);

      const win = windowRect(pane, layout);
      // 12 px under the measured bar, 12 px in from the right edge.
      expect(win.y).toBe(topBottom + PANEL_MARGIN_PX);
      expect(win.x + win.w).toBe(pane.width - PANEL_MARGIN_PX);
      // 12 px above the reset-view button.
      expect(resetView.y - (win.y + win.h)).toBe(PANEL_MARGIN_PX);
      expect(overlap(win, resetView)).toBe(false);
      expect(overlap(win, topControls)).toBe(false);
      expect(overlap(win, mapCredits)).toBe(false);
      expect(layout.smallMap).toBeNull();
      expect(layout.tagMaxWidth).toBeNull();
    });

    it('swapped: the window fills below the bar; the small map sits in its picture', () => {
      const layout = layoutAt(pane, topBottom, { swapped: true, windowChromePx: CHROME });
      const win = windowRect(pane, layout);
      expect(layout.window.width).toBe(pane.width - 2 * PANEL_MARGIN_PX);
      expect(win.y).toBe(topBottom + PANEL_MARGIN_PX);
      expect(pane.height - (win.y + win.h)).toBe(PANEL_MARGIN_PX);
      expect(overlap(win, topControls)).toBe(false);

      // Top right of the picture: 12 px under the rows, 12 px in from the window's inner edge.
      const small = smallMapRect(pane, layout);
      const rows = rect(win.x, win.y, win.w, WINDOW_BORDER_PX + CHROME);
      const picture = rect(win.x + WINDOW_BORDER_PX, win.y + WINDOW_BORDER_PX + CHROME,
        win.w - 2 * WINDOW_BORDER_PX, win.h - 2 * WINDOW_BORDER_PX - CHROME);
      expect(small.y).toBe(rows.y + rows.h + PANEL_MARGIN_PX);
      expect(picture.x + picture.w - (small.x + small.w)).toBe(PANEL_MARGIN_PX);
      expect(inside(small, picture)).toBe(true);
      expect(overlap(small, rows)).toBe(false);
      // 16:9, at most min(320 px, half the window), never below the minimum.
      expect(small.h).toBe(Math.round(small.w / PICTURE_ASPECT));
      expect(small.w).toBeLessThanOrEqual(Math.min(SMALL_MAP_MAX_PX, picture.w / 2));
      expect(small.w).toBeGreaterThanOrEqual(SMALL_MAP_MIN_PX);
      // The picture's top-left tag (SYNTHETIC · not camera) wraps 12 px left of the small map.
      expect(layout.tagMaxWidth).toBeGreaterThan(0);
      expect(picture.x + PICTURE_TAG_INSET_PX + layout.tagMaxWidth + PANEL_MARGIN_PX).toBe(small.x);
    });
  }

  it('swapped with a failure row and on Synthetic: under the failure row, 12 px above the window\'s credit line', () => {
    const chrome = CHROME + FAILURE_ROW;
    const layout = layoutAt(pane, topBottom, {
      swapped: true, windowChromePx: chrome, windowCreditsPx: WINDOW_CREDITS,
    });
    const win = windowRect(pane, layout);
    const small = smallMapRect(pane, layout);
    const rows = rect(win.x, win.y, win.w, WINDOW_BORDER_PX + chrome);
    const windowCredits = rect(win.x, win.y + win.h - WINDOW_BORDER_PX - WINDOW_CREDITS, win.w, WINDOW_CREDITS);
    expect(small.y).toBe(rows.y + rows.h + PANEL_MARGIN_PX);
    expect(overlap(small, rows)).toBe(false);
    expect(windowCredits.y - (small.y + small.h)).toBeGreaterThanOrEqual(PANEL_MARGIN_PX);
    expect(overlap(small, windowCredits)).toBe(false);
    expect(overlap(small, topControls)).toBe(false);
  });
});

describe('edges of the contract', () => {
  const at = (paneWidth, paneHeight, extra) => vehicleWindowLayout({
    paneWidth, paneHeight, topControlsBottom: DEFAULT_TOP_CONTROLS_BOTTOM_PX, windowChromePx: CHROME, ...extra,
  });

  it('the normal width is 320 px on a wide pane, pane - 24 below 344; the height follows the content', () => {
    expect(DOCKED_WIDTH_PX).toBe(320);
    expect(at(1600, 900).window.width).toBe(320);
    expect(at(1600, 900).window.height).toBeNull();
    expect(at(344, 900).window.width).toBe(320);
    expect(at(343, 900).window.width).toBe(319);
    expect(at(300, 900).window.width).toBe(276);
  });

  it('the whole viewing bar fits one line from a 294 px pane; below it Maximize may wrap', () => {
    expect(BAR_NEED_PX).toBe(268);
    const inner = (pane) => at(pane, 900).window.width - 2 * WINDOW_BORDER_PX;
    expect(inner(294)).toBeGreaterThanOrEqual(BAR_NEED_PX);
    expect(inner(293)).toBeLessThan(BAR_NEED_PX);
  });

  it('chrome not yet measured: no picture at normal size; swapped it does not matter', () => {
    const base = { paneWidth: 1600, paneHeight: 900, topControlsBottom: 44 };
    expect(vehicleWindowLayout({ ...base, windowChromePx: null }).pictureFits).toBe(false);
    expect(vehicleWindowLayout({ ...base, windowChromePx: 40 }).pictureFits).toBe(true);
    expect(vehicleWindowLayout({ ...base, windowChromePx: null, swapped: true }).pictureFits).toBe(true);
  });

  it('the picture floor is 82 px, the 16:9 height of the smallest small map', () => {
    expect(PICTURE_MIN_H_PX).toBe(82);
    expect(PICTURE_MIN_H_PX).toBeGreaterThanOrEqual(SMALL_MAP_MIN_PX / PICTURE_ASPECT);
    // One action-bar row, no warning rows: a 236 px pane leaves 82 px, 235 leaves 81.
    expect(at(640, 236).pictureFits).toBe(true);
    expect(at(640, 235).pictureFits).toBe(false);
    // A warning row moves the boundary by its height.
    expect(at(640, 236, { windowChromePx: CHROME + 24 }).pictureFits).toBe(false);
    expect(at(640, 260, { windowChromePx: CHROME + 24 }).pictureFits).toBe(true);
  });

  it('no room for the picture still offers the swap when the swapped layout has room', () => {
    const short = at(640, 235);
    expect(short.pictureFits).toBe(false);
    expect(short.canSwap).toBe(true);
    const swapped = at(640, 235, { swapped: true });
    expect(swapped.smallMap).not.toBeNull();
    expect(swapped.pictureFits).toBe(true);
  });

  it('a tall map credit line moves the bottom limit up: the window stays 12 px above it', () => {
    const credits = 80;
    const layout = at(1600, 900, { mapCreditsPx: credits });
    expect(900 - (layout.window.top + layout.window.maxHeight)).toBe(credits + PANEL_MARGIN_PX);
  });

  it('a short swapped pane: the small map shrinks by height to stay above the credit line', () => {
    const short = at(1600, 300, { swapped: true, windowCreditsPx: WINDOW_CREDITS });
    const bottomLimit = 300 - PANEL_MARGIN_PX - WINDOW_BORDER_PX - WINDOW_CREDITS - PANEL_MARGIN_PX;
    expect(short.canSwap).toBe(true);
    expect(short.smallMap.width).toBeLessThan(SMALL_MAP_MAX_PX);
    expect(short.smallMap.width).toBeGreaterThanOrEqual(SMALL_MAP_MIN_PX);
    expect(short.smallMap.top + short.smallMap.height).toBeLessThanOrEqual(bottomLimit);
  });

  it('no room for a whole small map (its credit line, the ion logo, 138 px): no swap, the window stays normal', () => {
    // A 375 x 200 pane with a failure row, a 160 px pane, a 300 px wide pane.
    for (const [width, height, topBottom, chrome, credits] of [
      [375, 200, 82, CHROME + 48, 24], [1600, 160, 44, CHROME, 0], [300, 700, 82, CHROME, 0],
    ]) {
      const args = {
        paneWidth: width, paneHeight: height, topControlsBottom: topBottom,
        windowChromePx: chrome, windowCreditsPx: credits,
      };
      const tiny = vehicleWindowLayout({ ...args, swapped: true });
      expect(tiny.canSwap).toBe(false);
      expect(tiny.smallMap).toBeNull();
      expect(tiny.tagMaxWidth).toBeNull();
      expect(tiny.window).toEqual(vehicleWindowLayout({ ...args, swapped: false }).window);
    }
    expect(SMALL_MAP_MIN_PX).toBeGreaterThanOrEqual(138 + 2 * WINDOW_BORDER_PX);
  });
});
