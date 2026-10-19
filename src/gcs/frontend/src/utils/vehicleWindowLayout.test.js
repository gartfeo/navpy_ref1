// Layout contract for the vehicle window, the swapped map and the monitor map
// controls (docs/design/gcs_video_streams.md, Test plan "Layout"). jsdom does
// no layout, so this checks the numbers the components render with; the
// measured browser layout is the live-tester screenshot check.
import { describe, it, expect } from 'vitest';
import { sidebarWidth, topBarHeight, bottomBarHeight } from '../styles';
import {
  DEFAULT_TOP_CONTROLS_BOTTOM_PX, DOCKED_WIDTH_PX, HEADER_GAP_PX, HEADER_PADDING_PX, MAP_CONTROL_PX,
  MENU_BUTTON_PX, NARROW_PANE_PX, PANEL_MARGIN_PX, PICTURE_ASPECT, RESET_VIEW_PX, SMALL_MAP_MAX_PX,
  PICTURE_TAG_INSET_PX, SMALL_MAP_MIN_PX, STATE_PX, TITLE_MIN_PX, TOUCH_TARGET_PX, WINDOW_BORDER_PX,
  vehicleWindowLayout,
} from './vehicleWindowLayout';

const DESKTOP = { width: 1920 - sidebarWidth, height: 1080 - topBarHeight - bottomBarHeight };
// A 375 px map pane (the fixed 320 px sidebar is outside this design).
const NARROW = { width: 375, height: 812 - topBarHeight - bottomBarHeight };
// One row of top controls on desktop; on the narrow pane they wrap to two.
const TOP_BOTTOM = { desktop: DEFAULT_TOP_CONTROLS_BOTTOM_PX, narrow: PANEL_MARGIN_PX + 2 * MAP_CONTROL_PX + 6 };
// Typical Cesium credit line heights.
const MAP_CREDITS = 26;
const WINDOW_CREDITS = 24;
// The window's rows above the picture: header and control row, plus a failure row.
const CHROME = 2 * TOUCH_TARGET_PX + 2 * HEADER_PADDING_PX;
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

function headerWidth(layout) {
  const badges = layout.visible.length * layout.badgeWidth + Math.max(0, layout.visible.length - 1) * HEADER_GAP_PX;
  const menu = layout.overflow.length > 0 ? HEADER_GAP_PX + MENU_BUTTON_PX : 0;
  return 2 * HEADER_PADDING_PX + badges + menu + HEADER_GAP_PX + TITLE_MIN_PX + HEADER_GAP_PX + STATE_PX;
}

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

  for (const badgeCount of [1, 3, 4]) {
    it(`${badgeCount} UAV(s), normal: top right under the bar, one header line, above the reset view`, () => {
      const layout = layoutAt(pane, topBottom, { badgeCount, selectedIndex: badgeCount - 1 });
      expect(headerWidth(layout)).toBeLessThanOrEqual(layout.window.width);
      // Every UAV is reachable, and the selected one is always visible.
      expect(layout.visible.length + layout.overflow.length).toBe(badgeCount);
      if (layout.visible.length > 0) expect(layout.visible).toContain(badgeCount - 1);
      expect(layout.badgeWidth).toBeGreaterThanOrEqual(TOUCH_TARGET_PX);
      expect(layout.badgeLabel).toBe(pane.width < NARROW_PANE_PX ? 'number' : 'full');
      expect(layout.narrow).toBe(pane.width < NARROW_PANE_PX);

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

    it(`${badgeCount} UAV(s), swapped: the window fills below the bar; the small map sits in its picture`, () => {
      const layout = layoutAt(pane, topBottom, { badgeCount, swapped: true, selectedIndex: 0, windowChromePx: CHROME });
      expect(headerWidth(layout)).toBeLessThanOrEqual(layout.window.width);
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
      badgeCount: 3, swapped: true, windowChromePx: chrome, windowCreditsPx: WINDOW_CREDITS,
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
  it('number badges and one menu button for the badges that do not fit', () => {
    const layout = vehicleWindowLayout({ paneWidth: 375, paneHeight: 700, topControlsBottom: 106, badgeCount: 5, selectedIndex: 4 });
    expect(layout.badgeLabel).toBe('number');
    expect(layout.overflow.length).toBeGreaterThan(0);
    expect(layout.visible).toContain(4);
    expect(headerWidth(layout)).toBeLessThanOrEqual(layout.window.width);
  });

  it('the normal width is the docked width on a wide pane, the height follows the content', () => {
    const layout = vehicleWindowLayout({ paneWidth: 1600, paneHeight: 900, topControlsBottom: 44, badgeCount: 3 });
    expect(layout.window.width).toBe(DOCKED_WIDTH_PX);
    expect(layout.window.height).toBeNull();
  });

  it('a tall map credit line moves the bottom limit up: the window stays 12 px above it', () => {
    const credits = 80;
    const layout = vehicleWindowLayout({ paneWidth: 1600, paneHeight: 900, topControlsBottom: 44, badgeCount: 3, mapCreditsPx: credits });
    expect(900 - (layout.window.top + layout.window.maxHeight)).toBe(credits + PANEL_MARGIN_PX);
  });

  it('a short swapped pane: the small map shrinks by height to stay above the credit line', () => {
    const short = vehicleWindowLayout({
      paneWidth: 1600, paneHeight: 360, topControlsBottom: 44, badgeCount: 3, swapped: true,
      windowChromePx: CHROME, windowCreditsPx: WINDOW_CREDITS,
    });
    const bottomLimit = 360 - PANEL_MARGIN_PX - WINDOW_BORDER_PX - WINDOW_CREDITS - PANEL_MARGIN_PX;
    expect(short.canSwap).toBe(true);
    expect(short.smallMap.width).toBeLessThan(SMALL_MAP_MAX_PX);
    expect(short.smallMap.width).toBeGreaterThanOrEqual(SMALL_MAP_MIN_PX);
    expect(short.smallMap.top + short.smallMap.height).toBeLessThanOrEqual(bottomLimit);
  });

  it('no room for a whole small map (its credit line, the ion logo, 138 px): no swap, the window stays normal', () => {
    // A 375 x 280 pane with a failure row, a 200 px pane, a 300 px wide pane.
    for (const [width, height, topBottom, chrome, credits] of [
      [375, 280, 82, 114, 24], [1600, 200, 44, CHROME, 0], [300, 700, 82, CHROME, 0],
    ]) {
      const args = {
        paneWidth: width, paneHeight: height, topControlsBottom: topBottom, badgeCount: 3,
        windowChromePx: chrome, windowCreditsPx: credits,
      };
      const tiny = vehicleWindowLayout({ ...args, swapped: true });
      expect(tiny.canSwap).toBe(false);
      expect(tiny.smallMap).toBeNull();
      expect(tiny.tagMaxWidth).toBeNull();
      expect(tiny.window).toEqual(vehicleWindowLayout({ ...args, swapped: false }).window);
    }
    expect(SMALL_MAP_MIN_PX).toBeGreaterThanOrEqual(138 + 2 * WINDOW_BORDER_PX);
    expect(Math.round(SMALL_MAP_MIN_PX / PICTURE_ASPECT)).toBeGreaterThanOrEqual(TOUCH_TARGET_PX);
  });
});
