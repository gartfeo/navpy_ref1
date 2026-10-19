// Layout contract for the vehicle window, the swapped map and the monitor map
// controls (docs/design/gcs_video_streams.md, Test plan "Layout"). jsdom does
// no layout, so this checks the numbers the components render with; the
// measured browser layout is the live-tester screenshot check.
import { describe, it, expect } from 'vitest';
import { sidebarWidth, topBarHeight, bottomBarHeight } from '../styles';
import {
  DEFAULT_TOP_CONTROLS_BOTTOM_PX, DOCKED_WIDTH_PX, HEADER_GAP_PX, HEADER_PADDING_PX, MAP_CONTROL_PX,
  MENU_BUTTON_PX, NARROW_PANE_PX, PANEL_MARGIN_PX, PICTURE_ASPECT, RESET_VIEW_PX, STATE_PX, TITLE_MIN_PX,
  TOUCH_TARGET_PX, vehicleWindowLayout,
} from './vehicleWindowLayout';

const DESKTOP = { width: 1920 - sidebarWidth, height: 1080 - topBarHeight - bottomBarHeight };
// A 375 px map pane (the fixed 320 px sidebar is outside this design).
const NARROW = { width: 375, height: 812 - topBarHeight - bottomBarHeight };
// One row of top controls on desktop; on the narrow pane they wrap to two.
const TOP_BOTTOM = { desktop: DEFAULT_TOP_CONTROLS_BOTTOM_PX, narrow: PANEL_MARGIN_PX + 2 * MAP_CONTROL_PX + 6 };
// Typical Cesium credit line heights.
const MAP_CREDITS = 26;
const WINDOW_CREDITS = 24;

const rect = (x, y, w, h) => ({ x, y, w, h });
const overlap = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;

function windowRect(pane, layout) {
  const box = layout.window;
  const height = box.height ?? box.maxHeight;
  return rect(PANEL_MARGIN_PX, pane.height - box.bottom - height, box.width, height);
}

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
    it(`${badgeCount} UAV(s), normal: one header line, clear of the controls and the map's credits`, () => {
      const layout = layoutAt(pane, topBottom, { badgeCount, selectedIndex: badgeCount - 1 });
      expect(headerWidth(layout)).toBeLessThanOrEqual(layout.window.width);
      // Every UAV is reachable, and the selected one is always visible.
      expect(layout.visible.length + layout.overflow.length).toBe(badgeCount);
      if (layout.visible.length > 0) expect(layout.visible).toContain(badgeCount - 1);
      expect(layout.badgeWidth).toBeGreaterThanOrEqual(TOUCH_TARGET_PX);
      expect(layout.badgeLabel).toBe(pane.width < NARROW_PANE_PX ? 'number' : 'full');
      expect(layout.narrow).toBe(pane.width < NARROW_PANE_PX);

      const win = windowRect(pane, layout);
      expect(overlap(win, resetView)).toBe(false);
      expect(overlap(win, topControls)).toBe(false);
      expect(overlap(win, mapCredits)).toBe(false);
      // 12 px above the top of the map's credit line.
      expect(layout.window.bottom).toBe(MAP_CREDITS + PANEL_MARGIN_PX);
      expect(layout.smallMap).toBeNull();
    });

    it(`${badgeCount} UAV(s), swapped: the window fills below the top controls; the map takes the normal box`, () => {
      const normal = layoutAt(pane, topBottom, { badgeCount });
      const layout = layoutAt(pane, topBottom, { badgeCount, swapped: true, selectedIndex: 0 });
      expect(headerWidth(layout)).toBeLessThanOrEqual(layout.window.width);
      const win = windowRect(pane, layout);
      expect(layout.window.width).toBe(pane.width - 2 * PANEL_MARGIN_PX);
      expect(win.y).toBe(topBottom + PANEL_MARGIN_PX);
      expect(overlap(win, topControls)).toBe(false);
      expect(layout.window.bottom).toBe(PANEL_MARGIN_PX);

      // Camera video has no credit line: the small map sits at the margin.
      expect(layout.smallMap).toEqual({
        left: PANEL_MARGIN_PX,
        bottom: PANEL_MARGIN_PX,
        width: normal.window.width,
        height: Math.round(normal.window.width / PICTURE_ASPECT),
      });
      expect(layout.smallMap.height).toBeGreaterThan(0);
    });
  }

  it('swapped on Synthetic: the small map sits 12 px above the window\'s own credit line', () => {
    const layout = layoutAt(pane, topBottom, { badgeCount: 3, swapped: true, windowCreditsPx: WINDOW_CREDITS });
    expect(layout.smallMap.bottom).toBe(PANEL_MARGIN_PX + WINDOW_CREDITS + PANEL_MARGIN_PX);
    const small = rect(PANEL_MARGIN_PX, pane.height - layout.smallMap.bottom - layout.smallMap.height, layout.smallMap.width, layout.smallMap.height);
    const windowCredits = rect(PANEL_MARGIN_PX, pane.height - PANEL_MARGIN_PX - WINDOW_CREDITS, layout.window.width, WINDOW_CREDITS);
    expect(overlap(small, windowCredits)).toBe(false);
    expect(overlap(small, topControls)).toBe(false);
  });
});

describe('narrow pane rules', () => {
  it('number badges and one menu button for the badges that do not fit', () => {
    const layout = vehicleWindowLayout({ paneWidth: 375, paneHeight: 700, topControlsBottom: 106, badgeCount: 3, selectedIndex: 2 });
    expect(layout.badgeLabel).toBe('number');
    expect(layout.overflow.length).toBeGreaterThan(0);
    expect(layout.visible).toContain(2);
  });

  it('the normal width is v8\'s docked width on a wide pane', () => {
    const layout = vehicleWindowLayout({ paneWidth: 1600, paneHeight: 900, topControlsBottom: 44, badgeCount: 3 });
    expect(layout.window.width).toBe(DOCKED_WIDTH_PX);
    expect(layout.window.height).toBeNull();
  });
});
