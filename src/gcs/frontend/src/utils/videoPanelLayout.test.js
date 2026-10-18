// Layout contract for the camera panel and the monitor map controls
// (docs/design/gcs_video_streams.md, Test plan "Layout"). jsdom does no layout,
// so this checks the numbers the components render with; the measured browser
// layout is the live-tester screenshot check.
import { describe, it, expect } from 'vitest';
import { sidebarWidth, topBarHeight, bottomBarHeight } from '../styles';
import {
  DEFAULT_TOP_CONTROLS_BOTTOM_PX, HEADER_GAP_PX, HEADER_PADDING_PX, MENU_BUTTON_PX,
  NARROW_PANE_PX, PANEL_MARGIN_PX, RESET_VIEW_PX, STATE_PX, TITLE_MIN_PX, TOUCH_TARGET_PX,
  videoPanelLayout,
} from './videoPanelLayout';

const DESKTOP = { width: 1920 - sidebarWidth, height: 1080 - topBarHeight - bottomBarHeight };
// The 375 x 812 touch viewport, applied to the map pane itself (see the report:
// the fixed 320 px sidebar is outside this design).
const TOUCH = { width: 375, height: 812 - topBarHeight - bottomBarHeight };
// One 44 px row of top controls on desktop; on the touch pane they wrap to two.
const TOP_BOTTOM = { desktop: DEFAULT_TOP_CONTROLS_BOTTOM_PX, touch: PANEL_MARGIN_PX + 2 * TOUCH_TARGET_PX + 6 };

const rect = (x, y, w, h) => ({ x, y, w, h });
const overlap = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;

function panelRect(pane, layout, height) {
  return rect(PANEL_MARGIN_PX, pane.height - PANEL_MARGIN_PX - height, layout.width, height);
}

function headerWidth(layout) {
  const badges = layout.visible.length * layout.badgeWidth + Math.max(0, layout.visible.length - 1) * HEADER_GAP_PX;
  const menu = layout.overflow.length > 0 ? HEADER_GAP_PX + MENU_BUTTON_PX : 0;
  return 2 * HEADER_PADDING_PX + badges + menu + HEADER_GAP_PX + TITLE_MIN_PX + HEADER_GAP_PX + STATE_PX;
}

describe.each([
  ['desktop', DESKTOP, TOP_BOTTOM.desktop],
  ['375 x 812 touch', TOUCH, TOP_BOTTOM.touch],
])('camera panel layout at %s', (_name, pane, topBottom) => {
  for (const badgeCount of [1, 3, 4]) {
    for (const expanded of [false, true]) {
      it(`${badgeCount} UAV(s) with video, ${expanded ? 'expanded' : 'docked'}: one header line, no overlap`, () => {
        const layout = videoPanelLayout({
          paneWidth: pane.width, paneHeight: pane.height, topControlsBottom: topBottom,
          badgeCount, selectedIndex: badgeCount - 1, expanded,
        });
        // Header on one line: everything fits the panel width.
        expect(headerWidth(layout)).toBeLessThanOrEqual(layout.width);
        // Every UAV is reachable, and the selected one is always visible.
        expect(layout.visible.length + layout.overflow.length).toBe(badgeCount);
        if (layout.visible.length > 0) expect(layout.visible).toContain(badgeCount - 1);
        // Targets.
        expect(layout.badgeWidth).toBeGreaterThanOrEqual(TOUCH_TARGET_PX);
        expect(layout.badgeLabel).toBe(pane.width < NARROW_PANE_PX ? 'number' : 'full');

        const panel = panelRect(pane, layout, layout.maxHeight);
        const resetView = rect(pane.width - PANEL_MARGIN_PX - RESET_VIEW_PX, pane.height - PANEL_MARGIN_PX - RESET_VIEW_PX, RESET_VIEW_PX, RESET_VIEW_PX);
        const topControls = rect(0, 0, pane.width, topBottom);
        expect(overlap(panel, resetView)).toBe(false);
        expect(overlap(panel, topControls)).toBe(false);
        expect(panel.x + panel.w).toBeLessThanOrEqual(pane.width - PANEL_MARGIN_PX);
      });
    }
  }

  it('expanded takes the pane width up to the reset-view column', () => {
    const layout = videoPanelLayout({ paneWidth: pane.width, paneHeight: pane.height, topControlsBottom: topBottom, badgeCount: 3, expanded: true });
    expect(layout.width).toBe(pane.width - 3 * PANEL_MARGIN_PX - RESET_VIEW_PX);
  });
});

describe('narrow pane rules', () => {
  it('number badges and one menu button for the badges that do not fit', () => {
    const layout = videoPanelLayout({ paneWidth: 375, paneHeight: 700, topControlsBottom: 106, badgeCount: 3, selectedIndex: 2 });
    expect(layout.badgeLabel).toBe('number');
    expect(layout.overflow.length).toBeGreaterThan(0);
    expect(layout.visible).toContain(2);
  });

});
