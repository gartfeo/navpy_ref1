// The handheld layout rule and boxes (docs/design/gcs_handheld_layout.md). jsdom
// does no layout, so this checks the numbers App renders with; the measured
// browser layout is the live check.
import { describe, it, expect } from 'vitest';
import { colors, sidebarWidth, topBarHeight, bottomBarHeight } from '../styles';
import {
  COLLAPSED_LIST_MIN_PX, DOCKED_MIN_VIEWPORT_PX, OVERLAY_Z_INDEX, COLLAPSED_LIST_Z_INDEX, RC_RAISED_Z_INDEX,
  RC_LIST_MIN_WIDTH_PX, collapsedListBox, panelHostStyle, panelOpenAtStart, rcListPlacement, sidePanelMode,
} from './appLayout';
import {
  DEFAULT_TOP_CONTROLS_BOTTOM_PX, PANEL_MARGIN_PX, RESET_VIEW_PX, vehicleWindowLayout,
} from './vehicleWindowLayout';

const paneOf = (w, h) => ({ width: w, height: h - topBarHeight - bottomBarHeight });
// A missing number must fail here: overlap() is simply false on NaN.
const rect = (x, y, w, h) => {
  if (![x, y, w, h].every(Number.isFinite)) throw new Error(`not a rectangle: ${[x, y, w, h]}`);
  return { x, y, w, h };
};
const overlap = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
const MAP_CREDITS = 26;
const TWO_LINE_CREDITS = 44;
const CHROME = 40;

describe('side panel rule', () => {
  it('needs a 375 px map pane beside the 320 px panel to dock', () => {
    expect(DOCKED_MIN_VIEWPORT_PX).toBe(695);
    expect(sidePanelMode(true, 695)).toBe('docked');
    expect(sidePanelMode(true, 694)).toBe('overlay');
    expect(sidePanelMode(true, 1920)).toBe('docked');
    expect(sidePanelMode(true, 375)).toBe('overlay');
  });

  it('a closed panel is collapsed at every width', () => {
    for (const width of [375, 694, 695, 1920]) expect(sidePanelMode(false, width)).toBe('collapsed');
  });

  it('starts open only where it docks', () => {
    expect(panelOpenAtStart(695)).toBe(true);
    expect(panelOpenAtStart(694)).toBe(false);
  });
});

describe('panel host style', () => {
  const box = { left: 12, top: 56, width: 307, height: 400 };

  it('docked is the panel as it always was: in the flow, no z-index', () => {
    expect(panelHostStyle({ mode: 'docked', roundsExist: true, box })).toEqual({
      width: sidebarWidth,
      height: '100%',
      background: colors.bgLight,
      borderLeft: `1px solid ${colors.border}`,
      flexShrink: 0,
    });
  });

  it('overlay lies over the map at the same width, above the map controls', () => {
    const style = panelHostStyle({ mode: 'overlay', roundsExist: false, box });
    expect(style).toMatchObject({
      position: 'absolute', top: 0, right: 0, bottom: 0, width: sidebarWidth, maxWidth: '100%', zIndex: OVERLAY_Z_INDEX,
      background: colors.bgLight,
    });
    expect(style.display).toBeUndefined();
    expect(OVERLAY_Z_INDEX).toBeGreaterThan(10);
    expect(OVERLAY_Z_INDEX).toBeLessThan(50);
  });

  it('collapsed with a round shows the list in its box, above the map controls', () => {
    const style = panelHostStyle({ mode: 'collapsed', roundsExist: true, box });
    expect(style).toMatchObject({
      position: 'absolute', left: 12, top: 56, width: 307, maxHeight: 400, zIndex: COLLAPSED_LIST_Z_INDEX,
      background: colors.bgLight,
    });
    expect(COLLAPSED_LIST_Z_INDEX).toBeGreaterThan(10);
    expect(style.display).toBeUndefined();
  });

  // Anchored at the bottom, a new round would lift the old card and put its own
  // Approve and Deny under the finger aimed at the old ones. A row still moves the
  // rows after it, as in the docked list (Limits in the note, todo 171).
  it('collapsed, the list grows down from its top, so a row does not move the rows above it', () => {
    const style = panelHostStyle({ mode: 'collapsed', roundsExist: true, box });
    expect(style.top).toBe(56);
    expect(style.bottom).toBeUndefined();
    expect(style.height).toBeUndefined();
  });

  it('collapsed with no round takes no room, and stays mounted', () => {
    expect(panelHostStyle({ mode: 'collapsed', roundsExist: false, box })).toEqual({ display: 'none' });
  });

  it('uses no transform or filter, so a fixed modal inside still means the viewport', () => {
    for (const mode of ['docked', 'overlay', 'collapsed']) {
      const style = panelHostStyle({ mode, roundsExist: true, box });
      expect(style.transform).toBeUndefined();
      expect(style.filter).toBeUndefined();
    }
  });
});

describe('collapsed list box', () => {
  const boxAt = (pane, extra) => collapsedListBox({
    paneWidth: pane.width, paneHeight: pane.height, topControlsBottom: DEFAULT_TOP_CONTROLS_BOTTOM_PX, ...extra,
  });
  const listRect = (box) => rect(box.left, box.top, box.width, box.height);
  const resetViewOf = (pane) => rect(
    pane.width - PANEL_MARGIN_PX - RESET_VIEW_PX, pane.height - PANEL_MARGIN_PX - RESET_VIEW_PX, RESET_VIEW_PX, RESET_VIEW_PX,
  );

  it('is 307 px on a phone: clear of the 32 px column on the right', () => {
    const pane = paneOf(375, 812);
    const box = boxAt(pane);
    expect(box.width).toBe(307);
    expect(box.left).toBe(PANEL_MARGIN_PX);
    expect(overlap(listRect(box), resetViewOf(pane))).toBe(false);
    expect(pane.width - (box.left + box.width)).toBe(PANEL_MARGIN_PX + RESET_VIEW_PX + PANEL_MARGIN_PX);
  });

  it('is at most the panel width on a wide pane', () => {
    expect(boxAt(paneOf(960, 448)).width).toBe(sidebarWidth);
    expect(boxAt(paneOf(1920, 1080)).width).toBe(sidebarWidth);
  });

  it('ends level with the reset view\'s reserve, clear of a two-line credit line, and starts under the top controls', () => {
    const pane = paneOf(812, 375);
    const box = boxAt(pane);
    expect(box.top).toBe(DEFAULT_TOP_CONTROLS_BOTTOM_PX + PANEL_MARGIN_PX);
    expect(pane.height - (box.top + box.height)).toBe(TWO_LINE_CREDITS + PANEL_MARGIN_PX);
    expect(pane.height - (box.top + box.height)).toBe(PANEL_MARGIN_PX + RESET_VIEW_PX + PANEL_MARGIN_PX);
  });

  it.each([[375, 812], [812, 375], [960, 448]])('at %i x %i it never leaves the pane', (w, h) => {
    const pane = paneOf(w, h);
    const box = boxAt(pane);
    expect(box.height).toBeGreaterThan(0);
    expect(box.left + box.width).toBeLessThanOrEqual(pane.width);
    expect(overlap(listRect(box), resetViewOf(pane))).toBe(false);
  });

  describe('with the vehicle window', () => {
    const windowAt = (pane) => vehicleWindowLayout({
      paneWidth: pane.width, paneHeight: pane.height, topControlsBottom: DEFAULT_TOP_CONTROLS_BOTTOM_PX,
      mapCreditsPx: MAP_CREDITS, windowChromePx: CHROME,
    }).window;
    const windowHeight = (win) => Math.min(win.maxHeight, 2 + CHROME + (win.width - 2) / (16 / 9));

    it('on a phone the list sits under the picture, the video above the card', () => {
      const pane = paneOf(375, 812);
      const win = windowAt(pane);
      const box = boxAt(pane, { windowBox: win });
      const list = listRect(box);
      const picture = rect(pane.width - win.right - win.width, win.top, win.width, windowHeight(win));
      expect(overlap(list, picture)).toBe(false);
      expect(list.y - (picture.y + picture.h)).toBeCloseTo(PANEL_MARGIN_PX, 0);
      expect(box.height).toBeGreaterThanOrEqual(COLLAPSED_LIST_MIN_PX);
    });

    // These come with no tap: the box must not move a row under a finger on its way.
    // Through the real window layout, as App computes them, on panes with and without room.
    it.each([[375, 812], [375, 601], [375, 600], [640, 500]])(
      'at %i x %i warning rows and the credit line do not change the box',
      (w, h) => {
        const pane = paneOf(w, h);
        const windowWith = (windowChromePx, mapCreditsPx) => vehicleWindowLayout({
          paneWidth: pane.width, paneHeight: pane.height, topControlsBottom: DEFAULT_TOP_CONTROLS_BOTTOM_PX,
          windowChromePx, mapCreditsPx,
        }).window;
        const box = boxAt(pane, { windowBox: windowWith(CHROME, MAP_CREDITS) });
        for (const chrome of [null, CHROME, CHROME + 25, CHROME + 50, CHROME + 120]) {
          for (const credits of [0, MAP_CREDITS, TWO_LINE_CREDITS, 80]) {
            const extra = { windowBox: windowWith(chrome, credits), windowChromePx: chrome, mapCreditsPx: credits };
            expect(boxAt(pane, extra)).toEqual(box);
          }
        }
      },
    );

    it('every warning row stays above the list: the window grows under it, over the picture\'s bottom', () => {
      const pane = paneOf(375, 812);
      const win = windowAt(pane);
      const box = boxAt(pane, { windowBox: win });
      const barAndRows = 120 + CHROME;
      expect(win.top + 1 + barAndRows).toBeLessThan(box.top);
    });

    it('under the window the box keeps at least the minimum', () => {
      const pane = { width: 375, height: 505 };
      const box = boxAt(pane, { windowBox: windowAt(pane) });
      expect(box.top).toBe(289);
      expect(box.height).toBe(COLLAPSED_LIST_MIN_PX);
    });

    it('on a wide pane the two sit side by side, so the list keeps the full height', () => {
      const pane = paneOf(960, 600);
      const win = windowAt(pane);
      const box = boxAt(pane, { windowBox: win });
      expect(box).toEqual(boxAt(pane));
      const picture = rect(pane.width - win.right - win.width, win.top, win.width, windowHeight(win));
      expect(overlap(listRect(box), picture)).toBe(false);
    });

    it('the side-by-side boundary is 12 + list + 12 + window + 12', () => {
      const need = PANEL_MARGIN_PX + sidebarWidth + PANEL_MARGIN_PX + sidebarWidth + PANEL_MARGIN_PX;
      expect(need).toBe(676);
      const at = (width) => {
        const pane = { width, height: 700 };
        return { box: boxAt(pane, { windowBox: windowAt(pane) }), plain: boxAt(pane) };
      };
      expect(at(need).box).toEqual(at(need).plain);
      expect(at(need - 1).box.height).toBeLessThan(at(need - 1).plain.height);
    });

    it('with too little room under the picture the cards win over the picture', () => {
      const pane = paneOf(375, 480);
      const box = boxAt(pane, { windowBox: windowAt(pane) });
      expect(box).toEqual(boxAt(pane));
    });

    it('needs 160 px from the picture to the box\'s end: with less it keeps the full height, over the picture', () => {
      expect(COLLAPSED_LIST_MIN_PX).toBe(160);
      const windowBox = { top: 100, width: sidebarWidth };
      const at = (height) => boxAt({ width: 375, height }, { windowBox });
      // 56 px below (the reset view and its margins), 333 px above: the window from 100,
      // 40 px of bar, 2 of border, its 318 x 179 picture, and a margin.
      expect(at(549).top).toBe(333);
      expect(at(548)).toEqual(boxAt({ width: 375, height: 548 }));
    });

    it('with no box (closed or maximized) the list runs from the top controls to the reset view\'s reserve', () => {
      const pane = paneOf(375, 812);
      const { height } = boxAt(pane, { windowBox: null });
      expect(height).toBe(pane.height - (DEFAULT_TOP_CONTROLS_BOTTOM_PX + PANEL_MARGIN_PX) - (TWO_LINE_CREDITS + PANEL_MARGIN_PX));
    });
  });
});

describe('RC list placement', () => {
  const placeAt = (pane) => rcListPlacement({
    paneWidth: pane.width,
    paneHeight: pane.height,
    left: 100,
    right: 182,
    top: 94,
    bottom: 85,
    gap: 8,
    heldBottom: 166,
  });

  it('at UniRC sizes it stays where it is: nothing overrides the placement', () => {
    for (const pane of [paneOf(960, 448), paneOf(960, 600), paneOf(1280, 648), paneOf(800, 1128), paneOf(1920, 1080)]) {
      const placed = placeAt(pane);
      expect(placed.style).toBeNull();
      expect(placed.heldRight).toBeNull();
      expect(placed.heightPx).toBe(pane.height - 94 - 85);
    }
  });

  it('with less than 320 px between the mode column and the stick it narrows, down to 260', () => {
    expect(RC_LIST_MIN_WIDTH_PX).toBe(260);
    expect(placeAt({ width: 602, height: 500 }).style).toBeNull();
    const narrow = placeAt({ width: 601, height: 500 });
    expect(narrow.style).toEqual({ width: 319 });
    expect(narrow.heldRight).toBe(182 + 319 + 8);
    expect(placeAt({ width: 542, height: 500 }).style).toEqual({ width: 260 });
  });

  it('with less room it moves above the sticks, below the held banner, over the altitude tape', () => {
    const pane = paneOf(375, 812);
    const placed = placeAt(pane);
    expect(placed.style).toMatchObject({
      top: 174, left: 100, right: 12, width: 'auto', zIndex: RC_RAISED_Z_INDEX, maxHeight: pane.height - 174 - 182,
    });
    expect(RC_RAISED_Z_INDEX).toBeGreaterThan(10);
    expect(placed.heightPx).toBe(pane.height - 174 - 182);
    // The banner takes the width: its row ends 8 px above the list.
    expect(placed.heldRight).toBe(PANEL_MARGIN_PX);
    // Right of the mode column, on screen, above the sticks.
    const list = rect(100, 174, pane.width - 100 - 12, placed.heightPx);
    expect(list.w).toBeGreaterThanOrEqual(RC_LIST_MIN_WIDTH_PX);
    expect(overlap(list, rect(pane.width - 174, pane.height - 174, 150, 150))).toBe(false);
    expect(overlap(list, rect(24, pane.height - 174, 150, 150))).toBe(false);
  });

  it('the boundary of the raised place is 100 + 260 + 182', () => {
    expect(placeAt({ width: 542, height: 800 }).style.width).toBe(260);
    expect(placeAt({ width: 542, height: 800 }).raised).toBe(false);
    expect(placeAt({ width: 541, height: 800 }).style.width).toBe('auto');
    expect(placeAt({ width: 541, height: 800 }).raised).toBe(true);
  });

  it('on a short landscape phone the height is what remains', () => {
    const placed = placeAt(paneOf(812, 375));
    expect(placed.style).toBeNull();
    expect(placed.heightPx).toBe(100);
  });
});
