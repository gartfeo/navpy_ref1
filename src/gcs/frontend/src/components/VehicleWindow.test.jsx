// The monitor view's vehicle window (docs/design/gcs_video_streams.md
// section 5, "Vehicle window", v12 and v12.1), driven through useVehicleWindow and
// vehicleWindowLayout as App wires them. The synthetic widget is stubbed here;
// SyntheticView.test.jsx covers it.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { useLayoutEffect, useState } from 'react';
import { render, fireEvent, act, within } from '@testing-library/react';
import VehicleWindow, { MAP_SWAPPED_Z_INDEX, MENU_Z_INDEX, Z_INDEX } from './VehicleWindow';
import useVehicleWindow from '../hooks/useVehicleWindow';
import {
  BAR_GAP_PX, BAR_PADDING_PX, CHIP_MAX_PX, MAP_CONTROL_PX, WINDOW_BORDER_PX, vehicleWindowLayout,
} from '../utils/vehicleWindowLayout';
import { hasVideo, vehicleWindowVisible } from '../utils/videoStreams';
import { VIDEO_FIRST_FRAME_DEADLINE_S, VIDEO_STALL_DEADLINE_S } from '../constants/video';

// Interpolating t, so the operator text can be checked for the UAV it names.
vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key, opts) => (opts ? `${key} ${Object.values(opts).join(' ')}` : key),
    i18n: { language: 'en' },
  }),
}));

const { readers, FakeReader, synthetic } = vi.hoisted(() => {
  const readers = [];
  class FakeReader {
    constructor(conf) {
      this.conf = conf;
      this.closeCount = 0;
      readers.push(this);
    }

    close() {
      this.closeCount += 1;
    }
  }
  return { readers, FakeReader, synthetic: { mounts: 0, unmounts: 0, onFail: null } };
});
vi.mock('../vendor/mediamtx/reader', () => ({ default: FakeReader }));
vi.mock('./SyntheticView', async () => {
  const { useEffect } = await import('react');
  return {
    default: function SyntheticStub({ sysId, onFail, tagMaxWidth }) {
      synthetic.onFail = onFail;
      useEffect(() => {
        synthetic.mounts += 1;
        return () => { synthetic.unmounts += 1; };
      }, []);
      return <div data-testid="synthetic-widget" data-tag-max-width={tagMaxWidth ?? 'none'}>{sysId}</div>;
    },
  };
});

const stream = (camera, id, extra = {}) => ({
  key: `${camera}:${id}`, camera_component: camera, stream_id: id, name: `overlay ${camera}/${id}`,
  encoding: 'H264', whep_url: `http://192.168.144.1${camera % 10}:8889/c${camera}s${id}/whep`, ...extra,
});
const entry = (sysId, streams = [], extra = {}) => ({
  sysId, index: sysId - 1, label: `UAV ${sysId}`, streams, linkOk: true, holdReason: null, ...extra,
});

/**
 * As App: the video icon (here a plain button) opens the window; the layout
 * decides its place. `measured` feeds the window's reported chrome back into
 * the layout as App does (null until measured); otherwise `windowChromePx` is fixed.
 */
function Harness({
  fleet, active = true, paneWidth = 1600, paneHeight = 900, windowChromePx = 40, onChromeHeight, measured = false,
}) {
  const win = useVehicleWindow({ fleet, active });
  const [reportedChrome, setReportedChrome] = useState(null);
  const reportChrome = (h) => { setReportedChrome(h); onChromeHeight?.(h); };
  const layout = vehicleWindowLayout({
    paneWidth,
    paneHeight,
    topControlsBottom: 44,
    swapped: win.swapped,
    mapCreditsPx: 0,
    windowChromePx: measured ? reportedChrome : windowChromePx,
  });
  // As App: a pane too short for a whole small map ends the swap.
  const { unswap } = win;
  useLayoutEffect(() => {
    if (win.swapped && !layout.canSwap) unswap();
  }, [win.swapped, layout.canSwap, unswap]);
  const shown = vehicleWindowVisible({ monitor: active, rcEnabled: false, fleetSize: fleet.length, open: win.open });
  return (
    <>
      <button type="button" data-testid="video-icon" aria-pressed={win.open} onClick={win.toggleOpen}>video</button>
      {shown && (
        <VehicleWindow
          fleet={fleet}
          win={win}
          layout={layout}
          poseSourceRef={{ current: null }}
          storeRef={{ current: null }}
          ionTerrain={null}
          onSyntheticCreditsHeight={() => {}}
          onChromeHeight={reportChrome}
        />
      )}
    </>
  );
}

const latest = () => readers[readers.length - 1];
const open = () => readers.filter((r) => r.closeCount === 0);
const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });
// requestVideoFrameCallback stub: each call to frame() presents one frame.
const frame = (container) => act(() => {
  const video = container.querySelector('video');
  const cb = video.__frameCallback;
  video.__frameCallback = null;
  cb?.(performance.now(), {});
});

// The UAV buttons' accessible names: the UAV and what its icon means.
const nameOf = (n, icon = 'camera') => `vehicleWindow.uav.${icon} UAV ${n}`;
const iconOf = (fleet, n) => (hasVideo(fleet.find((e) => e.sysId === n)) ? 'camera' : 'map');
const uavButton = (q, fleet, n) => q.getByRole('button', { name: nameOf(n, iconOf(fleet, n)) });
const toggle = (q) => fireEvent.click(q.getByTestId('video-icon'));
const chip = (q) => q.container.querySelector('[data-control="chip"]');
const chipName = (q) => q.getByTestId('chip-name');
const button = (q, key) => q.getByRole('button', { name: `vehicleWindow.${key}` });
const queryButton = (q, key) => q.queryByRole('button', { name: `vehicleWindow.${key}` });
const list = (q) => q.queryByRole('group', { name: 'vehicleWindow.uavList' });

/** Render and open the window on the picker. */
function renderOpen(fleet, props = {}) {
  const utils = render(<Harness fleet={fleet} {...props} />);
  toggle(utils);
  return utils;
}

/** Render, open, then pick UAV `n` in the picker. */
function renderPicked(fleet, n = 1, props = {}) {
  const utils = renderOpen(fleet, props);
  fireEvent.click(utils.container.querySelector(`[data-sysid="${n}"]`));
  return utils;
}

/** Open the UAV list from the chip and pick UAV `n`. */
function pickFromList(q, fleet, n) {
  fireEvent.click(chip(q));
  fireEvent.click(within(list(q)).getByRole('button', { name: nameOf(n, iconOf(fleet, n)) }));
}

beforeEach(() => {
  readers.length = 0;
  Object.assign(synthetic, { mounts: 0, unmounts: 0, onFail: null });
  vi.useFakeTimers();
  vi.spyOn(HTMLMediaElement.prototype, 'play').mockResolvedValue(undefined);
  HTMLVideoElement.prototype.requestVideoFrameCallback = function request(cb) {
    this.__frameCallback = cb;
    return 1;
  };
  HTMLVideoElement.prototype.cancelVideoFrameCallback = function cancel() {
    this.__frameCallback = null;
  };
});

afterEach(() => {
  vi.restoreAllMocks();
  delete HTMLVideoElement.prototype.requestVideoFrameCallback;
  delete HTMLVideoElement.prototype.cancelVideoFrameCallback;
  delete HTMLVideoElement.prototype.getVideoPlaybackQuality;
  delete globalThis.RTCRtpReceiver;
  delete globalThis.ResizeObserver;
});

const TWO = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)]), entry(3)];
const FIVE = [1, 2, 3, 4, 5].map((n) => entry(n, [stream(100, 1)]));

const region = (q) => q.getByRole('region', { name: 'vehicleWindow.region' });
const queryRegion = (q) => q.queryByRole('region', { name: 'vehicleWindow.region' });

describe('open, closed and the picker', () => {
  it('starts closed; the video icon opens it on the picker, one 32 px button per UAV; no reader', () => {
    const utils = render(<Harness fleet={TWO} />);
    expect(queryRegion(utils)).toBeNull();
    toggle(utils);
    expect(region(utils)).toBeInTheDocument();
    expect(utils.getByText('vehicleWindow.pickerPrompt')).toBeInTheDocument();
    const picker = utils.getByTestId('uav-picker');
    const buttons = within(picker).getAllByRole('button');
    expect(buttons).toHaveLength(3);
    expect(buttons[0].style.height).toBe(`${MAP_CONTROL_PX}px`);
    expect(picker.style.flexWrap).toBe('wrap');
    expect(readers).toHaveLength(0);
    // No bar in the picker.
    expect(queryButton(utils, 'maximize')).toBeNull();
    expect(chip(utils)).toBeNull();
  });

  it('closing stops the reader and keeps the UAV; opening again plays it with a fresh connection', () => {
    const utils = renderPicked(TWO, 2);
    const first = latest();
    toggle(utils);
    expect(queryRegion(utils)).toBeNull();
    expect(first.closeCount).toBe(1);
    expect(open()).toHaveLength(0);
    toggle(utils);
    expect(open()).toHaveLength(1);
    expect(latest()).not.toBe(first);
    expect(chipName(utils)).toHaveTextContent('UAV 2');
  });

  it('closing destroys the synthetic widget and swaps back; it reopens normal size on Synthetic', () => {
    const utils = renderPicked(TWO, 3);
    const normalWidth = region(utils).style.width;
    fireEvent.click(button(utils, 'maximize'));
    toggle(utils);
    expect(synthetic.unmounts).toBe(synthetic.mounts);
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    toggle(utils);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
    expect(region(utils).style.width).toBe(normalWidth);
    expect(region(utils).style.zIndex).toBe(String(Z_INDEX));
  });

  it('no window for an empty fleet or outside the active monitor view', () => {
    const empty = render(<Harness fleet={[]} />);
    toggle(empty);
    expect(queryRegion(empty)).toBeNull();
    empty.unmount();
    const inactive = render(<Harness fleet={TWO} active={false} />);
    toggle(inactive);
    expect(queryRegion(inactive)).toBeNull();
  });

  it('sits top right, 12 px under the action bar, 320 px wide, under the map controls', () => {
    const utils = renderOpen(TWO);
    const win = region(utils);
    expect(Z_INDEX).toBeLessThan(10);
    expect(win.style.zIndex).toBe(String(Z_INDEX));
    expect(win.style.top).toBe('56px');
    expect(win.style.right).toBe('12px');
    expect(win.style.width).toBe('320px');
    expect(win.style.bottom).toBe('');
    expect(win.style.left).toBe('');
  });

  it('the overlay fill and blur sit on a layer behind the content, never on the section', () => {
    const utils = renderPicked(TWO, 1);
    const win = region(utils);
    expect(win.style.backdropFilter).toBe('');
    expect(win.style.background).toBe('');
    const backdrop = utils.getByTestId('window-backdrop');
    expect(backdrop.style.backdropFilter).toBe('blur(4px)');
    expect(backdrop.style.background).toBe('rgba(22, 33, 62, 0.85)');
    // Every other child is positioned, so it paints above the layer.
    for (const child of win.children) expect(child.style.position).not.toBe('');
  });

  it('hiding the window (RC) closes its reader; RC exit brings it back on the same UAV', () => {
    const utils = renderPicked(TWO, 2);
    const first = latest();
    utils.rerender(<Harness fleet={TWO} active={false} />);
    expect(first.closeCount).toBe(1);
    utils.rerender(<Harness fleet={TWO} />);
    expect(open()).toHaveLength(1);
    expect(chipName(utils)).toHaveTextContent('UAV 2');
  });

  it('nothing is read from or written to browser storage; a v10 value left behind is ignored', () => {
    localStorage.setItem('gcs-vehicle-window', 'minimized');
    try {
      const get = vi.spyOn(Storage.prototype, 'getItem');
      const set = vi.spyOn(Storage.prototype, 'setItem');
      const utils = render(<Harness fleet={TWO} />);
      expect(queryRegion(utils)).toBeNull();
      toggle(utils);
      fireEvent.click(utils.container.querySelector('[data-sysid="1"]'));
      fireEvent.click(button(utils, 'maximize'));
      toggle(utils);
      expect(get).not.toHaveBeenCalled();
      expect(set).not.toHaveBeenCalled();
    } finally {
      localStorage.removeItem('gcs-vehicle-window');
    }
  });

  it('reports the height of the bar and the rows above the picture; the picker reports none', () => {
    globalThis.ResizeObserver = class { observe() {} disconnect() {} };
    vi.spyOn(HTMLElement.prototype, 'offsetHeight', 'get').mockReturnValue(100);
    const onChromeHeight = vi.fn();
    const utils = renderOpen(TWO, { onChromeHeight });
    expect(onChromeHeight).not.toHaveBeenCalled();
    fireEvent.click(uavButton(utils, TWO, 1));
    expect(onChromeHeight).toHaveBeenLastCalledWith(200);
  });
});

describe('UAV buttons, the chip and the list', () => {
  it('a camera icon for a UAV with streams, a map icon without; the names say which', () => {
    const utils = renderOpen(TWO);
    const one = utils.getByRole('button', { name: nameOf(1, 'camera') });
    const three = utils.getByRole('button', { name: nameOf(3, 'map') });
    expect(one.querySelector('[data-icon="camera"]')).not.toBeNull();
    expect(three.querySelector('[data-icon="map"]')).not.toBeNull();
    expect(one).toHaveTextContent('UAV 1');
    // The zone colour is a swatch; the text never sits on it.
    expect(one.querySelector('[data-swatch]').style.background).not.toBe('');
    expect(one.style.background).toBe('rgba(22, 33, 62, 0.85)');
  });

  it('the icon follows the stream announcements', () => {
    const utils = renderOpen(TWO);
    utils.rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)]), entry(3, [stream(100, 1)])]} />);
    expect(utils.getByRole('button', { name: nameOf(1, 'map') }).querySelector('[data-icon="map"]')).not.toBeNull();
    expect(utils.getByRole('button', { name: nameOf(3, 'camera') }).querySelector('[data-icon="camera"]')).not.toBeNull();
  });

  it('"link lost" in text on a UAV whose link is lost, and in its name', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false }), entry(2)];
    const utils = renderOpen(fleet);
    const one = utils.getByRole('button', { name: `vehicleWindow.uav.withLinkLost ${nameOf(1, 'camera')}` });
    expect(one).toHaveTextContent('vehicleWindow.uav.linkLost');
    expect(utils.getByRole('button', { name: nameOf(2, 'map') })).not.toHaveTextContent('linkLost');
  });

  it('a camera-icon pick opens Camera, a map-icon pick opens Synthetic, from the picker and the list', () => {
    const utils = renderPicked(TWO, 1);
    expect(latest().conf.url).toBe(stream(100, 1).whep_url);
    expect(chipName(utils)).toHaveTextContent('UAV 1');
    expect(utils.getByTestId('picture-tag')).toHaveTextContent('overlay 100/1');
    pickFromList(utils, TWO, 3);
    expect(open()).toHaveLength(0);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
    pickFromList(utils, TWO, 2);
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    expect(open()).toHaveLength(1);
    expect(chipName(utils)).toHaveTextContent('UAV 2');
    utils.unmount();

    const fromPicker = renderPicked([entry(4)], 4);
    expect(fromPicker.getAllByTestId('synthetic-widget')).toHaveLength(1);
  });

  it('a picker pick and a list pick move focus to the chip', () => {
    const utils = renderPicked(TWO, 1);
    expect(document.activeElement).toBe(chip(utils));
    fireEvent.click(chip(utils));
    const row = within(list(utils)).getByRole('button', { name: nameOf(3, 'map') });
    act(() => row.focus());
    fireEvent.click(row);
    expect(document.activeElement).toBe(chip(utils));
  });

  it('picking the selected UAV again in the list closes it and focuses the chip', () => {
    const utils = renderPicked(TWO, 1);
    fireEvent.click(chip(utils));
    const row = within(list(utils)).getByRole('button', { name: nameOf(1) });
    act(() => row.focus());
    fireEvent.click(row);
    expect(list(utils)).toBeNull();
    expect(document.activeElement).toBe(chip(utils));
    // Nothing left queued: a source switch afterwards keeps focus on its button.
    const synth = button(utils, 'source.synthetic');
    act(() => synth.focus());
    fireEvent.click(synth);
    expect(document.activeElement).toBe(synth);
  });

  it('the chip: swatch, name, link lost and a chevron, 32 px; its name names the UAV', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const utils = renderPicked(fleet, 1);
    const c = chip(utils);
    expect(c).toHaveAccessibleName('vehicleWindow.chip vehicleWindow.uav.withLinkLost UAV 1');
    expect(c).toHaveAttribute('aria-expanded', 'false');
    expect(c.style.height).toBe(`${MAP_CONTROL_PX}px`);
    // Capped at CHIP_MAX_PX, and shrinking with its group on a narrow window.
    expect(c.parentElement.style.maxWidth).toBe(`${CHIP_MAX_PX}px`);
    expect(c.style.maxWidth).toBe('100%');
    expect(c).toHaveTextContent('vehicleWindow.uav.linkLost');
  });

  it('a long name ellipsizes; the swatch, link lost and the chevron never shrink', () => {
    const long = 'Գյումրի-հյուսիս UAV 12 long name';
    const fleet = [entry(1, [stream(100, 1)], { label: long, linkOk: false })];
    const utils = renderPicked(fleet, 1);
    const name = chipName(utils);
    expect(name).toHaveTextContent(long);
    expect(name.style.textOverflow).toBe('ellipsis');
    expect(name.style.overflow).toBe('hidden');
    expect(name.style.minWidth).toBe('0px');
    const c = chip(utils);
    const fixed = [c.querySelector('[data-swatch]'), [...c.querySelectorAll('span')].find((s) => s.textContent === 'vehicleWindow.uav.linkLost'), c.querySelector('svg')];
    for (const el of fixed) expect(el.style.flexShrink).toBe('0');
    expect(c).toHaveAccessibleName(`vehicleWindow.chip vehicleWindow.uav.withLinkLost ${long}`);
  });

  it('the list: one row per UAV with what a pick opens; the selected row outlined, checked, pressed; picking it again does nothing', () => {
    const utils = renderPicked(TWO, 1);
    const reader = latest();
    fireEvent.click(chip(utils));
    expect(chip(utils)).toHaveAttribute('aria-expanded', 'true');
    const rows = within(list(utils)).getAllByRole('button');
    expect(rows).toHaveLength(3);
    expect(utils.queryByRole('menu')).toBeNull();
    const one = within(list(utils)).getByRole('button', { name: nameOf(1) });
    const three = within(list(utils)).getByRole('button', { name: nameOf(3, 'map') });
    expect(one).toHaveAttribute('aria-pressed', 'true');
    expect(one.style.outline).toContain('solid');
    expect(one.querySelector('[data-icon="check"]')).not.toBeNull();
    expect(one).toHaveTextContent('vehicleWindow.uav.opensCamera');
    expect(three).toHaveAttribute('aria-pressed', 'false');
    expect(three).toHaveTextContent('vehicleWindow.uav.opensSynthetic');
    fireEvent.click(one);
    expect(list(utils)).toBeNull();
    expect(reader.closeCount).toBe(0);
    expect(readers).toHaveLength(1);
  });

  it('opens downward above the small map; Escape closes it and returns focus to the chip; an outside tap closes it', () => {
    const utils = renderPicked(FIVE, 1);
    fireEvent.click(chip(utils));
    expect(list(utils).style.top).toBe(`calc(100% + ${BAR_GAP_PX}px)`);
    expect(Number(list(utils).style.zIndex)).toBe(MENU_Z_INDEX);
    expect(MENU_Z_INDEX).toBeGreaterThan(MAP_SWAPPED_Z_INDEX);
    fireEvent.keyDown(document, { key: 'Escape' });
    expect(list(utils)).toBeNull();
    expect(document.activeElement).toBe(chip(utils));
    fireEvent.click(chip(utils));
    fireEvent.pointerDown(document.body);
    expect(list(utils)).toBeNull();
  });

  it('swapped: the window has no stacking context, so the list can draw above the small map', () => {
    const utils = renderPicked(FIVE, 1);
    fireEvent.click(button(utils, 'maximize'));
    expect(region(utils).style.zIndex).toBe('');
    fireEvent.click(chip(utils));
    expect(list(utils)).not.toBeNull();
  });

  it('its height limit is the room from the chip down to the window\'s bottom limit, not its rendered bottom', () => {
    const SECTION_TOP = 56;
    const CHIP_BOTTOM = 100;
    vi.spyOn(Element.prototype, 'getBoundingClientRect').mockImplementation(function rectOf() {
      // A short rendered window (no picture): the limit still comes from the layout.
      return this.tagName === 'SECTION' ? { top: SECTION_TOP, bottom: 140 } : { top: 60, bottom: CHIP_BOTTOM };
    });
    const room = (layout) => SECTION_TOP + layout - WINDOW_BORDER_PX - CHIP_BOTTOM - BAR_GAP_PX;
    // Normal: the margin above reset view (900 - 56 - 56 = 788 below the window's top).
    const utils = renderPicked([entry(1), entry(2)], 1, { paneHeight: 900 });
    fireEvent.click(button(utils, 'source.camera'));
    fireEvent.click(chip(utils));
    expect(list(utils).style.maxHeight).toBe(`${room(788)}px`);
    fireEvent.keyDown(document, { key: 'Escape' });
    // Swapped: the window's bottom (900 - 56 - 12 = 832).
    fireEvent.click(button(utils, 'maximize'));
    fireEvent.click(chip(utils));
    expect(list(utils).style.maxHeight).toBe(`${room(832)}px`);
  });

  it('no room below the chip still bounds the list to one row, never unlimited', () => {
    vi.spyOn(Element.prototype, 'getBoundingClientRect').mockImplementation(function rectOf() {
      return this.tagName === 'SECTION' ? { top: 56, bottom: 96 } : { top: 860, bottom: 892 };
    });
    const utils = renderPicked(FIVE, 1);
    fireEvent.click(chip(utils));
    expect(list(utils).style.maxHeight).toBe(`${MAP_CONTROL_PX + 2 * BAR_PADDING_PX + 2 * WINDOW_BORDER_PX}px`);
  });
});

describe('the bar', () => {
  it('chip, then Camera | Synthetic and Maximize: 32 px map buttons with names and tooltips', () => {
    const utils = renderPicked(TWO, 1);
    const bar = utils.getByTestId('window-bar');
    expect(bar.style.flexWrap).toBe('wrap');
    for (const key of ['source.camera', 'source.synthetic', 'maximize']) {
      const b = button(utils, key);
      expect(b.style.width).toBe(`${MAP_CONTROL_PX}px`);
      expect(b.style.height).toBe(`${MAP_CONTROL_PX}px`);
      expect(b).toHaveAttribute('title', `vehicleWindow.${key}`);
      expect(bar.contains(b)).toBe(true);
    }
    // Close is gone: the video icon closes, the chip changes UAV.
    expect(queryButton(utils, 'close')).toBeNull();
  });

  it('the source switch: pressed, the active style, and focus stays on the pressed button', () => {
    const utils = renderPicked(TWO, 1);
    const camera = button(utils, 'source.camera');
    const synth = button(utils, 'source.synthetic');
    expect(camera).toHaveAttribute('aria-pressed', 'true');
    expect(camera.style.boxShadow).not.toBe(synth.style.boxShadow);
    act(() => synth.focus());
    fireEvent.click(synth);
    expect(button(utils, 'source.synthetic')).toBe(synth);
    expect(synth).toHaveAttribute('aria-pressed', 'true');
    expect(document.activeElement).toBe(synth);
    act(() => camera.focus());
    fireEvent.click(camera);
    expect(document.activeElement).toBe(camera);
  });

  it('one UAV group [chip | Camera | Synthetic]; Maximize apart; the group never splits', () => {
    const utils = renderPicked(TWO, 1);
    const group = utils.getByTestId('uav-group');
    expect(group.contains(chip(utils))).toBe(true);
    const sources = utils.getByRole('group', { name: 'vehicleWindow.source.groupFor UAV 1' });
    expect(group.contains(sources)).toBe(true);
    expect(within(sources).getAllByRole('button')).toHaveLength(2);
    expect(group.contains(button(utils, 'maximize'))).toBe(false);
    // The group shrinks (the chip's name ellipsizes) but never wraps inside;
    // the source buttons never shrink; Maximize wraps to its own line first.
    expect(group.style.flex).toBe('0 1 auto');
    expect(group.style.minWidth).toBe('0px');
    expect(group.style.maxWidth).toBe('100%');
    expect(group.style.flexWrap).toBe('');
    expect(sources.style.flexShrink).toBe('0');
    expect(button(utils, 'maximize').parentElement.style.flexShrink).toBe('0');
    expect(button(utils, 'maximize').parentElement.style.marginLeft).toBe('auto');
    // The group draws the outer border; the joined buttons only their dividers.
    // Drawn as an inset outline, so the group is 32 px high like Maximize.
    expect(group.style.outline).toMatch(new RegExp(`^${WINDOW_BORDER_PX}px solid`));
    expect(group.style.outlineOffset).toBe(`${-WINDOW_BORDER_PX}px`);
    expect(group.style.borderWidth).toBe('');
    expect(group.style.borderRadius).toBe('6px');
    expect(button(utils, 'source.camera').style.borderRadius).toBe('0px');
    expect(button(utils, 'maximize').style.borderRadius).toBe('6px');
  });

  it('the source group is named for the selected UAV', () => {
    const utils = renderPicked(TWO, 1);
    pickFromList(utils, TWO, 2);
    expect(utils.getByRole('group', { name: 'vehicleWindow.source.groupFor UAV 2' })).toBeInTheDocument();
  });

  it('a stream appearing on Synthetic changes only the icon and makes Camera available; nothing switches', () => {
    const fleet = [entry(1), entry(2)];
    const utils = renderPicked(fleet, 1);
    expect(button(utils, 'source.camera')).toHaveAttribute('aria-disabled', 'true');
    const next = [entry(1, [stream(100, 1)]), entry(2)];
    utils.rerender(<Harness fleet={next} />);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('1');
    expect(readers).toHaveLength(0);
    expect(button(utils, 'source.camera')).not.toHaveAttribute('aria-disabled');
    expect(button(utils, 'source.synthetic')).toHaveAttribute('aria-pressed', 'true');
    fireEvent.click(chip(utils));
    expect(within(list(utils)).getByRole('button', { name: nameOf(1, 'camera') })).toHaveAttribute('aria-pressed', 'true');
  });

  it('a narrow pane keeps every control at 32 px; the window is pane - 24 wide', () => {
    const utils = renderPicked(TWO, 1, { paneWidth: 280 });
    expect(region(utils).style.width).toBe('256px');
    expect(button(utils, 'source.synthetic').style.width).toBe(`${MAP_CONTROL_PX}px`);
    expect(chip(utils).style.height).toBe(`${MAP_CONTROL_PX}px`);
  });
});

describe('live state on the picture tag', () => {
  it('connecting names the UAV, LIVE once frames arrive, NO FRAMES N s after; the stream name on the tag', () => {
    const { container, getByText, queryByText, getByTestId } = renderPicked(TWO, 1);
    expect(getByText('vehicleWindow.connectingUav UAV 1')).toBeInTheDocument();
    expect(getByText('vehicleWindow.state.connecting')).toBeInTheDocument();
    expect(getByTestId('picture-tag')).toHaveTextContent('vehicleWindow.state.connecting · overlay 100/1');

    frame(container);
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
    expect(getByTestId('picture-tag')).toHaveTextContent('● vehicleWindow.state.live · overlay 100/1');
    expect(queryByText(/connectingUav/)).toBeNull();

    // Frames keep coming: stays LIVE.
    for (let i = 0; i < 3; i++) { advance(500); frame(container); }
    advance(500);
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();

    // Frames stop: the element raises no waiting/stalled event at all.
    // Last frame at 1.5 s; the clock ticks on whole seconds, so at 3 s the
    // picture has had no frame for 1.5 s and reads "NO FRAMES 1 s".
    advance(1000);
    expect(queryByText('vehicleWindow.state.live')).toBeNull();
    expect(getByText('vehicleWindow.noFrames 1')).toBeInTheDocument();

    // Frames again: LIVE at the next tick, the stale text goes.
    frame(container);
    advance(500);
    frame(container);
    advance(500);
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
    expect(queryByText(/noFrames/)).toBeNull();
  });

  it('stall: frames then freeze removes the picture at the stall deadline', () => {
    const { container, getByText, queryByText, queryByTestId } = renderPicked(TWO, 1);
    frame(container);
    const reader = latest();
    advance((VIDEO_STALL_DEADLINE_S - 1) * 1000);
    expect(container.querySelector('video')).not.toBeNull();
    advance(1000);
    expect(container.querySelector('video')).toBeNull();
    expect(queryByTestId('picture-tag')).toBeNull();
    expect(getByText(/vehicleWindow.videoLost UAV 1 — vehicleWindow.reason.stalled 5/)).toBeInTheDocument();
    expect(getByText('vehicleWindow.reconnect')).toBeInTheDocument();
    // Still advertised: Reconnect video only; Dismiss is for a withdrawn stream.
    expect(queryByText('vehicleWindow.dismiss')).toBeNull();
    // A failed feed holds no session.
    expect(reader.closeCount).toBe(1);
  });

  it('first-frame deadline: peer connected but no frames', () => {
    const { container, getByText } = renderPicked(TWO, 1);
    act(() => latest().conf.onTrack({ streams: [{ getTracks: () => [] }] }));
    advance((VIDEO_FIRST_FRAME_DEADLINE_S - 1) * 1000);
    expect(getByText('vehicleWindow.state.connecting')).toBeInTheDocument();
    advance(1000);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/vehicleWindow.reason.noFirstFrame 10/)).toBeInTheDocument();
  });

  it('falls back to getVideoPlaybackQuality without requestVideoFrameCallback', () => {
    delete HTMLVideoElement.prototype.requestVideoFrameCallback;
    let frames = 0;
    HTMLVideoElement.prototype.getVideoPlaybackQuality = () => ({ totalVideoFrames: frames });
    const { getByText, container } = renderPicked(TWO, 1);
    frames = 30;
    advance(1000);
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
    // Counter stops: stale, then failed.
    advance(2000);
    expect(getByText(/vehicleWindow.noFrames/)).toBeInTheDocument();
    advance(VIDEO_STALL_DEADLINE_S * 1000);
    expect(container.querySelector('video')).toBeNull();
  });

  it('codec not supported: no picture, the reason, no reader', () => {
    globalThis.RTCRtpReceiver = { getCapabilities: () => ({ codecs: [{ mimeType: 'video/H264' }] }) };
    const fleet = [entry(1, [stream(100, 1, { encoding: 'H265' })])];
    const { container, getByText } = renderPicked(fleet, 1);
    expect(readers).toHaveLength(0);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/vehicleWindow.reason.codec H265/)).toBeInTheDocument();
  });

  it('a reader error removes the picture and releases the reader; Reconnect video starts over', () => {
    const { container, getByText } = renderPicked(TWO, 1);
    const first = latest();
    act(() => first.conf.onError('peer connection closed'));
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/vehicleWindow.reason.connection/)).toBeInTheDocument();
    expect(first.closeCount).toBe(1);

    fireEvent.click(getByText('vehicleWindow.reconnect'));
    expect(readers).toHaveLength(2);
    expect(open()).toHaveLength(1);
    expect(container.querySelector('video')).not.toBeNull();
    expect(getByText('vehicleWindow.state.connecting')).toBeInTheDocument();
  });

  it('Reconnect video and Dismiss are 32 px text buttons in the plan button style', () => {
    const utils = renderPicked(TWO, 1);
    utils.rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)]), entry(3)]} />);
    for (const key of ['reconnect', 'dismiss']) {
      expect(utils.getByText(`vehicleWindow.${key}`).style.height).toBe(`${MAP_CONTROL_PX}px`);
      expect(utils.getByText(`vehicleWindow.${key}`).style.fontSize).toBe('12px');
    }
  });
});

describe('source switch', () => {
  it('Synthetic: closes the reader, mounts the synthetic view; Camera reconnects', () => {
    const utils = renderPicked(TWO, 1);
    const reader = latest();
    fireEvent.click(button(utils, 'source.synthetic'));
    expect(reader.closeCount).toBe(1);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('1');
    expect(button(utils, 'source.synthetic')).toHaveAttribute('aria-pressed', 'true');
    expect(within(utils.getByRole('group', { name: 'vehicleWindow.source.groupFor UAV 1' })).getAllByRole('button')).toHaveLength(2);

    fireEvent.click(button(utils, 'source.camera'));
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    expect(synthetic.unmounts).toBe(synthetic.mounts);
    expect(open()).toHaveLength(1);
  });

  it('Camera on a UAV without a stream: slashed, dimmed, focusable; activating it changes nothing and says why', () => {
    // UAV 3 has none, though UAVs 1 and 2 do: availability is UAV 3's own.
    const utils = renderPicked(TWO, 3);
    const camera = button(utils, 'source.camera');
    expect(camera).toHaveAttribute('aria-disabled', 'true');
    expect(camera).toHaveAttribute('title', 'vehicleWindow.source.noCameraFor UAV 3');
    expect(camera.querySelector('[data-icon="camera-off"]')).not.toBeNull();
    expect(camera.style.color).not.toBe(button(utils, 'source.synthetic').style.color);
    // A native, enabled button, so focusable. jsdom does not turn Enter or
    // Space into a click; the live check covers keyboard activation.
    expect(camera.disabled).toBe(false);
    act(() => camera.focus());
    expect(document.activeElement).toBe(camera);
    fireEvent.click(camera);
    expect(button(utils, 'source.synthetic')).toHaveAttribute('aria-pressed', 'true');
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
    expect(readers).toHaveLength(0);
    const reason = utils.getByText('vehicleWindow.source.noCameraFor UAV 3', { selector: '[role="status"]' });
    expect(camera).toHaveAttribute('aria-describedby', reason.id);
    expect(camera).toHaveAccessibleDescription('vehicleWindow.source.noCameraFor UAV 3');
    expect(document.activeElement).toBe(camera);
    // The next pick clears the reason.
    pickFromList(utils, TWO, 1);
    expect(utils.queryByText('vehicleWindow.source.noCameraFor UAV 1', { selector: '[role="status"]' })).toBeNull();
  });

  it('a synthetic failure keeps Synthetic: the widget goes, a row with Retry; resizing or a new stream never retries', () => {
    const fleet = [entry(1), entry(2, [stream(100, 1)])];
    const utils = renderPicked(fleet, 1);
    act(() => synthetic.onFail());
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    expect(synthetic.unmounts).toBe(synthetic.mounts);
    expect(utils.getByText('vehicleWindow.syntheticUnavailable')).toHaveAttribute('role', 'alert');
    expect(button(utils, 'source.synthetic')).toHaveAttribute('aria-pressed', 'true');
    expect(readers).toHaveLength(0);
    const mounts = synthetic.mounts;
    utils.rerender(<Harness fleet={fleet} paneHeight={700} />);
    utils.rerender(<Harness fleet={[entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])]} paneHeight={900} />);
    expect(synthetic.mounts).toBe(mounts);
    expect(utils.getByText('vehicleWindow.syntheticUnavailable')).toBeInTheDocument();
    expect(button(utils, 'source.synthetic')).toHaveAttribute('aria-pressed', 'true');
    // Retry remounts the widget and focuses the Synthetic button.
    const retry = button(utils, 'retry');
    expect(retry.style.height).toBe(`${MAP_CONTROL_PX}px`);
    fireEvent.click(retry);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('1');
    expect(synthetic.mounts).toBe(mounts + 1);
    expect(utils.queryByText('vehicleWindow.syntheticUnavailable')).toBeNull();
    expect(document.activeElement).toBe(button(utils, 'source.synthetic'));
  });

  it('held pose on Synthetic: the held row names the reason', () => {
    const fleet = [entry(1, [], { linkOk: false, holdReason: 'linkLost' })];
    const { getByText } = renderPicked(fleet, 1);
    expect(getByText('fpvView.held.linkLost 0')).toBeInTheDocument();
  });

  it('the held time restarts for a newly picked UAV held for the same reason', () => {
    const held = { linkOk: false, holdReason: 'linkLost' };
    const fleet = [entry(1, [], held), entry(2, [], held)];
    const utils = renderPicked(fleet, 1);
    advance(3000);
    expect(utils.getByText('fpvView.held.linkLost 3')).toBeInTheDocument();
    fireEvent.click(chip(utils));
    fireEvent.click(within(list(utils)).getByRole('button', { name: `vehicleWindow.uav.withLinkLost ${nameOf(2, 'map')}` }));
    expect(utils.getByText('fpvView.held.linkLost 0')).toBeInTheDocument();
  });
});

describe('size', () => {
  it('Maximize swaps (labelled Restore, pressed); Restore goes back; the reader stays', () => {
    const utils = renderPicked(TWO, 1);
    const reader = latest();
    const normalWidth = parseInt(region(utils).style.width, 10);
    fireEvent.click(button(utils, 'maximize'));
    expect(button(utils, 'restore')).toHaveAttribute('aria-pressed', 'true');
    expect(parseInt(region(utils).style.width, 10)).toBeGreaterThan(normalWidth);
    fireEvent.click(button(utils, 'restore'));
    expect(parseInt(region(utils).style.width, 10)).toBe(normalWidth);
    expect(reader.closeCount).toBe(0);
  });

  it('a pick while maximized keeps the swap', () => {
    const utils = renderPicked(TWO, 1);
    fireEvent.click(button(utils, 'maximize'));
    pickFromList(utils, TWO, 3);
    expect(button(utils, 'restore')).toHaveAttribute('aria-pressed', 'true');
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
  });

  it('swapped: the top-left tags of the picture wrap left of the small map; normal size: no limit', () => {
    const expected = vehicleWindowLayout({
      paneWidth: 1600, paneHeight: 900, topControlsBottom: 44, swapped: true, mapCreditsPx: 0,
    }).tagMaxWidth;
    expect(expected).toBeGreaterThan(0);
    // SYNTHETIC · not camera (a map-icon pick).
    const synth = renderPicked(TWO, 3);
    expect(synth.getByTestId('synthetic-widget').dataset.tagMaxWidth).toBe('none');
    fireEvent.click(button(synth, 'maximize'));
    expect(Number(synth.getByTestId('synthetic-widget').dataset.tagMaxWidth)).toBe(expected);
    synth.unmount();

    // The camera's tag.
    const cam = renderPicked(TWO, 1);
    frame(cam.container);
    advance(2000);
    expect(cam.getByTestId('picture-tag').style.maxWidth).toBe('');
    fireEvent.click(button(cam, 'maximize'));
    expect(cam.getByTestId('picture-tag').style.maxWidth).toBe(`${expected}px`);
  });

  it('a pane too short for a whole small map: no Maximize; a swap ends there, focus moves to the chip, and growing does not swap again', () => {
    const CHROME = 40;
    const utils = renderPicked(TWO, 1, { windowChromePx: CHROME });
    const normalWidth = region(utils).style.width;
    fireEvent.click(button(utils, 'maximize'));
    act(() => button(utils, 'restore').focus());
    // The pane shrinks while swapped.
    utils.rerender(<Harness fleet={TWO} paneHeight={200} windowChromePx={CHROME} />);
    expect(region(utils).style.width).toBe(normalWidth);
    expect(queryButton(utils, 'restore')).toBeNull();
    expect(queryButton(utils, 'maximize')).toBeNull();
    expect(document.activeElement).toBe(chip(utils));
    // It grows again: still normal size, Maximize offered again.
    utils.rerender(<Harness fleet={TWO} windowChromePx={CHROME} />);
    expect(region(utils).style.width).toBe(normalWidth);
    expect(button(utils, 'maximize')).toHaveAttribute('aria-pressed', 'false');
  });

  it('swapped with no picture: the large area states it in text', () => {
    const { getByText, getAllByText, getByRole } = renderPicked([entry(1)], 1);
    fireEvent.click(getByRole('button', { name: 'vehicleWindow.maximize' }));
    act(() => synthetic.onFail());
    expect(getAllByText('vehicleWindow.syntheticUnavailable')).toHaveLength(2);
    expect(getByText('vehicleWindow.syntheticUnavailable', { selector: 'div[style*="inset"]' })).toBeInTheDocument();
  });
});

describe('too little room for the picture', () => {
  // A 640 px pane, one action-bar row, the 40 px bar: 236 leaves 82 px, 235 leaves 81.
  const at = (paneHeight) => ({ paneWidth: 640, paneHeight, windowChromePx: 40 });

  it('camera: no reader and a row says so; Maximize still offered; room again is a fresh connection', () => {
    const utils = renderPicked(TWO, 1, at(236));
    const first = latest();
    expect(utils.container.querySelector('video')).not.toBeNull();
    utils.rerender(<Harness fleet={TWO} {...at(235)} />);
    expect(first.closeCount).toBe(1);
    expect(utils.container.querySelector('video')).toBeNull();
    expect(utils.getByText('vehicleWindow.noRoom')).toBeInTheDocument();
    expect(button(utils, 'maximize')).toBeInTheDocument();
    utils.rerender(<Harness fleet={TWO} {...at(236)} />);
    expect(utils.queryByText('vehicleWindow.noRoom')).toBeNull();
    expect(open()).toHaveLength(1);
    expect(latest()).not.toBe(first);
  });

  it('Synthetic: the widget is unmounted (WebGL released); Maximize gives the large picture', () => {
    const utils = renderPicked(TWO, 3, at(235));
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    expect(utils.getByText('vehicleWindow.noRoom')).toBeInTheDocument();
    fireEvent.click(button(utils, 'maximize'));
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
    expect(utils.queryByText('vehicleWindow.noRoom')).toBeNull();
  });

  it('the no-room row is not measured with the rows above the picture', () => {
    globalThis.ResizeObserver = class { observe() {} disconnect() {} };
    vi.spyOn(HTMLElement.prototype, 'offsetHeight', 'get').mockImplementation(function h() {
      return this.textContent === 'vehicleWindow.noRoom' ? 30 : 20;
    });
    const onChromeHeight = vi.fn();
    const utils = renderPicked(TWO, 1, { ...at(235), onChromeHeight });
    expect(utils.getByText('vehicleWindow.noRoom')).toBeInTheDocument();
    expect(onChromeHeight).toHaveBeenLastCalledWith(40);
  });
});

describe('with the chrome measured and fed back, as App', () => {
  let resizes;
  beforeEach(() => {
    resizes = [];
    globalThis.ResizeObserver = class {
      constructor(cb) { resizes.push(cb); }
      observe() {}
      disconnect() {}
    };
    // The bar is 40 px; the rows block 40 px per warning row.
    vi.spyOn(HTMLElement.prototype, 'offsetHeight', 'get').mockImplementation(function h() {
      if (this.dataset.testid === 'window-bar') return 40;
      return this.querySelectorAll('[role="alert"]').length * 40;
    });
  });
  const remeasure = () => act(() => { for (const cb of resizes) cb(); });

  it('a pane too short from the start never opens a reader', () => {
    const utils = renderPicked(TWO, 1, { paneWidth: 640, paneHeight: 235, measured: true });
    expect(readers).toHaveLength(0);
    expect(utils.getByText('vehicleWindow.noRoom')).toBeInTheDocument();
  });

  it('room for the picture: the reader opens once the bar is measured', () => {
    const utils = renderPicked(TWO, 1, { paneWidth: 640, paneHeight: 236, measured: true });
    expect(readers).toHaveLength(1);
    expect(utils.container.querySelector('video')).not.toBeNull();
  });

  it('a failure whose row leaves no picture room stays shown and does not reconnect by itself', () => {
    // 260: room with the bar alone (40), none with the failure row too (80).
    const utils = renderPicked(TWO, 1, { paneWidth: 640, paneHeight: 260, measured: true });
    expect(readers).toHaveLength(1);
    act(() => latest().conf.onError('peer connection closed'));
    remeasure();
    remeasure();
    expect(utils.getByText(/vehicleWindow.reason.connection/)).toBeInTheDocument();
    expect(readers).toHaveLength(1);
    expect(open()).toHaveLength(0);
    expect(utils.container.querySelector('video')).toBeNull();
    // Reconnect video is the operator's.
    fireEvent.click(utils.getByText('vehicleWindow.reconnect'));
    remeasure();
    expect(readers).toHaveLength(2);
  });

  it('closing the window forgets the chrome; reopening measures before a reader opens', () => {
    const onChromeHeight = vi.fn();
    const utils = renderPicked(TWO, 1, { paneWidth: 640, paneHeight: 236, measured: true, onChromeHeight });
    toggle(utils);
    expect(onChromeHeight).toHaveBeenLastCalledWith(null);
    toggle(utils);
    expect(open()).toHaveLength(1);
  });
});

describe('loss', () => {
  it('stream removal: the same UAV\'s other stream', () => {
    const fleet = [entry(1, [stream(100, 1), stream(101, 2)]), entry(2, [stream(100, 1)])];
    const utils = renderPicked(fleet, 1);
    utils.rerender(<Harness fleet={[entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])]} />);
    expect(utils.getByTestId('picture-tag')).toHaveTextContent('overlay 101/2');
    expect(latest().conf.url).toBe(stream(101, 2).whep_url);
  });

  it('else "Camera video lost · UAV n" with Reconnect video and Dismiss; Dismiss opens Synthetic for that UAV', () => {
    const { getByText, getByTestId, rerender, container } = renderPicked(TWO, 1);
    rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)]), entry(3)]} />);
    expect(getByTestId('chip-name')).toHaveTextContent('UAV 1');
    expect(getByText(/vehicleWindow.videoLost UAV 1/)).toBeInTheDocument();
    expect(getByText('vehicleWindow.reconnect')).toBeDisabled();
    expect(open()).toHaveLength(0);

    fireEvent.click(getByText('vehicleWindow.dismiss'));
    // UAV 2 still advertises: no silent switch.
    expect(container.querySelector('video')).toBeNull();
    expect(getByTestId('synthetic-widget')).toHaveTextContent('1');
  });

  it('the picked UAV leaves the fleet: not in the list, selection kept with link and video lost; it resumes on return', () => {
    const utils = renderPicked(TWO, 1);
    utils.rerender(<Harness fleet={TWO.slice(1)} />);
    fireEvent.click(chip(utils));
    expect(within(list(utils)).queryByRole('button', { name: nameOf(1) })).toBeNull();
    fireEvent.keyDown(document, { key: 'Escape' });
    expect(chipName(utils)).toHaveTextContent('UAV 1');
    expect(utils.getByText('vehicleWindow.linkLost UAV 1')).toBeInTheDocument();
    expect(utils.getByText(/vehicleWindow.videoLost UAV 1/)).toBeInTheDocument();
    utils.rerender(<Harness fleet={TWO} />);
    expect(open()).toHaveLength(1);
    expect(utils.queryByText(/vehicleWindow.videoLost/)).toBeNull();
  });

  it('"Vehicle link lost" alone while the video still plays', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const utils = renderPicked(fleet, 1);
    frame(utils.container);
    expect(utils.getByText('vehicleWindow.linkLost UAV 1')).toBeInTheDocument();
    expect(utils.queryByText(/vehicleWindow.videoLost/)).toBeNull();
    expect(utils.getByText('vehicleWindow.state.live')).toBeInTheDocument();
  });

  it('both lost: both shown, link first', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const utils = renderPicked(fleet, 1);
    frame(utils.container);
    advance(VIDEO_STALL_DEADLINE_S * 1000);
    const alerts = within(region(utils)).getAllByRole('alert');
    expect(alerts.map((a) => a.textContent.split(' ')[0])).toEqual(['vehicleWindow.linkLost', 'vehicleWindow.videoLost']);
  });
});
