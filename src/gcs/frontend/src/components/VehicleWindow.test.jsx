// The monitor view's vehicle window (docs/design/gcs_video_streams.md
// section 5, "Vehicle window"), driven through useVehicleWindow and
// vehicleWindowLayout as App wires them. The synthetic widget is stubbed here;
// SyntheticView.test.jsx covers it.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { useLayoutEffect } from 'react';
import { render, fireEvent, act, within } from '@testing-library/react';
import VehicleWindow, { MAP_SWAPPED_Z_INDEX, MENU_Z_INDEX, Z_INDEX } from './VehicleWindow';
import useVehicleWindow from '../hooks/useVehicleWindow';
import { WINDOW_BORDER_PX, vehicleWindowLayout } from '../utils/vehicleWindowLayout';
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

function Harness({ fleet, active = true, paneWidth = 1600, paneHeight = 900, windowChromePx = 0, onChromeHeight }) {
  const win = useVehicleWindow({ fleet, active });
  const layout = vehicleWindowLayout({
    paneWidth,
    paneHeight,
    topControlsBottom: 44,
    badgeCount: fleet.length,
    selectedIndex: fleet.findIndex((e) => e.sysId === win.sysId),
    swapped: win.swapped,
    mapCreditsPx: 0,
    windowChromePx,
  });
  // As App: a pane too short for a whole small map ends the swap.
  const { unswap } = win;
  useLayoutEffect(() => {
    if (win.swapped && !layout.canSwap) unswap();
  }, [win.swapped, layout.canSwap, unswap]);
  if (!vehicleWindowVisible({ monitor: active, rcEnabled: false, fleetSize: fleet.length })) return null;
  return (
    <VehicleWindow
      fleet={fleet}
      win={win}
      layout={layout}
      poseSourceRef={{ current: null }}
      storeRef={{ current: null }}
      ionTerrain={null}
      onSyntheticCreditsHeight={() => {}}
      onChromeHeight={onChromeHeight}
    />
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

/** Render, then pick UAV `n` (a tap on its button). */
function renderPicked(fleet, n = 1, props = {}) {
  const utils = render(<Harness fleet={fleet} {...props} />);
  fireEvent.click(utils.container.querySelector(`[data-sysid="${n}"]`));
  return utils;
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

const region = (getByRole) => getByRole('region', { name: 'vehicleWindow.region' });

describe('window visibility, place and the picker', () => {
  it('starts on the picker, one button per UAV, also when UAVs have streams; no reader', () => {
    const { getByRole, getByText, getByTestId, queryByText } = render(<Harness fleet={TWO} />);
    expect(region(getByRole)).toBeInTheDocument();
    expect(getByText('vehicleWindow.pickerPrompt')).toBeInTheDocument();
    const picker = getByTestId('uav-picker');
    expect(within(picker).getAllByRole('button')).toHaveLength(3);
    expect(picker.style.flexWrap).toBe('wrap');
    expect(readers).toHaveLength(0);
    // Maximize and Close are absent in the picker.
    expect(queryByText('vehicleWindow.maximize')).toBeNull();
    expect(queryByText('vehicleWindow.close')).toBeNull();
  });

  it('no window for an empty fleet or outside the active monitor view', () => {
    expect(render(<Harness fleet={[]} />).container).toBeEmptyDOMElement();
    expect(render(<Harness fleet={TWO} active={false} />).container).toBeEmptyDOMElement();
  });

  it('sits top right, 12 px under the action bar, under the map controls', () => {
    const { getByRole } = render(<Harness fleet={TWO} />);
    const win = region(getByRole);
    expect(Z_INDEX).toBeLessThan(10);
    expect(win.style.zIndex).toBe(String(Z_INDEX));
    expect(win.style.top).toBe('56px');
    expect(win.style.right).toBe('12px');
    expect(win.style.bottom).toBe('');
    expect(win.style.left).toBe('');
  });

  it('hiding the window closes its reader; RC exit brings it back on the same UAV', () => {
    const { rerender, getByTestId } = renderPicked(TWO, 2);
    const first = latest();
    rerender(<Harness fleet={TWO} active={false} />);
    expect(first.closeCount).toBe(1);
    rerender(<Harness fleet={TWO} />);
    expect(open()).toHaveLength(1);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 2');
  });

  it('nothing is read from or written to browser storage; a v10 value left behind is ignored', () => {
    localStorage.setItem('gcs-vehicle-window', 'minimized');
    try {
      const get = vi.spyOn(Storage.prototype, 'getItem');
      const set = vi.spyOn(Storage.prototype, 'setItem');
      const { getByText } = render(<Harness fleet={TWO} />);
      expect(getByText('vehicleWindow.pickerPrompt')).toBeInTheDocument();
      fireEvent.click(getByText('UAV 1'));
      fireEvent.click(getByText('vehicleWindow.maximize'));
      fireEvent.click(getByText('vehicleWindow.close'));
      expect(get).not.toHaveBeenCalled();
      expect(set).not.toHaveBeenCalled();
    } finally {
      localStorage.removeItem('gcs-vehicle-window');
    }
  });

  it('reports the height of the rows above the picture; the picker reports none', () => {
    globalThis.ResizeObserver = class { observe() {} disconnect() {} };
    vi.spyOn(HTMLElement.prototype, 'offsetHeight', 'get').mockReturnValue(100);
    const onChromeHeight = vi.fn();
    const utils = render(<Harness fleet={TWO} onChromeHeight={onChromeHeight} />);
    expect(onChromeHeight).not.toHaveBeenCalled();
    fireEvent.click(uavButton(utils, TWO, 1));
    expect(onChromeHeight).toHaveBeenCalledWith(100);
  });
});

describe('UAV buttons', () => {
  it('a camera icon for a UAV with streams, a map icon without; the names say which', () => {
    const { getByRole } = render(<Harness fleet={TWO} />);
    const one = getByRole('button', { name: nameOf(1, 'camera') });
    const three = getByRole('button', { name: nameOf(3, 'map') });
    expect(one.querySelector('[data-icon="camera"]')).not.toBeNull();
    expect(three.querySelector('[data-icon="map"]')).not.toBeNull();
    expect(one).toHaveTextContent('UAV 1');
    expect(parseInt(one.style.minHeight, 10)).toBeGreaterThanOrEqual(44);
  });

  it('the icon follows the stream announcements', () => {
    const { getByRole, rerender } = render(<Harness fleet={TWO} />);
    rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)]), entry(3, [stream(100, 1)])]} />);
    expect(getByRole('button', { name: nameOf(1, 'map') }).querySelector('[data-icon="map"]')).not.toBeNull();
    expect(getByRole('button', { name: nameOf(3, 'camera') }).querySelector('[data-icon="camera"]')).not.toBeNull();
  });

  it('"link lost" in text on a UAV whose link is lost, and in its name', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false }), entry(2)];
    const { getByRole } = render(<Harness fleet={fleet} />);
    const one = getByRole('button', { name: `vehicleWindow.uav.withLinkLost ${nameOf(1, 'camera')}` });
    expect(one).toHaveTextContent('vehicleWindow.uav.linkLost');
    expect(getByRole('button', { name: nameOf(2, 'map') })).not.toHaveTextContent('linkLost');
  });

  it('a camera-icon pick opens Camera, a map-icon pick opens Synthetic, from the picker and the header', () => {
    const utils = renderPicked(TWO, 1);
    expect(latest().conf.url).toBe(stream(100, 1).whep_url);
    expect(utils.getByTestId('video-title')).toHaveTextContent('UAV 1 · overlay 100/1');
    fireEvent.click(uavButton(utils, TWO, 3));
    expect(open()).toHaveLength(0);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
    fireEvent.click(uavButton(utils, TWO, 2));
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    expect(open()).toHaveLength(1);
    expect(utils.getByTestId('video-title')).toHaveTextContent('UAV 2');

    const fromPicker = renderPicked([entry(4)], 4);
    expect(fromPicker.getByText('vehicleWindow.state.synthetic')).toBeInTheDocument();
  });

  it('selected: fill, outline, full label and pressed; picking it again does nothing', () => {
    const utils = renderPicked(TWO, 1);
    const reader = latest();
    const one = uavButton(utils, TWO, 1);
    const two = uavButton(utils, TWO, 2);
    expect(one).toHaveAttribute('aria-pressed', 'true');
    expect(one.style.outline).toContain('solid');
    expect(two).toHaveAttribute('aria-pressed', 'false');
    expect(two.style.outline).toBe('none');
    fireEvent.click(one);
    expect(reader.closeCount).toBe(0);
    expect(readers).toHaveLength(1);
  });

  it('a pick while maximized keeps the swap', () => {
    const utils = renderPicked(TWO, 1);
    fireEvent.click(utils.getByText('vehicleWindow.maximize'));
    fireEvent.click(uavButton(utils, TWO, 3));
    expect(utils.getByText('vehicleWindow.restore')).toHaveAttribute('aria-pressed', 'true');
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('3');
  });

  it('header names the view from the stream metadata, 14 px on an opaque background', () => {
    const { container } = renderPicked(TWO, 1);
    const header = container.querySelector('header');
    expect(header.style.fontSize).toBe('14px');
    expect(header.style.background).not.toContain('rgba');
  });

  it('a stream appearing on Synthetic changes only the icon', () => {
    const fleet = [entry(1), entry(2)];
    const utils = renderPicked(fleet, 1);
    const next = [entry(1, [stream(100, 1)]), entry(2)];
    utils.rerender(<Harness fleet={next} />);
    expect(utils.getByTestId('synthetic-widget')).toHaveTextContent('1');
    expect(readers).toHaveLength(0);
    expect(utils.getByRole('button', { name: nameOf(1, 'camera') })).toHaveAttribute('aria-pressed', 'true');
  });

  it('narrow pane: glyph buttons keep their names; number labels keep the full accessible name', () => {
    const utils = renderPicked(TWO, 1, { paneWidth: 375 });
    expect(utils.getByRole('button', { name: 'vehicleWindow.maximize' })).toHaveTextContent('□');
    expect(utils.getByRole('button', { name: 'vehicleWindow.close' })).toHaveTextContent('×');
    expect(uavButton(utils, TWO, 1)).toHaveTextContent(/^1$/);
  });
});

describe('Close', () => {
  it('closes the reader, shows the picker and focuses that UAV\'s button', () => {
    const utils = renderPicked(TWO, 2);
    const reader = latest();
    fireEvent.click(utils.getByText('vehicleWindow.close'));
    expect(reader.closeCount).toBe(1);
    expect(utils.getByText('vehicleWindow.pickerPrompt')).toBeInTheDocument();
    expect(document.activeElement).toBe(within(utils.getByTestId('uav-picker')).getByRole('button', { name: nameOf(2) }));
  });

  it('destroys the synthetic widget and swaps back', () => {
    const utils = renderPicked(TWO, 3);
    const normalWidth = parseInt(region(utils.getByRole).style.width, 10);
    fireEvent.click(utils.getByText('vehicleWindow.maximize'));
    fireEvent.click(utils.getByText('vehicleWindow.close'));
    expect(synthetic.unmounts).toBe(synthetic.mounts);
    expect(utils.queryByTestId('synthetic-widget')).toBeNull();
    expect(parseInt(region(utils.getByRole).style.width, 10)).toBe(normalWidth);
    expect(region(utils.getByRole).style.zIndex).toBe(String(Z_INDEX));
  });

  it('after the picked UAV left the fleet: focus goes to the first UAV button', () => {
    const utils = renderPicked(TWO, 1);
    const rest = TWO.slice(1);
    utils.rerender(<Harness fleet={rest} />);
    fireEvent.click(utils.getByText('vehicleWindow.close'));
    expect(document.activeElement).toBe(within(utils.getByTestId('uav-picker')).getAllByRole('button')[0]);
    expect(document.activeElement).toHaveAccessibleName(nameOf(2));
  });
});

describe('overflow menu', () => {
  const menuButton = (utils) => utils.getByRole('button', { name: 'vehicleWindow.moreUavsLabel' });

  it('opens downward inside the window, above the small map, and a pick closes it', () => {
    const utils = renderPicked(FIVE, 1, { paneWidth: 375 });
    fireEvent.click(menuButton(utils));
    const menu = utils.getByRole('menu');
    expect(menu.style.top).toBe('100%');
    expect(Number(menu.style.zIndex)).toBe(MENU_Z_INDEX);
    expect(MENU_Z_INDEX).toBeGreaterThan(MAP_SWAPPED_Z_INDEX);
    fireEvent.click(within(menu).getByRole('button', { name: nameOf(4) }));
    expect(utils.queryByRole('menu')).toBeNull();
    expect(uavButton(utils, FIVE, 4)).toHaveAttribute('aria-pressed', 'true');
  });

  it('Escape closes it and returns focus to its button; an outside tap closes it', () => {
    const utils = renderPicked(FIVE, 1, { paneWidth: 375 });
    fireEvent.click(menuButton(utils));
    fireEvent.keyDown(document, { key: 'Escape' });
    expect(utils.queryByRole('menu')).toBeNull();
    expect(document.activeElement).toBe(menuButton(utils));
    fireEvent.click(menuButton(utils));
    fireEvent.pointerDown(document.body);
    expect(utils.queryByRole('menu')).toBeNull();
  });

  it('swapped: the window has no stacking context, so the menu can draw above the small map', () => {
    const utils = renderPicked(FIVE, 1, { paneWidth: 375 });
    fireEvent.click(utils.getByRole('button', { name: 'vehicleWindow.maximize' }));
    expect(region(utils.getByRole).style.zIndex).toBe('');
  });

  it('its height is the room down to the bottom edge of the window, measured again when the window resizes', () => {
    const observers = [];
    globalThis.ResizeObserver = class {
      constructor(cb) { this.cb = cb; this.targets = []; observers.push(this); }
      observe(target) { this.targets.push(target); }
      disconnect() { this.targets = []; }
    };
    let windowBottom = 400;
    const BUTTON_BOTTOM = 100;
    vi.spyOn(Element.prototype, 'getBoundingClientRect').mockImplementation(function rectOf() {
      return { bottom: this.tagName === 'SECTION' ? windowBottom : BUTTON_BOTTOM };
    });
    const resizeWindow = (bottom) => act(() => {
      windowBottom = bottom;
      observers.filter((o) => o.targets.some((el) => el.tagName === 'SECTION')).forEach((o) => o.cb([]));
    });
    const utils = renderPicked(FIVE, 1, { paneWidth: 375 });
    fireEvent.click(menuButton(utils));
    expect(utils.getByRole('menu').style.maxHeight).toBe(`${400 - WINDOW_BORDER_PX - BUTTON_BOTTOM}px`);
    // A failed feed collapses the picture: the window gets shorter while the menu is open.
    resizeWindow(250);
    expect(utils.getByRole('menu').style.maxHeight).toBe(`${250 - WINDOW_BORDER_PX - BUTTON_BOTTOM}px`);
    resizeWindow(BUTTON_BOTTOM);
    expect(utils.getByRole('menu').style.maxHeight).toBe('');
  });
});

describe('live state from frame progress', () => {
  it('connecting names the UAV, LIVE once frames arrive, NO FRAMES N s after', () => {
    const { container, getByText, queryByText } = renderPicked(TWO, 1);
    expect(getByText('vehicleWindow.connectingUav UAV 1')).toBeInTheDocument();
    expect(getByText('vehicleWindow.state.connecting')).toBeInTheDocument();

    frame(container);
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
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
    expect(getByText('vehicleWindow.state.stale')).toBeInTheDocument();
    expect(getByText('vehicleWindow.noFrames 1')).toBeInTheDocument();

    // Frames again: LIVE at the next tick, the stale text goes.
    frame(container);
    advance(500);
    frame(container);
    advance(500);
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
    expect(queryByText(/noFrames/)).toBeNull();
  });

  it('stall: frames then freeze collapses to the header at the stall deadline', () => {
    const { container, getByText, queryByText } = renderPicked(TWO, 1);
    frame(container);
    const reader = latest();
    advance((VIDEO_STALL_DEADLINE_S - 1) * 1000);
    expect(container.querySelector('video')).not.toBeNull();
    advance(1000);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText('vehicleWindow.state.failed')).toBeInTheDocument();
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

  it('codec not supported: collapses to the header with the reason, no reader', () => {
    globalThis.RTCRtpReceiver = { getCapabilities: () => ({ codecs: [{ mimeType: 'video/H264' }] }) };
    const fleet = [entry(1, [stream(100, 1, { encoding: 'H265' })])];
    const { container, getByText } = renderPicked(fleet, 1);
    expect(readers).toHaveLength(0);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/vehicleWindow.reason.codec H265/)).toBeInTheDocument();
  });

  it('a reader error collapses to the header and releases the reader; Reconnect video starts over', () => {
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
});

describe('source switch', () => {
  it('Synthetic: closes the reader, mounts the synthetic view with its state; Camera reconnects', () => {
    const { getByRole, getByTestId, getByText, queryByTestId } = renderPicked(TWO, 1);
    const reader = latest();
    // The control row remounts with the body: query the group afresh each time.
    const group = () => getByRole('group', { name: 'vehicleWindow.source.group' });
    fireEvent.click(within(group()).getByText('vehicleWindow.source.synthetic'));
    expect(reader.closeCount).toBe(1);
    expect(getByTestId('synthetic-widget')).toHaveTextContent('1');
    expect(getByText('vehicleWindow.state.synthetic')).toBeInTheDocument();
    expect(getByTestId('video-title')).toHaveTextContent('vehicleWindow.syntheticTitle UAV 1');
    expect(within(group()).getByText('vehicleWindow.source.synthetic')).toHaveAttribute('aria-pressed', 'true');

    fireEvent.click(within(group()).getByText('vehicleWindow.source.camera'));
    expect(queryByTestId('synthetic-widget')).toBeNull();
    expect(synthetic.unmounts).toBe(synthetic.mounts);
    expect(open()).toHaveLength(1);
  });

  it('Camera on a map-icon UAV: "No camera · UAV n", no reader, no Dismiss', () => {
    const { getByText, queryByText } = renderPicked(TWO, 3);
    fireEvent.click(getByText('vehicleWindow.source.camera'));
    expect(getByText('vehicleWindow.noCamera UAV 3')).toBeInTheDocument();
    expect(getByText('vehicleWindow.state.noCamera')).toBeInTheDocument();
    expect(queryByText('vehicleWindow.dismiss')).toBeNull();
    expect(readers).toHaveLength(0);
  });

  it('a synthetic failure falls back to Camera with a notice', () => {
    const { getByText, queryByTestId } = renderPicked(TWO, 3);
    act(() => synthetic.onFail());
    expect(queryByTestId('synthetic-widget')).toBeNull();
    expect(getByText('vehicleWindow.syntheticUnavailable')).toHaveAttribute('role', 'alert');
    expect(getByText('vehicleWindow.source.camera')).toHaveAttribute('aria-pressed', 'true');
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
    fireEvent.click(utils.getByRole('button', { name: `vehicleWindow.uav.withLinkLost ${nameOf(2, 'map')}` }));
    expect(utils.getByText('fpvView.held.linkLost 0')).toBeInTheDocument();
  });
});

describe('size', () => {
  it('Maximize swaps (labelled Restore, pressed); Restore goes back; the reader stays', () => {
    const { getByText, getByRole } = renderPicked(TWO, 1);
    const reader = latest();
    const normalWidth = parseInt(region(getByRole).style.width, 10);
    fireEvent.click(getByText('vehicleWindow.maximize'));
    expect(getByText('vehicleWindow.restore')).toHaveAttribute('aria-pressed', 'true');
    expect(parseInt(region(getByRole).style.width, 10)).toBeGreaterThan(normalWidth);
    fireEvent.click(getByText('vehicleWindow.restore'));
    expect(parseInt(region(getByRole).style.width, 10)).toBe(normalWidth);
    expect(reader.closeCount).toBe(0);
  });

  it('swapped: the top-left tags of the picture wrap left of the small map; normal size: no limit', () => {
    const expected = (selectedIndex) => vehicleWindowLayout({
      paneWidth: 1600, paneHeight: 900, topControlsBottom: 44, badgeCount: TWO.length, selectedIndex,
      swapped: true, mapCreditsPx: 0,
    }).tagMaxWidth;
    // SYNTHETIC · not camera (a map-icon pick).
    const synth = renderPicked(TWO, 3);
    expect(synth.getByTestId('synthetic-widget').dataset.tagMaxWidth).toBe('none');
    fireEvent.click(synth.getByText('vehicleWindow.maximize'));
    expect(Number(synth.getByTestId('synthetic-widget').dataset.tagMaxWidth)).toBe(expected(2));
    expect(expected(2)).toBeGreaterThan(0);
    synth.unmount();

    // The camera's NO FRAMES tag.
    const cam = renderPicked(TWO, 1);
    frame(cam.container);
    advance(2000);
    expect(cam.getByText(/vehicleWindow\.noFrames/).style.maxWidth).toBe('');
    fireEvent.click(cam.getByText('vehicleWindow.maximize'));
    expect(cam.getByText(/vehicleWindow\.noFrames/).style.maxWidth).toBe(`${expected(0)}px`);
  });

  it('a pane too short for a whole small map: no Maximize; a swap ends there, focus moves to Close, and growing does not swap again', () => {
    const CHROME = 100;
    const utils = renderPicked(TWO, 1, { windowChromePx: CHROME });
    const normalWidth = region(utils.getByRole).style.width;
    fireEvent.click(utils.getByText('vehicleWindow.maximize'));
    act(() => utils.getByText('vehicleWindow.restore').focus());
    // The pane shrinks while swapped.
    utils.rerender(<Harness fleet={TWO} paneHeight={250} windowChromePx={CHROME} />);
    expect(region(utils.getByRole).style.width).toBe(normalWidth);
    expect(utils.queryByText('vehicleWindow.restore')).toBeNull();
    expect(utils.queryByText('vehicleWindow.maximize')).toBeNull();
    expect(document.activeElement).toBe(utils.getByText('vehicleWindow.close'));
    // It grows again: still normal size, Maximize offered again.
    utils.rerender(<Harness fleet={TWO} windowChromePx={CHROME} />);
    expect(region(utils.getByRole).style.width).toBe(normalWidth);
    expect(utils.getByText('vehicleWindow.maximize')).toHaveAttribute('aria-pressed', 'false');
  });

  it('swapped with no picture: the large area states it in text', () => {
    const { getByText, getAllByText } = renderPicked([entry(1)], 1);
    fireEvent.click(getByText('vehicleWindow.source.camera'));
    fireEvent.click(getByText('vehicleWindow.maximize'));
    expect(getAllByText('vehicleWindow.noCamera UAV 1')).toHaveLength(2);
  });
});

describe('loss', () => {
  it('stream removal: the same UAV\'s other stream', () => {
    const fleet = [entry(1, [stream(100, 1), stream(101, 2)]), entry(2, [stream(100, 1)])];
    const { getByTestId, rerender } = renderPicked(fleet, 1);
    rerender(<Harness fleet={[entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])]} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1 · overlay 101/2');
    expect(latest().conf.url).toBe(stream(101, 2).whep_url);
  });

  it('else "Camera video lost · UAV n" with Reconnect video and Dismiss; Dismiss opens Synthetic for that UAV', () => {
    const { getByText, getByTestId, rerender, container } = renderPicked(TWO, 1);
    rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)]), entry(3)]} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1');
    expect(getByText(/vehicleWindow.videoLost UAV 1/)).toBeInTheDocument();
    expect(getByText('vehicleWindow.reconnect')).toBeDisabled();
    expect(open()).toHaveLength(0);

    fireEvent.click(getByText('vehicleWindow.dismiss'));
    // UAV 2 still advertises: no silent switch.
    expect(container.querySelector('video')).toBeNull();
    expect(getByTestId('synthetic-widget')).toHaveTextContent('1');
    expect(getByText('vehicleWindow.state.synthetic')).toBeInTheDocument();
  });

  it('the picked UAV leaves the fleet: button gone, selection kept with link and video lost; it resumes on return', () => {
    const utils = renderPicked(TWO, 1);
    utils.rerender(<Harness fleet={TWO.slice(1)} />);
    expect(utils.queryByRole('button', { name: nameOf(1) })).toBeNull();
    expect(utils.getByTestId('video-title')).toHaveTextContent('UAV 1');
    expect(utils.getByText('vehicleWindow.linkLost UAV 1')).toBeInTheDocument();
    expect(utils.getByText(/vehicleWindow.videoLost UAV 1/)).toBeInTheDocument();
    utils.rerender(<Harness fleet={TWO} />);
    expect(open()).toHaveLength(1);
    expect(utils.queryByText(/vehicleWindow.videoLost/)).toBeNull();
  });

  it('"Vehicle link lost" alone while the video still plays', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const { container, getByText, queryByText } = render(<Harness fleet={fleet} />);
    fireEvent.click(container.querySelector('[data-sysid="1"]'));
    frame(container);
    expect(getByText('vehicleWindow.linkLost UAV 1')).toBeInTheDocument();
    expect(queryByText(/vehicleWindow.videoLost/)).toBeNull();
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
  });

  it('both lost: both shown, link first', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const { container, getByRole } = render(<Harness fleet={fleet} />);
    fireEvent.click(container.querySelector('[data-sysid="1"]'));
    frame(container);
    advance(VIDEO_STALL_DEADLINE_S * 1000);
    const alerts = within(region(getByRole)).getAllByRole('alert');
    expect(alerts.map((a) => a.textContent.split(' ')[0])).toEqual(['vehicleWindow.linkLost', 'vehicleWindow.videoLost']);
  });
});
