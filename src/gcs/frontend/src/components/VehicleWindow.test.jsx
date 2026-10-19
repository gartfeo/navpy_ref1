// The monitor view's vehicle window (docs/design/gcs_video_streams.md
// section 5, "Vehicle window"), driven through useVehicleWindow and
// vehicleWindowLayout as App wires them. The synthetic widget is stubbed here;
// SyntheticView.test.jsx covers it.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, fireEvent, act, within } from '@testing-library/react';
import VehicleWindow, { Z_INDEX } from './VehicleWindow';
import useVehicleWindow from '../hooks/useVehicleWindow';
import { vehicleWindowLayout } from '../utils/vehicleWindowLayout';
import { vehicleWindowVisible } from '../utils/videoStreams';
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
    default: function SyntheticStub({ sysId, onFail }) {
      synthetic.onFail = onFail;
      useEffect(() => {
        synthetic.mounts += 1;
        return () => { synthetic.unmounts += 1; };
      }, []);
      return <div data-testid="synthetic-widget">{sysId}</div>;
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

function Harness({ fleet, followSysId = null, active = true, paneWidth = 1600 }) {
  const win = useVehicleWindow({ fleet, followSysId, active });
  if (!vehicleWindowVisible({ monitor: active, rcEnabled: false, fleetSize: fleet.length })) return null;
  const layout = vehicleWindowLayout({
    paneWidth,
    paneHeight: 900,
    topControlsBottom: 44,
    badgeCount: fleet.length,
    selectedIndex: fleet.findIndex((e) => e.sysId === win.sysId),
    swapped: win.swapped,
    mapCreditsPx: 0,
  });
  return (
    <VehicleWindow
      fleet={fleet}
      win={win}
      layout={layout}
      poseSourceRef={{ current: null }}
      storeRef={{ current: null }}
      ionTerrain={null}
      onSyntheticCreditsHeight={() => {}}
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

beforeEach(() => {
  readers.length = 0;
  localStorage.clear();
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
  delete HTMLVideoElement.prototype.requestVideoFrameCallback;
  delete HTMLVideoElement.prototype.cancelVideoFrameCallback;
  delete HTMLVideoElement.prototype.getVideoPlaybackQuality;
  delete globalThis.RTCRtpReceiver;
});

const TWO = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)]), entry(3)];

const region = (getByRole) => getByRole('region', { name: 'vehicleWindow.region' });

describe('window visibility and place', () => {
  it('shown whenever the fleet has a UAV, even with no streams; no reader then', () => {
    const { getByRole, getByText } = render(<Harness fleet={[entry(1), entry(2)]} />);
    expect(region(getByRole)).toBeInTheDocument();
    expect(getByText('vehicleWindow.noCamera UAV 1')).toBeInTheDocument();
    expect(getByText('vehicleWindow.state.noCamera')).toBeInTheDocument();
    // "No camera" offers no Reconnect or Dismiss.
    expect(within(region(getByRole)).queryByText('vehicleWindow.dismiss')).toBeNull();
    expect(readers).toHaveLength(0);
  });

  it('no window for an empty fleet or outside the active monitor view', () => {
    expect(render(<Harness fleet={[]} />).container).toBeEmptyDOMElement();
    expect(render(<Harness fleet={TWO} active={false} />).container).toBeEmptyDOMElement();
  });

  it('hiding the window closes its reader; showing it again reconnects', () => {
    const { rerender } = render(<Harness fleet={TWO} />);
    const first = latest();
    rerender(<Harness fleet={TWO} active={false} />);
    expect(first.closeCount).toBe(1);
    rerender(<Harness fleet={TWO} />);
    expect(open()).toHaveLength(1);
  });

  it('sits bottom-left, under the map controls', () => {
    const { getByRole } = render(<Harness fleet={TWO} />);
    const win = region(getByRole);
    expect(Z_INDEX).toBeLessThan(10);
    expect(win.style.bottom).toBe('12px');
    expect(win.style.left).toBe('12px');
  });
});

describe('badges and header', () => {
  it('a badge for every UAV, with video or not; selected has fill, outline and the full label', () => {
    const { getByRole } = render(<Harness fleet={TWO} />);
    const one = getByRole('button', { name: 'UAV 1' });
    const two = getByRole('button', { name: 'UAV 2' });
    expect(getByRole('button', { name: 'UAV 3' })).toBeInTheDocument();
    expect(one).toHaveAttribute('aria-pressed', 'true');
    expect(one).toHaveTextContent('UAV 1');
    expect(one.style.outline).toContain('solid');
    expect(two).toHaveAttribute('aria-pressed', 'false');
    expect(two.style.outline).toBe('none');
    expect(parseInt(one.style.minHeight, 10)).toBeGreaterThanOrEqual(44);
  });

  it('header names the view from the stream metadata, 14 px on an opaque background', () => {
    const { getByTestId, container } = render(<Harness fleet={TWO} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1 · overlay 100/1');
    const header = container.querySelector('header');
    expect(header.style.fontSize).toBe('14px');
    expect(header.style.background).not.toContain('rgba');
  });

  it('switching: a badge plays that UAV, one reader at a time; a UAV without video shows "No camera"', () => {
    const { getByRole, getByTestId, getByText } = render(<Harness fleet={TWO} />);
    const first = latest();
    fireEvent.click(getByRole('button', { name: 'UAV 2' }));
    expect(first.closeCount).toBe(1);
    expect(open()).toHaveLength(1);
    expect(latest().conf.url).toBe(stream(100, 1).whep_url);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 2');
    fireEvent.click(getByRole('button', { name: 'UAV 3' }));
    expect(open()).toHaveLength(0);
    expect(getByText('vehicleWindow.noCamera UAV 3')).toBeInTheDocument();
  });

  it('the follow selector picks only the first UAV; later follow changes keep the video', () => {
    const { getByTestId, rerender } = render(<Harness fleet={TWO} followSysId={2} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 2');
    const reader = latest();
    rerender(<Harness fleet={TWO} followSysId={1} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 2');
    expect(reader.closeCount).toBe(0);
  });
});

describe('live state from frame progress', () => {
  it('connecting names the UAV, LIVE once frames arrive, NO FRAMES N s after', () => {
    const { container, getByText, queryByText } = render(<Harness fleet={TWO} />);
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
    const { container, getByText, queryByText } = render(<Harness fleet={TWO} />);
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
    const { container, getByText } = render(<Harness fleet={TWO} />);
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
    const { getByText, container } = render(<Harness fleet={TWO} />);
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
    const { container, getByText } = render(<Harness fleet={fleet} />);
    expect(readers).toHaveLength(0);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/vehicleWindow.reason.codec H265/)).toBeInTheDocument();
  });

  it('a reader error collapses to the header and releases the reader; Reconnect video starts over', () => {
    const { container, getByText } = render(<Harness fleet={TWO} />);
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
    const { getByRole, getByTestId, getByText, queryByTestId } = render(<Harness fleet={TWO} />);
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

  it('a badge on Synthetic keeps Synthetic for the new UAV', () => {
    const { getByRole, getByTestId, getByText } = render(<Harness fleet={TWO} />);
    fireEvent.click(getByText('vehicleWindow.source.synthetic'));
    fireEvent.click(getByRole('button', { name: 'UAV 3' }));
    expect(getByTestId('synthetic-widget')).toHaveTextContent('3');
  });

  it('a synthetic failure falls back to Camera with a notice', () => {
    const { getByText, queryByTestId } = render(<Harness fleet={TWO} />);
    fireEvent.click(getByText('vehicleWindow.source.synthetic'));
    act(() => synthetic.onFail());
    expect(queryByTestId('synthetic-widget')).toBeNull();
    expect(getByText('vehicleWindow.syntheticUnavailable')).toHaveAttribute('role', 'alert');
    expect(getByText('vehicleWindow.source.camera')).toHaveAttribute('aria-pressed', 'true');
  });

  it('held pose on Synthetic: the held row names the reason', () => {
    const fleet = [entry(1, [], { linkOk: false, holdReason: 'linkLost' })];
    const { getByText } = render(<Harness fleet={fleet} />);
    fireEvent.click(getByText('vehicleWindow.source.synthetic'));
    expect(getByText('fpvView.held.linkLost 0')).toBeInTheDocument();
  });
});

describe('size', () => {
  it('Maximize swaps (labelled Restore, pressed); Restore goes back; the reader stays', () => {
    const { getByText, getByRole } = render(<Harness fleet={TWO} />);
    const reader = latest();
    const normalWidth = parseInt(region(getByRole).style.width, 10);
    fireEvent.click(getByText('vehicleWindow.maximize'));
    expect(getByText('vehicleWindow.restore')).toHaveAttribute('aria-pressed', 'true');
    expect(parseInt(region(getByRole).style.width, 10)).toBeGreaterThan(normalWidth);
    expect(region(getByRole).style.zIndex).toBe(String(Z_INDEX - 1));
    fireEvent.click(getByText('vehicleWindow.restore'));
    expect(parseInt(region(getByRole).style.width, 10)).toBe(normalWidth);
    expect(reader.closeCount).toBe(0);
  });

  it('swapped with no picture: the large area states it in text', () => {
    const { getByText, getAllByText } = render(<Harness fleet={[entry(1)]} />);
    fireEvent.click(getByText('vehicleWindow.maximize'));
    expect(getAllByText('vehicleWindow.noCamera UAV 1')).toHaveLength(2);
  });

  it('Minimize: a handle naming the UAV and its state; the reader closes; the handle restores', () => {
    const { getByText, getByRole, container } = render(<Harness fleet={TWO} />);
    const reader = latest();
    fireEvent.click(getByText('vehicleWindow.minimize'));
    expect(reader.closeCount).toBe(1);
    expect(container.querySelector('video')).toBeNull();
    const handle = getByRole('button', { name: 'vehicleWindow.handle.restoreLabel UAV 1 vehicleWindow.handle.camera' });
    expect(handle).toHaveTextContent('vehicleWindow.handle.label UAV 1 vehicleWindow.handle.camera');
    fireEvent.click(handle);
    expect(open()).toHaveLength(1);
  });

  it('the handle names Synthetic, No camera and a lost link', () => {
    const syn = render(<Harness fleet={[entry(1)]} />);
    fireEvent.click(syn.getByText('vehicleWindow.source.synthetic'));
    fireEvent.click(syn.getByText('vehicleWindow.minimize'));
    expect(syn.getByText(/handle.synthetic/)).toBeInTheDocument();
    expect(synthetic.unmounts).toBe(synthetic.mounts);
    syn.unmount();
    localStorage.clear();

    const none = render(<Harness fleet={[entry(1)]} />);
    fireEvent.click(none.getByText('vehicleWindow.minimize'));
    expect(none.getByText(/handle.noCamera/)).toBeInTheDocument();
    none.unmount();
    localStorage.clear();

    const lost = render(<Harness fleet={[entry(1, [], { linkOk: false })]} />);
    fireEvent.click(lost.getByText('vehicleWindow.minimize'));
    expect(lost.getByText(/handle.linkLost/)).toBeInTheDocument();
  });

  it('narrow pane: glyph buttons keep their names', () => {
    const { getByRole } = render(<Harness fleet={TWO} paneWidth={375} />);
    expect(getByRole('button', { name: 'vehicleWindow.maximize' })).toHaveTextContent('□');
    expect(getByRole('button', { name: 'vehicleWindow.minimize' })).toHaveTextContent('–');
    // Number badges, the full name stays the accessible name.
    expect(getByRole('button', { name: 'UAV 1' })).toHaveTextContent(/^1$/);
  });
});

describe('loss', () => {
  it('stream removal: the same UAV\'s other stream', () => {
    const fleet = [entry(1, [stream(100, 1), stream(101, 2)]), entry(2, [stream(100, 1)])];
    const { getByTestId, rerender } = render(<Harness fleet={fleet} />);
    rerender(<Harness fleet={[entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])]} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1 · overlay 101/2');
    expect(latest().conf.url).toBe(stream(101, 2).whep_url);
  });

  it('else "Camera video lost · UAV n" with Reconnect video and Dismiss; Dismiss goes to "No camera · UAV n"', () => {
    const { getByText, getByTestId, rerender, container } = render(<Harness fleet={TWO} />);
    rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)]), entry(3)]} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1');
    expect(getByText(/vehicleWindow.videoLost UAV 1/)).toBeInTheDocument();
    expect(getByText('vehicleWindow.reconnect')).toBeDisabled();
    expect(open()).toHaveLength(0);

    fireEvent.click(getByText('vehicleWindow.dismiss'));
    // UAV 2 still advertises: no silent switch.
    expect(container.querySelector('video')).toBeNull();
    expect(getByText('vehicleWindow.noCamera UAV 1')).toBeInTheDocument();
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1');
  });

  it('"Vehicle link lost" alone while the video still plays', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const { container, getByText, queryByText } = render(<Harness fleet={fleet} />);
    frame(container);
    expect(getByText('vehicleWindow.linkLost UAV 1')).toBeInTheDocument();
    expect(queryByText(/vehicleWindow.videoLost/)).toBeNull();
    expect(getByText('vehicleWindow.state.live')).toBeInTheDocument();
  });

  it('both lost: both shown, link first', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const { container, getByRole } = render(<Harness fleet={fleet} />);
    frame(container);
    advance(VIDEO_STALL_DEADLINE_S * 1000);
    const alerts = within(region(getByRole)).getAllByRole('alert');
    expect(alerts.map((a) => a.textContent.split(' ')[0])).toEqual(['vehicleWindow.linkLost', 'vehicleWindow.videoLost']);
  });
});
