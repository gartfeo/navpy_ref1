import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, fireEvent, act, within } from '@testing-library/react';
import VideoPanel, { Z_INDEX } from './VideoPanel';
import useVideoSelection from '../hooks/useVideoSelection';
import { cameraPanelVisible } from '../utils/videoStreams';
import { VIDEO_FIRST_FRAME_DEADLINE_S, VIDEO_STALL_DEADLINE_S } from '../constants/video';

// Interpolating t, so the operator text can be checked for the UAV it names.
vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key, opts) => (opts ? `${key} ${Object.values(opts).join(' ')}` : key),
    i18n: { language: 'en' },
  }),
}));

const { readers, FakeReader } = vi.hoisted(() => {
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
  return { readers, FakeReader };
});
vi.mock('../vendor/mediamtx/reader', () => ({ default: FakeReader }));

const stream = (camera, id, extra = {}) => ({
  key: `${camera}:${id}`, camera_component: camera, stream_id: id, name: `overlay ${camera}/${id}`,
  encoding: 'H264', whep_url: `http://192.168.144.1${camera % 10}:8889/c${camera}s${id}/whep`, ...extra,
});
const entry = (sysId, streams = [], extra = {}) => ({
  sysId, index: sysId - 1, label: `UAV ${sysId}`, streams, linkOk: true, fpvEligible: true, holdReason: null, ...extra,
});

function Harness({ fleet, followSysId = null }) {
  const video = useVideoSelection({ fleet, followSysId });
  if (!video.hasPanel) return null;
  return (
    <VideoPanel
      fleet={fleet}
      selection={video.selection}
      shown={video.shown}
      session={video.session}
      collapsed={video.collapsed}
      onCollapse={video.setCollapsed}
      onSelectUav={video.selectUav}
      onReconnect={video.reconnect}
      onDismiss={video.dismiss}
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

describe('camera panel visibility', () => {
  it('no panel and no reader without streams', () => {
    const { container } = render(<Harness fleet={[entry(1), entry(2)]} />);
    expect(container).toBeEmptyDOMElement();
    expect(readers).toHaveLength(0);
  });

  it('monitor only, hidden in manual control and in FPV', () => {
    const base = { monitor: true, rcEnabled: false, fpvOn: false, hasPanel: true };
    expect(cameraPanelVisible(base)).toBe(true);
    expect(cameraPanelVisible({ ...base, rcEnabled: true })).toBe(false);
    expect(cameraPanelVisible({ ...base, fpvOn: true })).toBe(false);
    expect(cameraPanelVisible({ ...base, monitor: false })).toBe(false);
    expect(cameraPanelVisible({ ...base, hasPanel: false })).toBe(false);
  });

  it('hiding the panel (FPV, RC) closes its reader; showing it again reconnects', () => {
    const { rerender } = render(<Harness fleet={TWO} />);
    const first = latest();
    rerender(<></>);
    expect(first.closeCount).toBe(1);
    rerender(<Harness fleet={TWO} />);
    expect(open()).toHaveLength(1);
  });

  it('sits bottom-left, under the map controls', () => {
    const { getByRole } = render(<Harness fleet={TWO} />);
    const panel = getByRole('region', { name: 'videoPanel.region' });
    expect(Z_INDEX).toBeLessThan(10);
    expect(panel.style.bottom).toBe('12px');
    expect(panel.style.left).toBe('12px');
  });
});

describe('badges and header', () => {
  it('badges only for vehicles with streams; selected has fill, outline and the full label', () => {
    const { getByRole, queryByRole } = render(<Harness fleet={TWO} />);
    const one = getByRole('button', { name: 'UAV 1' });
    const two = getByRole('button', { name: 'UAV 2' });
    expect(queryByRole('button', { name: 'UAV 3' })).toBeNull();
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

  it('switching: a badge plays that UAV, one reader at a time', () => {
    const { getByRole, getByTestId } = render(<Harness fleet={TWO} />);
    const first = latest();
    fireEvent.click(getByRole('button', { name: 'UAV 2' }));
    expect(first.closeCount).toBe(1);
    expect(open()).toHaveLength(1);
    expect(latest().conf.url).toBe(stream(100, 1).whep_url);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 2');
  });

  it('the follow selector picks only the first feed; later follow changes keep the video', () => {
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
    expect(getByText('videoPanel.connectingUav UAV 1')).toBeInTheDocument();
    expect(getByText('videoPanel.state.connecting')).toBeInTheDocument();

    frame(container);
    expect(getByText('videoPanel.state.live')).toBeInTheDocument();
    expect(queryByText(/connectingUav/)).toBeNull();

    // Frames keep coming: stays LIVE.
    for (let i = 0; i < 3; i++) { advance(500); frame(container); }
    advance(500);
    expect(getByText('videoPanel.state.live')).toBeInTheDocument();

    // Frames stop: the element raises no waiting/stalled event at all.
    // Last frame at 1.5 s; the clock ticks on whole seconds, so at 3 s the
    // picture has had no frame for 1.5 s and reads "NO FRAMES 1 s".
    advance(1000);
    expect(queryByText('videoPanel.state.live')).toBeNull();
    expect(getByText('videoPanel.state.stale')).toBeInTheDocument();
    expect(getByText('videoPanel.noFrames 1')).toBeInTheDocument();

    // Frames again: LIVE at the next tick, the stale text goes.
    frame(container);
    advance(500);
    frame(container);
    advance(500);
    expect(getByText('videoPanel.state.live')).toBeInTheDocument();
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
    expect(getByText('videoPanel.state.failed')).toBeInTheDocument();
    expect(getByText(/videoPanel.videoLost UAV 1 — videoPanel.reason.stalled 5/)).toBeInTheDocument();
    expect(getByText('videoPanel.reconnect')).toBeInTheDocument();
    expect(getByText('videoPanel.dismiss')).toBeInTheDocument();
    // A failed feed holds no session.
    expect(reader.closeCount).toBe(1);
    expect(queryByText('videoPanel.expand')).toBeNull();
  });

  it('first-frame deadline: peer connected but no frames', () => {
    const { container, getByText } = render(<Harness fleet={TWO} />);
    act(() => latest().conf.onTrack({ streams: [{ getTracks: () => [] }] }));
    advance((VIDEO_FIRST_FRAME_DEADLINE_S - 1) * 1000);
    expect(getByText('videoPanel.state.connecting')).toBeInTheDocument();
    advance(1000);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/videoPanel.reason.noFirstFrame 10/)).toBeInTheDocument();
  });

  it('falls back to getVideoPlaybackQuality without requestVideoFrameCallback', () => {
    delete HTMLVideoElement.prototype.requestVideoFrameCallback;
    let frames = 0;
    HTMLVideoElement.prototype.getVideoPlaybackQuality = () => ({ totalVideoFrames: frames });
    const { getByText, container } = render(<Harness fleet={TWO} />);
    frames = 30;
    advance(1000);
    expect(getByText('videoPanel.state.live')).toBeInTheDocument();
    // Counter stops: stale, then failed.
    advance(2000);
    expect(getByText(/videoPanel.noFrames/)).toBeInTheDocument();
    advance(VIDEO_STALL_DEADLINE_S * 1000);
    expect(container.querySelector('video')).toBeNull();
  });

  it('codec not supported: collapses to the header with the reason, no reader', () => {
    globalThis.RTCRtpReceiver = { getCapabilities: () => ({ codecs: [{ mimeType: 'video/H264' }] }) };
    const fleet = [entry(1, [stream(100, 1, { encoding: 'H265' })])];
    const { container, getByText } = render(<Harness fleet={fleet} />);
    expect(readers).toHaveLength(0);
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/videoPanel.reason.codec H265/)).toBeInTheDocument();
  });

  it('a reader error collapses to the header and releases the reader; Reconnect video starts over', () => {
    const { container, getByText } = render(<Harness fleet={TWO} />);
    const first = latest();
    act(() => first.conf.onError('peer connection closed'));
    expect(container.querySelector('video')).toBeNull();
    expect(getByText(/videoPanel.reason.connection/)).toBeInTheDocument();
    expect(first.closeCount).toBe(1);

    fireEvent.click(getByText('videoPanel.reconnect'));
    expect(readers).toHaveLength(2);
    expect(open()).toHaveLength(1);
    expect(container.querySelector('video')).not.toBeNull();
    expect(getByText('videoPanel.state.connecting')).toBeInTheDocument();
  });
});

describe('expand, collapse and loss', () => {
  it('expand/restore and collapse to the header of a healthy feed (labelled buttons)', () => {
    const { container, getByText, getByRole, queryByText } = render(<Harness fleet={TWO} />);
    frame(container);
    const panel = getByRole('region', { name: 'videoPanel.region' });
    const docked = parseInt(panel.style.width, 10);

    fireEvent.click(getByText('videoPanel.expand'));
    expect(parseInt(panel.style.width, 10)).toBeGreaterThan(docked);
    expect(getByText('videoPanel.restore')).toHaveAttribute('aria-pressed', 'true');
    fireEvent.click(getByText('videoPanel.restore'));
    expect(parseInt(panel.style.width, 10)).toBe(docked);

    const reader = latest();
    fireEvent.click(getByText('videoPanel.collapse'));
    expect(container.querySelector('video')).toBeNull();
    expect(getByText('videoPanel.state.hidden')).toBeInTheDocument();
    expect(reader.closeCount).toBe(1);
    fireEvent.click(getByText('videoPanel.show'));
    expect(container.querySelector('video')).not.toBeNull();
    expect(open()).toHaveLength(1);
    expect(queryByText('videoPanel.show')).toBeNull();
  });

  it('stream removal: the same UAV\'s other stream', () => {
    const fleet = [entry(1, [stream(100, 1), stream(101, 2)]), entry(2, [stream(100, 1)])];
    const { getByTestId, rerender } = render(<Harness fleet={fleet} />);
    rerender(<Harness fleet={[entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])]} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1 · overlay 101/2');
    expect(latest().conf.url).toBe(stream(101, 2).whep_url);
  });

  it('else "Camera video lost · UAV n" with Reconnect video and Dismiss, never another UAV', () => {
    const { getByText, getByTestId, rerender, container } = render(<Harness fleet={TWO} />);
    rerender(<Harness fleet={[entry(1), entry(2, [stream(100, 1)])]} />);
    expect(getByTestId('video-title')).toHaveTextContent('UAV 1');
    expect(getByText(/videoPanel.videoLost UAV 1/)).toBeInTheDocument();
    expect(getByText('videoPanel.reconnect')).toBeDisabled();
    expect(open()).toHaveLength(0);

    fireEvent.click(getByText('videoPanel.dismiss'));
    // UAV 2 still advertises, so the panel stays, idle: no silent switch.
    expect(container.querySelector('video')).toBeNull();
    expect(getByText('videoPanel.pickUav')).toBeInTheDocument();
  });

  it('panel gone only after the last lost feed is dismissed and no stream is advertised', () => {
    const fleet = [entry(1, [stream(100, 1)])];
    const { getByText, rerender, container } = render(<Harness fleet={fleet} />);
    rerender(<Harness fleet={[entry(1)]} />);
    expect(container).not.toBeEmptyDOMElement();
    fireEvent.click(getByText('videoPanel.dismiss'));
    expect(container).toBeEmptyDOMElement();
  });

  it('"Vehicle link lost" alone while the video still plays', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const { container, getByText, queryByText } = render(<Harness fleet={fleet} />);
    frame(container);
    expect(getByText('videoPanel.linkLost UAV 1')).toBeInTheDocument();
    expect(queryByText(/videoPanel.videoLost/)).toBeNull();
    expect(getByText('videoPanel.state.live')).toBeInTheDocument();
  });

  it('both lost: both shown, link first', () => {
    const fleet = [entry(1, [stream(100, 1)], { linkOk: false })];
    const { container, getByRole } = render(<Harness fleet={fleet} />);
    frame(container);
    advance(VIDEO_STALL_DEADLINE_S * 1000);
    const alerts = within(getByRole('region', { name: 'videoPanel.region' })).getAllByRole('alert');
    expect(alerts.map((a) => a.textContent.split(' ')[0])).toEqual(['videoPanel.linkLost', 'videoPanel.videoLost']);
  });
});
