import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, fireEvent, act } from '@testing-library/react';
import useWhepStream from './useWhepStream';

// The reader's own signalling is covered in src/vendor/mediamtx/reader.test.js.
// Here it is a stub, so these tests are about the hook's state machine and the
// resources it must let go of.
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

function Probe({ url }) {
  const { status, error, videoRef, retry } = useWhepStream(url);
  return (
    <div>
      <span>status:{status}</span>
      <span>error:{error ?? '-'}</span>
      <video ref={videoRef} />
      <button type="button" onClick={retry}>retry</button>
    </div>
  );
}

const fakeStream = () => {
  const track = { stop: vi.fn() };
  return { stream: { getTracks: () => [track] }, track };
};

const latest = () => readers[readers.length - 1];

beforeEach(() => {
  readers.length = 0;
  vi.spyOn(HTMLMediaElement.prototype, 'play').mockResolvedValue(undefined);
});

describe('useWhepStream', () => {
  it('reports unavailable and opens no session when unconfigured', () => {
    const { getByText } = render(<Probe url={null} />);
    expect(getByText('status:unavailable')).toBeInTheDocument();
    expect(readers).toHaveLength(0);
  });

  it('connects to the configured URL and goes live when the element plays', () => {
    const { getByText, container } = render(<Probe url="http://host/whep" />);
    expect(getByText('status:connecting')).toBeInTheDocument();
    expect(readers).toHaveLength(1);
    expect(latest().conf.url).toBe('http://host/whep');

    const video = container.querySelector('video');
    const { stream } = fakeStream();
    act(() => latest().conf.onTrack({ streams: [stream] }));
    expect(video.srcObject).toBe(stream);

    fireEvent.playing(video);
    expect(getByText('status:live')).toBeInTheDocument();
  });

  it('reports a stall only once playback has started', () => {
    const { getByText, container } = render(<Probe url="http://host/whep" />);
    const video = container.querySelector('video');

    // Before playback the reader's state already describes the situation.
    fireEvent.waiting(video);
    expect(getByText('status:connecting')).toBeInTheDocument();

    fireEvent.playing(video);
    fireEvent.waiting(video);
    expect(getByText('status:stalled')).toBeInTheDocument();
  });

  it('surfaces a reader error and lets the operator start over', () => {
    const { getByText } = render(<Probe url="http://host/whep" />);

    act(() => latest().conf.onError('peer connection closed, retrying in some seconds'));
    expect(getByText('status:error')).toBeInTheDocument();
    expect(getByText('error:peer connection closed, retrying in some seconds')).toBeInTheDocument();

    const first = latest();
    fireEvent.click(getByText('retry'));
    expect(readers).toHaveLength(2);
    expect(first.closeCount).toBe(1);
    expect(getByText('status:connecting')).toBeInTheDocument();
    expect(getByText('error:-')).toBeInTheDocument();
  });

  // React detaches the ref before the cleanup runs, so the tracks have to be
  // reachable without the element — the discarded element itself is harmless.
  it('closes the session and stops the tracks on unmount', () => {
    const { stream, track } = fakeStream();
    const { unmount } = render(<Probe url="http://host/whep" />);
    act(() => latest().conf.onTrack({ streams: [stream] }));
    const reader = latest();

    unmount();

    expect(reader.closeCount).toBe(1);
    expect(track.stop).toHaveBeenCalledTimes(1);
  });

  it('releases everything on pagehide, and does not release twice on unmount', () => {
    const { container, unmount } = render(<Probe url="http://host/whep" />);
    const video = container.querySelector('video');
    const { stream, track } = fakeStream();
    act(() => latest().conf.onTrack({ streams: [stream] }));
    const reader = latest();

    fireEvent(window, new Event('pagehide'));

    expect(reader.closeCount).toBe(1);
    expect(track.stop).toHaveBeenCalledTimes(1);
    expect(video.srcObject).toBeNull();

    unmount();
    expect(reader.closeCount).toBe(1);
    expect(track.stop).toHaveBeenCalledTimes(1);
  });

  // A back-forward-cache restore leaves the component mounted, so without a
  // pageshow recovery the feed stays dead with no effect left to rerun.
  it('rebuilds exactly one session when the page is restored after pagehide', () => {
    const { getByText, container } = render(<Probe url="http://host/whep" />);
    const video = container.querySelector('video');
    const { stream } = fakeStream();
    act(() => latest().conf.onTrack({ streams: [stream] }));
    fireEvent.playing(video);
    const first = latest();

    fireEvent(window, new Event('pagehide'));
    expect(first.closeCount).toBe(1);

    act(() => {
      fireEvent(window, new Event('pageshow'));
    });

    expect(readers).toHaveLength(2);
    expect(latest()).not.toBe(first);
    expect(latest().conf.url).toBe('http://host/whep');
    expect(first.closeCount).toBe(1);
    expect(getByText('status:connecting')).toBeInTheDocument();

    // The fresh session plays as usual.
    const second = fakeStream();
    act(() => latest().conf.onTrack({ streams: [second.stream] }));
    expect(container.querySelector('video').srcObject).toBe(second.stream);
  });

  it('does not recover on a pageshow that follows no pagehide', () => {
    render(<Probe url="http://host/whep" />);
    act(() => {
      fireEvent(window, new Event('pageshow'));
    });
    expect(readers).toHaveLength(1);
  });

  it('does not reconnect on pagehide/pageshow after unmount', () => {
    const { unmount } = render(<Probe url="http://host/whep" />);
    const reader = latest();

    unmount();
    act(() => {
      fireEvent(window, new Event('pagehide'));
      fireEvent(window, new Event('pageshow'));
    });

    expect(readers).toHaveLength(1);
    expect(reader.closeCount).toBe(1);
  });

  it('ignores a late track that arrives after release', () => {
    const { container } = render(<Probe url="http://host/whep" />);
    const video = container.querySelector('video');
    const reader = latest();

    fireEvent(window, new Event('pagehide'));

    const { stream, track } = fakeStream();
    act(() => reader.conf.onTrack({ streams: [stream] }));
    expect(video.srcObject).toBeNull();
    expect(track.stop).not.toHaveBeenCalled();
  });

  it('tears the session down when the URL disappears', () => {
    const { getByText, rerender } = render(<Probe url="http://host/whep" />);
    const reader = latest();

    rerender(<Probe url={null} />);

    expect(reader.closeCount).toBe(1);
    expect(getByText('status:unavailable')).toBeInTheDocument();
    expect(readers).toHaveLength(1);
  });
});
