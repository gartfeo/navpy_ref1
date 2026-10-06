import { useCallback, useEffect, useRef, useState } from 'react';
import MediaMTXWebRTCReader from '../vendor/mediamtx/reader';

/**
 * Playback state of the single video feed. `unavailable` means the feed is not
 * configured at all (no build-time WHEP URL) — not that it failed.
 */
export const WHEP_STATUS = {
  UNAVAILABLE: 'unavailable',
  CONNECTING: 'connecting',
  LIVE: 'live',
  STALLED: 'stalled',
  ERROR: 'error',
};

/**
 * Plays one WHEP stream into a <video> element.
 *
 * Reconnect on a lost stream is the vendored reader's job (it retries on its
 * own and says so through onError), so nothing here loops or polls. `retry()`
 * is the operator's explicit "start over": it tears the reader down and builds
 * a fresh one, which is the only way out of the reader's terminal failure.
 *
 * @param {string|null} url - WHEP endpoint, or null when unconfigured.
 */
export default function useWhepStream(url) {
  const videoRef = useRef(null);
  // The attached stream is held here, not read back off the element: React has
  // already detached the ref by the time an unmount cleanup runs, so the
  // element is no longer reachable when the tracks must be stopped.
  const streamRef = useRef(null);
  const [status, setStatus] = useState(url ? WHEP_STATUS.CONNECTING : WHEP_STATUS.UNAVAILABLE);
  const [error, setError] = useState(null);
  const [attempt, setAttempt] = useState(0);

  const retry = useCallback(() => setAttempt((n) => n + 1), []);

  // Signalling + teardown. One reader per (url, attempt).
  useEffect(() => {
    if (!url) {
      setStatus(WHEP_STATUS.UNAVAILABLE);
      setError(null);
      return undefined;
    }

    setStatus(WHEP_STATUS.CONNECTING);
    setError(null);

    let released = false;
    const reader = new MediaMTXWebRTCReader({
      url,
      onError: (err) => {
        if (released) return;
        setError(String(err));
        setStatus(WHEP_STATUS.ERROR);
      },
      onTrack: (evt) => {
        const video = videoRef.current;
        if (released || !video) return;
        const stream = evt?.streams?.[0];
        if (!stream || video.srcObject === stream) return;
        streamRef.current = stream;
        video.srcObject = stream;
        // Muted autoplay is allowed, but an explicit play() also covers the
        // case where the element was paused by a previous teardown.
        try {
          video.play?.()?.catch?.(() => {});
        } catch { /* element not playable yet; the autoplay attribute covers it */ }
      },
    });

    // Drop the session, the peer connection and the remote tracks together, so
    // neither an unmount nor a page transition leaves the publisher streaming.
    const release = () => {
      if (released) return;
      released = true;
      reader.close();
      const stream = streamRef.current;
      streamRef.current = null;
      if (stream?.getTracks) {
        for (const track of stream.getTracks()) track.stop();
      }
      if (videoRef.current) videoRef.current.srcObject = null;
    };

    // pagehide, not unload: it is the event that fires for a backgrounded or
    // discarded page in mobile WebViews, where an unmount never happens.
    let releasedByPagehide = false;
    let recovering = false;
    const onPageHide = () => {
      if (released) return;
      releasedByPagehide = true;
      release();
    };
    // A back-forward-cache restore keeps the component mounted, so no effect
    // reruns by itself and the session pagehide dropped would never come back.
    // Bumping the attempt counter rebuilds exactly one fresh reader, and only
    // for a hook that released on its own pagehide and is still listening.
    const onPageShow = () => {
      if (!releasedByPagehide || recovering) return;
      recovering = true;
      setAttempt((n) => n + 1);
    };

    window.addEventListener('pagehide', onPageHide);
    window.addEventListener('pageshow', onPageShow);
    return () => {
      window.removeEventListener('pagehide', onPageHide);
      window.removeEventListener('pageshow', onPageShow);
      release();
    };
  }, [url, attempt]);

  // The element's own events are the only honest source of "is it playing".
  useEffect(() => {
    const video = videoRef.current;
    if (!video) return undefined;

    const onPlaying = () => {
      setStatus(WHEP_STATUS.LIVE);
      setError(null);
    };
    // Frames stopped arriving. Only meaningful once playback had started;
    // before that the reader's own state already describes the situation.
    const onStall = () => setStatus((s) => (s === WHEP_STATUS.LIVE ? WHEP_STATUS.STALLED : s));

    video.addEventListener('playing', onPlaying);
    video.addEventListener('waiting', onStall);
    video.addEventListener('stalled', onStall);
    return () => {
      video.removeEventListener('playing', onPlaying);
      video.removeEventListener('waiting', onStall);
      video.removeEventListener('stalled', onStall);
    };
  }, [url, attempt]);

  return { status, error, videoRef, retry };
}
