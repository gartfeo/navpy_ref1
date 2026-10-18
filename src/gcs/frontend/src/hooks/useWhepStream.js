import { useEffect, useRef, useState } from 'react';
import MediaMTXWebRTCReader from '../vendor/mediamtx/reader';

/**
 * Plays one WHEP stream into a <video> element: signalling, track attach and
 * release only. Whether frames actually arrive is measured by the caller from
 * frame progress (useVideoFeed), not from this hook or the element's events.
 *
 * One reader per (url, session). A null url opens nothing and releases the
 * current reader. A reader error is reported through `error`; the caller
 * decides what to do (the camera panel treats it as failed and drops the url).
 *
 * `sessionId` counts the readers this hook has opened, so the caller can tell
 * a fresh session (e.g. after a back-forward-cache restore) from the old one.
 *
 * @param {string|null} url - WHEP endpoint, or null for no stream.
 */
export default function useWhepStream(url) {
  const videoRef = useRef(null);
  // The attached stream is held here, not read back off the element: React has
  // already detached the ref by the time an unmount cleanup runs, so the
  // element is no longer reachable when the tracks must be stopped.
  const streamRef = useRef(null);
  const [error, setError] = useState(null);
  const [attempt, setAttempt] = useState(0);
  const [sessionId, setSessionId] = useState(0);

  useEffect(() => {
    setError(null);
    if (!url) return undefined;

    setSessionId((n) => n + 1);

    let released = false;
    const reader = new MediaMTXWebRTCReader({
      url,
      onError: (err) => {
        if (released) return;
        setError(String(err));
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

  return { videoRef, error, sessionId };
}
