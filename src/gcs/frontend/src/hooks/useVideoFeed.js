import { useEffect, useRef, useState } from 'react';
import useWhepStream from './useWhepStream';
import { VIDEO_CLOCK_TICK_MS } from '../constants/video';
import { codecSupported } from '../utils/videoStreams';
import { FEED_FAILURE, FEED_STATUS, feedTiming } from '../utils/videoFeed';

/**
 * Live state of one camera stream, from frame progress.
 *
 * - The codec is checked before connecting; unsupported fails at once, with no
 *   reader opened.
 * - Frames are counted with requestVideoFrameCallback, else by sampling
 *   getVideoPlaybackQuality().totalVideoFrames once per clock tick.
 * - A reader error, no first frame in time, or a stall latches FAILED and
 *   drops the url, which releases the reader: a failed feed holds no session.
 *   Starting over is the caller's job (it remounts with a new key).
 *
 * @param {{ whep_url: string, encoding?: string|null }} stream
 */
export default function useVideoFeed(stream) {
  const [failure, setFailure] = useState(() => (codecSupported(stream.encoding) ? null : FEED_FAILURE.CODEC));
  const [errorText, setErrorText] = useState(null);
  const { videoRef, error, sessionId } = useWhepStream(failure ? null : stream.whep_url);

  const startedAtRef = useRef(Date.now());
  const lastFrameAtRef = useRef(null);
  // Set by pagehide: the reader was released by the page, not by a fault, so
  // the deadlines wait for the next session instead of failing the feed.
  const suspendedRef = useRef(false);
  const sampleRef = useRef(null);
  const [, setFirstFrame] = useState(false);
  const [now, setNow] = useState(() => Date.now());

  // Each new session restarts the first-frame deadline.
  useEffect(() => {
    if (sessionId === 0) return;
    suspendedRef.current = false;
    startedAtRef.current = Date.now();
    lastFrameAtRef.current = null;
    setFirstFrame(false);
    setNow(Date.now());
  }, [sessionId]);

  useEffect(() => {
    const onHide = () => { suspendedRef.current = true; };
    window.addEventListener('pagehide', onHide);
    return () => window.removeEventListener('pagehide', onHide);
  }, []);

  // Frame progress.
  useEffect(() => {
    if (failure) return undefined;
    const video = videoRef.current;
    if (!video) return undefined;
    const markFrame = () => {
      lastFrameAtRef.current = Date.now();
      setFirstFrame(true);
    };
    if (typeof video.requestVideoFrameCallback === 'function') {
      let handle = null;
      const onFrame = () => {
        markFrame();
        handle = video.requestVideoFrameCallback(onFrame);
      };
      handle = video.requestVideoFrameCallback(onFrame);
      return () => video.cancelVideoFrameCallback?.(handle);
    }
    let lastCount = 0;
    sampleRef.current = () => {
      const count = video.getVideoPlaybackQuality?.()?.totalVideoFrames;
      if (Number.isFinite(count) && count > lastCount) markFrame();
      if (Number.isFinite(count)) lastCount = count;
    };
    return () => { sampleRef.current = null; };
  }, [failure, videoRef]);

  // One clock for the "N s" counter and both deadlines.
  useEffect(() => {
    if (failure) return undefined;
    const id = setInterval(() => {
      sampleRef.current?.();
      setNow(Date.now());
    }, VIDEO_CLOCK_TICK_MS);
    return () => clearInterval(id);
  }, [failure]);

  let timing;
  if (failure) timing = { status: FEED_STATUS.FAILED, reason: failure };
  else if (error) timing = { status: FEED_STATUS.FAILED, reason: FEED_FAILURE.CONNECTION };
  else if (suspendedRef.current) timing = { status: FEED_STATUS.CONNECTING };
  else {
    timing = feedTiming({
      startedAt: startedAtRef.current,
      lastFrameAt: lastFrameAtRef.current,
      now: Math.max(now, lastFrameAtRef.current ?? now),
    });
  }

  // Latch a failure: the url drops and the reader is released.
  const failedReason = timing.status === FEED_STATUS.FAILED ? timing.reason : null;
  useEffect(() => {
    if (failure || !failedReason) return;
    if (failedReason === FEED_FAILURE.CONNECTION) setErrorText(error);
    setFailure(failedReason);
  }, [failure, failedReason, error]);

  return { videoRef, ...timing, errorText };
}
