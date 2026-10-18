import {
  VIDEO_FIRST_FRAME_DEADLINE_S,
  VIDEO_STALE_AFTER_S,
  VIDEO_STALL_DEADLINE_S,
} from '../constants/video';

/** Live state of the one camera feed on screen. LIVE means frames arrive, not low latency. */
export const FEED_STATUS = {
  CONNECTING: 'connecting',
  LIVE: 'live',
  STALE: 'stale',
  FAILED: 'failed',
};

/** Why a feed failed; each maps to a short operator-facing reason. */
export const FEED_FAILURE = {
  NO_FIRST_FRAME: 'noFirstFrame',
  STALLED: 'stalled',
  CODEC: 'codec',
  CONNECTION: 'connection',
};

/**
 * Feed state from frame progress alone (all times in ms, same clock).
 * `lastFrameAt` is null until the first frame was presented.
 */
export function feedTiming({ startedAt, lastFrameAt, now }) {
  if (lastFrameAt == null) {
    return now - startedAt >= VIDEO_FIRST_FRAME_DEADLINE_S * 1000
      ? { status: FEED_STATUS.FAILED, reason: FEED_FAILURE.NO_FIRST_FRAME }
      : { status: FEED_STATUS.CONNECTING };
  }
  const since = (now - lastFrameAt) / 1000;
  if (since >= VIDEO_STALL_DEADLINE_S) return { status: FEED_STATUS.FAILED, reason: FEED_FAILURE.STALLED };
  if (since >= VIDEO_STALE_AFTER_S) return { status: FEED_STATUS.STALE, seconds: Math.floor(since) };
  return { status: FEED_STATUS.LIVE };
}
