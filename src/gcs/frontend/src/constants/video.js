/**
 * Timing of the monitor-view camera panel (docs/design/gcs_video_streams.md,
 * section 5 "Camera panel"). Every value is measured from frame progress
 * (requestVideoFrameCallback, else getVideoPlaybackQuality), not from the video
 * element's waiting/stalled events, which do not fire for every freeze.
 */

/**
 * No first frame this long after the session starts = failed. ICE setup plus
 * one keyframe interval; the display stream keeps that interval at 2 s or less.
 */
export const VIDEO_FIRST_FRAME_DEADLINE_S = 10;

/** No new frame for this long after frames were arriving = failed. A few keyframe intervals. */
export const VIDEO_STALL_DEADLINE_S = 5;

/**
 * The picture is called stale ("NO FRAMES N s") from the first whole second
 * without a new frame. One second is the resolution of that counter: at the
 * display stream's frame rate it is dozens of missed frames, far above jitter.
 */
export const VIDEO_STALE_AFTER_S = 1;

/**
 * The panel re-reads frame progress once per second: it drives the "N s"
 * counter and the two deadlines above, all of which are whole seconds.
 */
export const VIDEO_CLOCK_TICK_MS = 1000;
