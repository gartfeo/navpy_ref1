import { describe, it, expect } from 'vitest';
import {
  codecSupported, defaultStream, pickFirstFeed, resolveShownStream, streamsOf, vehicleLabel,
} from './videoStreams';
import { fpvEligible, fpvHoldReason, pickFpvTarget, poseFinite } from './fpvView';
import { FEED_FAILURE, FEED_STATUS, feedTiming } from './videoFeed';
import {
  VIDEO_FIRST_FRAME_DEADLINE_S, VIDEO_STALE_AFTER_S, VIDEO_STALL_DEADLINE_S,
} from '../constants/video';

const stream = (camera, id, extra = {}) => ({
  key: `${camera}:${id}`, camera_component: camera, stream_id: id, name: `cam ${camera}/${id}`,
  encoding: 'H264', whep_url: `http://192.168.144.10:8889/c${camera}s${id}/whep`, ...extra,
});
const entry = (sysId, streams = [], extra = {}) => ({ sysId, index: sysId - 1, label: `UAV ${sysId}`, streams, linkOk: true, fpvEligible: true, ...extra });

describe('video stream helpers', () => {
  it('names the design constants', () => {
    expect(VIDEO_FIRST_FRAME_DEADLINE_S).toBe(10);
    expect(VIDEO_STALL_DEADLINE_S).toBe(5);
  });

  it('treats an absent or malformed field as no streams', () => {
    expect(streamsOf({})).toEqual([]);
    expect(streamsOf({ video_streams: null })).toEqual([]);
    expect(streamsOf({ video_streams: [{ key: '100:1', whep_url: '' }] })).toEqual([]);
    expect(vehicleLabel({ sys_id: 3 })).toBe('UAV 3');
    expect(vehicleLabel({ sys_id: 3, name: 'Alpha' })).toBe('Alpha');
  });

  it('plays the lowest stream id, the lower camera on a tie', () => {
    expect(defaultStream([stream(100, 2), stream(101, 1)]).key).toBe('101:1');
    expect(defaultStream([stream(100, 1), stream(101, 1)]).key).toBe('100:1');
    expect(defaultStream([])).toBeNull();
  });

  it('first feed: the followed UAV if it has video, else the first UAV with video', () => {
    const fleet = [entry(1), entry(2, [stream(100, 1)]), entry(3, [stream(100, 1)])];
    expect(pickFirstFeed(3, fleet)).toBe(3);
    expect(pickFirstFeed(1, fleet)).toBe(2);
    expect(pickFirstFeed(null, fleet)).toBe(2);
    expect(pickFirstFeed(null, [entry(1)])).toBeNull();
  });

  it('a vanished stream falls back to the same UAV, never another', () => {
    const fleet = [entry(1, [stream(101, 1)]), entry(2, [stream(100, 1)])];
    const switched = resolveShownStream({ sysId: 1, key: '100:1' }, fleet);
    expect(switched).toMatchObject({ kind: 'stream', switched: true });
    expect(switched.stream.key).toBe('101:1');

    const lost = resolveShownStream({ sysId: 1, key: '100:1' }, [entry(1), entry(2, [stream(100, 1)])]);
    expect(lost.kind).toBe('lost');
    expect(lost.stream).toBeUndefined();
    expect(resolveShownStream(null, fleet).kind).toBe('idle');
  });

  it('checks the announced codec against the receiver capabilities', () => {
    const h264Only = { getCapabilities: () => ({ codecs: [{ mimeType: 'video/H264' }] }) };
    expect(codecSupported('H264', h264Only)).toBe(true);
    expect(codecSupported('H265', h264Only)).toBe(false);
    // Unknown or missing encoding, or no capability API: try.
    expect(codecSupported(null, h264Only)).toBe(true);
    expect(codecSupported('VP9', h264Only)).toBe(true);
    expect(codecSupported('H265', undefined)).toBe(true);
  });
});

describe('FPV eligibility', () => {
  const linked = { link_ok: true, lat: 32, lon: 34, alt_rel: 100, roll: 1, pitch: 2, heading: 90 };

  it('is link_ok plus a finite position and attitude', () => {
    expect(fpvEligible(linked)).toBe(true);
    expect(fpvHoldReason({ ...linked, link_ok: false })).toBe('linkLost');
    expect(fpvHoldReason(undefined)).toBe('linkLost');
    expect(fpvHoldReason({ ...linked, lat: null })).toBe('poseUnavailable');
    expect(fpvHoldReason({ ...linked, roll: NaN })).toBe('poseUnavailable');
    expect(poseFinite({ ...linked, alt_rel: undefined, alt: 400 })).toBe(true);
    // Link first, even when the pose is also missing.
    expect(fpvHoldReason({ ...linked, link_ok: false, heading: null })).toBe('linkLost');
  });

  it('picks the followed UAV, else from Free the panel UAV, else the first eligible', () => {
    const fleet = [entry(1, [], { fpvEligible: false }), entry(2), entry(3)];
    expect(pickFpvTarget({ followSysId: 3, panelSysId: 2, fleet })).toEqual({ sysId: 3 });
    expect(pickFpvTarget({ followSysId: 1, panelSysId: 2, fleet })).toEqual({ ineligible: 1 });
    expect(pickFpvTarget({ followSysId: null, panelSysId: 3, fleet })).toEqual({ sysId: 3 });
    expect(pickFpvTarget({ followSysId: null, panelSysId: 1, fleet })).toEqual({ sysId: 2 });
    expect(pickFpvTarget({ followSysId: null, panelSysId: null, fleet: [entry(1, [], { fpvEligible: false })] })).toEqual({ none: true });
  });
});

describe('feed timing from frame progress', () => {
  const t0 = 1_000_000;
  it('connecting until the first-frame deadline, then failed', () => {
    const at = (s) => feedTiming({ startedAt: t0, lastFrameAt: null, now: t0 + s * 1000 });
    expect(at(VIDEO_FIRST_FRAME_DEADLINE_S - 0.001).status).toBe(FEED_STATUS.CONNECTING);
    expect(at(VIDEO_FIRST_FRAME_DEADLINE_S)).toEqual({ status: FEED_STATUS.FAILED, reason: FEED_FAILURE.NO_FIRST_FRAME });
  });

  it('live while frames arrive, stale with whole seconds, failed at the stall deadline', () => {
    const since = (s) => feedTiming({ startedAt: t0, lastFrameAt: t0, now: t0 + s * 1000 });
    expect(since(VIDEO_STALE_AFTER_S - 0.001).status).toBe(FEED_STATUS.LIVE);
    expect(since(VIDEO_STALE_AFTER_S)).toEqual({ status: FEED_STATUS.STALE, seconds: 1 });
    expect(since(VIDEO_STALL_DEADLINE_S - 0.001)).toEqual({ status: FEED_STATUS.STALE, seconds: 4 });
    expect(since(VIDEO_STALL_DEADLINE_S)).toEqual({ status: FEED_STATUS.FAILED, reason: FEED_FAILURE.STALLED });
  });
});
