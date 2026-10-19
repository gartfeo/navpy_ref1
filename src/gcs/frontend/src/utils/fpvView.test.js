// FPV hold reasons (docs/design/gcs_video_streams.md section 5 "Synthetic view").
import { describe, it, expect } from 'vitest';
import { fleetEntryHoldReason, fpvHoldReason } from './fpvView';

const linked = { link_ok: true, lat: 40, lon: 44, alt: 100, roll: 0, pitch: 0, heading: 0 };

describe('fleetEntryHoldReason', () => {
  it('a live entry (holdReason null) shows no banner', () => {
    expect(fleetEntryHoldReason({ holdReason: null })).toBeNull();
  });
  it('keeps the entry reason', () => {
    expect(fleetEntryHoldReason({ holdReason: 'poseUnavailable' })).toBe('poseUnavailable');
  });
  it('a UAV no longer in the fleet is link lost', () => {
    expect(fleetEntryHoldReason(undefined)).toBe('linkLost');
  });
});

describe('fpvHoldReason', () => {
  it('a linked vehicle with a finite pose is live', () => {
    expect(fpvHoldReason(linked, true)).toBeNull();
  });
  it('a snapshot that is not current holds first', () => {
    expect(fpvHoldReason(linked, false)).toBe('telemetryDisconnected');
  });
  it('link first, then pose', () => {
    expect(fpvHoldReason({ ...linked, link_ok: false }, true)).toBe('linkLost');
    expect(fpvHoldReason({ ...linked, lat: NaN }, true)).toBe('poseUnavailable');
  });
});
