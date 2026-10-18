// FPV hold reasons (docs/design/gcs_video_streams.md section 5 "FPV").
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
    expect(fpvHoldReason(linked)).toBeNull();
  });
  it('link first, then pose', () => {
    expect(fpvHoldReason({ ...linked, link_ok: false })).toBe('linkLost');
    expect(fpvHoldReason({ ...linked, lat: NaN })).toBe('poseUnavailable');
  });
});
