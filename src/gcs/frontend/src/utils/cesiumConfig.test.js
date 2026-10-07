import { describe, it, expect } from 'vitest';
import { cesiumIonToken } from './cesiumConfig';

describe('cesiumIonToken', () => {
  it('is null when unconfigured, so the map stays on OSM', () => {
    expect(cesiumIonToken({})).toBeNull();
    expect(cesiumIonToken({ VITE_CESIUM_ION_TOKEN: '   ' })).toBeNull();
    expect(cesiumIonToken(null)).toBeNull();
  });

  it('returns the trimmed build-time token when configured', () => {
    expect(cesiumIonToken({ VITE_CESIUM_ION_TOKEN: ' abc.def ' })).toBe('abc.def');
  });
});
