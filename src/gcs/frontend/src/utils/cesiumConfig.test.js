import { describe, it, expect, vi, afterEach } from 'vitest';
import { parseIonToken, fetchCesiumIonToken, MAP_CONFIG_TIMEOUT_MS } from './cesiumConfig';

const okJson = (body) => vi.fn(async () => ({ ok: true, json: async () => body }));

describe('parseIonToken', () => {
  it('is null when unconfigured, so the map has no Ion layers', () => {
    expect(parseIonToken({ cesium_ion_token: null })).toBeNull();
    expect(parseIonToken({ cesium_ion_token: '   ' })).toBeNull();
    expect(parseIonToken({})).toBeNull();
    expect(parseIonToken(null)).toBeNull();
  });

  it('returns the trimmed token', () => {
    expect(parseIonToken({ cesium_ion_token: ' abc.def ' })).toBe('abc.def');
  });
});

describe('fetchCesiumIonToken', () => {
  afterEach(() => { vi.unstubAllGlobals(); vi.useRealTimers(); });
  const answer = (fetchImpl) => { vi.stubGlobal('fetch', fetchImpl); return fetchCesiumIonToken(); };

  it('reads the token from the backend map config', async () => {
    const fetchImpl = okJson({ cesium_ion_token: 'abc.def' });
    await expect(answer(fetchImpl)).resolves.toBe('abc.def');
    expect(fetchImpl).toHaveBeenCalledWith('/api/map-config', expect.objectContaining({
      signal: expect.any(AbortSignal),
    }));
  });

  it('is null on an error status or an unreachable backend', async () => {
    await expect(answer(vi.fn(async () => ({ ok: false, status: 404 })))).resolves.toBeNull();
    await expect(answer(vi.fn(async () => { throw new TypeError('refused'); }))).resolves.toBeNull();
  });

  it('gives up after the timeout instead of holding the map back', async () => {
    vi.useFakeTimers();
    const hung = vi.fn((url, { signal }) => new Promise((_, reject) => {
      signal.addEventListener('abort', () => reject(signal.reason));
    }));
    const result = answer(hung);
    await vi.advanceTimersByTimeAsync(MAP_CONFIG_TIMEOUT_MS);
    await expect(result).resolves.toBeNull();
  });
});
