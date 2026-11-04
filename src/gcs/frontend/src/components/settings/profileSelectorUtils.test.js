import { describe, expect, it, vi } from 'vitest';
import { putVisionProfile } from './profileSelectorUtils.js';

const answer = (status, body) => vi.fn(async () => ({
  ok: status >= 200 && status < 300,
  status,
  json: async () => body,
}));

describe('putVisionProfile', () => {
  it('returns the catalog of an ok answer and sends the body as JSON', async () => {
    const fetchFn = answer(200, { profiles: { sim: {} } });
    await expect(putVisionProfile('/api/vision-profiles/sim', { a: 1 }, fetchFn))
      .resolves.toEqual({ profiles: { sim: {} } });
    expect(fetchFn).toHaveBeenCalledWith('/api/vision-profiles/sim', {
      method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: '{"a":1}',
    });
  });

  it('sends no body when there is none', async () => {
    const fetchFn = answer(200, {});
    await putVisionProfile('/api/vision-profiles/default/sim', undefined, fetchFn);
    expect(fetchFn).toHaveBeenCalledWith('/api/vision-profiles/default/sim', { method: 'PUT' });
  });

  it('throws the detail of a refusal, so its body never becomes the catalog', async () => {
    const fetchFn = answer(501, { detail: 'not supported in simulation: writes vision_profiles.json' });
    await expect(putVisionProfile('/api/vision-profiles/sim', {}, fetchFn))
      .rejects.toThrow('not supported in simulation: writes vision_profiles.json');
  });

  it('throws the status when an error has no JSON detail', async () => {
    const fetchFn = vi.fn(async () => ({ ok: false, status: 502, json: async () => { throw new Error('html'); } }));
    await expect(putVisionProfile('/x', {}, fetchFn)).rejects.toThrow('HTTP 502');
  });
});
