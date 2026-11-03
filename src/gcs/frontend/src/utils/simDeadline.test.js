import { afterEach, describe, expect, it, vi } from 'vitest';
import fetchWithRetry from './fetchRetry';
import { DEADLINE_HEADER, deadlineHeaders, isLockstepRun } from './simDeadline';

afterEach(() => {
  vi.unstubAllEnvs();
  vi.unstubAllGlobals();
});

describe('simDeadline', () => {
  it('carries the timeout in seconds', () => {
    expect(deadlineHeaders(20000)).toEqual({ [DEADLINE_HEADER]: '20' });
  });

  it('knows a lockstep page only by the flag the app stack sets', () => {
    expect(isLockstepRun({})).toBe(false);
    expect(isLockstepRun({ VITE_NAVPY_LOCKSTEP: '0' })).toBe(false);
    expect(isLockstepRun({ VITE_NAVPY_LOCKSTEP: '1' })).toBe(true);
  });
});

describe('fetchWithRetry on a lockstep run', () => {
  const failing = () => vi.fn(async () => ({ ok: false, status: 503 }));

  it('retries a server error off lockstep', async () => {
    const fetch = failing();
    vi.stubGlobal('fetch', fetch);
    await fetchWithRetry('/x', {}, { maxRetries: 1, baseDelay: 0 });
    expect(fetch).toHaveBeenCalledTimes(2);
  });

  it('sends once on lockstep: no wall delay decides a resend', async () => {
    vi.stubEnv('VITE_NAVPY_LOCKSTEP', '1');
    const fetch = failing();
    vi.stubGlobal('fetch', fetch);
    const res = await fetchWithRetry('/x', {}, { maxRetries: 2, baseDelay: 0 });
    expect(res.status).toBe(503);
    expect(fetch).toHaveBeenCalledTimes(1);
  });
});
