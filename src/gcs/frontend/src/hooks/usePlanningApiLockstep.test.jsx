/**
 * On a lockstep run no wall timer decides what the page sends next (dev sim
 * step 11): the connect and mission-download limits travel as the run's
 * sim-time deadline_s, and only the run's own deadline answer (504) counts as
 * the page's timeout. Off lockstep the wall abort still frees the page.
 */
import { act, renderHook } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import usePlanningApi from './usePlanningApi';

function api() {
  return renderHook(() => usePlanningApi({ setAnalysis: () => {}, setPlan: () => {} })).result.current;
}

// A fetch that answers only when the test says, and records each request.
function heldFetch() {
  const calls = [];
  const fetch = vi.fn((url, options = {}) => new Promise((resolve, reject) => {
    calls.push({ url, options, resolve });
    options.signal?.addEventListener('abort', () => reject(new DOMException('aborted', 'AbortError')));
  }));
  return { fetch, calls, backend: () => calls.filter((c) => !String(c.url).includes('/api/diagnostics')) };
}

const reply = (status, body = {}) => ({ ok: status < 300, status, json: async () => body });

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllEnvs();
  vi.unstubAllGlobals();
});

describe('usePlanningApi on a lockstep run', () => {
  it.each([
    ['connect', (a) => a.connectVehicle('lockstep:router', 1), 20],
    ['mission download', (a) => a.downloadMission(1), 45],
  ])('%s: wall time past the limit aborts nothing; the sim deadline is the timeout', async (_name, call, limitS) => {
    vi.stubEnv('VITE_NAVPY_LOCKSTEP', '1');
    vi.useFakeTimers();
    const held = heldFetch();
    vi.stubGlobal('fetch', held.fetch);
    let result;
    const pending = call(api()).then((r) => { result = r; });
    await act(async () => { await vi.advanceTimersByTimeAsync(limitS * 1000 * 3); });
    const [request] = held.backend();
    expect(request.options.signal.aborted).toBe(false);
    expect(request.options.headers['X-NavPy-Deadline-S']).toBe(String(limitS));
    expect(result).toBeUndefined();
    await act(async () => { request.resolve(reply(504, { detail: 'sim-time deadline passed' })); await pending; });
    expect(result.error).toBe('timeout');
    expect(held.backend()).toHaveLength(1); // no retry
  });
});

describe('usePlanningApi off lockstep', () => {
  it('the wall abort still ends a hung connect', async () => {
    vi.useFakeTimers();
    const held = heldFetch();
    vi.stubGlobal('fetch', held.fetch);
    let result;
    const pending = api().connectVehicle('udp:x', 1).then((r) => { result = r; });
    await act(async () => { await vi.advanceTimersByTimeAsync(20000); await pending; });
    expect(result.error).toBe('timeout');
  });

  it('a 504 from the backend is an ordinary error', async () => {
    const held = heldFetch();
    vi.stubGlobal('fetch', held.fetch);
    const pending = api().connectVehicle('udp:x', 1);
    await act(async () => { held.backend()[0].resolve(reply(504, { detail: 'gateway' })); });
    expect((await pending).error).toBe('gateway');
  });
});
