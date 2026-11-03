// The app's clock for anything it sends: the backend's telemetry `ts`
// (deterministic_dev_sim.md section 5). On a lockstep run that is sim time.
import { afterEach, describe, expect, it, vi } from 'vitest';
import { backendNowMs, noteBackendTs, resetBackendClock } from './backendClock';
import { confirmNow } from './confirmClock';
import { DEADLINE_HEADER, deadlineHeaders } from './simDeadline';

describe('backend clock', () => {
  afterEach(() => { resetBackendClock(); vi.useRealTimers(); });

  it('is the page clock until the first telemetry ts, then the backend ts only', () => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date('2026-11-03T12:00:00Z'));
    expect(backendNowMs()).toBe(Date.now());
    noteBackendTs(1767225600.5); // sim time: 2026-01-01
    vi.advanceTimersByTime(5000); // wall time passes; the run has not
    expect(backendNowMs()).toBe(1767225600500);
    expect(confirmNow()).toBe(1767225600500);
    noteBackendTs(undefined);
    noteBackendTs(Number.NaN);
    expect(backendNowMs()).toBe(1767225600500);
    noteBackendTs(1767225601);
    expect(confirmNow()).toBe(1767225601000);
  });

  it('carries a fetch time limit as the sim-time deadline header', () => {
    expect(deadlineHeaders(45000)).toEqual({ [DEADLINE_HEADER]: '45' });
  });
});
