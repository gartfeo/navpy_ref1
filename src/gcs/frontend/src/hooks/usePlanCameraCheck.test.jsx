import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { usePlanCameraCheckResult, usePlanCameraGate } from './usePlanCameraCheck';

const track = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }];
const plan = (altitude_m) => ({ altitude_m, zones: [{ track }] });
const getAas = (_sysId, name) => ({ targ_alt: 0, use_trn: false })[name];
const vehicleList = [{ sys_id: 5 }];

describe('usePlanCameraGate', () => {
  it('checks the mission held now, not the one at first render', async () => {
    const check = vi.fn().mockResolvedValue({ status: 'ok', zones: [] });
    const props = {
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      vehicleTargWps: { 5: 0b10 }, vehicleNavLastWp: {}, getAas, check,
    };
    const { result, rerender } = renderHook((p) => usePlanCameraGate(p), { initialProps: props });
    rerender({ ...props, plan: plan(403), vehicleTargWps: { 5: 0b1 } });
    let ok;
    await act(async () => { ok = await result.current(); });
    expect(ok).toBe(true);
    expect(check).toHaveBeenCalledWith([expect.objectContaining({
      altitude_m: 403, docks: [{ ...track[0], wp: 1 }],
    })]);
  });

  it('goes ahead without a check when disabled', async () => {
    const check = vi.fn();
    const { result } = renderHook(() => usePlanCameraGate({ enabled: false, check }));
    expect(await result.current()).toBe(true);
    expect(check).not.toHaveBeenCalled();
  });
});

describe('usePlanCameraCheckResult', () => {
  it('checks once per edit burst and keeps the latest answer', async () => {
    vi.useFakeTimers();
    const check = vi.fn((zones) => Promise.resolve({ status: 'ok', zones: [], alt: zones[0].altitude_m }));
    const props = {
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      simDockWps: { 0: [0] }, detectAfterWps: {}, getAas, check,
    };
    const { result, rerender } = renderHook((p) => usePlanCameraCheckResult(p), { initialProps: props });
    rerender({ ...props, plan: plan(250) });
    rerender({ ...props, plan: plan(300) });
    await act(async () => { await vi.runAllTimersAsync(); });
    expect(check).toHaveBeenCalledOnce();
    expect(result.current.alt).toBe(300);
  });

  it('does not re-check when inputs are rebuilt with the same content', async () => {
    vi.useFakeTimers();
    const check = vi.fn().mockResolvedValue({ status: 'ok', zones: [] });
    const props = {
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      simDockWps: { 0: [0] }, detectAfterWps: {}, getAas, check,
    };
    const { rerender } = renderHook((p) => usePlanCameraCheckResult(p), { initialProps: props });
    await act(async () => { await vi.runAllTimersAsync(); });
    // Telemetry and param updates hand in new arrays and getters with equal values.
    rerender({ ...props, plan: plan(200), vehicleList: [{ sys_id: 5 }], getAas: (s, n) => getAas(s, n) });
    await act(async () => { await vi.runAllTimersAsync(); });
    expect(check).toHaveBeenCalledOnce();
  });

  function deferred() {
    let resolve; let reject;
    const promise = new Promise((res, rej) => { resolve = res; reject = rej; });
    return { promise, resolve, reject };
  }

  it.each(['resolves', 'rejects'])('ignores an older answer that %s after a newer one', async (late) => {
    vi.useFakeTimers();
    const calls = [];
    const check = vi.fn(() => { const d = deferred(); calls.push(d); return d.promise; });
    const props = {
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      simDockWps: { 0: [0] }, detectAfterWps: {}, getAas, check,
    };
    const { result, rerender } = renderHook((p) => usePlanCameraCheckResult(p), { initialProps: props });
    await act(async () => { await vi.advanceTimersByTimeAsync(500); });
    rerender({ ...props, plan: plan(300) });
    await act(async () => { await vi.advanceTimersByTimeAsync(500); });
    expect(calls).toHaveLength(2);
    await act(async () => { calls[1].resolve({ status: 'ok', zones: [], tag: 'new' }); });
    await act(async () => {
      if (late === 'resolves') calls[0].resolve({ status: 'ok', zones: [], tag: 'old' });
      else calls[0].reject(new Error('old'));
    });
    expect(result.current.tag).toBe('new');
  });

  it('re-checks when only the camera changes', async () => {
    vi.useFakeTimers();
    const check = vi.fn().mockResolvedValue({ status: 'ok', zones: [] });
    const props = {
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      simDockWps: { 0: [0] }, detectAfterWps: {}, getAas, check, cameraKey: 'a',
    };
    const { rerender } = renderHook((p) => usePlanCameraCheckResult(p), { initialProps: props });
    await act(async () => { await vi.runAllTimersAsync(); });
    rerender({ ...props, cameraKey: 'b' });
    await act(async () => { await vi.runAllTimersAsync(); });
    expect(check).toHaveBeenCalledTimes(2);
  });

  it('shows an unavailable check as could-not-run', async () => {
    vi.useFakeTimers();
    const check = vi.fn().mockResolvedValue({ status: 'skipped', reason: 'no_profile', zones: [] });
    const { result } = renderHook(() => usePlanCameraCheckResult({
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      simDockWps: { 0: [0] }, detectAfterWps: {}, getAas, check,
    }));
    await act(async () => { await vi.runAllTimersAsync(); });
    expect(result.current).toMatchObject({ status: 'error', error: 'no_profile' });
  });

  it('is null without sim docks', () => {
    const check = vi.fn();
    const { result } = renderHook(() => usePlanCameraCheckResult({
      enabled: true, plan: plan(200), searchPattern: 'corridor', vehicleList,
      simDockWps: {}, detectAfterWps: {}, getAas, check,
    }));
    expect(result.current).toBeNull();
    expect(check).not.toHaveBeenCalled();
  });
});
