import { describe, it, expect, vi, beforeEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useSimAutoConnect, { AUTO_CONNECT_ATTEMPTS, AUTO_CONNECT_RETRY_MS } from './useSimAutoConnect';

const base = { enabled: true, simMode: true, device: 'udp:sitl' };

beforeEach(() => {
  vi.useFakeTimers();
});

// Let the awaited scan settle, then run the next retry timer.
const nextAttempt = async () => {
  await act(async () => { await vi.advanceTimersByTimeAsync(AUTO_CONNECT_RETRY_MS); });
};

describe('useSimAutoConnect', () => {
  it('retries while SITL has no vehicles and stops once a scan finds them', async () => {
    const scan = vi.fn()
      .mockResolvedValueOnce({ found: 0 })
      .mockResolvedValueOnce({ found: 3 });
    renderHook(() => useSimAutoConnect({ ...base, scan, vehicleCount: 0 }));
    await nextAttempt();
    await nextAttempt();
    await nextAttempt();
    expect(scan).toHaveBeenCalledTimes(2);
  });

  it('stops scanning once the fleet is connected although scans find 0', async () => {
    // The backend auto-connected the fleet; discover skips connected vehicles.
    const scan = vi.fn().mockResolvedValue({ found: 0 });
    const { rerender } = renderHook(
      (props) => useSimAutoConnect({ ...base, scan, ...props }),
      { initialProps: { vehicleCount: 0 } },
    );
    await act(async () => { await Promise.resolve(); });
    expect(scan).toHaveBeenCalledTimes(1);
    rerender({ vehicleCount: 3 });
    await nextAttempt();
    await nextAttempt();
    expect(scan).toHaveBeenCalledTimes(1);
  });

  it('does not scan at all when the fleet is already connected', async () => {
    const scan = vi.fn().mockResolvedValue({ found: 0 });
    renderHook(() => useSimAutoConnect({ ...base, scan, vehicleCount: 3 }));
    await nextAttempt();
    expect(scan).not.toHaveBeenCalled();
  });

  it('gives up after the retry budget', async () => {
    const scan = vi.fn().mockResolvedValue({ found: 0 });
    renderHook(() => useSimAutoConnect({ ...base, scan, vehicleCount: 0 }));
    for (let i = 0; i < AUTO_CONNECT_ATTEMPTS + 3; i += 1) await nextAttempt();
    expect(scan).toHaveBeenCalledTimes(AUTO_CONNECT_ATTEMPTS);
  });

  it('does nothing outside sim mode', async () => {
    const scan = vi.fn();
    renderHook(() => useSimAutoConnect({ ...base, simMode: false, scan, vehicleCount: 0 }));
    await nextAttempt();
    expect(scan).not.toHaveBeenCalled();
  });
});
