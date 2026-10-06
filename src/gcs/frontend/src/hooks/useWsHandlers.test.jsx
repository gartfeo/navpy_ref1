import { describe, it, expect, vi } from 'vitest';
import { act, renderHook } from '@testing-library/react';
import useWsHandlers from './useWsHandlers';

describe('legacy completion event handling', () => {
  it('handles the event without a visual-effect dependency or timer', () => {
    const messageHandlersRef = { current: {} };
    const cleanup = vi.fn();
    const clearCard = vi.fn();
    const timeout = vi.spyOn(globalThis, 'setTimeout');
    renderHook(() => useWsHandlers({
      messageHandlersRef,
      taskConfirm: { handleDisarmAfterGuided: clearCard },
      taskAssign: { handleDisarmAfterGuidedCleanup: cleanup },
      setUploadProgress: vi.fn(),
    }));
    timeout.mockClear();

    const event = { type: 'disarm_after_guided', sys_id: 1, lat: 0, lon: 0 };
    act(() => messageHandlersRef.current.disarm_after_guided(event));

    expect(cleanup).toHaveBeenCalledExactlyOnceWith(event);
    expect(clearCard).toHaveBeenCalledExactlyOnceWith(event);
    expect(timeout).not.toHaveBeenCalled();
  });
});
