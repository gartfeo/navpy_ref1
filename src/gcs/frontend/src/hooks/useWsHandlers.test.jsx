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

describe('task assignment routing', () => {
  function setup() {
    const messageHandlersRef = { current: {} };
    const taskConfirm = {
      handleConfirmRequest: vi.fn(),
      handleConfirmResponse: vi.fn(),
    };
    const taskAssign = {
      handleAssignAck: vi.fn(),
      handleConfirmingStatus: vi.fn(),
      handleConfirmedStatusByTask: vi.fn(),
      handleResolvedCleanupByTask: vi.fn(),
    };
    renderHook(() => useWsHandlers({
      messageHandlersRef, taskConfirm, taskAssign, setUploadProgress: vi.fn(),
    }));
    return { handlers: messageHandlersRef.current, taskAssign };
  }

  it('routes the owner APPLIED to the assignment state', () => {
    const { handlers, taskAssign } = setup();
    const ack = { type: 'task_assign_ack', owner_id: 1, helper_id: 2, task_id: 5 };

    act(() => handlers.task_assign_ack(ack));

    expect(taskAssign.handleAssignAck).toHaveBeenCalledExactlyOnceWith(ack);
  });

  it('keys confirm events by UAV and task', () => {
    const { handlers, taskAssign } = setup();

    act(() => handlers.task_confirm_request({ sys_id: 2, task_id: 5 }));
    act(() => handlers.task_confirm_response({ sys_id: 2, task_id: 5, is_confirmed: true }));
    act(() => handlers.task_confirm_response({ sys_id: 3, task_id: 6, is_confirmed: false }));

    expect(taskAssign.handleConfirmingStatus).toHaveBeenCalledExactlyOnceWith(2, 5);
    expect(taskAssign.handleConfirmedStatusByTask).toHaveBeenCalledExactlyOnceWith(2, 5);
    expect(taskAssign.handleResolvedCleanupByTask).toHaveBeenCalledExactlyOnceWith(3, 6);
  });
});
