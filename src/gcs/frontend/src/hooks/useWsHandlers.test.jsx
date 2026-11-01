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

  it('keys confirm events by UAV, POI and round', () => {
    const { handlers, taskAssign } = setup();

    act(() => handlers.task_confirm_request({ sys_id: 2, task_id: 5, round_uid: '3:40' }));
    act(() => handlers.task_confirm_response({
      sys_id: 2, task_id: 5, round_uid: '3:40', is_confirmed: true,
    }));
    act(() => handlers.task_confirm_response({ sys_id: 3, task_id: 6, is_confirmed: false }));

    expect(taskAssign.handleConfirmingStatus).toHaveBeenCalledExactlyOnceWith(2, 5, '3:40');
    expect(taskAssign.handleConfirmedStatusByTask).toHaveBeenCalledExactlyOnceWith(2, 5, '3:40');
    expect(taskAssign.handleResolvedCleanupByTask).toHaveBeenCalledExactlyOnceWith(3, 6, undefined);
  });
});

describe('navpy sim status', () => {
  it('keeps the closest-approach records beside the companion status, one new list per status', () => {
    const messageHandlersRef = { current: {} };
    const { result } = renderHook(() => useWsHandlers({
      messageHandlersRef, taskConfirm: {}, taskAssign: {}, setUploadProgress: vi.fn(),
    }));
    expect(result.current.simClosestApproaches).toBeNull();
    const records = [{ key: 'b:1:1', sys_id: 2 }];
    const status = { type: 'navpy_sim_status', instances: [{ sys_id: 2, running: true }], closest_approaches: records };

    act(() => messageHandlersRef.current.navpy_sim_status(status));
    const first = result.current.simClosestApproaches;
    act(() => messageHandlersRef.current.navpy_sim_status(status));

    expect(result.current.navpyStatus).toEqual({ 2: { sys_id: 2, running: true } });
    expect(result.current.simClosestApproaches).toEqual(records);
    expect(result.current.simClosestApproaches).not.toBe(first);
    act(() => messageHandlersRef.current.navpy_sim_status({ instances: [] }));
    expect(result.current.simClosestApproaches).toEqual([]);
  });
});
