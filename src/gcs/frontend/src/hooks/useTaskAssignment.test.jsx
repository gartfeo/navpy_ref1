import { beforeEach, describe, expect, it, vi } from 'vitest';
import { act, renderHook } from '@testing-library/react';
import useTaskAssignment from './useTaskAssignment';

// The hook reads each vehicle's swarm heartbeat from the telemetry store.
const telemetry = vi.hoisted(() => ({ listeners: new Set(), vehicles: [], connected: true }));
vi.mock('../stores/telemetryStore', () => ({
  default: {
    subscribe: (listener) => {
      telemetry.listeners.add(listener);
      return () => telemetry.listeners.delete(listener);
    },
    getVehicleList: () => telemetry.vehicles,
    isConnected: () => telemetry.connected,
  },
}));

const uid = (boot_id, msg_seq) => ({ boot_id, msg_seq });

function publish(vehicles) {
  telemetry.vehicles = vehicles;
  act(() => {
    for (const listener of telemetry.listeners) listener();
  });
}

const beat = (sysId, seq, phase, outcome) => ({
  sys_id: sysId, swarm: { state: 'BUSY', phase, outcome, boot: 3, seq, stale: false },
});

function askOwnDock(result, sysId = 2) {
  act(() => result.current.handleConfirmingStatus({
    sys_id: sysId, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41, lon: 45, alt: 0,
  }));
}

function approveOwnDock(result, sysId = 2) {
  askOwnDock(result, sysId);
  act(() => result.current.handleConfirmedStatusByTask(sysId, 7, '3:60'));
}

function offerAndAccept(result) {
  act(() => result.current.handleAssignRequest({
    sender_id: 1, receiver_id: 2, task_id: 5, task_type: 'DOCK',
    lat: 40, lon: 44, alt: 0, uid: uid(7, 20),
  }));
  act(() => result.current.handleAssignResponse({
    sender_id: 2, receiver_id: 1, task_id: 5, is_accepted: true, uid: uid(3, 40),
  }));
}

describe('useTaskAssignment', () => {
  beforeEach(() => {
    telemetry.listeners.clear();
    telemetry.vehicles = [];
    telemetry.connected = true;
  });

  it('shows the helper waiting until the owner applies its answer', () => {
    const { result } = renderHook(() => useTaskAssignment());

    offerAndAccept(result);
    expect(result.current.assignments['1:5'].status).toBe('waiting');

    act(() => result.current.handleAssignAck({
      owner_id: 1, helper_id: 2, task_id: 5, status: 'APPLIED',
      ref: uid(3, 40), uid: uid(7, 90),
    }));
    expect(result.current.assignments['1:5']).toBeUndefined();
    expect(result.current.assignments['helper:2'].status).toBe('assigned');
  });

  it('retires a round once its helper reports FREE after its answer', () => {
    const { result } = renderHook(() => useTaskAssignment());
    offerAndAccept(result);

    publish([{ sys_id: 2, swarm: { state: 'FREE', boot: 3, seq: 39, stale: false } }]);
    expect(result.current.assignments['1:5']).toBeDefined();

    publish([{ sys_id: 2, swarm: { state: 'FREE', boot: 3, seq: 41, stale: false } }]);
    expect(result.current.assignments['1:5']).toBeUndefined();
  });

  it('takes a UAV\'s own dock with its position, approved, then handled by phase beats', () => {
    const { result } = renderHook(() => useTaskAssignment());
    act(() => result.current.handleConfirmingStatus({
      sys_id: 2, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41, lon: 45, alt: 0,
    }));
    expect(result.current.assignments['own:2:3:60'].status).toBe('confirming');
    act(() => result.current.handleConfirmedStatusByTask(2, 7, '3:60'));
    expect(result.current.assignments['own:2:3:60'].status).toBe('confirmed');

    publish([{ sys_id: 2, swarm: { state: 'BUSY', phase: 'APPROACH', outcome: null, boot: 3, seq: 61, stale: false } }]);
    publish([{ sys_id: 2, swarm: { state: 'BUSY', phase: 'HOLD', outcome: 'docked', boot: 3, seq: 62, stale: false } }]);

    expect(result.current.assignments['own:2:3:60']).toBeUndefined();
    expect(result.current.assignments['handled:2:3:60'].outcome).toBe('docked');
  });

  it('takes the outcome word away from a UAV whose beats went stale', () => {
    const { result } = renderHook(() => useTaskAssignment());
    for (const sysId of [2, 3]) {
      act(() => result.current.handleConfirmingStatus({
        sys_id: sysId, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41 + sysId, lon: 45, alt: 0,
      }));
      act(() => result.current.handleConfirmedStatusByTask(sysId, 7, '3:60'));
      publish([{ sys_id: sysId, swarm: { state: 'BUSY', phase: 'APPROACH', outcome: null, boot: 3, seq: 61, stale: false } }]);
    }
    // The same beat, now past its TTL: nothing new to read, but the gap counts.
    publish([{ sys_id: 2, swarm: { state: 'BUSY', phase: 'APPROACH', outcome: null, boot: 3, seq: 61, stale: true } }]);
    publish([
      { sys_id: 2, swarm: { state: 'BUSY', phase: 'HOLD', outcome: 'docked', boot: 3, seq: 90, stale: false } },
      { sys_id: 3, swarm: { state: 'BUSY', phase: 'HOLD', outcome: 'docked', boot: 3, seq: 90, stale: false } },
    ]);

    expect(result.current.assignments['handled:2:3:60'].outcome).toBe('unknown');
    expect(result.current.assignments['handled:3:3:60'].outcome).toBe('docked');
  });

  it.each([[5001, 'unknown'], [5000, 'docked']])(
    'after beats %i ms apart with no stale snapshot between them, reads %s',
    (gapMs, word) => {
      const now = vi.spyOn(performance, 'now').mockReturnValue(1000);
      const { result } = renderHook(() => useTaskAssignment());
      for (const sysId of [2, 3]) {
        act(() => result.current.handleConfirmingStatus({
          sys_id: sysId, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41 + sysId, lon: 45, alt: 0,
        }));
        act(() => result.current.handleConfirmedStatusByTask(sysId, 7, '3:60'));
      }
      publish([{ sys_id: 2, swarm: { state: 'BUSY', phase: 'APPROACH', outcome: null, boot: 3, seq: 61, stale: false } }]);
      now.mockReturnValue(1000 + gapMs);
      publish([
        { sys_id: 2, swarm: { state: 'BUSY', phase: 'HOLD', outcome: 'docked', boot: 3, seq: 90, stale: false } },
        { sys_id: 3, swarm: { state: 'BUSY', phase: 'APPROACH', outcome: null, boot: 3, seq: 61, stale: false } },
      ]);
      now.mockRestore();

      expect(result.current.assignments['handled:2:3:60'].outcome).toBe(word);
      // Its first beat here is no gap, and another UAV's gap is not its own.
      expect(result.current.assignments['own:3:3:60'].noWord).toBeUndefined();
    },
  );

  it('takes the outcome word away when the socket drops', () => {
    const { result } = renderHook(() => useTaskAssignment());
    act(() => result.current.handleConfirmingStatus({
      sys_id: 2, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41, lon: 45, alt: 0,
    }));
    act(() => result.current.handleConfirmedStatusByTask(2, 7, '3:60'));

    telemetry.connected = false;
    publish([]);
    telemetry.connected = true;
    publish([{ sys_id: 2, swarm: { state: 'BUSY', phase: 'APPROACH', outcome: null, boot: 3, seq: 71, stale: false } }]);
    publish([{ sys_id: 2, swarm: { state: 'BUSY', phase: 'HOLD', outcome: 'docked', boot: 3, seq: 72, stale: false } }]);

    expect(result.current.assignments['handled:2:3:60'].outcome).toBe('unknown');
  });

  it('takes no word away when the socket connects, only when it drops', () => {
    telemetry.connected = false;
    const { result } = renderHook(() => useTaskAssignment());
    approveOwnDock(result);

    telemetry.connected = true;
    publish([beat(2, 61, 'APPROACH', null)]);
    publish([beat(2, 62, 'HOLD', 'docked')]);

    expect(result.current.assignments['handled:2:3:60'].outcome).toBe('docked');
  });

  it('reads a beat once, however often the store publishes it again', () => {
    const { result } = renderHook(() => useTaskAssignment());
    askOwnDock(result);
    const beatBeforeTheAnswer = [beat(2, 61, 'HOLD', null)];
    publish(beatBeforeTheAnswer);

    act(() => result.current.handleConfirmedStatusByTask(2, 7, '3:60'));
    publish(beatBeforeTheAnswer);

    expect(result.current.assignments['own:2:3:60'].status).toBe('confirmed');
  });

  it('counts each UAV\'s beats on their own', () => {
    const { result } = renderHook(() => useTaskAssignment());
    approveOwnDock(result, 2);
    approveOwnDock(result, 3);

    publish([beat(2, 61, 'HOLD', null), beat(3, 61, 'HOLD', null)]);

    expect(Object.keys(result.current.assignments).sort()).toEqual(['handled:2:3:60', 'handled:3:3:60']);
  });

  it('tells E-STOP from a mission restart', () => {
    const { result } = renderHook(() => useTaskAssignment());
    act(() => result.current.handleConfirmingStatus({
      sys_id: 2, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41, lon: 45, alt: 0,
    }));
    act(() => result.current.handleConfirmedStatusByTask(2, 7, '3:60'));

    act(() => result.current.handleDeniedCleanup(2));
    expect(result.current.assignments['handled:2:3:60'].outcome).toBe('unknown');

    act(() => result.current.reset());
    expect(result.current.assignments).toEqual({});
  });

  it('keeps its handlers across renders and unsubscribes on unmount', () => {
    const { result, rerender, unmount } = renderHook(() => useTaskAssignment());
    const first = result.current.handleAssignAck;
    rerender();
    expect(result.current.handleAssignAck).toBe(first);

    expect(telemetry.listeners.size).toBe(1);
    unmount();
    expect(telemetry.listeners.size).toBe(0);
  });
});
