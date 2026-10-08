import { beforeEach, describe, expect, it, vi } from 'vitest';
import { act, renderHook } from '@testing-library/react';
import useTaskAssignment from './useTaskAssignment';

// The hook reads each vehicle's swarm heartbeat from the telemetry store.
const telemetry = vi.hoisted(() => ({ listeners: new Set(), vehicles: [] }));
vi.mock('../stores/telemetryStore', () => ({
  default: {
    subscribe: (listener) => {
      telemetry.listeners.add(listener);
      return () => telemetry.listeners.delete(listener);
    },
    getVehicleList: () => telemetry.vehicles,
  },
}));

const uid = (boot_id, msg_seq) => ({ boot_id, msg_seq });

function publish(vehicles) {
  telemetry.vehicles = vehicles;
  act(() => {
    for (const listener of telemetry.listeners) listener();
  });
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
