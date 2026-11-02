import { useEffect, useMemo, useReducer, useRef } from 'react';
import telemetryStore from '../stores/telemetryStore';
import {
  INITIAL_ASSIGNMENT_STATE,
  reduceTaskAssignment,
} from '../utils/taskAssignmentState';

/**
 * Task-assignment state for the map and the vehicle cards: a thin wrapper
 * over the pure taskAssignmentState reducer, which owns the rules.
 *
 * availableTasks — advertised tasks keyed `${owner}:${task}`.
 * assignments — pending rounds keyed `${owner}:${task}` (waiting), assigned
 *   tasks keyed `helper:${id}`, and the map's own docks and handled docks
 *   (`own:...`, `handled:...`); see the reducer.
 */
// The sender's TTL for SWARM_HEARTBEAT (`ttl_defaults.py`).
const BEAT_TTL_MS = 5000;

export default function useTaskAssignment() {
  const [state, dispatch] = useReducer(reduceTaskAssignment, INITIAL_ASSIGNMENT_STATE);
  const lastHeardAt = useRef({});
  // A gap ends at what ended it, and that message's round is not behind the gap.
  const closeGap = useRef((sysId) => {
    const now = performance.now();
    if (now - lastHeardAt.current[sysId] > BEAT_TTL_MS) {
      dispatch({ type: 'link_gap', data: { sys_id: sysId } });
    }
    lastHeardAt.current[sysId] = now;
  }).current;

  // Heartbeats retire rounds and end approved docks; each beat once. A dropped
  // socket, or a gap between a UAV's beats longer than their TTL (seen as a stale
  // beat, or as a jump when snapshots stalled too), may have hidden rounds, which
  // the reducer must know.
  useEffect(() => {
    const lastBeat = {};
    const wasStale = {};
    let connected = telemetryStore.isConnected();
    return telemetryStore.subscribe(() => {
      const nowConnected = telemetryStore.isConnected();
      if (connected && !nowConnected) dispatch({ type: 'link_gap' });
      connected = nowConnected;
      for (const vehicle of telemetryStore.getVehicleList()) {
        const swarm = vehicle.swarm;
        if (!swarm) continue;
        if (swarm.stale && !wasStale[vehicle.sys_id]) {
          dispatch({ type: 'link_gap', data: { sys_id: vehicle.sys_id } });
        }
        wasStale[vehicle.sys_id] = swarm.stale;
        const beat = `${swarm.boot}:${swarm.seq}`;
        if (lastBeat[vehicle.sys_id] === beat) continue;
        lastBeat[vehicle.sys_id] = beat;
        closeGap(vehicle.sys_id);
        dispatch({ type: 'swarm_heartbeat', data: { sys_id: vehicle.sys_id, swarm } });
      }
    });
  }, [closeGap]);

  const handlers = useMemo(() => {
    const on = (type) => (data) => dispatch({ type, data });
    const onTask = (type) => (sysId, taskId, roundUid = null) => dispatch({
      type, data: { sys_id: sysId, task_id: taskId, round_uid: roundUid },
    });
    return {
      handleAvailableTaskRequest: on('available_task_request'),
      handleAssignRequest: on('task_assign_request'),
      handleAssignResponse: on('task_assign_response'),
      handleAssignAck: on('task_assign_ack'),
      handleDisarmAfterGuidedCleanup: on('disarm_after_guided'),
      handleConfirmingStatus: (data) => {
        closeGap(data.sys_id);
        dispatch({ type: 'task_confirming', data });
      },
      handleConfirmedStatusByTask: onTask('task_confirmed'),
      handleResolvedCleanupByTask: onTask('task_resolved'),
      handleDeniedCleanup: (sysId) => dispatch({ type: 'vehicle_reset', data: { sys_id: sysId } }),
      reset: () => dispatch({ type: 'reset' }),
    };
  }, [closeGap]);

  return {
    assignments: state.assignments,
    availableTasks: state.availableTasks,
    ...handlers,
  };
}
