import { useEffect, useMemo, useReducer } from 'react';
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
export default function useTaskAssignment() {
  const [state, dispatch] = useReducer(reduceTaskAssignment, INITIAL_ASSIGNMENT_STATE);

  // Heartbeats retire rounds and end approved docks; each beat once. A dropped
  // socket may have hidden rounds, which the reducer must know.
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
        dispatch({ type: 'swarm_heartbeat', data: { sys_id: vehicle.sys_id, swarm } });
      }
    });
  }, []);

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
      handleConfirmingStatus: on('task_confirming'),
      handleConfirmedStatusByTask: onTask('task_confirmed'),
      handleResolvedCleanupByTask: onTask('task_resolved'),
      handleDeniedCleanup: (sysId) => dispatch({ type: 'vehicle_reset', data: { sys_id: sysId } }),
      reset: () => dispatch({ type: 'reset' }),
    };
  }, []);

  return {
    assignments: state.assignments,
    availableTasks: state.availableTasks,
    ...handlers,
  };
}
