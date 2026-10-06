import { useCallback, useState } from 'react';

/**
 * Manages task assignment state: peer-assigned targets from swarm protocol.
 *
 * availableTasks — keyed by task_id: { taskId, taskType, lat, lon, alt, senderId }
 *   Populated when `available_task_request` arrives (targets detected, pre-assignment).
 *
 * assignments — keyed by task_id: { taskId, taskType, lat, lon, alt, senderId, receiverId, status, receivedAt }
 *   Populated when `task_assign_request` arrives (Hungarian algorithm assigned target).
 */
export default function useTaskAssignment() {
  const [assignments, setAssignments] = useState({});
  const [availableTasks, setAvailableTasks] = useState({});

  const handleAvailableTaskRequest = useCallback((data) => {
    setAvailableTasks((prev) => {
      const next = { ...prev };
      for (const t of data.tasks || []) {
        next[t.task_id] = {
          taskId: t.task_id,
          taskType: t.task_type,
          lat: t.lat,
          lon: t.lon,
          alt: t.alt,
          senderId: data.sender_id,
        };
      }
      return next;
    });
  }, []);

  const handleAssignRequest = useCallback((data) => {
    setAvailableTasks((prev) => {
      if (!(data.task_id in prev)) return prev;
      const next = { ...prev };
      delete next[data.task_id];
      return next;
    });
    setAssignments((prev) => ({
      ...prev,
      [data.task_id]: {
        taskId: data.task_id,
        taskType: data.task_type,
        lat: data.lat,
        lon: data.lon,
        alt: data.alt,
        senderId: data.sender_id,
        receiverId: data.receiver_id,
        status: 'assigning',
        receivedAt: Date.now(),
      },
    }));
  }, []);

  const handleAssignResponse = useCallback((data) => {
    setAssignments((prev) => {
      const entry = prev[data.task_id];
      if (!entry) return prev;
      if (data.is_accepted) {
        return {
          ...prev,
          [data.task_id]: { ...entry, status: 'assigned' },
        };
      }
      // Rejected — remove
      const next = { ...prev };
      delete next[data.task_id];
      return next;
    });
  }, []);

  const handleDisarmAfterGuidedCleanup = useCallback((data) => {
    setAvailableTasks((prev) => {
      const toRemove = Object.keys(prev).filter(
        (tid) => prev[tid].senderId === data.sys_id,
      );
      if (toRemove.length === 0) return prev;
      const next = { ...prev };
      for (const tid of toRemove) delete next[tid];
      return next;
    });
    setAssignments((prev) => {
      const toRemove = Object.keys(prev).filter(
        (tid) => prev[tid].receiverId === data.sys_id,
      );
      if (toRemove.length === 0) return prev;
      const next = { ...prev };
      for (const tid of toRemove) delete next[tid];
      return next;
    });
  }, []);

  const handleConfirmingStatus = useCallback((sysId) => {
    setAssignments((prev) => {
      const next = { ...prev };
      let changed = false;
      for (const tid of Object.keys(next)) {
        if (next[tid].receiverId === sysId) {
          next[tid] = { ...next[tid], status: 'confirming' };
          changed = true;
        }
      }
      return changed ? next : prev;
    });
  }, []);

  const handleDeniedCleanup = useCallback((sysId) => {
    setAssignments((prev) => {
      const toRemove = Object.keys(prev).filter(
        (tid) => prev[tid].receiverId === sysId,
      );
      if (toRemove.length === 0) return prev;
      const next = { ...prev };
      for (const tid of toRemove) delete next[tid];
      return next;
    });
  }, []);

  // ---- Task-id-keyed convergence (assignments/availableTasks are keyed by
  // task_id, so these are precise: a stale event for an old task cannot touch
  // a newer task's assignment). Used by the WS task_confirm_response handler
  // so passive clients converge, not just the initiating client's wrappers. ----

  const handleConfirmedStatusByTask = useCallback((taskId) => {
    setAssignments((prev) => {
      const entry = prev[taskId];
      if (!entry || entry.status === 'confirmed') return prev;
      return { ...prev, [taskId]: { ...entry, status: 'confirmed' } };
    });
  }, []);

  const handleResolvedCleanupByTask = useCallback((taskId) => {
    setAssignments((prev) => {
      if (!(taskId in prev)) return prev;
      const next = { ...prev };
      delete next[taskId];
      return next;
    });
    setAvailableTasks((prev) => {
      if (!(taskId in prev)) return prev;
      const next = { ...prev };
      delete next[taskId];
      return next;
    });
  }, []);

  const reset = useCallback(() => {
    setAssignments({});
    setAvailableTasks({});
  }, []);

  return {
    assignments,
    availableTasks,
    handleAvailableTaskRequest,
    handleAssignRequest,
    handleAssignResponse,
    handleDisarmAfterGuidedCleanup,
    handleConfirmingStatus,
    handleDeniedCleanup,
    handleConfirmedStatusByTask,
    handleResolvedCleanupByTask,
    reset,
  };
}
