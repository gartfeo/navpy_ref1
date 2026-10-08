import { useCallback, useEffect, useRef, useState } from 'react';

/**
 * Registers all WebSocket message handlers and owns the state they mutate:
 * navpyStatus, launchStates, launching.
 *
 * Also provides handleTaskApprove/handleTaskDeny which bridge
 * taskConfirm and taskAssign.
 */
export default function useWsHandlers({
  messageHandlersRef,
  taskConfirm, taskAssign,
  setUploadProgress,
  fullParams,
  compassCal,
}) {
  // ---- Refs for circular deps ----
  const confirmingStatusRef = useRef(null);
  confirmingStatusRef.current = taskAssign.handleConfirmingStatus;

  const disarmAfterGuidedCleanupRef = useRef(taskAssign.handleDisarmAfterGuidedCleanup);
  disarmAfterGuidedCleanupRef.current = taskAssign.handleDisarmAfterGuidedCleanup;

  const fullParamProgressRef = useRef(fullParams?.handleDownloadProgress);
  fullParamProgressRef.current = fullParams?.handleDownloadProgress;

  const fullParamWriteProgressRef = useRef(fullParams?.handleWriteProgress);
  fullParamWriteProgressRef.current = fullParams?.handleWriteProgress;

  const compassCalProgressRef = useRef(compassCal?.handleProgress);
  compassCalProgressRef.current = compassCal?.handleProgress;

  // ---- Task confirm handlers ----
  useEffect(() => {
    messageHandlersRef.current['task_confirm_request'] = (data) => {
      taskConfirm.handleConfirmRequest(data);
      confirmingStatusRef.current?.(data.sys_id, data.task_id);
    };
    messageHandlersRef.current['task_confirm_image'] = taskConfirm.handleConfirmImage;
    // Response converges on ALL clients (not just the initiator's wrapper):
    // the card state AND the (UAV, task)-keyed assignment state are updated here.
    messageHandlersRef.current['task_confirm_response'] = (data) => {
      taskConfirm.handleConfirmResponse(data);
      if (data.is_confirmed) taskAssign.handleConfirmedStatusByTask(data.sys_id, data.task_id);
      else taskAssign.handleResolvedCleanupByTask(data.sys_id, data.task_id);
    };
    // Mission restart / E-STOP (incl. a per-UAV E-STOP): clear cards +
    // assignments for the listed UAVs.
    messageHandlersRef.current['task_confirm_reset'] = (data) => {
      taskConfirm.handleConfirmReset(data);
      for (const sid of data.sys_ids || []) taskAssign.handleDeniedCleanup(sid);
    };
  }, [
    taskConfirm.handleConfirmRequest, taskConfirm.handleConfirmImage,
    taskConfirm.handleConfirmResponse, taskConfirm.handleConfirmReset,
    taskAssign.handleConfirmedStatusByTask, taskAssign.handleResolvedCleanupByTask,
    taskAssign.handleDeniedCleanup,
  ]);

  // ---- Task assignment handlers ----
  useEffect(() => {
    messageHandlersRef.current['available_task_request'] = taskAssign.handleAvailableTaskRequest;
    messageHandlersRef.current['task_assign_request'] = taskAssign.handleAssignRequest;
    messageHandlersRef.current['task_assign_response'] = taskAssign.handleAssignResponse;
    messageHandlersRef.current['task_assign_ack'] = taskAssign.handleAssignAck;
  }, [
    taskAssign.handleAvailableTaskRequest, taskAssign.handleAssignRequest,
    taskAssign.handleAssignResponse, taskAssign.handleAssignAck,
  ]);

  // ---- Existing task cleanup event ----
  const disarmAfterGuidedCardRef = useRef(taskConfirm.handleDisarmAfterGuided);
  disarmAfterGuidedCardRef.current = taskConfirm.handleDisarmAfterGuided;
  useEffect(() => {
    messageHandlersRef.current['disarm_after_guided'] = (data) => {
      disarmAfterGuidedCleanupRef.current(data);
      disarmAfterGuidedCardRef.current(data); // clear the (approved) confirmation card
    };
  }, [messageHandlersRef]);

  // ---- NavPy sim status ----
  const [navpyStatus, setNavpyStatus] = useState({});
  useEffect(() => {
    messageHandlersRef.current['navpy_sim_status'] = (msg) => {
      const map = {};
      for (const inst of msg.instances || []) map[inst.sys_id] = inst;
      setNavpyStatus(map);
    };
  }, []);

  // ---- Launch state tracking ----
  const [launchStates, setLaunchStates] = useState({});
  const [launching, setLaunching] = useState(false);
  useEffect(() => {
    messageHandlersRef.current['launch_progress'] = (msg) => {
      setLaunchStates((prev) => ({
        ...prev,
        [msg.sys_id]: { state: msg.state, error: msg.error || null },
      }));
      setLaunching(true);
    };
    messageHandlersRef.current['launch_complete'] = () => {
      setLaunching(false);
    };
    messageHandlersRef.current['launch_aborted'] = () => {
      setLaunching(false);
    };
  }, []);

  // ---- Upload progress ----
  useEffect(() => {
    messageHandlersRef.current['upload_progress'] = (data) => {
      setUploadProgress((prev) => ({
        ...prev,
        [data.sys_id]: {
          ...prev[data.sys_id],
          stage: data.stage,
          progress: data.progress ?? prev[data.sys_id]?.progress ?? 0,
          done: data.done ?? false,
          error: data.error ?? null,
          wp_sent: data.wp_sent ?? prev[data.sys_id]?.wp_sent,
          wp_total: data.wp_total ?? prev[data.sys_id]?.wp_total,
        },
      }));
    };
  }, [setUploadProgress]);

  // ---- Full parameter download progress ----
  useEffect(() => {
    messageHandlersRef.current['full_param_progress'] = (data) => {
      fullParamProgressRef.current?.(data);
    };
    messageHandlersRef.current['full_param_write_progress'] = (data) => {
      fullParamWriteProgressRef.current?.(data);
    };
  }, [messageHandlersRef]);

  // ---- Compass / magnetometer calibration progress ----
  useEffect(() => {
    messageHandlersRef.current['compass_cal_progress'] = (data) => {
      compassCalProgressRef.current?.(data);
    };
  }, [messageHandlersRef]);

  // ---- Approve / deny / cancel wrappers ----
  // Card-only on purpose: assignment state converges for ALL clients
  // (including this one) via the task_confirm_response WS handler, keyed by
  // task_id. Mutating assignment here too would diverge card vs assignment
  // state on a failed POST (the card is gated on res.ok; assignment was not).
  const handleTaskApprove = useCallback((sysId) => {
    taskConfirm.approve(sysId);
  }, [taskConfirm.approve]);

  const handleTaskDeny = useCallback((sysId) => {
    taskConfirm.deny(sysId);
  }, [taskConfirm.deny]);

  const handleTaskCancel = useCallback((sysId) => {
    taskConfirm.cancel(sysId);
  }, [taskConfirm.cancel]);

  return {
    navpyStatus,
    launchStates, setLaunchStates,
    launching,
    handleTaskApprove,
    handleTaskDeny,
    handleTaskCancel,
  };
}
