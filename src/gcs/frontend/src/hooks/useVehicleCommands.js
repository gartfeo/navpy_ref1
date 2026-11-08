import { useCallback, useState } from 'react';
import useConfirm from './useConfirm';

/**
 * Vehicle command callbacks: estop, set mode, arm/disarm, launch, trigger,
 * restart, abort launch, clear trail and camera-mark history. Owns trackResetKey
 * and historyResetKey.
 *
 * `confirmBeforeStart` (async, resolves true to go ahead) gates every path that
 * starts a mission flight — launch (container and per-vehicle, force included)
 * and restart (bungee Start, Restart) — before any command or local reset.
 */
export default function useVehicleCommands({
  api, vehicleList,
  taskConfirmReset, taskAssignReset,
  setLaunchStates,
  resumeHistory,
  confirmBeforeStart,
}) {
  const confirm = useConfirm();
  const [trackResetKey, setTrackResetKey] = useState(0);
  const [historyResetKey, setHistoryResetKey] = useState(0);

  // sysIds omitted/undefined => all vehicles (backend targets fallback);
  // [sysId] => force-disarm only that one UAV (per-UAV E-STOP).
  const handleEstop = useCallback((sysIds) => {
    api.sendCommand('estop', sysIds);
  }, [api.sendCommand]);

  const handleSetMode = useCallback((sysId, mode) => {
    api.sendCommand('set_mode', [sysId], { mode });
  }, [api.sendCommand]);

  const handleSetModeAll = useCallback(async (mode) => {
    const ids = vehicleList.map(v => v.sys_id);
    if (ids.length === 0) return;
    if (!(await confirm(`Set ALL vehicles to ${mode}?`))) return;
    api.sendCommand('set_mode', ids, { mode });
  }, [api.sendCommand, vehicleList, confirm]);

  const handleArmDisarm = useCallback((sysId, shouldArm, opts) => {
    const cmd = shouldArm
      ? (opts?.force ? 'force_arm' : 'arm')
      : (opts?.force ? 'force_disarm' : 'disarm');
    api.sendCommand(cmd, [sysId]);
  }, [api.sendCommand]);

  // Reboot the autopilot (flight controller) on a single vehicle. The backend
  // refuses armed vehicles; callers (e.g. compass cal) can reuse this directly.
  const handleReboot = useCallback(
    (sysId) => api.sendCommand('reboot', [sysId]),
    [api.sendCommand]
  );

  // opts.signal: the launch or start control's unmount signal. Its camera
  // question closes with the control, and an answer that comes after the
  // control is gone launches nothing.
  const handleLaunch = useCallback(
    async (sysIds, opts) => {
      const { signal, ...rest } = opts ?? {};
      if (confirmBeforeStart && !(await confirmBeforeStart(signal))) return null;
      if (signal?.aborted) return null;
      setLaunchStates({});
      return api.launchSequence(sysIds, opts && rest);
    },
    [api.launchSequence, setLaunchStates, confirmBeforeStart]
  );

  const handleTriggerVehicle = useCallback(
    (sysId) => api.triggerVehicle(sysId),
    [api.triggerVehicle]
  );

  const handleRestart = useCallback(
    async (sysIds, opts) => {
      const { signal, ...rest } = opts ?? {};
      if (confirmBeforeStart && !(await confirmBeforeStart(signal))) return null;
      if (signal?.aborted) return null;
      setTrackResetKey((k) => k + 1);
      // A new mission is recorded: Restart clears history and resumes it.
      resumeHistory?.();
      taskConfirmReset();
      taskAssignReset();
      return api.restartMission(sysIds, opts && rest);
    },
    [api.restartMission, taskConfirmReset, taskAssignReset, resumeHistory, confirmBeforeStart]
  );

  const handleAbortLaunch = useCallback(async () => {
    await fetch('/api/control/launch/abort', { method: 'POST' });
  }, []);

  // Clear all: trail and camera marks of every UAV, shown or hidden.
  const handleClearHistory = useCallback(() => setHistoryResetKey((k) => k + 1), []);

  return {
    handleEstop, handleSetMode, handleSetModeAll, handleArmDisarm, handleReboot,
    handleLaunch, handleTriggerVehicle, handleRestart,
    handleAbortLaunch, handleClearHistory,
    trackResetKey, historyResetKey,
  };
}
