import { useCallback, useState } from 'react';

/**
 * Vehicle command callbacks: estop, set mode, arm/disarm, launch, trigger,
 * restart, abort launch, clear coverage/trails. Owns coverageResetKey, trackResetKey and trailResetKey.
 */
export default function useVehicleCommands({
  api, vehicleList,
  taskConfirmReset, taskAssignReset,
  setLaunchStates,
}) {
  const [coverageResetKey, setCoverageResetKey] = useState(0);
  const [trackResetKey, setTrackResetKey] = useState(0);
  const [trailResetKey, setTrailResetKey] = useState(0);

  // sysIds omitted/undefined => all vehicles (backend targets fallback);
  // [sysId] => force-disarm only that one UAV (per-UAV E-STOP).
  const handleEstop = useCallback((sysIds) => {
    api.sendCommand('estop', sysIds);
  }, [api.sendCommand]);

  const handleSetMode = useCallback((sysId, mode) => {
    api.sendCommand('set_mode', [sysId], { mode });
  }, [api.sendCommand]);

  const handleSetModeAll = useCallback((mode) => {
    const ids = vehicleList.map(v => v.sys_id);
    if (ids.length === 0) return;
    if (!window.confirm(`Set ALL vehicles to ${mode}?`)) return;
    api.sendCommand('set_mode', ids, { mode });
  }, [api.sendCommand, vehicleList]);

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

  const handleLaunch = useCallback(
    (sysIds, opts) => {
      setLaunchStates({});
      return api.launchSequence(sysIds, opts);
    },
    [api.launchSequence, setLaunchStates]
  );

  const handleTriggerVehicle = useCallback(
    (sysId) => api.triggerVehicle(sysId),
    [api.triggerVehicle]
  );

  const handleRestart = useCallback(
    (sysIds, opts) => {
      setTrackResetKey((k) => k + 1);
      taskConfirmReset();
      taskAssignReset();
      return api.restartMission(sysIds, opts);
    },
    [api.restartMission, taskConfirmReset, taskAssignReset]
  );

  const handleAbortLaunch = useCallback(async () => {
    await fetch('/api/control/launch/abort', { method: 'POST' });
  }, []);

  const handleClearCoverage = useCallback(() => setCoverageResetKey((k) => k + 1), []);
  const handleClearTrails = useCallback(() => setTrailResetKey((k) => k + 1), []);

  return {
    handleEstop, handleSetMode, handleSetModeAll, handleArmDisarm, handleReboot,
    handleLaunch, handleTriggerVehicle, handleRestart,
    handleAbortLaunch, handleClearCoverage, handleClearTrails,
    coverageResetKey, trackResetKey, trailResetKey,
  };
}
