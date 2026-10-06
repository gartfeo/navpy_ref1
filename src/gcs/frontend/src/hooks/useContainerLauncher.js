import { useCallback, useEffect, useState } from 'react';
import { applyLaunchOrder, resolvePreparedLaunchOrder } from '../utils/launchSequence';
import { containersFromSysIds } from '../utils/containers';

/**
 * Container launcher state and logic: ESP32 connection checking,
 * container launch orchestration, and auto-dismiss on completion.
 */
export default function useContainerLauncher({
  isContainer, vehicleList, launchStates,
  onLaunch, onTriggerVehicle, settings,
}) {
  const [containerLaunching, setContainerLaunching] = useState(false);
  const [launcherConnected, setLauncherConnected] = useState(false);
  const [simulatorRunning, setSimulatorRunning] = useState(false);
  const [checkingLauncher, setCheckingLauncher] = useState(false);

  const checkLauncher = useCallback(async () => {
    setCheckingLauncher(true);
    try {
      const res = await fetch('/api/control/launch/esp32-status');
      if (res.ok) {
        const data = await res.json();
        setLauncherConnected(data.reachable);
        setSimulatorRunning(data.simulator_running ?? false);
      }
    } catch {
      setLauncherConnected(false);
    }
    setCheckingLauncher(false);
  }, []);

  // Auto-check launcher connection when container mode and vehicles are present
  useEffect(() => {
    if (!isContainer || vehicleList.length === 0) return;
    checkLauncher();
  }, [isContainer, vehicleList.length, checkLauncher]);

  // Auto-dismiss container LaunchPanel when all vehicles terminal
  useEffect(() => {
    if (!containerLaunching) return;
    const allTerminal = vehicleList.length > 0 && vehicleList.every((v) => {
      const s = launchStates?.[v.sys_id]?.state;
      return s === 'airborne' || s === 'launched' || s === 'failed';
    });
    if (allTerminal) setContainerLaunching(false);
  }, [containerLaunching, vehicleList, launchStates]);

  // Container mode: START prepares ESP32 session then triggers all vehicles
  const doContainerLaunch = useCallback(async (opts) => {
    setContainerLaunching(true);
    // Launch vehicles in the operator-defined launch order (settings.launch.launch_order).
    const requestedSysIds = applyLaunchOrder(
      vehicleList.map((v) => v.sys_id),
      settings?.launch?.launch_order,
    );
    const result = await onLaunch(requestedSysIds, opts);
    if (result?.status !== 'container_prepared') {
      setContainerLaunching(false);
      return;
    }
    const sysIds = resolvePreparedLaunchOrder(
      requestedSysIds,
      result.sys_ids,
      settings?.launch?.launch_order,
    );
    console.log('[launch] container launch order:', sysIds);
    // With a container gap configured, trigger container-by-container (grouped
    // by sys_id) so the backend applies the gap at the right boundaries; within
    // a container the operator's launch order still applies. The gap itself is
    // enforced abort-aware in launch_controller, not here. With no gap, launch in
    // the flat launch order exactly as before (no behavior change).
    const containerGap = settings?.launch?.container_gap_s || 0;
    const groups = containerGap > 0
      ? containersFromSysIds(sysIds).map((c) => applyLaunchOrder(c, settings?.launch?.launch_order))
      : [sysIds];
    for (const group of groups) {
      for (const sysId of group) {
        console.log('[launch] triggering vehicle', sysId);
        await onTriggerVehicle(sysId);
      }
    }
  }, [vehicleList, onLaunch, onTriggerVehicle, settings]);

  const esp32Host = settings?.launch?.esp32_host || '192.168.4.1';
  const esp32Port = settings?.launch?.esp32_port || 80;

  return {
    containerLaunching,
    launcherConnected,
    simulatorRunning,
    checkingLauncher,
    checkLauncher,
    doContainerLaunch,
    esp32Host,
    esp32Port,
  };
}
