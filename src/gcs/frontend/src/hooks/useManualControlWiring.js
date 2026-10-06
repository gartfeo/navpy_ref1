import { useCallback, useEffect, useRef, useSyncExternalStore } from 'react';
import useManualControl from './useManualControl';
import { resolveFollowTarget } from '../components/map/hooks/useFollowUav';
import { PHASES } from './useMissionState';

/**
 * Wraps useManualControl with App-level integration: confirm dialogs,
 * mode capture/restore, vehicle selection, mcVehicle subscription,
 * and auto-disable effects on phase change / vehicle disconnect.
 */
export default function useManualControlWiring({
  sendWsMessage, storeRef,
  phase, vehicleList,
  handleSetMode,
  setFollowSysId, setNotification,
}) {
  const setModeRef = useRef(null);
  setModeRef.current = handleSetMode;

  const {
    manualControlEnabled, manualControlTarget, setManualControlTarget,
    toggleManualControl, handleLeftMove, handleRightMove,
    capturePrevMode, peekRestoreMode, releasePoi,
  } = useManualControl(sendWsMessage, storeRef, setModeRef);

  // Subscribe to the manual-control target vehicle so JSX using its
  // data (FlightModeColumn, ManualControlOverlay, TelemetryHud) stays fresh.
  const mcVehicle = useSyncExternalStore(
    storeRef.current.subscribe,
    () => {
      if (!manualControlEnabled || manualControlTarget == null) return null;
      return storeRef.current.getVehicles()[manualControlTarget] ?? null;
    },
  );

  // Wrap toggleManualControl to reset follow mode when exiting
  const handleToggleManualControl = useCallback(() => {
    if (manualControlEnabled && manualControlTarget != null) {
      const restoreMode = peekRestoreMode(manualControlTarget);
      if (restoreMode) {
        const cur = storeRef.current.getVehicles()[manualControlTarget]?.mode;
        if (!window.confirm(
          `UAV ${manualControlTarget} is in ${cur} — will be set to ${restoreMode}. Exit RC?`
        )) return;
      }
      const restored = releasePoi(manualControlTarget);
      if (restored) setNotification(`UAV ${manualControlTarget} → ${restored}`);
      setManualControlTarget(null);
      setFollowSysId(null);
    } else if (!manualControlEnabled && manualControlTarget != null) {
      capturePrevMode(manualControlTarget);
    }
    toggleManualControl();
  }, [manualControlEnabled, manualControlTarget, toggleManualControl, releasePoi, capturePrevMode, peekRestoreMode]);

  const handleVehicleSelect = useCallback((sysId) => {
    if (manualControlTarget != null && manualControlTarget !== sysId) {
      const restoreMode = peekRestoreMode(manualControlTarget);
      if (restoreMode) {
        const cur = storeRef.current.getVehicles()[manualControlTarget]?.mode;
        if (!window.confirm(
          `UAV ${manualControlTarget} is in ${cur} — will be set to ${restoreMode}. Switch?`
        )) return;
      }
      const restored = releasePoi(manualControlTarget);
      if (restored) setNotification(`UAV ${manualControlTarget} → ${restored}`);
    }
    setManualControlTarget(sysId);
    capturePrevMode(sysId);
    setFollowSysId(sysId);
  }, [manualControlTarget, setManualControlTarget, releasePoi, capturePrevMode, peekRestoreMode]);

  // Auto-disable manual control on phase change or target disconnect
  useEffect(() => {
    if (phase !== PHASES.MONITOR && manualControlEnabled) {
      handleToggleManualControl();
    }
  }, [phase]);

  useEffect(() => {
    if (manualControlEnabled && manualControlTarget != null
        && resolveFollowTarget(manualControlTarget, vehicleList) === null) {
      handleToggleManualControl();
    }
  }, [vehicleList]);

  return {
    manualControlEnabled, manualControlTarget, setManualControlTarget,
    handleToggleManualControl, handleVehicleSelect,
    handleLeftMove, handleRightMove,
    mcVehicle,
  };
}
