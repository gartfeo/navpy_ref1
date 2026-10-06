import { useState, useRef, useCallback, useEffect } from 'react';
import { buildManualControlPayload } from '../utils/joystickMath';
import { RADIO_MODES } from '../utils/flightModes';

/**
 * Hook that manages manual-control state and rate-limited sending.
 *
 * Returns controls for toggling manual control, setting the target vehicle,
 * and handlers for the left/right joystick onMove callbacks.
 *
 * Sends manual_control messages over the telemetry WebSocket only when
 * joystick values change. The backend repeater handles 20 Hz MAVLink delivery.
 */
export default function useManualControl(sendWsMessage, storeRef, setModeRef) {
  const [manualControlEnabled, setManualControlEnabled] = useState(false);
  const [manualControlTarget, setManualControlTarget] = useState(null);

  const leftRef = useRef({ x: 0, y: -1 });
  const rightRef = useRef({ x: 0, y: 0 });
  const intervalRef = useRef(null);
  const lastSentRef = useRef(null);
  const prevModeRef = useRef({});

  const handleLeftMove = useCallback((pos) => { leftRef.current = pos; }, []);
  const handleRightMove = useCallback((pos) => { rightRef.current = pos; }, []);

  // Start / stop the 20 Hz check interval
  useEffect(() => {
    if (!manualControlEnabled || manualControlTarget == null) {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
      return;
    }

    const send = () => {
      const payload = buildManualControlPayload(leftRef.current, rightRef.current);
      const key = `${payload.x},${payload.y},${payload.z},${payload.r}`;
      if (key === lastSentRef.current) return;
      lastSentRef.current = key;
      payload.type = 'manual_control';
      payload.sys_id = manualControlTarget;
      sendWsMessage(payload);
    };

    // Send immediately, then check at 20 Hz
    send();
    intervalRef.current = setInterval(send, 50);

    return () => {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    };
  }, [manualControlEnabled, manualControlTarget, sendWsMessage]);

  /** Capture a vehicle's current mode as its "previous mode" before RC takes over. */
  const capturePrevMode = useCallback((sysId) => {
    if (sysId == null) return;
    const mode = storeRef.current.getVehicles()[sysId]?.mode;
    if (mode && !(sysId in prevModeRef.current)) {
      prevModeRef.current[sysId] = mode;
    }
  }, [storeRef]);

  /**
   * Compute what mode would be restored for a vehicle without acting.
   * Returns the target mode name, or null if no change needed.
   */
  const peekRestoreMode = useCallback((sysId) => {
    if (sysId == null) return null;
    const v = storeRef.current.getVehicles()[sysId];
    if (!v?.armed) return null;
    const currentMode = v.mode;
    if (!currentMode || !RADIO_MODES.includes(currentMode)) return null;
    const prev = prevModeRef.current[sysId];
    if (prev === currentMode) return null; // back in original mode, no change needed
    return (prev && !RADIO_MODES.includes(prev)) ? prev : 'LOITER';
  }, [storeRef]);

  /**
   * Release RC control for a vehicle and restore its previous safe mode.
   * Returns the restored mode name if a mode change was made, or null.
   */
  const releaseTarget = useCallback((sysId) => {
    if (sysId == null) return null;
    sendWsMessage({ type: 'manual_control_stop', sys_id: sysId });

    const restoredMode = peekRestoreMode(sysId);
    if (restoredMode) {
      setModeRef.current?.(sysId, restoredMode);
    }

    delete prevModeRef.current[sysId];
    return restoredMode;
  }, [sendWsMessage, peekRestoreMode, setModeRef]);

  const toggleManualControl = useCallback(() => {
    setManualControlEnabled((prev) => {
      if (prev) {
        // Disabling — reset joystick state (caller handles releaseTarget)
        leftRef.current = { x: 0, y: -1 };
        rightRef.current = { x: 0, y: 0 };
        lastSentRef.current = null;
      }
      return !prev;
    });
  }, []);

  return {
    manualControlEnabled,
    manualControlTarget,
    setManualControlTarget,
    toggleManualControl,
    handleLeftMove,
    handleRightMove,
    capturePrevMode,
    peekRestoreMode,
    releaseTarget,
  };
}
