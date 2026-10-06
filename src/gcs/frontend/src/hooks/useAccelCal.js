import { useCallback, useEffect, useRef, useState } from 'react';
import { initCalState, reduceAccelCalStep } from '../utils/accelCal';

/**
 * Owns accelerometer / level calibration: registers the `accel_cal_step`
 * WebSocket handler and exposes per-vehicle actions that drive the autopilot
 * via the existing `/api/control/command` dispatch (sendCommand).
 *
 * State shape per sys_id:
 *   { active, mode: 'level'|'full', step: 1..6|null, prompt, result, busy }
 *
 * The hook holds only UI state; the autopilot is the source of truth for
 * which position it expects next (relayed as `accel_cal_step` prompts).
 *
 * Re-authored from feat/uav-accel-cal (466 commits behind dev, INTEG-04,
 * D-04) — drop-in logic, no dev-side collision. Registers directly on
 * `messageHandlersRef` (the telemetry store's shared handler map) rather than
 * through useWsHandlers.js, matching the source branch's own wiring.
 */
export default function useAccelCal({ messageHandlersRef, sendCommand }) {
  const [calByVehicle, setCalByVehicle] = useState({});

  // Mirror of the latest state so action callbacks can read the current step
  // without depending on (and re-creating with) every state change.
  const calRef = useRef(calByVehicle);
  calRef.current = calByVehicle;

  // Register the WebSocket handler once. messageHandlersRef.current is the
  // telemetry store's shared (stable) handler map; setCalByVehicle is stable.
  useEffect(() => {
    const handlers = messageHandlersRef.current;
    handlers['accel_cal_step'] = (msg) => {
      setCalByVehicle((prev) => ({
        ...prev,
        [msg.sys_id]: reduceAccelCalStep(prev[msg.sys_id], msg),
      }));
    };
    return () => { delete handlers['accel_cal_step']; };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const setBusy = useCallback((sysId, busy) => {
    setCalByVehicle((prev) => (
      prev[sysId] ? { ...prev, [sysId]: { ...prev[sysId], busy } } : prev
    ));
  }, []);

  const startLevel = useCallback(async (sysId) => {
    setCalByVehicle((prev) => ({ ...prev, [sysId]: initCalState('level') }));
    await sendCommand('accel_level', [sysId]);
    setBusy(sysId, false);
  }, [sendCommand, setBusy]);

  const startFullCal = useCallback(async (sysId) => {
    setCalByVehicle((prev) => ({ ...prev, [sysId]: initCalState('full') }));
    await sendCommand('accel_cal_start', [sysId]);
    setBusy(sysId, false);
  }, [sendCommand, setBusy]);

  const confirmPosition = useCallback(async (sysId) => {
    const pos = calRef.current[sysId]?.step ?? null;
    if (pos == null) return;
    // Mark busy and clear the consumed prompt so the wizard reads "waiting"
    // until the autopilot sends the next position prompt (or final result).
    setCalByVehicle((prev) => (
      prev[sysId]
        ? { ...prev, [sysId]: { ...prev[sysId], busy: true, prompt: '', step: null } }
        : prev
    ));
    await sendCommand('accel_cal_pos', [sysId], { pos });
    setBusy(sysId, false);
  }, [sendCommand, setBusy]);

  /** Dismiss a vehicle's wizard locally (does not abort the autopilot). */
  const dismiss = useCallback((sysId) => {
    setCalByVehicle((prev) => {
      if (!(sysId in prev)) return prev;
      const next = { ...prev };
      delete next[sysId];
      return next;
    });
  }, []);

  return { calByVehicle, startLevel, startFullCal, confirmPosition, dismiss };
}
