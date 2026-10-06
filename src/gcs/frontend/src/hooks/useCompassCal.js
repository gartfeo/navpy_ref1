import { useCallback, useEffect, useMemo, useState } from 'react';
import { startCal, cancelCal, reduceCompassCal } from '../utils/compassCal';

/**
 * Owns onboard compass-calibration state and command dispatch.
 *
 * `calByVehicle: { [sysId]: { compasses, status, rebootRequired } }`.
 * The WS `compass_cal_progress` message is routed here via
 * `handleProgress` (wired in useWsHandlers). State for vehicles that leave
 * `vehicleList` is pruned.
 *
 * Ported from feat/uav-compass-cal (INTEG-04, D-04) into the unified
 * Calibration tab shell — drop-in, no dev-side collision.
 */
async function sendControl(command, sysIds, params) {
  try {
    const res = await fetch('/api/control/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command, sys_ids: sysIds, params: params ?? null }),
    });
    if (res.ok) return await res.json();
  } catch (e) {
    console.error('Compass cal command error:', e);
  }
  return null;
}

export default function useCompassCal(vehicleList) {
  const [calByVehicle, setCalByVehicle] = useState({});

  // Prune state for disconnected vehicles.
  useEffect(() => {
    const live = new Set((vehicleList || []).map((v) => v.sys_id));
    setCalByVehicle((prev) => {
      let changed = false;
      const next = {};
      for (const k of Object.keys(prev)) {
        if (live.has(Number(k))) next[k] = prev[k];
        else changed = true;
      }
      return changed ? next : prev;
    });
  }, [vehicleList]);

  const start = useCallback(async (sysId) => {
    setCalByVehicle((prev) => ({ ...prev, [sysId]: startCal() }));
    return sendControl('compass_cal_start', [sysId], null);
  }, []);

  const cancel = useCallback(async (sysId) => {
    setCalByVehicle((prev) => ({ ...prev, [sysId]: cancelCal(prev[sysId]) }));
    return sendControl('compass_cal_cancel', [sysId], null);
  }, []);

  const accept = useCallback(async (sysId) => sendControl('compass_cal_accept', [sysId], null), []);

  const reboot = useCallback(async (sysId) => sendControl('compass_cal_reboot', [sysId], null), []);

  const handleProgress = useCallback((event) => {
    const sid = Number(event?.sys_id);
    if (!Number.isFinite(sid)) return;
    setCalByVehicle((prev) => ({ ...prev, [sid]: reduceCompassCal(prev[sid], event) }));
  }, []);

  return useMemo(
    () => ({ calByVehicle, start, cancel, accept, reboot, handleProgress }),
    [calByVehicle, start, cancel, accept, reboot, handleProgress],
  );
}
