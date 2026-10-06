/**
 * True while the backend still reports mission work for a vehicle. A timed-out
 * browser request does not cancel the backend's blocking MAVLink transaction,
 * so callers can wait for this telemetry to settle before reading its cache.
 */
export function backendMissionDownloadInFlight(vehicle) {
  return !!vehicle && (
    vehicle.is_probing || Array.isArray(vehicle.mission_download_progress)
  );
}

/**
 * Subscribe directly to the live telemetry store until backend mission work
 * settles. `vehicleList` is intentionally unsuitable here because it only
 * updates when the connected roster changes, not on 5 Hz telemetry updates.
 */
export function waitForBackendMissionSettlement(store, sysId, signal) {
  return new Promise((resolve) => {
    let unsubscribe = () => {};
    let finished = false;
    const finish = () => {
      if (finished) return;
      finished = true;
      unsubscribe();
      signal?.removeEventListener('abort', finish);
      resolve();
    };
    const check = () => {
      const vehicle = store.getVehicles()[sysId];
      if (signal?.aborted || !backendMissionDownloadInFlight(vehicle)) finish();
    };
    unsubscribe = store.subscribe(check);
    signal?.addEventListener('abort', finish, { once: true });
    check();
  });
}

function normalizeResult(result, sysId) {
  if (!result || typeof result !== 'object') return { sys_id: sysId, error: 'empty response' };
  return { ...result, sys_id: sysId };
}

async function safeDownload(downloadMission, sysId) {
  try {
    return normalizeResult(await downloadMission(sysId), sysId);
  } catch (error) {
    return { sys_id: sysId, error: error?.message || String(error) };
  }
}

/**
 * Download a fleet concurrently and publish ordered snapshots as each vehicle
 * succeeds. A client timeout is special: the backend transaction may still be
 * running, so wait for its telemetry to settle and then read the cached result.
 * Other failures settle immediately.
 */
export async function reconcileMissionDownloads({
  vehicles,
  downloadMission,
  waitForBackendSettlement,
  onResultsChanged,
  onAwaitingLate,
  onVehicleSettled,
  isActive = () => true,
}) {
  const orderedIds = vehicles.map((vehicle) => vehicle.sys_id);
  const resultsById = new Map();

  const publish = async (result) => {
    resultsById.set(result.sys_id, result);
    const orderedResults = orderedIds
      .filter((sysId) => resultsById.has(sysId))
      .map((sysId) => resultsById.get(sysId));
    await onResultsChanged?.(orderedResults, result);
  };

  await Promise.all(vehicles.map(async (vehicle) => {
    const sysId = vehicle.sys_id;
    let result = await safeDownload(downloadMission, sysId);
    if (!isActive()) return;

    if (result.error === 'timeout') {
      onAwaitingLate?.(sysId);
      await waitForBackendSettlement?.(sysId);
      if (!isActive()) return;
      result = await safeDownload(downloadMission, sysId);
      if (!isActive()) return;
    }

    // Only successful missions change the rendered plan. Final errors remain
    // visible through onVehicleSettled without needlessly republishing it.
    if (!result.error) await publish(result);
    if (!isActive()) return;
    onVehicleSettled?.(sysId, result);
  }));
}
