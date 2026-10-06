/**
 * Frontend-only placeholder vehicles for the narrow gap between backend
 * discovery/connect returning and the first telemetry snapshot arriving over
 * WebSocket.
 */

export function pendingVehicleFromDiscovery(vehicle) {
  const sysId = Number(vehicle?.sys_id);
  if (!Number.isFinite(sysId)) return null;
  return {
    sys_id: sysId,
    name: vehicle?.name || `UAV ${sysId}`,
    pending: true,
    link_ok: false,
    link_quality: 0,
    armed: false,
    companion_status: 'checking',
    companion_ok: true,
  };
}

export function upsertPendingVehicles(current, vehicles) {
  const byId = new Map((current || []).map((v) => [Number(v.sys_id), v]));
  for (const vehicle of vehicles || []) {
    const pending = pendingVehicleFromDiscovery(vehicle);
    if (!pending) continue;
    byId.set(pending.sys_id, { ...byId.get(pending.sys_id), ...pending });
  }
  return [...byId.values()].sort((a, b) => a.sys_id - b.sys_id);
}

export function prunePendingVehicles(current, vehicleList, removedIds = []) {
  const existing = current || [];
  const liveIds = new Set((vehicleList || []).map((v) => Number(v.sys_id)));
  const removed = new Set((removedIds || []).map((id) => Number(id)));
  const next = existing.filter((v) => {
    const sysId = Number(v.sys_id);
    return !liveIds.has(sysId) && !removed.has(sysId);
  });
  if (next.length === existing.length && next.every((v, i) => v === existing[i])) {
    return existing;
  }
  return next;
}

export function mergePendingVehicles(vehicleList, pendingVehicles) {
  const live = vehicleList || [];
  const liveIds = new Set(live.map((v) => Number(v.sys_id)));
  const pending = (pendingVehicles || []).filter((v) => !liveIds.has(Number(v.sys_id)));
  return [...live, ...pending];
}
