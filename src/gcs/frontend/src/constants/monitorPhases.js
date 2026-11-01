export const MONITOR_PHASES = {
  NO_VEHICLES: 'NO_VEHICLES',
  PRE_LAUNCH: 'PRE_LAUNCH',
  CONTAINER_LAUNCHING: 'CONTAINER_LAUNCHING',
  IN_FLIGHT: 'IN_FLIGHT',
};

export function computeMonitorPhase({ vehicleList, containerLaunching }) {
  if (vehicleList.length === 0) return MONITOR_PHASES.NO_VEHICLES;
  if (containerLaunching)       return MONITOR_PHASES.CONTAINER_LAUNCHING;
  const allDisarmed = vehicleList.every(v => !v.armed);
  if (allDisarmed) return MONITOR_PHASES.PRE_LAUNCH;
  return MONITOR_PHASES.IN_FLIGHT;
}
