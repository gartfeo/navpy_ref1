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

// Planning entry point shown on the monitor map (top-right overlay) — the
// ubiquitous "Edit Plan" / "Start Planning" affordance that used to live in the
// UAV-status sidebar header (a long translated label there could crowd E-STOP).
// `show` is true whenever planning should stay reachable; `isEdit` picks the
// "Edit Plan" label when a plan exists vs. "Start Planning" otherwise.
//
// It hides only in the idle pre-launch, no-plan case, where the sidebar already
// renders the primary full-width "Start Planning" button — a duplicate CTA. App
// passes `allDisarmed` as a proxy for the PRE_LAUNCH phase because the true
// container-launch flag lives in useContainerLauncher and is not lifted to App.
// The proxy is safe for every reachable state: suppression also requires
// `!vehiclesHaveMission`, and a real container launch always starts vehicles that
// carry a mission, so `show` stays true throughout any reachable container
// launch (see the truth-table test in tests/gcs/frontend/test_plan_entry.py). If
// container launch is revived beyond the 3-UAV demo, thread the real phase here.
export function computePlanEntry({ hasPlan, vehiclesHaveMission, isBusyConnecting, allDisarmed }) {
  const showsStartPlanningPrimary =
    allDisarmed && !hasPlan && !vehiclesHaveMission && !isBusyConnecting;
  return { show: hasPlan || !showsStartPlanningPrimary, isEdit: hasPlan };
}
