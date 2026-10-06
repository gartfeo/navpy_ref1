/**
 * Node.js tests for restart/start/launch button visibility logic
 * from MonitoringSidebar ActiveMonitoring component.
 *
 * Extracts the pure visibility logic and verifies:
 * - Bungee mode: restart button visible when ANY vehicle is armed (IN_FLIGHT)
 * - Bungee mode: restart only includes armed vehicle sys_ids
 * - Bungee mode: per-vehicle launch buttons visible for disarmed vehicles
 *   in both PRE_LAUNCH and IN_FLIGHT phases
 * - Container mode: restart button visible whenever phase is IN_FLIGHT
 *
 * Run via: node tests/gcs/frontend/test_restart_button_logic.js
 */

const assert = require('assert');

const MONITOR_PHASES = {
  NO_VEHICLES: 'NO_VEHICLES',
  PRE_LAUNCH: 'PRE_LAUNCH',
  CONTAINER_LAUNCHING: 'CONTAINER_LAUNCHING',
  IN_FLIGHT: 'IN_FLIGHT',
};

function computeMonitorPhase({ vehicleList, containerLaunching }) {
  if (vehicleList.length === 0) return MONITOR_PHASES.NO_VEHICLES;
  if (containerLaunching)       return MONITOR_PHASES.CONTAINER_LAUNCHING;
  const allDisarmed = vehicleList.every(v => !v.armed);
  if (allDisarmed) return MONITOR_PHASES.PRE_LAUNCH;
  return MONITOR_PHASES.IN_FLIGHT;
}

/**
 * Whether the RESTART MISSION button is visible.
 * Matches: phase === IN_FLIGHT
 */
function restartButtonVisible(vehicleList, isContainer, containerLaunching) {
  const phase = computeMonitorPhase({ vehicleList, containerLaunching });
  return phase === MONITOR_PHASES.IN_FLIGHT;
}

/**
 * Compute the sys_ids sent to the restart endpoint.
 * Container: all vehicles. Bungee: only armed vehicles.
 */
function restartSysIds(vehicleList, isContainer) {
  if (isContainer) return vehicleList.map(v => v.sys_id);
  return vehicleList.filter(v => v.armed).map(v => v.sys_id);
}

/**
 * Whether a per-vehicle bungee LAUNCH button is visible for a given vehicle.
 * Matches: !isContainer && !v.armed && (phase === PRE_LAUNCH || phase === IN_FLIGHT)
 */
function canBungeeLaunch(vehicle, vehicleList, isContainer, containerLaunching) {
  const phase = computeMonitorPhase({ vehicleList, containerLaunching });
  return !isContainer && !vehicle.armed && (phase === MONITOR_PHASES.PRE_LAUNCH || phase === MONITOR_PHASES.IN_FLIGHT);
}

// ---- Restart button: bungee mode ----

(function testBungeeAllArmedShowsRestart() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: true },
  ];
  assert.strictEqual(restartButtonVisible(vehicles, false, false), true,
    'Bungee: all armed → restart visible');
})();

(function testBungeeMixedShowsRestart() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(restartButtonVisible(vehicles, false, false), true,
    'Bungee: mixed armed/disarmed → restart visible (at least one armed)');
})();

(function testBungeeAllDisarmedHidesRestart() {
  const vehicles = [
    { sys_id: 1, armed: false },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(restartButtonVisible(vehicles, false, false), false,
    'Bungee: all disarmed → restart hidden');
})();

(function testBungeeSingleArmedShowsRestart() {
  const vehicles = [{ sys_id: 1, armed: true }];
  assert.strictEqual(restartButtonVisible(vehicles, false, false), true,
    'Bungee: single vehicle armed → restart visible');
})();

// ---- Restart sysIds filtering: bungee ----

(function testRestartSysIdsBungeeOnlyArmed() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: false },
    { sys_id: 3, armed: true },
  ];
  assert.deepStrictEqual(restartSysIds(vehicles, false), [1, 3],
    'Bungee: only armed vehicle sys_ids are included');
})();

(function testRestartSysIdsBungeeAllArmed() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: true },
  ];
  assert.deepStrictEqual(restartSysIds(vehicles, false), [1, 2],
    'Bungee: all armed → all sys_ids included');
})();

(function testRestartSysIdsBungeeNoneArmed() {
  const vehicles = [
    { sys_id: 1, armed: false },
    { sys_id: 2, armed: false },
  ];
  assert.deepStrictEqual(restartSysIds(vehicles, false), [],
    'Bungee: none armed → empty list');
})();

// ---- Restart sysIds filtering: container ----

(function testRestartSysIdsContainerAll() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: false },
    { sys_id: 3, armed: true },
  ];
  assert.deepStrictEqual(restartSysIds(vehicles, true), [1, 2, 3],
    'Container: all sys_ids included regardless of armed state');
})();

// ---- Restart button: container mode ----

(function testContainerMixedShowsRestart() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(restartButtonVisible(vehicles, true, false), true,
    'Container: mixed armed → restart visible');
})();

(function testContainerAllArmedShowsRestart() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: true },
  ];
  assert.strictEqual(restartButtonVisible(vehicles, true, false), true,
    'Container: all armed → restart visible');
})();

// ---- Per-vehicle bungee launch button ----

(function testBungeeLaunchPreLaunch() {
  const vehicles = [
    { sys_id: 1, armed: false },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(canBungeeLaunch(vehicles[0], vehicles, false, false), true,
    'Bungee PRE_LAUNCH: disarmed vehicle shows LAUNCH');
})();

(function testBungeeLaunchInFlightForDisarmed() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(canBungeeLaunch(vehicles[1], vehicles, false, false), true,
    'Bungee IN_FLIGHT: disarmed vehicle still shows LAUNCH');
})();

(function testBungeeLaunchHiddenForArmed() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(canBungeeLaunch(vehicles[0], vehicles, false, false), false,
    'Bungee IN_FLIGHT: armed vehicle hides LAUNCH');
})();

(function testContainerNeverShowsBungeeLaunch() {
  const vehicles = [
    { sys_id: 1, armed: false },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(canBungeeLaunch(vehicles[0], vehicles, true, false), false,
    'Container: disarmed vehicle does not show bungee LAUNCH');
})();

(function testBungeeLaunchHiddenDuringContainerLaunching() {
  const vehicles = [
    { sys_id: 1, armed: false },
    { sys_id: 2, armed: false },
  ];
  assert.strictEqual(canBungeeLaunch(vehicles[0], vehicles, false, true), false,
    'Bungee during CONTAINER_LAUNCHING phase: LAUNCH hidden');
})();

(function testEmptyListNoRestart() {
  assert.strictEqual(restartButtonVisible([], false, false), false,
    'Empty list → no restart');
})();

(function testThreeVehiclesOneUnarmed() {
  const vehicles = [
    { sys_id: 1, armed: true },
    { sys_id: 2, armed: true },
    { sys_id: 3, armed: false },
  ];
  assert.strictEqual(restartButtonVisible(vehicles, false, false), true,
    'Bungee: 2/3 armed → restart visible');
  assert.deepStrictEqual(restartSysIds(vehicles, false), [1, 2],
    'Bungee: 2/3 armed → only armed sys_ids in restart');
  assert.strictEqual(canBungeeLaunch(vehicles[2], vehicles, false, false), true,
    'Bungee: third disarmed vehicle shows LAUNCH');
})();

console.log('PASS');
