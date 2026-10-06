/**
 * Node.js tests for computeMonitorPhase state machine logic
 * from MonitoringSidebar.
 *
 * Run via: node tests/gcs/frontend/test_monitor_phase_logic.js
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

// ---- NO_VEHICLES ----

(function testEmptyList() {
  const result = computeMonitorPhase({ vehicleList: [], containerLaunching: false });
  assert.strictEqual(result, 'NO_VEHICLES', 'Empty list → NO_VEHICLES');
})();

(function testEmptyListWithContainerLaunching() {
  const result = computeMonitorPhase({ vehicleList: [], containerLaunching: true });
  assert.strictEqual(result, 'NO_VEHICLES', 'Empty list + containerLaunching → NO_VEHICLES (priority)');
})();

// ---- PRE_LAUNCH ----

(function testAllDisarmed() {
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: false },
      { sys_id: 2, armed: false },
    ],
    containerLaunching: false,
  });
  assert.strictEqual(result, 'PRE_LAUNCH', 'All disarmed → PRE_LAUNCH');
})();

(function testSingleDisarmed() {
  const result = computeMonitorPhase({
    vehicleList: [{ sys_id: 1, armed: false }],
    containerLaunching: false,
  });
  assert.strictEqual(result, 'PRE_LAUNCH', 'Single disarmed → PRE_LAUNCH');
})();

// ---- CONTAINER_LAUNCHING ----

(function testContainerLaunchingAllDisarmed() {
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: false },
      { sys_id: 2, armed: false },
    ],
    containerLaunching: true,
  });
  assert.strictEqual(result, 'CONTAINER_LAUNCHING', 'All disarmed + launching → CONTAINER_LAUNCHING');
})();

(function testContainerLaunchingSomeArmed() {
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: true },
      { sys_id: 2, armed: false },
    ],
    containerLaunching: true,
  });
  assert.strictEqual(result, 'CONTAINER_LAUNCHING', 'Some armed + launching → CONTAINER_LAUNCHING');
})();

(function testContainerLaunchingAllArmed() {
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: true },
      { sys_id: 2, armed: true },
    ],
    containerLaunching: true,
  });
  assert.strictEqual(result, 'CONTAINER_LAUNCHING', 'All armed + launching → CONTAINER_LAUNCHING');
})();

// ---- IN_FLIGHT ----

(function testSomeArmed() {
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: true },
      { sys_id: 2, armed: false },
    ],
    containerLaunching: false,
  });
  assert.strictEqual(result, 'IN_FLIGHT', 'Some armed → IN_FLIGHT');
})();

(function testAllArmed() {
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: true },
      { sys_id: 2, armed: true },
    ],
    containerLaunching: false,
  });
  assert.strictEqual(result, 'IN_FLIGHT', 'All armed → IN_FLIGHT');
})();

// ---- Priority tests ----

(function testContainerLaunchingBeatsArmed() {
  // containerLaunching should keep phase as CONTAINER_LAUNCHING even with armed vehicles
  const result = computeMonitorPhase({
    vehicleList: [
      { sys_id: 1, armed: true },
      { sys_id: 2, armed: true },
      { sys_id: 3, armed: false },
    ],
    containerLaunching: true,
  });
  assert.strictEqual(result, 'CONTAINER_LAUNCHING', 'containerLaunching beats armed state');
})();

(function testNoVehiclesBeatsContainerLaunching() {
  // NO_VEHICLES has highest priority
  const result = computeMonitorPhase({ vehicleList: [], containerLaunching: true });
  assert.strictEqual(result, 'NO_VEHICLES', 'NO_VEHICLES beats containerLaunching');
})();

console.log('PASS');
