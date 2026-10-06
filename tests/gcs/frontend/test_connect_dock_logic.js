/**
 * Node.js tests for the DOCK-preserving target reconstruction in handleConnect.
 *
 * Extracts and tests the pure logic that builds the `targets` array when
 * connecting a new vehicle: existing zones should preserve their DOCK
 * assignments while the newly connected zone gets the downloaded fallback_delivery_location.
 *
 * Run via: node tests/gcs/frontend/test_connect_dock_logic.js
 */
const assert = require('assert');

/**
 * Pure function matching the target-reconstruction logic in handleConnect.
 * Given merged zones, DOCK assignments, existing Docks, and the downloaded
 * fallback_delivery_location, builds the targets array for derivePlanPolygon.
 */
function buildTargets(mergedZones, fallbackLocationAssignments, existingFallbackLocations, missionFallbackLocation) {
  return mergedZones.map((z, i) => {
    if (i === mergedZones.length - 1) return missionFallbackLocation || null;
    const fallbackLocationIdx = fallbackLocationAssignments?.[i];
    if (fallbackLocationIdx != null && existingFallbackLocations[fallbackLocationIdx]) {
      return { lat: existingFallbackLocations[fallbackLocationIdx].lat, lon: existingFallbackLocations[fallbackLocationIdx].lon };
    }
    return null;
  });
}

// ---- Test: single zone (new) gets its fallback_delivery_location ----
(function testSingleNewZone() {
  const zones = [{ zone_index: 0, sys_id: 5 }];
  const targets = buildTargets(zones, [], [], { lat: 32.0, lon: 34.0 });
  assert.deepStrictEqual(targets, [{ lat: 32.0, lon: 34.0 }]);
})();

// ---- Test: two zones — existing preserves DOCK, new gets fallback_delivery_location ----
(function testExistingPreservesDock() {
  const zones = [
    { zone_index: 0, sys_id: 1 },
    { zone_index: 1, sys_id: 5 }, // newly connected
  ];
  const fallbackLocationAssignments = [0]; // zone 0 assigned to DOCK index 0
  const existingFallbackLocations = [
    { lat: 10.0, lon: 20.0, name: 'HQ', type: 'building' },
  ];
  const missionFallbackLocation = { lat: 32.0, lon: 34.0 };
  const targets = buildTargets(zones, fallbackLocationAssignments, existingFallbackLocations, missionFallbackLocation);
  assert.deepStrictEqual(targets, [
    { lat: 10.0, lon: 20.0 },  // preserved from DOCK
    { lat: 32.0, lon: 34.0 },  // new vehicle's fallback_delivery_location
  ]);
})();

// ---- Test: three zones — unassigned middle zone stays null ----
(function testUnassignedMiddleZone() {
  const zones = [
    { zone_index: 0, sys_id: 1 },
    { zone_index: 1, sys_id: 2 },
    { zone_index: 2, sys_id: 5 }, // newly connected
  ];
  const fallbackLocationAssignments = [0, null]; // zone 0 assigned, zone 1 unassigned
  const existingFallbackLocations = [
    { lat: 10.0, lon: 20.0, name: 'Alpha', type: 'vehicle' },
  ];
  const missionFallbackLocation = { lat: 50.0, lon: 60.0 };
  const targets = buildTargets(zones, fallbackLocationAssignments, existingFallbackLocations, missionFallbackLocation);
  assert.deepStrictEqual(targets, [
    { lat: 10.0, lon: 20.0 },  // preserved from DOCK
    null,                        // unassigned — stays null
    { lat: 50.0, lon: 60.0 },  // new vehicle's fallback_delivery_location
  ]);
})();

// ---- Test: no DOCK assignments at all — only last zone gets target ----
(function testNoFallbackLocationAssignments() {
  const zones = [
    { zone_index: 0, sys_id: 1 },
    { zone_index: 1, sys_id: 5 },
  ];
  const targets = buildTargets(zones, null, [], { lat: 32.0, lon: 34.0 });
  assert.deepStrictEqual(targets, [null, { lat: 32.0, lon: 34.0 }]);
})();

// ---- Test: no fallback_delivery_location — last zone gets null ----
(function testNoMissionFallbackLocation() {
  const zones = [{ zone_index: 0, sys_id: 5 }];
  const targets = buildTargets(zones, [], [], null);
  assert.deepStrictEqual(targets, [null]);
})();

console.log('PASS');
