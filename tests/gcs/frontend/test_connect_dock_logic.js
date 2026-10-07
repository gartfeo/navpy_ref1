/**
 * Node.js tests for the DOCK-preserving POI reconstruction in handleConnect.
 *
 * Extracts and tests the pure logic that builds the `pois` array when
 * connecting a new vehicle: existing zones should preserve their DOCK
 * assignments while the newly connected zone gets the downloaded default_delivery_hub.
 *
 * Run via: node tests/gcs/frontend/test_connect_dock_logic.js
 */
const assert = require('assert');

/**
 * Pure function matching the POI-reconstruction logic in handleConnect.
 * Given merged zones, DOCK assignments, existing Docks, and the downloaded
 * default_delivery_hub, builds the pois array for derivePlanPolygon.
 */
function buildPois(mergedZones, deliveryHubAssignments, existingDeliveryHubs, missionDeliveryHub) {
  return mergedZones.map((z, i) => {
    if (i === mergedZones.length - 1) return missionDeliveryHub || null;
    const deliveryHubIdx = deliveryHubAssignments?.[i];
    if (deliveryHubIdx != null && existingDeliveryHubs[deliveryHubIdx]) {
      return { lat: existingDeliveryHubs[deliveryHubIdx].lat, lon: existingDeliveryHubs[deliveryHubIdx].lon };
    }
    return null;
  });
}

// ---- Test: single zone (new) gets its default_delivery_hub ----
(function testSingleNewZone() {
  const zones = [{ zone_index: 0, sys_id: 5 }];
  const pois = buildPois(zones, [], [], { lat: 32.0, lon: 34.0 });
  assert.deepStrictEqual(pois, [{ lat: 32.0, lon: 34.0 }]);
})();

// ---- Test: two zones — existing preserves DOCK, new gets default_delivery_hub ----
(function testExistingPreservesDock() {
  const zones = [
    { zone_index: 0, sys_id: 1 },
    { zone_index: 1, sys_id: 5 }, // newly connected
  ];
  const deliveryHubAssignments = [0]; // zone 0 assigned to DOCK index 0
  const existingDeliveryHubs = [
    { lat: 10.0, lon: 20.0, name: 'HQ', type: 'building' },
  ];
  const missionDeliveryHub = { lat: 32.0, lon: 34.0 };
  const pois = buildPois(zones, deliveryHubAssignments, existingDeliveryHubs, missionDeliveryHub);
  assert.deepStrictEqual(pois, [
    { lat: 10.0, lon: 20.0 },  // preserved from DOCK
    { lat: 32.0, lon: 34.0 },  // new vehicle's default_delivery_hub
  ]);
})();

// ---- Test: three zones — unassigned middle zone stays null ----
(function testUnassignedMiddleZone() {
  const zones = [
    { zone_index: 0, sys_id: 1 },
    { zone_index: 1, sys_id: 2 },
    { zone_index: 2, sys_id: 5 }, // newly connected
  ];
  const deliveryHubAssignments = [0, null]; // zone 0 assigned, zone 1 unassigned
  const existingDeliveryHubs = [
    { lat: 10.0, lon: 20.0, name: 'Alpha', type: 'vehicle' },
  ];
  const missionDeliveryHub = { lat: 50.0, lon: 60.0 };
  const pois = buildPois(zones, deliveryHubAssignments, existingDeliveryHubs, missionDeliveryHub);
  assert.deepStrictEqual(pois, [
    { lat: 10.0, lon: 20.0 },  // preserved from DOCK
    null,                        // unassigned — stays null
    { lat: 50.0, lon: 60.0 },  // new vehicle's default_delivery_hub
  ]);
})();

// ---- Test: no DOCK assignments at all — only last zone gets POI ----
(function testNoDeliveryHubAssignments() {
  const zones = [
    { zone_index: 0, sys_id: 1 },
    { zone_index: 1, sys_id: 5 },
  ];
  const pois = buildPois(zones, null, [], { lat: 32.0, lon: 34.0 });
  assert.deepStrictEqual(pois, [null, { lat: 32.0, lon: 34.0 }]);
})();

// ---- Test: no default_delivery_hub — last zone gets null ----
(function testNoMissionDeliveryHub() {
  const zones = [{ zone_index: 0, sys_id: 5 }];
  const pois = buildPois(zones, [], [], null);
  assert.deepStrictEqual(pois, [null]);
})();

console.log('PASS');
