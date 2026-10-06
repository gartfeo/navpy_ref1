/**
 * Node.js tests for disconnect zone cleanup logic.
 *
 * Simulates the zone-filtering logic from handleDisconnect in
 * useVehicleConnection.js: zones with sys_id are filtered by match,
 * zones without sys_id fall back to count-based trimming.
 *
 * Run via: node tests/gcs/frontend/test_disconnect_zone_cleanup_logic.js
 */

const assert = require('assert');

/**
 * Replicate the zone-filtering logic from handleDisconnect.
 * Returns { remaining, keptOldIndices }.
 */
function filterZonesOnDisconnect(oldZones, removedIds) {
  const removedSet = new Set(removedIds);
  const keptOldIndices = [];
  for (let i = 0; i < oldZones.length; i++) {
    if (!removedSet.has(oldZones[i].sys_id)) keptOldIndices.push(i);
  }
  let remaining = oldZones
    .filter((z) => !removedSet.has(z.sys_id))
    .map((z, i) => ({ ...z, zone_index: i }));
  // Robustness: if sys_id filtering didn't remove anything (zones lack sys_id),
  // fall back to removing the last N zones to match the reduced vehicle count.
  if (remaining.length === oldZones.length && oldZones.length > 0 && removedIds.length > 0) {
    const keepCount = Math.max(0, oldZones.length - removedIds.length);
    remaining = oldZones.slice(0, keepCount).map((z, i) => ({ ...z, zone_index: i }));
    keptOldIndices.length = 0;
    for (let i = 0; i < keepCount; i++) keptOldIndices.push(i);
  }
  return { remaining, keptOldIndices };
}

// ---- Test: zones with sys_id are filtered correctly ----

(function testFilterBySystemId() {
  const zones = [
    { zone_index: 0, sys_id: 10, track: [] },
    { zone_index: 1, sys_id: 20, track: [] },
    { zone_index: 2, sys_id: 30, track: [] },
  ];
  const { remaining, keptOldIndices } = filterZonesOnDisconnect(zones, [20]);
  assert.strictEqual(remaining.length, 2, 'Should keep 2 zones');
  assert.strictEqual(remaining[0].sys_id, 10);
  assert.strictEqual(remaining[1].sys_id, 30);
  assert.strictEqual(remaining[0].zone_index, 0, 'Reindexed to 0');
  assert.strictEqual(remaining[1].zone_index, 1, 'Reindexed to 1');
  assert.deepStrictEqual(keptOldIndices, [0, 2]);
})();

// ---- Test: disconnect all by sys_id clears everything ----

(function testDisconnectAllBySystemId() {
  const zones = [
    { zone_index: 0, sys_id: 10, track: [] },
    { zone_index: 1, sys_id: 20, track: [] },
  ];
  const { remaining } = filterZonesOnDisconnect(zones, [10, 20]);
  assert.strictEqual(remaining.length, 0, 'Should clear all zones');
})();

// ---- Test: zones WITHOUT sys_id fall back to count-based trimming ----

(function testFallbackWithoutSystemId() {
  const zones = [
    { zone_index: 0, track: [{ lat: 1, lon: 2 }] },
    { zone_index: 1, track: [{ lat: 3, lon: 4 }] },
    { zone_index: 2, track: [{ lat: 5, lon: 6 }] },
  ];
  // Disconnect one vehicle (sys_id 99 doesn't match any zone)
  const { remaining, keptOldIndices } = filterZonesOnDisconnect(zones, [99]);
  assert.strictEqual(remaining.length, 2, 'Should trim to 2 zones');
  assert.deepStrictEqual(remaining[0].track, [{ lat: 1, lon: 2 }]);
  assert.deepStrictEqual(remaining[1].track, [{ lat: 3, lon: 4 }]);
  assert.strictEqual(remaining[0].zone_index, 0);
  assert.strictEqual(remaining[1].zone_index, 1);
  assert.deepStrictEqual(keptOldIndices, [0, 1]);
})();

// ---- Test: zones without sys_id — disconnect all ----

(function testFallbackDisconnectAll() {
  const zones = [
    { zone_index: 0, track: [] },
    { zone_index: 1, track: [] },
  ];
  const { remaining } = filterZonesOnDisconnect(zones, [10, 20]);
  assert.strictEqual(remaining.length, 0, 'Should clear all zones when 2 vehicles removed');
})();

// ---- Test: empty plan — no zones ----

(function testEmptyPlan() {
  const { remaining } = filterZonesOnDisconnect([], [10]);
  assert.strictEqual(remaining.length, 0, 'Empty plan stays empty');
})();

// ---- Test: mixed zones (some with sys_id, some without) ----

(function testMixedZones() {
  const zones = [
    { zone_index: 0, sys_id: 10, track: [] },
    { zone_index: 1, track: [] },  // no sys_id
    { zone_index: 2, sys_id: 30, track: [] },
  ];
  // Remove vehicle 10 — zone 0 matched by sys_id, zone 1 survives (no sys_id, not matched)
  const { remaining } = filterZonesOnDisconnect(zones, [10]);
  assert.strictEqual(remaining.length, 2, 'Should remove matched zone only');
  // Zone 1 (no sys_id) and zone 2 (sys_id 30) survive
  assert.strictEqual(remaining[0].sys_id, undefined);
  assert.strictEqual(remaining[1].sys_id, 30);
})();

// ---- Test: disconnect more vehicles than zones (no crash) ----

(function testDisconnectMoreThanZones() {
  const zones = [
    { zone_index: 0, track: [] },
  ];
  const { remaining } = filterZonesOnDisconnect(zones, [10, 20, 30]);
  assert.strictEqual(remaining.length, 0, 'Should clamp to 0');
})();

// ---- Test: single vehicle disconnect with sys_id ----

(function testSingleVehicleDisconnect() {
  const zones = [
    { zone_index: 0, sys_id: 42, track: [] },
  ];
  const { remaining } = filterZonesOnDisconnect(zones, [42]);
  assert.strictEqual(remaining.length, 0, 'Last zone removed');
})();

console.log('PASS');
