/**
 * Tests for assembleMissionsFromResults pure helper.
 *
 * Functions loaded from production source:
 *   src/gcs/frontend/src/utils/missionAssembly.js
 *
 * Run via: node tests/gcs/frontend/test_mission_assembly_logic.js
 */
const assert = require('assert');
const path = require('path');
const { readFileSync } = require('fs');

function loadEsm(relPath) {
  const abs = path.resolve(__dirname, '..', '..', '..', 'src', 'gcs', 'frontend', 'src', relPath);
  let src = readFileSync(abs, 'utf8');
  src = src.replace(/import\s+\{[^}]*\}\s+from\s+['"][^'"]+['"];?\n?/g, '');
  src = src.replace(/export\s+(const|function|class)\s+/g, '$1 ');
  src = src.replace(/export\s*\{[^}]*\}/g, '');
  src = src.replace(/export\s+default\s+/g, '');
  const names = [];
  src.replace(/^(?:const|function|class)\s+(\w+)/gm, (_, n) => { names.push(n); return _; });
  src += '\nmodule.exports = { ' + names.join(', ') + ' };\n';
  const m = { exports: {} };
  new Function('module', 'exports', 'require', src)(m, m.exports, require);
  return m.exports;
}

const { assembleMissionsFromResults, mergeDownloadedMissionZones } = loadEsm('utils/missionAssembly.js');

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
}

test('empty results returns null', () => {
  assert.strictEqual(assembleMissionsFromResults([]), null);
});

test('all errors returns null', () => {
  const results = [
    { sys_id: 1, error: 'timeout' },
    { sys_id: 2, error: 'refused' },
  ];
  assert.strictEqual(assembleMissionsFromResults(results), null);
});

test('errors filtered, valid missions kept', () => {
  const results = [
    { sys_id: 1, error: 'timeout' },
    { sys_id: 2, waypoints: [{ lat: 1, lon: 2 }], altitude_m: 150 },
  ];
  const r = assembleMissionsFromResults(results);
  assert.ok(r !== null);
  assert.strictEqual(r.zones.length, 1);
  assert.strictEqual(r.zones[0].sys_id, 2);
});

test('single mission produces correct zone structure', () => {
  const results = [{
    sys_id: 5,
    waypoints: [{ lat: 10, lon: 20 }, { lat: 11, lon: 21, alt: 100 }],
    altitude_m: 200,
    corridor_end_index: 1,
    search_pattern: 'corridor',
    polygon: [{ lat: 0, lon: 0 }, { lat: 1, lon: 0 }, { lat: 0, lon: 1 }],
    launch_point: { lat: 5, lon: 5 },
    fallback_delivery_location: { lat: 10, lon: 20 },
  }];
  const r = assembleMissionsFromResults(results);
  assert.strictEqual(r.zones.length, 1);
  assert.strictEqual(r.zones[0].zone_index, 0);
  assert.strictEqual(r.zones[0].sys_id, 5);
  assert.strictEqual(r.zones[0].corridor_end_index, 1);
  assert.strictEqual(r.zones[0].track.length, 2);
  assert.deepStrictEqual(r.zones[0].track[1], { lat: 11, lon: 21, alt: 100 });
  assert.strictEqual(r.altitude, 200);
  assert.strictEqual(r.searchPattern, 'corridor');
  assert.strictEqual(r.polygon.length, 3);
  assert.deepStrictEqual(r.launchPoint, { lat: 5, lon: 5 });
  assert.strictEqual(Object.hasOwn(r, 'dockClasses'), false);
  assert.deepStrictEqual(r.missionFallbackLocations, [{ lat: 10, lon: 20 }]);
});

test('multiple missions indexed correctly', () => {
  const results = [
    { sys_id: 1, waypoints: [{ lat: 0, lon: 0 }], altitude_m: 100 },
    { sys_id: 2, waypoints: [{ lat: 1, lon: 1 }], altitude_m: 120 },
    { sys_id: 3, waypoints: [{ lat: 2, lon: 2 }], altitude_m: 80 },
  ];
  const r = assembleMissionsFromResults(results);
  assert.strictEqual(r.zones.length, 3);
  assert.strictEqual(r.zones[0].zone_index, 0);
  assert.strictEqual(r.zones[1].zone_index, 1);
  assert.strictEqual(r.zones[2].zone_index, 2);
  assert.strictEqual(r.zones[0].sys_id, 1);
  assert.strictEqual(r.zones[2].sys_id, 3);
});

test('metadata extracted from first available mission', () => {
  const results = [
    { sys_id: 1, waypoints: [{ lat: 0, lon: 0 }], altitude_m: 100 },
    { sys_id: 2, waypoints: [{ lat: 1, lon: 1 }], altitude_m: 120,
      search_pattern: 'corridor', polygon: [{ lat: 0, lon: 0 }, { lat: 1, lon: 0 }, { lat: 0, lon: 1 }] },
  ];
  const r = assembleMissionsFromResults(results);
  assert.strictEqual(r.searchPattern, 'corridor');
  assert.strictEqual(r.polygon.length, 3);
});

test('missions without waypoints skipped', () => {
  const results = [
    { sys_id: 1, waypoints: [] },
    { sys_id: 2 },
    { sys_id: 3, waypoints: [{ lat: 1, lon: 1 }], altitude_m: 100 },
  ];
  const r = assembleMissionsFromResults(results);
  assert.strictEqual(r.zones.length, 1);
  assert.strictEqual(r.zones[0].sys_id, 3);
  assert.strictEqual(r.zones[0].zone_index, 0);
});

test('corridor_end_index defaults to 0', () => {
  const results = [{ sys_id: 1, waypoints: [{ lat: 0, lon: 0 }] }];
  const r = assembleMissionsFromResults(results);
  assert.strictEqual(r.zones[0].corridor_end_index, 0);
});

test('default altitude is 100 when not provided', () => {
  const results = [{ sys_id: 1, waypoints: [{ lat: 0, lon: 0 }] }];
  const r = assembleMissionsFromResults(results);
  assert.strictEqual(r.altitude, 100);
});

// --- mergeDownloadedMissionZones (single-vehicle download merge) ---

test('merge appends a new zone and re-indexes', () => {
  const existing = [{ zone_index: 0, sys_id: 4, track: [{ lat: 0, lon: 0 }] }];
  const zones = mergeDownloadedMissionZones(existing, { waypoints: [{ lat: 1, lon: 1 }], altitude_m: 120 }, 5);
  assert.strictEqual(zones.length, 2);
  assert.deepStrictEqual(zones.map((z) => z.sys_id), [4, 5]);
  assert.deepStrictEqual(zones.map((z) => z.zone_index), [0, 1]);
});

test('merge into empty/undefined plan yields one zone', () => {
  const zones = mergeDownloadedMissionZones(undefined, { waypoints: [{ lat: 1, lon: 1 }] }, 7);
  assert.strictEqual(zones.length, 1);
  assert.strictEqual(zones[0].sys_id, 7);
  assert.strictEqual(zones[0].zone_index, 0);
  assert.strictEqual(zones[0].corridor_end_index, 0);
});

test('merge preserves altitude only when present in waypoint', () => {
  const zones = mergeDownloadedMissionZones([], { waypoints: [{ lat: 1, lon: 1, alt: 90 }, { lat: 2, lon: 2 }] }, 7);
  assert.deepStrictEqual(zones[0].track[0], { lat: 1, lon: 1, alt: 90 });
  assert.deepStrictEqual(zones[0].track[1], { lat: 2, lon: 2 });
});

test('re-downloading a vehicle replaces its zone (no duplicate)', () => {
  const existing = [
    { zone_index: 0, sys_id: 4, track: [{ lat: 0, lon: 0 }] },
    { zone_index: 1, sys_id: 5, track: [{ lat: 1, lon: 1 }] },
  ];
  const zones = mergeDownloadedMissionZones(existing, { waypoints: [{ lat: 9, lon: 9 }] }, 4);
  assert.strictEqual(zones.length, 2, 'no duplicate zone for sys_id 4');
  assert.deepStrictEqual(zones.map((z) => z.sys_id), [5, 4]);
  assert.deepStrictEqual(zones.map((z) => z.zone_index), [0, 1]);
});

test('concurrent single downloads accumulate when threaded through a synchronous ref', () => {
  // Emulates downloadSingleMission: each call reads planRef.current, merges its
  // own zone, and writes the result straight back BEFORE React re-renders. Two
  // vehicles connected in quick succession run this back-to-back; the ref write
  // is what makes the second read see the first zone.
  const planRef = { current: null };
  const commit = (mission, sysId) => {
    const zones = mergeDownloadedMissionZones(planRef.current?.zones, mission, sysId);
    planRef.current = { zones };
    return zones;
  };
  commit({ waypoints: [{ lat: 4, lon: 4 }] }, 4);
  commit({ waypoints: [{ lat: 5, lon: 5 }] }, 5);
  commit({ waypoints: [{ lat: 6, lon: 6 }] }, 6);
  assert.strictEqual(planRef.current.zones.length, 3, 'all 3 vehicles kept');
  assert.deepStrictEqual(planRef.current.zones.map((z) => z.sys_id), [4, 5, 6]);

  // Documents the bug this fix prevents: reusing a stale snapshot (never writing
  // it back) makes each merge build off the empty plan, so only the last wins.
  const stale = null;
  const a = mergeDownloadedMissionZones(stale, { waypoints: [{ lat: 4, lon: 4 }] }, 4);
  const b = mergeDownloadedMissionZones(stale, { waypoints: [{ lat: 5, lon: 5 }] }, 5);
  assert.strictEqual(a.length, 1);
  assert.strictEqual(b.length, 1, 'stale reads drop a zone (last-write-wins)');
});

console.log(`All ${passed} mission assembly tests passed`);
