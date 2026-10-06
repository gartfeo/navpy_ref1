/**
 * Tests for sim target utilities used in useMissionUpload and useVehicleConnection.
 *
 * All functions are loaded from production source files:
 * - reindexZoneMap, buildTargWpsUpdates, buildNavLastWpUpdates
 *     from src/gcs/frontend/src/utils/simDockReindex.js
 * - decodeBitmaskArray, encodeBitmask
 *     from src/gcs/frontend/src/utils/bitmask.js
 *
 * Run via: node tests/gcs/frontend/test_sim_target_persist_logic.js
 */
const assert = require('assert');
const path = require('path');
const { readFileSync } = require('fs');

// ---- Load production ESM modules as CJS ----

function loadEsm(relPath) {
  const abs = path.resolve(__dirname, '..', '..', '..', 'src', 'gcs', 'frontend', 'src', relPath);
  let src = readFileSync(abs, 'utf8');
  src = src.replace(/export\s+function\s+(\w+)/g, 'function $1');
  src = src.replace(/export\s*\{[^}]*\}/g, '');
  const names = [];
  src.replace(/^function\s+(\w+)/gm, (_, n) => { names.push(n); return _; });
  src += '\nmodule.exports = { ' + names.join(', ') + ' };\n';
  const m = { exports: {} };
  new Function('module', 'exports', 'require', src)(m, m.exports, require);
  return m.exports;
}

const { reindexZoneMap, buildTargWpsUpdates, buildNavLastWpUpdates } = loadEsm('utils/simDockReindex.js');
const { decodeBitmaskArray, encodeBitmask } = loadEsm('utils/bitmask.js');

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
}

// ---- reindexZoneMap ----

test('reindex array: remove middle zone', () => {
  const map = { 0: [1, 3], 1: [2], 2: [0, 4] };
  assert.deepStrictEqual(reindexZoneMap(map, [0, 2], 'array'), { 0: [1, 3], 1: [0, 4] });
});

test('reindex array: remove first zone', () => {
  assert.deepStrictEqual(
    reindexZoneMap({ 0: [5], 1: [2], 2: [0] }, [1, 2], 'array'),
    { 0: [2], 1: [0] },
  );
});

test('reindex array: remove last zone', () => {
  assert.deepStrictEqual(
    reindexZoneMap({ 0: [1], 1: [3], 2: [7] }, [0, 1], 'array'),
    { 0: [1], 1: [3] },
  );
});

test('reindex array: empty arrays dropped', () => {
  assert.deepStrictEqual(reindexZoneMap({ 0: [], 1: [2], 2: [] }, [0, 1, 2], 'array'), { 1: [2] });
});

test('reindex array: empty input', () => {
  assert.deepStrictEqual(reindexZoneMap({}, [0, 2], 'array'), {});
});

test('reindex scalar: remove middle zone', () => {
  assert.deepStrictEqual(reindexZoneMap({ 0: 5, 1: 3, 2: 7 }, [0, 2], 'scalar'), { 0: 5, 1: 7 });
});

test('reindex scalar: null dropped, zero preserved', () => {
  assert.deepStrictEqual(
    reindexZoneMap({ 0: null, 1: 0, 2: 7 }, [0, 1, 2], 'scalar'),
    { 1: 0, 2: 7 },
  );
});

test('reindex: single zone kept', () => {
  assert.deepStrictEqual(reindexZoneMap({ 0: [1], 1: [2], 2: [3] }, [1], 'array'), { 0: [2] });
});

// ---- buildTargWpsUpdates (production upload logic) ----

const zones2 = [{ set_index: 0 }, { set_index: 0 }];
const vehicles2 = [{ sys_id: 10 }, { sys_id: 11 }];
const corridorLens2 = [2, 2];

test('targ_wps: bitmask 0 when all targets removed', () => {
  const { updates, calls } = buildTargWpsUpdates({ 0: [], 1: [] }, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 0);
  assert.strictEqual(updates[11], 0);
  assert.strictEqual(calls.length, 2);
  assert.strictEqual(calls[0].targ_wps, 0);
  assert.strictEqual(calls[1].targ_wps, 0);
});

test('targ_wps: normal case computes correct bitmasks', () => {
  // Zone 0: WP indices [0,2] + corridorLen 2 → wpNums 3,5 → bits 2,4 → 4+16=20
  // Zone 1: WP index [1] + corridorLen 2 → wpNum 4 → bit 3 → 8
  const { updates } = buildTargWpsUpdates({ 0: [0, 2], 1: [1] }, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 20);
  assert.strictEqual(updates[11], 8);
});

test('targ_wps: empty object writes 0 for all zones', () => {
  const { updates, calls } = buildTargWpsUpdates({}, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 0);
  assert.strictEqual(updates[11], 0);
  assert.strictEqual(calls.length, 2);
});

test('targ_wps: null simDockWps writes 0', () => {
  const { updates } = buildTargWpsUpdates(null, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 0);
  assert.strictEqual(updates[11], 0);
});

test('targ_wps: undefined simDockWps writes 0', () => {
  const { updates } = buildTargWpsUpdates(undefined, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 0);
});

test('targ_wps: corridor mode forces corridorLen to 0', () => {
  const { updates } = buildTargWpsUpdates({ 0: [1] }, zones2, vehicles2, true, corridorLens2);
  assert.strictEqual(updates[10], 2); // wpNum=2, bit 1 → 2
});

test('targ_wps: skips missing vehicle', () => {
  const { updates, calls } = buildTargWpsUpdates({ 0: [0], 1: [0] }, zones2, [{ sys_id: 10 }, null], false, corridorLens2);
  assert.strictEqual(Object.keys(updates).length, 1);
  assert.strictEqual(updates[10], 4);
  assert.strictEqual(calls.length, 1);
});

// ---- buildNavLastWpUpdates (production upload logic) ----

test('nav_last_wp: cleared when no entry', () => {
  const { updates, calls } = buildNavLastWpUpdates({}, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 0);
  assert.strictEqual(updates[11], 0);
  assert.strictEqual(calls.length, 2);
  assert.strictEqual(calls[0].nav_last_wp, 0);
});

test('nav_last_wp: null detectAfterWps writes 0', () => {
  const { updates } = buildNavLastWpUpdates(null, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 0);
  assert.strictEqual(updates[11], 0);
});

test('nav_last_wp: normal case', () => {
  // Zone 0: detect after WP 3, corridorLen=2 → 2+3+1 = 6
  // Zone 1: detect after WP 0, corridorLen=2 → 2+0+1 = 3
  const { updates } = buildNavLastWpUpdates({ 0: 3, 1: 0 }, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 6);
  assert.strictEqual(updates[11], 3);
});

test('nav_last_wp: partial clear', () => {
  const { updates, calls } = buildNavLastWpUpdates({ 0: 2 }, zones2, vehicles2, false, corridorLens2);
  assert.strictEqual(updates[10], 5); // 2+2+1
  assert.strictEqual(updates[11], 0); // cleared
  assert.strictEqual(calls.length, 2);
});

test('nav_last_wp: corridor mode', () => {
  const { updates } = buildNavLastWpUpdates({ 0: 4 }, zones2, vehicles2, true, corridorLens2);
  assert.strictEqual(updates[10], 5); // 0+4+1
});

// ---- bitmask round-trip (production imports) ----

test('bitmask round-trip: encodeBitmask ↔ decodeBitmaskArray', () => {
  const bitmask = encodeBitmask('3,5');
  assert.strictEqual(bitmask, 20);
  assert.deepStrictEqual(decodeBitmaskArray(bitmask), [3, 5]);
});

test('decodeBitmaskArray: 0 → empty', () => {
  assert.deepStrictEqual(decodeBitmaskArray(0), []);
});

console.log(`PASS -- All ${passed} sim target tests passed (production imports).`);
