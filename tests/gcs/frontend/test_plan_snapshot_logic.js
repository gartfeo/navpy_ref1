/**
 * Tests for planSnapshot.js pure helpers (buildPlanSnapshot, isPlanDirty).
 *
 * Functions loaded from production source:
 *   src/gcs/frontend/src/utils/planSnapshot.js
 *
 * Run via: node tests/gcs/frontend/test_plan_snapshot_logic.js
 */
const assert = require('assert');
const path = require('path');
const { readFileSync } = require('fs');

// ---- Load production ESM module as CJS ----

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

const { buildPlanSnapshot, isPlanDirty } = loadEsm('utils/planSnapshot.js');

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
}

// ---- Helper: build a complete baseline state ----

function makeBaseline() {
  return {
    plan: { zones: [{ track: [[1, 2], [3, 4]], set_index: 0 }], altitude_m: 100 },
    polygon: [{ lat: 10, lon: 20 }, { lat: 11, lon: 21 }, { lat: 12, lon: 22 }],
    searchPattern: 'distributed',
    analysis: { min_uavs: 2, max_uavs: 4, sets: 1 },
    uavCount: 3,
    partitionAngleDeg: 45,
    setLaunchPoints: [{ lat: 10, lon: 20 }],
    setCorridorPointsArr: [[{ lat: 10.5, lon: 20.5 }]],
    deliveryHubAssignments: [0, 1],
    simDockWps: { 0: [1, 3] },
    detectAfterWps: { 0: 2 },
  };
}

// ---- isPlanDirty tests (13 cases) ----

test('isPlanDirty: identical baseline is not dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  assert.strictEqual(isPlanDirty(state, snap), false);
});

test('isPlanDirty: null snapshot returns false', () => {
  assert.strictEqual(isPlanDirty(makeBaseline(), null), false);
});

test('isPlanDirty: modified plan -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.plan.zones[0].track = [[5, 6]];
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified polygon -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.polygon[0] = { lat: 99, lon: 99 };
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified searchPattern -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.searchPattern = 'corridor';
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified analysis -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.analysis = { min_uavs: 5, max_uavs: 8, sets: 2 };
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified uavCount -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.uavCount = 7;
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified partitionAngleDeg -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.partitionAngleDeg = 90;
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified setLaunchPoints -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.setLaunchPoints = [{ lat: 99, lon: 99 }];
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified setCorridorPointsArr -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.setCorridorPointsArr = [[{ lat: 99, lon: 99 }]];
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified deliveryHubAssignments -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.deliveryHubAssignments = [2, 3];
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified simDockWps -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.simDockWps = { 0: [5] };
  assert.strictEqual(isPlanDirty(state, snap), true);
});

test('isPlanDirty: modified detectAfterWps -> dirty', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.detectAfterWps = { 0: 7 };
  assert.strictEqual(isPlanDirty(state, snap), true);
});

// ---- buildPlanSnapshot tests (6 cases) ----

test('buildPlanSnapshot: deep-copies plan', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.plan.zones[0].track.push([99, 99]);
  assert.strictEqual(snap.plan.zones[0].track.length, 2, 'snapshot plan must not be affected by mutation');
});

test('buildPlanSnapshot: normalizes polygon to { lat, lon } only', () => {
  const state = makeBaseline();
  state.polygon = [{ lat: 1, lon: 2, alt: 100, extra: 'junk' }];
  const snap = buildPlanSnapshot(state);
  assert.deepStrictEqual(snap.polygon, [{ lat: 1, lon: 2 }]);
});

test('buildPlanSnapshot: deep-copies launch/corridor arrays', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.setLaunchPoints[0] = { lat: 99, lon: 99 };
  state.setCorridorPointsArr[0].push({ lat: 99, lon: 99 });
  assert.notDeepStrictEqual(snap.setLaunchPoints[0], { lat: 99, lon: 99 });
  assert.strictEqual(snap.setCorridorPointsArr[0].length, 1);
});

test('buildPlanSnapshot: deep-copies analysis; null -> null', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.analysis.min_uavs = 999;
  assert.strictEqual(snap.analysis.min_uavs, 2, 'snapshot analysis must not be affected by mutation');

  const nullState = { ...makeBaseline(), analysis: null };
  const nullSnap = buildPlanSnapshot(nullState);
  assert.strictEqual(nullSnap.analysis, null);
});

test('buildPlanSnapshot: deep-copies DOCK/sim-POI/detect-after state', () => {
  const state = makeBaseline();
  const snap = buildPlanSnapshot(state);
  state.deliveryHubAssignments.push(5);
  state.simDockWps[0].push(99);
  state.detectAfterWps[1] = 9;
  assert.strictEqual(snap.deliveryHubAssignments.length, 2);
  assert.strictEqual(snap.simDockWps[0].length, 2);
  assert.strictEqual(snap.detectAfterWps[1], undefined);
});

test('buildPlanSnapshot: null-safe defaults for missing values', () => {
  const snap = buildPlanSnapshot({});
  assert.deepStrictEqual(snap.polygon, []);
  assert.strictEqual(snap.plan, null);
  assert.strictEqual(snap.searchPattern, null);
  // Every zone targets the single dock class; there is no class selection to snapshot.
  assert.strictEqual(Object.hasOwn(snap, 'dockClasses'), false);
  assert.strictEqual(Object.hasOwn(snap, 'perUavDockClasses'), false);
  assert.strictEqual(snap.analysis, null);
  assert.strictEqual(snap.uavCount, null);
  assert.strictEqual(snap.partitionAngleDeg, null);
  assert.deepStrictEqual(snap.setLaunchPoints, [null]);
  assert.deepStrictEqual(snap.setCorridorPointsArr, [[]]);
  assert.deepStrictEqual(snap.deliveryHubAssignments, []);
  assert.deepStrictEqual(snap.simDockWps, {});
  assert.deepStrictEqual(snap.detectAfterWps, {});
});

console.log(`PASS -- All ${passed} plan snapshot tests passed.`);
