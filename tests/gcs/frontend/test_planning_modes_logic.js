/**
 * Tests for planningModes.js pure helpers.
 *
 * Functions loaded from production source:
 *   src/gcs/frontend/src/utils/planningModes.js
 *
 * Run via: node tests/gcs/frontend/test_planning_modes_logic.js
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

const { nextPlacementState, toggleSimDock } = loadEsm('utils/planningModes.js');

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
}

// ---- nextPlacementState ----

const allIdle = { placingLaunchPoint: false, placingCorridor: false, placingFallbackLocation: false, isDrawing: false };
const drawing = { ...allIdle, isDrawing: true };

test('startDraw: clears all placement modes, sets shouldStartDraw', () => {
  const state = { placingLaunchPoint: true, placingCorridor: true, placingFallbackLocation: true, isDrawing: false };
  const result = nextPlacementState(state, 'startDraw');
  assert.strictEqual(result.placingLaunchPoint, false);
  assert.strictEqual(result.placingCorridor, false);
  assert.strictEqual(result.placingFallbackLocation, false);
  assert.strictEqual(result.shouldStartDraw, true);
  assert.strictEqual(result.shouldStopDraw, false);
});

test('toggleCorridor on: clears other modes, sets shouldStopDraw when drawing', () => {
  const state = { placingLaunchPoint: true, placingCorridor: false, placingFallbackLocation: true, isDrawing: true };
  const result = nextPlacementState(state, 'toggleCorridor');
  assert.strictEqual(result.placingCorridor, true);
  assert.strictEqual(result.placingLaunchPoint, false);
  assert.strictEqual(result.placingFallbackLocation, false);
  assert.strictEqual(result.shouldStopDraw, true);
  assert.strictEqual(result.shouldStartDraw, false);
});

test('startFallbackLocationFromSettings: clears all modes, sets placingFallbackLocation, shouldStopDraw when drawing', () => {
  const state = { placingLaunchPoint: true, placingCorridor: true, placingFallbackLocation: false, isDrawing: true };
  const result = nextPlacementState(state, 'startFallbackLocationFromSettings');
  assert.strictEqual(result.placingFallbackLocation, true);
  assert.strictEqual(result.placingLaunchPoint, false);
  assert.strictEqual(result.placingCorridor, false);
  assert.strictEqual(result.shouldStopDraw, true);
  assert.strictEqual(result.shouldStartDraw, false);
});

test('toggleCorridor off: only clears corridor, preserves other state', () => {
  const state = { placingLaunchPoint: false, placingCorridor: true, placingFallbackLocation: false, isDrawing: false };
  const result = nextPlacementState(state, 'toggleCorridor');
  assert.strictEqual(result.placingCorridor, false);
  assert.strictEqual(result.placingLaunchPoint, false);
  assert.strictEqual(result.placingFallbackLocation, false);
  assert.strictEqual(result.shouldStopDraw, false);
  assert.strictEqual(result.shouldStartDraw, false);
});

test('toggleFallbackLocation off: only clears DOCK, preserves other state', () => {
  const state = { placingLaunchPoint: false, placingCorridor: true, placingFallbackLocation: true, isDrawing: false };
  const result = nextPlacementState(state, 'toggleFallbackLocation');
  assert.strictEqual(result.placingFallbackLocation, false);
  assert.strictEqual(result.placingCorridor, true);
  assert.strictEqual(result.placingLaunchPoint, false);
  assert.strictEqual(result.shouldStopDraw, false);
});

test('shouldStopDraw is false when draw was not active', () => {
  const result = nextPlacementState(allIdle, 'toggleCorridor');
  assert.strictEqual(result.shouldStopDraw, false);
  assert.strictEqual(result.placingCorridor, true);

  const result2 = nextPlacementState(allIdle, 'toggleFallbackLocation');
  assert.strictEqual(result2.shouldStopDraw, false);
  assert.strictEqual(result2.placingFallbackLocation, true);

  const result3 = nextPlacementState(allIdle, 'startFallbackLocationFromSettings');
  assert.strictEqual(result3.shouldStopDraw, false);
  assert.strictEqual(result3.placingFallbackLocation, true);
});

test('startDraw: shouldStopDraw false even when already drawing', () => {
  // startDraw doesn't need to stop — useDrawing handles re-entry
  const result = nextPlacementState(drawing, 'startDraw');
  assert.strictEqual(result.shouldStartDraw, true);
  assert.strictEqual(result.shouldStopDraw, false);
});

test('toggleFallbackLocation on: clears other modes, sets shouldStopDraw when drawing', () => {
  const state = { placingLaunchPoint: true, placingCorridor: true, placingFallbackLocation: false, isDrawing: true };
  const result = nextPlacementState(state, 'toggleFallbackLocation');
  assert.strictEqual(result.placingFallbackLocation, true);
  assert.strictEqual(result.placingLaunchPoint, false);
  assert.strictEqual(result.placingCorridor, false);
  assert.strictEqual(result.shouldStopDraw, true);
});

// ---- toggleSimDock ----

test('toggleSimDock: add wpIndex to empty zone', () => {
  const result = toggleSimDock({}, 0, 5);
  assert.deepStrictEqual(result, { 0: [5] });
});

test('toggleSimDock: remove existing wpIndex, zone key stays with empty array', () => {
  const result = toggleSimDock({ 0: [3] }, 0, 3);
  assert.deepStrictEqual(result, { 0: [] });
  // Zone key must remain — usePlanPersistence serializes based on key presence
  assert.ok(result.hasOwnProperty('0'), 'zone key must stay present with empty array');
});

test('toggleSimDock: add second wpIndex to existing zone', () => {
  const result = toggleSimDock({ 0: [2] }, 0, 5);
  assert.deepStrictEqual(result, { 0: [2, 5] });
});

test('toggleSimDock: preserves other zones', () => {
  const result = toggleSimDock({ 0: [1], 1: [3, 4] }, 0, 7);
  assert.deepStrictEqual(result, { 0: [1, 7], 1: [3, 4] });
});

console.log(`PASS -- All ${passed} planning modes tests passed.`);
