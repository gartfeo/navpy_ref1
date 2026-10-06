/**
 * Tests for missionProgress.js pure helpers (computeZoneDistances, interpolatedDistance).
 *
 * Functions loaded from production source:
 *   src/gcs/frontend/src/utils/geo.js          (flatDist dependency)
 *   src/gcs/frontend/src/utils/missionProgress.js
 *
 * Run via: node tests/gcs/frontend/test_mission_progress_logic.js
 */
const assert = require('assert');
const path = require('path');
const { readFileSync } = require('fs');

// ---- Load production ESM modules as CJS ----

function loadEsm(relPath, deps) {
  const abs = path.resolve(__dirname, '..', '..', '..', 'src', 'gcs', 'frontend', 'src', relPath);
  let src = readFileSync(abs, 'utf8');
  // Strip import statements
  src = src.replace(/import\s+\{[^}]*\}\s+from\s+['"][^'"]+['"];?\n?/g, '');
  src = src.replace(/import\s+\w+\s+from\s+['"][^'"]+['"];?\n?/g, '');
  // Strip export keywords
  src = src.replace(/export\s+(const|function|class)\s+/g, '$1 ');
  src = src.replace(/export\s*\{[^}]*\}/g, '');
  src = src.replace(/export\s+default\s+/g, '');
  // Discover top-level names
  const names = [];
  src.replace(/^(?:const|function|class)\s+(\w+)/gm, (_, n) => { names.push(n); return _; });
  src += '\nmodule.exports = { ' + names.join(', ') + ' };\n';
  const m = { exports: {} };
  // Inject dependencies into scope
  const depKeys = Object.keys(deps || {});
  const depVals = depKeys.map(k => deps[k]);
  new Function('module', 'exports', 'require', ...depKeys, src)(m, m.exports, require, ...depVals);
  return m.exports;
}

const geo = loadEsm('utils/geo.js');
const { flatDist } = geo;
const { computeZoneDistances, interpolatedDistance } = loadEsm('utils/missionProgress.js', { flatDist });

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
}

// ---- computeZoneDistances ----

test('null plan returns empty array', () => {
  assert.deepStrictEqual(computeZoneDistances(null), []);
});

test('empty zones returns empty array', () => {
  assert.deepStrictEqual(computeZoneDistances({ zones: [] }), []);
});

test('single waypoint zone returns total 0', () => {
  const plan = { zones: [{ track: [{ lat: 0, lon: 0 }] }] };
  const result = computeZoneDistances(plan);
  assert.strictEqual(result.length, 1);
  assert.strictEqual(result[0].total, 0);
  assert.deepStrictEqual(result[0].cumulative, []);
});

test('two waypoint zone computes correct distance', () => {
  const a = { lat: 0, lon: 0 };
  const b = { lat: 0, lon: 1 };
  const plan = { zones: [{ track: [a, b] }] };
  const result = computeZoneDistances(plan);
  assert.strictEqual(result.length, 1);
  assert.strictEqual(result[0].cumulative.length, 2);
  assert.strictEqual(result[0].cumulative[0], 0);
  const expected = flatDist(a, b);
  assert.ok(Math.abs(result[0].total - expected) < 0.01, `total should be ~${expected}`);
});

test('multi waypoint cumulative distances are monotonic', () => {
  const pts = [
    { lat: 0, lon: 0 },
    { lat: 0, lon: 1 },
    { lat: 0, lon: 3 },
  ];
  const plan = { zones: [{ track: pts }] };
  const result = computeZoneDistances(plan);
  const d01 = flatDist(pts[0], pts[1]);
  const d12 = flatDist(pts[1], pts[2]);
  assert.strictEqual(result[0].cumulative[0], 0);
  assert.ok(Math.abs(result[0].cumulative[1] - d01) < 0.01);
  assert.ok(Math.abs(result[0].cumulative[2] - (d01 + d12)) < 0.01);
  assert.ok(Math.abs(result[0].total - (d01 + d12)) < 0.01);
});

test('multiple zones computed independently', () => {
  const plan = {
    zones: [
      { track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }] },
      { track: [{ lat: 10, lon: 10 }, { lat: 10, lon: 11 }] },
    ],
  };
  const result = computeZoneDistances(plan);
  assert.strictEqual(result.length, 2);
  assert.ok(result[0].total > 0);
  assert.ok(result[1].total > 0);
});

// ---- interpolatedDistance ----

test('null zone distance returns 0', () => {
  assert.strictEqual(interpolatedDistance({ mission_progress: 5 }, null, 0), 0);
});

test('vehicle not started returns 0', () => {
  const zd = {
    track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }],
    cumulative: [0, 100],
    total: 100,
  };
  assert.strictEqual(interpolatedDistance({ mission_progress: 0 }, zd, 0), 0);
});

test('vehicle before wpOffset returns 0', () => {
  const zd = {
    track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }],
    cumulative: [0, 100],
    total: 100,
  };
  assert.strictEqual(interpolatedDistance({ mission_progress: 2 }, zd, 5), 0);
});

test('mid-segment interpolation', () => {
  const zd = {
    track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }],
    cumulative: [0, 100],
    total: 100,
  };
  const v = { mission_progress: 1, lat: 0, lon: 0.5 };
  const dist = interpolatedDistance(v, zd, 0);
  assert.ok(Math.abs(dist - 50) < 0.01, `midpoint should be ~50, got ${dist}`);
});

test('end of track returns total distance', () => {
  const zd = {
    track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 0, lon: 2 }],
    cumulative: [0, 100, 200],
    total: 200,
  };
  const v = { mission_progress: 2, lat: 0, lon: 2 };
  const dist = interpolatedDistance(v, zd, 0);
  assert.ok(Math.abs(dist - 200) < 0.01, `end should be ~200, got ${dist}`);
});

test('wpOffset shifts track index correctly', () => {
  const zd = {
    track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }],
    cumulative: [0, 100],
    total: 100,
  };
  const v = { mission_progress: 4, lat: 0, lon: 0.5 };
  const dist = interpolatedDistance(v, zd, 3);
  assert.ok(Math.abs(dist - 50) < 0.01, `with wpOffset=3, midpoint should be ~50, got ${dist}`);
});

test('null lat/lon returns base distance', () => {
  const zd = {
    track: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }],
    cumulative: [0, 100],
    total: 100,
  };
  const v = { mission_progress: 1, lat: null, lon: null };
  const dist = interpolatedDistance(v, zd, 0);
  assert.strictEqual(dist, 0, 'null lat/lon returns baseDist of segment start');
});

console.log(`All ${passed} mission progress tests passed`);
