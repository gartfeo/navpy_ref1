/**
 * Tests for trailAccum.js pure utility functions.
 * Run via: node tests/gcs/frontend/test_uav_trails_logic.js
 *
 * Functions are inlined here since ES module imports can't be used directly.
 */

const MAX_TRAIL_LENGTH = 200;
const MIN_TRAIL_STEP_DEG2 = 3.2e-10;

function distDeg2(a, b) {
  const dx = a.lon - b.lon;
  const dy = a.lat - b.lat;
  return dx * dx + dy * dy;
}

function accumulate(trail, pt) {
  if (trail.length === 0) {
    trail.push(pt);
    return trail;
  }
  if (distDeg2(trail[trail.length - 1], pt) < MIN_TRAIL_STEP_DEG2) {
    return trail;
  }
  trail.push(pt);
  if (trail.length > MAX_TRAIL_LENGTH) {
    trail.splice(0, trail.length - MAX_TRAIL_LENGTH);
  }
  return trail;
}

// --- tests ---

let passed = 0;
let failed = 0;

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg);
    failed++;
  } else {
    console.log('PASS:', msg);
    passed++;
  }
}

function assertClose(a, b, msg, tol) {
  tol = tol || 1e-12;
  if (Math.abs(a - b) > tol) {
    console.error('FAIL:', msg, '— got', a, 'expected', b);
    failed++;
  } else {
    console.log('PASS:', msg);
    passed++;
  }
}

// Test 1: first point always added
{
  const trail = [];
  accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
  assert(trail.length === 1, 'first point always added to empty trail');
}

// Test 2: stationary vehicle doesn't duplicate
{
  const trail = [];
  const pt = { lon: 35.0, lat: 32.0, alt: 100 };
  accumulate(trail, pt);
  accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
  accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
  assert(trail.length === 1, 'stationary vehicle does not add duplicates');
}

// Test 3: moving vehicle accumulates points
{
  const trail = [];
  accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
  accumulate(trail, { lon: 35.001, lat: 32.0, alt: 100 });
  accumulate(trail, { lon: 35.002, lat: 32.0, alt: 100 });
  assert(trail.length === 3, 'moving vehicle accumulates points');
}

// Test 4: trail truncates beyond MAX_TRAIL_LENGTH
{
  const trail = [];
  for (let i = 0; i <= MAX_TRAIL_LENGTH + 10; i++) {
    accumulate(trail, { lon: 35.0 + i * 0.001, lat: 32.0, alt: 100 });
  }
  assert(trail.length === MAX_TRAIL_LENGTH, 'trail truncates to MAX_TRAIL_LENGTH=' + MAX_TRAIL_LENGTH);
}

// Test 5: distDeg2 returns correct squared distance
{
  const a = { lon: 0, lat: 0 };
  const b = { lon: 3, lat: 4 };
  assertClose(distDeg2(a, b), 25, 'distDeg2 returns 3^2 + 4^2 = 25');
}

// Test 6: sub-threshold movement rejected
{
  const trail = [];
  accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
  accumulate(trail, { lon: 35.000005, lat: 32.0, alt: 100 });
  assert(trail.length === 1, 'sub-threshold movement rejected');
}

// Summary
console.log('\n' + passed + ' passed, ' + failed + ' failed');
if (failed > 0) process.exit(1);
