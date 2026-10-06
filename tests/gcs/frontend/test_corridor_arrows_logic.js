/**
 * Node.js tests for per-segment arrow polyline logic.
 *
 * Both the corridor (useCorridorLayer.js) and track (useTrackLayer.js) use
 * per-segment PolylineArrowMaterialProperty polylines for direction indication.
 * Each segment connects consecutive positions, producing one arrowhead per segment.
 *
 * Run via: node tests/gcs/frontend/test_corridor_arrows_logic.js
 */

const assert = require('assert');

/**
 * Replicate the per-segment splitting used in both useCorridorLayer and
 * useTrackLayer: given an array of positions, return pairs of consecutive
 * positions (one per segment).
 */
function perSegmentPairs(positions) {
  const pairs = [];
  for (let i = 0; i < positions.length - 1; i++) {
    pairs.push([positions[i], positions[i + 1]]);
  }
  return pairs;
}

// ---- Test: correct number of segments ----
(function testSegmentCount() {
  const pts = [{ lon: 0, lat: 0 }, { lon: 1, lat: 0 }, { lon: 2, lat: 0 }];
  const pairs = perSegmentPairs(pts);
  assert.strictEqual(pairs.length, 2, `3 points should produce 2 segments, got ${pairs.length}`);
})();

// ---- Test: single segment from 2 points ----
(function testTwoPoints() {
  const pts = [{ lon: 34, lat: 32 }, { lon: 35, lat: 33 }];
  const pairs = perSegmentPairs(pts);
  assert.strictEqual(pairs.length, 1, `2 points should produce 1 segment`);
  assert.deepStrictEqual(pairs[0][0], pts[0]);
  assert.deepStrictEqual(pairs[0][1], pts[1]);
})();

// ---- Test: empty or single-point returns no segments ----
(function testTooFewPoints() {
  assert.strictEqual(perSegmentPairs([]).length, 0, 'Empty array should produce 0 segments');
  assert.strictEqual(perSegmentPairs([{ lon: 0, lat: 0 }]).length, 0, 'Single point should produce 0 segments');
})();

// ---- Test: each pair connects consecutive points ----
(function testConsecutivePairs() {
  const pts = [
    { lon: 0, lat: 0 },
    { lon: 1, lat: 1 },
    { lon: 2, lat: 0 },
    { lon: 3, lat: 1 },
  ];
  const pairs = perSegmentPairs(pts);
  assert.strictEqual(pairs.length, 3);
  for (let i = 0; i < pairs.length; i++) {
    assert.deepStrictEqual(pairs[i][0], pts[i], `Pair ${i} start should be point ${i}`);
    assert.deepStrictEqual(pairs[i][1], pts[i + 1], `Pair ${i} end should be point ${i + 1}`);
  }
})();

// ---- Test: corridor segment count (LP + CPs + trackStart) ----
(function testCorridorSegmentCount() {
  // Corridor has LP → CP1 → CP2 → trackStart = 3 segments from 4 points
  const lp = { lon: 34.0, lat: 32.0 };
  const cps = [{ lon: 34.1, lat: 32.1 }, { lon: 34.2, lat: 32.2 }];
  const trackStart = { lon: 34.3, lat: 32.3 };
  const allPts = [lp, ...cps, trackStart];
  const pairs = perSegmentPairs(allPts);
  assert.strictEqual(pairs.length, cps.length + 1, `LP + ${cps.length} CPs + trackStart should produce ${cps.length + 1} segments`);
})();

console.log('PASS');
