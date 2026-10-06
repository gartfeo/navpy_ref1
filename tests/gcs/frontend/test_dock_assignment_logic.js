/**
 * Node.js tests for buildFallbackLocationsFromDownload merge logic.
 *
 * Verifies that downloaded POIs are matched against existing Docks by
 * coordinates, preserving type/name for matches and keeping unmatched
 * existing Docks intact.
 *
 * Run via: node tests/gcs/frontend/test_dock_assignment_logic.js
 */

const assert = require('assert');
const { pathToFileURL } = require('url');
const path = require('path');

async function loadModule() {
  const src = path.resolve(
    __dirname,
    '../../../src/gcs/frontend/src/utils/fallbackLocationAssignment.js',
  );
  const mod = await import(pathToFileURL(src).href);
  return mod;
}

(async () => {
  const { buildFallbackLocationsFromDownload } = await loadModule();

  // ---- Test: backward compat — no existingFallbackLocations behaves like before ----
  (function testNoExistingDocks() {
    const pois = [
      { lat: 32.0, lon: 34.0 },
      { lat: 33.0, lon: 35.0 },
    ];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois);
    assert.strictEqual(fallbackLocations.length, 2, 'Should create 2 Docks');
    assert.strictEqual(fallbackLocations[0].type, 'other', 'Default type should be other');
    assert.strictEqual(fallbackLocations[0].name, 'Fallback delivery location 1');
    assert.strictEqual(fallbackLocations[1].name, 'Fallback delivery location 2');
    assert.deepStrictEqual(assignments, [0, 1]);
  })();

  // ---- Test: matching existing DOCK preserves type and name ----
  (function testMatchExistingPreservesTypeAndName() {
    const existing = [
      { name: 'HQ', type: 'building', lat: 32.0, lon: 34.0 },
    ];
    const pois = [{ lat: 32.0, lon: 34.0 }];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(fallbackLocations.length, 1, 'Should not create a duplicate');
    assert.strictEqual(fallbackLocations[0].type, 'building', 'Type should be preserved');
    assert.strictEqual(fallbackLocations[0].name, 'HQ', 'Name should be preserved');
    assert.deepStrictEqual(assignments, [0]);
  })();

  // ---- Test: new POI creates type "other" DOCK ----
  (function testNewPoiCreatesOther() {
    const existing = [
      { name: 'HQ', type: 'building', lat: 32.0, lon: 34.0 },
    ];
    const pois = [{ lat: 40.0, lon: 50.0 }];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(fallbackLocations.length, 2, 'Should add one new DOCK');
    assert.strictEqual(fallbackLocations[0].name, 'HQ', 'Existing preserved');
    assert.strictEqual(fallbackLocations[1].type, 'other', 'New DOCK should be type other');
    assert.strictEqual(fallbackLocations[1].name, 'Fallback delivery location 2', 'Name based on total count');
    assert.deepStrictEqual(assignments, [1]);
  })();

  // ---- Test: downloaded type is preserved for new DOCK ----
  (function testDownloadedTypePreservedForNewPoi() {
    const existing = [
      { name: 'HQ', type: 'building', lat: 32.0, lon: 34.0 },
    ];
    const pois = [{ lat: 40.0, lon: 50.0, type: 'bridge' }];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(fallbackLocations.length, 2, 'Should add one new DOCK');
    assert.strictEqual(fallbackLocations[1].type, 'bridge', 'Downloaded type should be preserved');
    assert.deepStrictEqual(assignments, [1]);
  })();

  // ---- Test: existing Docks not in download are preserved ----
  (function testExistingDocksPreserved() {
    const existing = [
      { name: 'Alpha', type: 'vehicle', lat: 10.0, lon: 20.0 },
      { name: 'Bravo', type: 'person', lat: 30.0, lon: 40.0 },
    ];
    const pois = [{ lat: 50.0, lon: 60.0 }];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(fallbackLocations.length, 3, 'Existing 2 + new 1');
    assert.strictEqual(fallbackLocations[0].name, 'Alpha');
    assert.strictEqual(fallbackLocations[1].name, 'Bravo');
    assert.strictEqual(fallbackLocations[2].type, 'other');
    assert.deepStrictEqual(assignments, [2]);
  })();

  // ---- Test: dedup within downloaded POIs still works ----
  (function testDedupWithinDownload() {
    const pois = [
      { lat: 32.0, lon: 34.0 },
      { lat: 32.0, lon: 34.0 }, // duplicate
      { lat: 33.0, lon: 35.0 },
    ];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois);
    assert.strictEqual(fallbackLocations.length, 2, 'Duplicates should be merged');
    assert.deepStrictEqual(assignments, [0, 0, 1]);
  })();

  // ---- Test: empty missionFallbackLocations with existingFallbackLocations returns copy of existing ----
  (function testEmptyPoisReturnsExisting() {
    const existing = [
      { name: 'HQ', type: 'building', lat: 32.0, lon: 34.0 },
    ];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload([], existing);
    assert.strictEqual(fallbackLocations.length, 1, 'Should return existing Docks');
    assert.strictEqual(fallbackLocations[0].name, 'HQ');
    assert.deepStrictEqual(assignments, []);
  })();

  // ---- Test: null POIs in array are handled ----
  (function testNullPoisInArray() {
    const existing = [
      { name: 'HQ', type: 'building', lat: 32.0, lon: 34.0 },
    ];
    const pois = [null, { lat: 32.0, lon: 34.0 }, null];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(fallbackLocations.length, 1, 'Only one valid POI matches existing');
    assert.deepStrictEqual(assignments, [null, 0, null]);
  })();

  // ---- Test: does not mutate existingFallbackLocations input ----
  (function testDoesNotMutateInput() {
    const existing = [
      { name: 'HQ', type: 'building', lat: 32.0, lon: 34.0 },
    ];
    const pois = [{ lat: 50.0, lon: 60.0 }];
    buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(existing.length, 1, 'Original array should not be modified');
  })();

  // ---- Test: mixed match and new POIs ----
  (function testMixedMatchAndNew() {
    const existing = [
      { name: 'Alpha', type: 'vehicle', lat: 10.0, lon: 20.0 },
      { name: 'Bravo', type: 'person', lat: 30.0, lon: 40.0 },
    ];
    const pois = [
      { lat: 30.0, lon: 40.0 }, // matches Bravo
      { lat: 50.0, lon: 60.0 }, // new
      { lat: 10.0, lon: 20.0 }, // matches Alpha
    ];
    const { fallbackLocations, assignments } = buildFallbackLocationsFromDownload(pois, existing);
    assert.strictEqual(fallbackLocations.length, 3, 'Existing 2 + new 1');
    assert.strictEqual(fallbackLocations[0].name, 'Alpha');
    assert.strictEqual(fallbackLocations[1].name, 'Bravo');
    assert.strictEqual(fallbackLocations[2].type, 'other');
    assert.deepStrictEqual(assignments, [1, 2, 0]);
  })();

  console.log('PASS');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
