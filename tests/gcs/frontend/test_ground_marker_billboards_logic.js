/**
 * Node.js tests for ground-marker SVG icon helpers.
 *
 * Marker label text is rendered as a separate Cesium label (not baked into the
 * SVG), so these tests cover the icon-only helpers that the marker hooks actually
 * use: makeAvailableTaskIcon and makeAssignmentIcon.
 *
 * Run via: node tests/gcs/frontend/test_ground_marker_billboards_logic.js
 */

const assert = require('assert');
const { pathToFileURL } = require('url');
const path = require('path');

async function loadModule() {
  const src = path.resolve(
    __dirname,
    '../../../src/gcs/frontend/src/components/map/constants/icons.js',
  );
  return import(pathToFileURL(src).href);
}

(async () => {
  const {
    makeAvailableTaskIcon,
    makeAssignmentIcon,
  } = await loadModule();

  (function testAvailableTaskIconIsDashedOrangeCircle() {
    const uri = makeAvailableTaskIcon(32);
    assert.ok(uri.startsWith('data:image/svg+xml,'));
    const svg = decodeURIComponent(uri.split(',')[1]);
    assert.ok(svg.includes('width="32"'));
    assert.ok(svg.includes('height="32"'));
    assert.ok(svg.includes('stroke="#ff9800"'));
    assert.ok(svg.includes('stroke-dasharray="4,3"'));
    // Center dot uses the same orange fill.
    assert.ok(svg.includes('fill="#ff9800"'));
  })();

  (function testAssignmentIconUsesGivenColorAndDockPad() {
    const uri = makeAssignmentIcon(32, '#00ff88');
    assert.ok(uri.startsWith('data:image/svg+xml,'));
    const svg = decodeURIComponent(uri.split(',')[1]);
    assert.ok(svg.includes('width="32"'));
    assert.ok(svg.includes('height="32"'));
    assert.ok(svg.includes('stroke="#00ff88"'));
    // Dock pad outline and D-shaped path, with the assignment color retained.
    assert.strictEqual((svg.match(/<rect/g) || []).length, 1);
    assert.strictEqual((svg.match(/<path/g) || []).length, 1);
    assert.ok(svg.includes('M9 7h3a5 5 0 0 1 0 10H9z'));
  })();

  console.log('PASS');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
