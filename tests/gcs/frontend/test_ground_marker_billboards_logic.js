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
    makeAwaitingDockIcon,
    makeApprovedDockIcon,
    makeHandledDockIcon,
  } = await loadModule();
  const svgOf = (uri) => decodeURIComponent(uri.split(',')[1]);

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

  (function testEachDockStateHasItsOwnIcon() {
    const icons = [
      makeAvailableTaskIcon(32),
      makeAwaitingDockIcon(32),
      makeAssignmentIcon(32, '#00ff88'),
      makeApprovedDockIcon(32, '#00ff88'),
      makeHandledDockIcon(32, 'docked'),
      makeHandledDockIcon(32, 'failed'),
      makeHandledDockIcon(32, 'canceled'),
    ];
    assert.strictEqual(new Set(icons).size, icons.length);
    assert.strictEqual(makeHandledDockIcon(32, 'unknown'), makeHandledDockIcon(32, 'canceled'));
    const handled = decodeURIComponent(makeHandledDockIcon(32, 'docked').split(',')[1]);
    assert.ok(handled.includes('opacity="0.55"'), 'handled is dimmed');
  })();

  (function testEachOutcomeDrawsItsOwnBadge() {
    const CHECK = 'M15.5 6l2 2 3-4';
    const CROSS = 'M16 4l4 4m0-4l-4 4';
    const DASH = 'M15.5 6h5';
    assert.ok(svgOf(makeHandledDockIcon(32, 'docked')).includes(CHECK));
    assert.ok(svgOf(makeApprovedDockIcon(32, '#00ff88')).includes(CHECK));
    assert.ok(svgOf(makeHandledDockIcon(32, 'failed')).includes(CROSS));
    assert.ok(svgOf(makeHandledDockIcon(32, 'canceled')).includes(DASH));
    assert.ok(svgOf(makeHandledDockIcon(32, 'unknown')).includes(DASH));
    assert.ok(svgOf(makeHandledDockIcon(32, undefined)).includes(DASH), 'no word it knows claims nothing');
  })();

  (function testApprovedIsTheUavColourUndimmedAndHandledIsGrey() {
    const approved = svgOf(makeApprovedDockIcon(32, '#00ff88'));
    assert.ok(approved.includes('stroke="#00ff88"'));
    assert.ok(approved.includes('<g opacity="1"'));
    assert.ok(svgOf(makeHandledDockIcon(32, 'docked')).includes('stroke="#9aa7b4"'));
  })();

  (function testAwaitingIsAmberWithAQuestionMark() {
    const svg = svgOf(makeAwaitingDockIcon(32));
    assert.ok(svg.includes('stroke="#ffc107"'));
    assert.ok(svg.includes('>?</text>'));
  })();

  (function testHandledPadIsOpaqueUnderItsDimming() {
    const svg = decodeURIComponent(makeHandledDockIcon(32, 'docked').split(',')[1]);
    const pad = svg.indexOf('fill="#16364a"');
    assert.ok(pad >= 0 && pad < svg.indexOf('<g opacity="0.55"'), 'nothing below the marker shows through');
  })();

  console.log('PASS');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
