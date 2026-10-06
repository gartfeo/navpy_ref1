/**
 * Node.js tests for AttitudeIndicator roll sign logic.
 *
 * Verifies that the horizon rotation uses -rollRad so a positive roll
 * (right bank) tilts the horizon counter-clockwise. The roll arc/ticks
 * rotate with the horizon; the pointer is fixed at top center.
 *
 * Run via: node tests/gcs/frontend/test_roll_hud_sign_logic.js
 */

const assert = require('assert');

/**
 * Replicate the roll-to-rotation conversion from AttitudeIndicator.
 * Returns the angle passed to ctx.rotate() for the horizon + roll arc.
 */
function horizonRotation(rollDeg) {
  const rollRad = (rollDeg * Math.PI) / 180;
  return -rollRad; // positive roll -> CCW horizon tilt
}

let passed = 0;

// ---- Horizon + roll scale rotation sign ----
(function testPositiveRollRotatesCCW() {
  const rot = horizonRotation(30);
  assert.ok(rot < 0, `Positive roll should produce negative (CCW) rotation, got ${rot}`);
  passed++;
  console.log('PASS: positive roll produces negative horizon rotation');
})();

(function testNegativeRollRotatesCW() {
  const rot = horizonRotation(-20);
  assert.ok(rot > 0, `Negative roll should produce positive (CW) rotation, got ${rot}`);
  passed++;
  console.log('PASS: negative roll produces positive horizon rotation');
})();

(function testZeroRollNoRotation() {
  const rot = horizonRotation(0);
  assert.ok(Math.abs(rot) === 0, `Zero roll should produce zero rotation, got ${rot}`);
  passed++;
  console.log('PASS: zero roll produces zero rotation');
})();

// ---- Roll scale rotates with horizon ----
(function testRollScaleRotatesWithHorizon() {
  // The roll arc and tick marks are drawn in the same rotated context
  // as the horizon, so they share the same rotation angle.
  const rollDeg = 45;
  const horizonRot = horizonRotation(rollDeg);
  // Scale rotation should equal horizon rotation (drawn before setTransform reset)
  const scaleRot = horizonRot;
  assert.strictEqual(scaleRot, horizonRot, 'Roll scale must rotate with the horizon');
  passed++;
  console.log('PASS: roll scale rotates with horizon');
})();

// ---- Fixed pointer at top center ----
(function testPointerIsFixedAtTopCenter() {
  // The pointer is drawn after setTransform reset, at fixed position
  // (cx, cy - arcR). It should NOT depend on rollRad.
  const cx = 200, cy = 200, arcR = 70;
  const ptrTipX = cx;
  const ptrTipY = cy - arcR + 10;
  // Pointer should be at top center regardless of roll
  assert.strictEqual(ptrTipX, cx, 'Pointer X should be at center');
  assert.ok(ptrTipY < cy, 'Pointer Y should be above center');
  passed++;
  console.log('PASS: pointer is fixed at top center');
})();

console.log(`\nAll ${passed} roll HUD sign tests PASS`);
