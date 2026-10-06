/**
 * Frontend unit tests for TaskConfirmCard presentation helpers.
 * Run via: node tests/gcs/test_task_confirm_frontend.js
 *
 * Card lifecycle / reducer logic (request, image, approve/deny/cancel,
 * reset, expiry, selectors) is tested against the real module
 * in test_task_confirmation_state.js. This file covers only the card-local
 * presentation math: the countdown, the timeout fallback, the timeout
 * auto-action, and the timer color thresholds.
 */

const assert = require('assert');

// ---- Countdown calculation ----
function calcRemaining(receivedAt, nowMs, timeoutSec = 30) {
  const elapsed = (nowMs - receivedAt) / 1000;
  return Math.max(0, timeoutSec - elapsed);
}

(function testCountdownFresh() {
  const now = Date.now();
  assert.strictEqual(calcRemaining(now, now), 30, 'Fresh request should have 30s remaining');
})();

(function testCountdownPartial() {
  const now = Date.now();
  assert.strictEqual(calcRemaining(now - 10000, now), 20, 'After 10s should have 20s remaining');
})();

(function testCountdownExpired() {
  const now = Date.now();
  assert.strictEqual(calcRemaining(now - 35000, now), 0, 'Expired request should have 0s remaining');
})();

(function testCountdownNeverNegative() {
  const now = Date.now();
  assert.strictEqual(calcRemaining(now - 100000, now), 0, 'Remaining should never be negative');
})();

(function testCountdownRespectsCustomTimeout() {
  const now = Date.now();
  assert.strictEqual(calcRemaining(now - 5000, now, 60), 55, 'Custom 60s timeout gives 55s after 5s');
})();

(function testCountdownDefaultFallback() {
  // Mirrors TaskConfirmCard's effectiveTimeout = (finite && >=0) ? prop : DEFAULT_TIMEOUT_SEC.
  // 0 is honored as "immediate timeout" since the settings UI allows nav_cwt min 0.
  const effective = (val) => (Number.isFinite(val) && val >= 0 ? val : 30);
  assert.strictEqual(effective(undefined), 30, 'Undefined timeout falls back to 30');
  assert.strictEqual(effective(null), 30, 'Null timeout falls back to 30');
  assert.strictEqual(effective(NaN), 30, 'NaN timeout falls back to 30');
  assert.strictEqual(effective(0), 0, 'Zero timeout is honored (immediate timeout)');
  assert.strictEqual(effective(-5), 30, 'Negative timeout falls back to 30');
  assert.strictEqual(effective(45), 45, 'Finite positive timeout is used as-is');
})();

// ---- Auto-action on expiry ----
function autoAction(autoApprove) {
  return autoApprove ? 'approve' : 'deny';
}

(function testAutoApproveWhenCmFl() {
  assert.strictEqual(autoAction(true), 'approve', 'Should auto-approve when nav_cm_fl is true');
})();

(function testAutoDenyWhenNoCmFl() {
  assert.strictEqual(autoAction(false), 'deny', 'Should auto-deny when nav_cm_fl is false');
})();

// ---- Timer color thresholds ----
function timerColor(remaining) {
  if (remaining > 10) return 'green';
  if (remaining > 5) return 'orange';
  return 'red';
}

(function testTimerColorGreen() {
  assert.strictEqual(timerColor(15), 'green');
  assert.strictEqual(timerColor(11), 'green');
})();

(function testTimerColorOrange() {
  assert.strictEqual(timerColor(10), 'orange');
  assert.strictEqual(timerColor(6), 'orange');
})();

(function testTimerColorRed() {
  assert.strictEqual(timerColor(5), 'red');
  assert.strictEqual(timerColor(0), 'red');
})();

console.log('All task confirmation frontend presentation tests passed.');
