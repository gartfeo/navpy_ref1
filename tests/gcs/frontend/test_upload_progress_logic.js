/**
 * Node.js tests for upload progress state management logic.
 *
 * Simulates the upload_progress WebSocket message handler and
 * the UploadProgressItem label/percentage logic from BottomBar.
 *
 * Run via: node tests/gcs/frontend/test_upload_progress_logic.js
 */

const assert = require('assert');

// ---- Simulate the upload_progress message handler from App.jsx ----

function applyUploadProgress(prev, data) {
  return {
    ...prev,
    [data.sys_id]: {
      ...prev[data.sys_id],
      stage: data.stage,
      progress: data.progress ?? prev[data.sys_id]?.progress ?? 0,
      done: data.done ?? false,
      error: data.error ?? null,
      wp_sent: data.wp_sent ?? prev[data.sys_id]?.wp_sent,
      wp_total: data.wp_total ?? prev[data.sys_id]?.wp_total,
    },
  };
}

// ---- Simulate the UploadProgressItem label logic from BottomBar ----

const STAGE_LABELS = {
  clearing: 'Clearing',
  uploading: 'Uploading',
  verifying: 'Verifying',
  reconnecting: 'Reconnecting',
  complete: 'Done',
  failed: 'Failed',
};

function getLabel(progress) {
  if (!progress) return 'Waiting';
  const { stage, wp_sent, wp_total, done, error } = progress;
  if (error) return 'Failed';
  if (done) return 'Done';
  if (stage === 'uploading' && wp_sent != null && wp_total) {
    return `${wp_sent}/${wp_total} wps`;
  }
  return STAGE_LABELS[stage] || 'Waiting';
}

function getPct(progress) {
  if (!progress) return 0;
  const { stage, wp_sent, wp_total, done } = progress;
  if (stage === 'uploading' && wp_total) {
    return Math.round((wp_sent || 0) / wp_total * 100);
  }
  if (done) return 100;
  if (stage === 'verifying') return 95;
  if (stage === 'clearing') return 5;
  return 0;
}

// ---- Tests ----

// Test: initial upload start message
(function testInitialUploadStart() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    progress: 0.1,
    done: false,
  });
  assert.strictEqual(state[1].progress, 0.1);
  assert.strictEqual(state[1].done, false);
  assert.strictEqual(state[1].error, null);
})();

// Test: clearing stage
(function testClearingStage() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'clearing',
    attempt: 1,
  });
  assert.strictEqual(state[1].stage, 'clearing');
  assert.strictEqual(getLabel(state[1]), 'Clearing');
  assert.strictEqual(getPct(state[1]), 5);
})();

// Test: uploading stage with per-waypoint progress
(function testUploadingWithWaypoints() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'uploading',
    wp_sent: 5,
    wp_total: 20,
    progress: 0.25,
  });
  assert.strictEqual(state[1].wp_sent, 5);
  assert.strictEqual(state[1].wp_total, 20);
  assert.strictEqual(getLabel(state[1]), '5/20 wps');
  assert.strictEqual(getPct(state[1]), 25);
})();

// Test: waypoint progress increments
(function testWaypointProgressIncrements() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'uploading',
    wp_sent: 10,
    wp_total: 50,
    progress: 0.2,
  });
  assert.strictEqual(getLabel(state[1]), '10/50 wps');
  assert.strictEqual(getPct(state[1]), 20);

  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'uploading',
    wp_sent: 25,
    wp_total: 50,
    progress: 0.5,
  });
  assert.strictEqual(getLabel(state[1]), '25/50 wps');
  assert.strictEqual(getPct(state[1]), 50);

  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'uploading',
    wp_sent: 50,
    wp_total: 50,
    progress: 1.0,
  });
  assert.strictEqual(getLabel(state[1]), '50/50 wps');
  assert.strictEqual(getPct(state[1]), 100);
})();

// Test: verifying stage
(function testVerifyingStage() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'verifying',
    attempt: 1,
  });
  assert.strictEqual(getLabel(state[1]), 'Verifying');
  assert.strictEqual(getPct(state[1]), 95);
})();

// Test: complete
(function testComplete() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'complete',
    progress: 1.0,
    done: true,
  });
  assert.strictEqual(getLabel(state[1]), 'Done');
  assert.strictEqual(getPct(state[1]), 100);
})();

// Test: failed
(function testFailed() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'failed',
    progress: 0.0,
    done: true,
    error: 'Upload failed after 3 attempts',
  });
  assert.strictEqual(getLabel(state[1]), 'Failed');
  assert.strictEqual(getPct(state[1]), 100); // done=true → 100
})();

// Test: multiple vehicles tracked independently
(function testMultipleVehicles() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'uploading',
    wp_sent: 5,
    wp_total: 20,
    progress: 0.25,
  });
  state = applyUploadProgress(state, {
    sys_id: 2,
    stage: 'clearing',
    attempt: 1,
  });
  assert.strictEqual(getLabel(state[1]), '5/20 wps');
  assert.strictEqual(getLabel(state[2]), 'Clearing');
  assert.strictEqual(getPct(state[1]), 25);
  assert.strictEqual(getPct(state[2]), 5);
})();

// Test: wp_total preserved across stage messages without wp fields
(function testWpTotalPreserved() {
  let state = {};
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'uploading',
    wp_sent: 10,
    wp_total: 30,
    progress: 0.33,
  });
  // Stage transition without wp fields should keep wp_total
  state = applyUploadProgress(state, {
    sys_id: 1,
    stage: 'verifying',
    attempt: 1,
  });
  assert.strictEqual(state[1].wp_total, 30, 'wp_total should persist');
  assert.strictEqual(state[1].stage, 'verifying');
})();

// Test: null progress object returns defaults
(function testNullProgress() {
  assert.strictEqual(getLabel(null), 'Waiting');
  assert.strictEqual(getLabel(undefined), 'Waiting');
  assert.strictEqual(getPct(null), 0);
  assert.strictEqual(getPct(undefined), 0);
})();

console.log('PASS');
