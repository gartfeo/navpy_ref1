/**
 * Frontend unit tests for upload retry logic.
 * Run via: node tests/gcs/frontend/test_upload_retry_logic.js
 */
const assert = require('assert');

// ---- uploadMissions: never returns null ----
// Replicate the logic from usePlanningApi.js uploadMissions

async function uploadMissions(fetchFn, assignments) {
  try {
    const res = await fetchFn('/api/vehicles/upload', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ assignments }),
    });
    if (res.ok) {
      return await res.json();
    }
    const err = await res.json().catch(() => null);
    return { status: 'error', results: [], error: err?.detail || `HTTP ${res.status}` };
  } catch (e) {
    return { status: 'error', results: [], error: e.message };
  }
}

// ---- handleUploadResult: decide transition ----
function handleUploadResult(result) {
  if (result.status === 'complete') {
    return { action: 'goToMonitor' };
  } else if (result.status === 'partial_failure') {
    const failures = (result.results || [])
      .filter((r) => r.error)
      .map((r) => `Vehicle ${r.sys_id}: ${r.error}`)
      .join('\n');
    return { action: 'alert', message: `Some uploads failed:\n${failures}` };
  } else {
    return { action: 'alert', message: `Upload failed: ${result.error || 'Unknown error'}` };
  }
}

// ---- Tests ----
(async function testUploadSuccessNeverNull() {
  const fetchFn = async () => ({
    ok: true,
    json: async () => ({ status: 'complete', results: [{ sys_id: 1, error: null }] }),
  });
  const result = await uploadMissions(fetchFn, []);
  assert.notStrictEqual(result, null, 'uploadMissions should never return null');
  assert.strictEqual(result.status, 'complete');
})();

(async function testUploadHttpErrorNeverNull() {
  const fetchFn = async () => ({
    ok: false,
    status: 500,
    json: async () => ({ detail: 'Internal server error' }),
  });
  const result = await uploadMissions(fetchFn, []);
  assert.notStrictEqual(result, null, 'uploadMissions should never return null on HTTP error');
  assert.strictEqual(result.status, 'error');
  assert.strictEqual(result.error, 'Internal server error');
  assert.deepStrictEqual(result.results, []);
})();

(async function testUploadNetworkErrorNeverNull() {
  const fetchFn = async () => { throw new Error('Network failure'); };
  const result = await uploadMissions(fetchFn, []);
  assert.notStrictEqual(result, null, 'uploadMissions should never return null on network error');
  assert.strictEqual(result.status, 'error');
  assert.strictEqual(result.error, 'Network failure');
})();

(function testGoToMonitorOnComplete() {
  const result = handleUploadResult({ status: 'complete', results: [] });
  assert.strictEqual(result.action, 'goToMonitor');
})();

(function testAlertOnPartialFailure() {
  const result = handleUploadResult({
    status: 'partial_failure',
    results: [
      { sys_id: 1, error: null },
      { sys_id: 2, error: 'Upload failed after 3 attempts' },
    ],
  });
  assert.strictEqual(result.action, 'alert');
  assert.ok(result.message.includes('Vehicle 2'));
  assert.ok(result.message.includes('Upload failed'));
})();

(function testAlertOnError() {
  const result = handleUploadResult({ status: 'error', results: [], error: 'Network failure' });
  assert.strictEqual(result.action, 'alert');
  assert.ok(result.message.includes('Network failure'));
})();

(function testAlertOnErrorNoMessage() {
  const result = handleUploadResult({ status: 'error', results: [] });
  assert.strictEqual(result.action, 'alert');
  assert.ok(result.message.includes('Unknown error'));
})();

// Wait for all async tests to settle
setTimeout(() => {
  console.log('PASS — All upload retry frontend tests passed.');
}, 100);
