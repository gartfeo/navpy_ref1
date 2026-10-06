/**
 * Node.js tests for fetchWithRetry logic.
 *
 * Mocks the global fetch to verify retry behaviour: success on first try,
 * retry on 5xx, no retry on 4xx, retry on network errors, and exhaust retries.
 *
 * Run via: node tests/gcs/frontend/test_fetch_retry_logic.js
 */
const assert = require('assert');
const { pathToFileURL } = require('url');
const path = require('path');

async function loadModule() {
  const src = path.resolve(
    __dirname,
    '../../../src/gcs/frontend/src/utils/fetchRetry.js',
  );
  const mod = await import(pathToFileURL(src).href);
  return mod.default;
}

(async () => {
  const fetchWithRetry = await loadModule();

  // ---- Test: succeeds on first try (1 fetch call) ----
  await (async function testSucceedsFirstTry() {
    let calls = 0;
    globalThis.fetch = async () => { calls++; return { ok: true, status: 200 }; };
    const res = await fetchWithRetry('/test');
    assert.strictEqual(calls, 1, 'Should call fetch once');
    assert.strictEqual(res.ok, true);
  })();

  // ---- Test: fails twice then succeeds (3 calls) ----
  await (async function testRetriesOnServerError() {
    let calls = 0;
    globalThis.fetch = async () => {
      calls++;
      if (calls <= 2) return { ok: false, status: 500 };
      return { ok: true, status: 200 };
    };
    const res = await fetchWithRetry('/test', {}, { maxRetries: 2, baseDelay: 1 });
    assert.strictEqual(calls, 3, 'Should retry twice then succeed');
    assert.strictEqual(res.ok, true);
  })();

  // ---- Test: does not retry on 404 (1 call) ----
  await (async function testNoRetryOn404() {
    let calls = 0;
    globalThis.fetch = async () => { calls++; return { ok: false, status: 404 }; };
    const res = await fetchWithRetry('/test', {}, { maxRetries: 2, baseDelay: 1 });
    assert.strictEqual(calls, 1, 'Should not retry on 404');
    assert.strictEqual(res.status, 404);
  })();

  // ---- Test: retries on network error (thrown exception) ----
  await (async function testRetriesOnNetworkError() {
    let calls = 0;
    globalThis.fetch = async () => {
      calls++;
      if (calls <= 1) throw new TypeError('Failed to fetch');
      return { ok: true, status: 200 };
    };
    const res = await fetchWithRetry('/test', {}, { maxRetries: 2, baseDelay: 1 });
    assert.strictEqual(calls, 2, 'Should retry after network error');
    assert.strictEqual(res.ok, true);
  })();

  // ---- Test: throws after exhausting retries ----
  await (async function testThrowsAfterExhaust() {
    let calls = 0;
    globalThis.fetch = async () => { calls++; throw new TypeError('Failed to fetch'); };
    let threw = false;
    try {
      await fetchWithRetry('/test', {}, { maxRetries: 2, baseDelay: 1 });
    } catch (e) {
      threw = true;
      assert.strictEqual(e.message, 'Failed to fetch');
    }
    assert.strictEqual(threw, true, 'Should throw after exhausting retries');
    assert.strictEqual(calls, 3, 'Should have called fetch maxRetries+1 times');
  })();

  // ---- Test: retries on 408 (Request Timeout) ----
  await (async function testRetriesOn408() {
    let calls = 0;
    globalThis.fetch = async () => {
      calls++;
      if (calls <= 1) return { ok: false, status: 408 };
      return { ok: true, status: 200 };
    };
    const res = await fetchWithRetry('/test', {}, { maxRetries: 2, baseDelay: 1 });
    assert.strictEqual(calls, 2, 'Should retry on 408');
    assert.strictEqual(res.ok, true);
  })();

  console.log('PASS');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
