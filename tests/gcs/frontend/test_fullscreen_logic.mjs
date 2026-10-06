/**
 * Node.js tests for the fullscreen helpers behind the TopBar toggle.
 *
 * Run via: node tests/gcs/frontend/test_fullscreen_logic.mjs
 */
import assert from 'node:assert';
import {
  isFullscreenActive,
  isFullscreenSupported,
  isInstalledApp,
  toggleFullscreen,
} from '../../../src/gcs/frontend/src/utils/fullscreen.js';

function fakeWindow(displayMode) {
  return { matchMedia: (q) => ({ matches: q === `(display-mode: ${displayMode})` }) };
}

// Installed app = launched from the home screen, never a plain browser tab.
assert.strictEqual(isInstalledApp(fakeWindow('standalone')), true);
// display-mode fullscreen also matches a desktop browser in F11.
assert.strictEqual(isInstalledApp(fakeWindow('fullscreen')), false);
assert.strictEqual(isInstalledApp(fakeWindow('browser')), false);
assert.strictEqual(isInstalledApp(fakeWindow('minimal-ui')), false);
// jsdom and old browsers: no matchMedia, or it returns nothing.
assert.strictEqual(isInstalledApp({}), false);
assert.strictEqual(isInstalledApp({ matchMedia: () => undefined }), false);
assert.strictEqual(isInstalledApp(undefined), false);

function fakeDocument({ enabled = true, element = null } = {}) {
  const calls = [];
  return {
    calls,
    fullscreenEnabled: enabled,
    fullscreenElement: element,
    exitFullscreen: () => { calls.push('exit'); return Promise.resolve('exited'); },
    documentElement: {
      requestFullscreen: () => { calls.push('request'); return Promise.resolve('entered'); },
    },
  };
}

// Supported only when the browser reports fullscreen as enabled.
assert.strictEqual(isFullscreenSupported(fakeDocument()), true);
assert.strictEqual(isFullscreenSupported(fakeDocument({ enabled: false })), false);
assert.strictEqual(isFullscreenSupported({}), false);
assert.strictEqual(isFullscreenSupported(undefined), false);

// Active follows document.fullscreenElement.
assert.strictEqual(isFullscreenActive(fakeDocument()), false);
assert.strictEqual(isFullscreenActive(fakeDocument({ element: {} })), true);

// Not fullscreen: the toggle requests fullscreen on the whole page.
{
  const doc = fakeDocument();
  assert.strictEqual(await toggleFullscreen(doc), 'entered');
  assert.deepStrictEqual(doc.calls, ['request']);
}

// Already fullscreen: the toggle exits.
{
  const doc = fakeDocument({ element: {} });
  assert.strictEqual(await toggleFullscreen(doc), 'exited');
  assert.deepStrictEqual(doc.calls, ['exit']);
}

// A refused request surfaces as a rejected promise for the caller to report.
{
  const doc = fakeDocument();
  doc.documentElement.requestFullscreen = () => Promise.reject(new Error('denied'));
  await assert.rejects(toggleFullscreen(doc), /denied/);
}

console.log('fullscreen logic: all tests passed');
