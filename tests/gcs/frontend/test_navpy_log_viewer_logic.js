/**
 * Node.js tests for navpyLogViewerVisibility logic
 * from MonitoringSidebar's NavpySimButton.
 *
 * Run via: node tests/gcs/frontend/test_navpy_log_viewer_logic.js
 */

const assert = require('assert');

function navpyLogViewerVisibility(instance, showLogs) {
  const hasLogs = instance?.log?.length > 0;
  return {
    linkVisible: hasLogs,
    panelVisible: hasLogs && showLogs,
  };
}

// ---- linkVisible ----

(function testNoInstance() {
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(undefined, false);
  assert.strictEqual(linkVisible, false, 'No instance → link hidden');
  assert.strictEqual(panelVisible, false, 'No instance → panel hidden');
})();

(function testNullInstance() {
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(null, false);
  assert.strictEqual(linkVisible, false, 'Null instance → link hidden');
  assert.strictEqual(panelVisible, false, 'Null instance → panel hidden');
})();

(function testEmptyLog() {
  const { linkVisible, panelVisible } = navpyLogViewerVisibility({ log: [] }, false);
  assert.strictEqual(linkVisible, false, 'Empty log → link hidden');
  assert.strictEqual(panelVisible, false, 'Empty log → panel hidden');
})();

(function testNoLogProperty() {
  const { linkVisible, panelVisible } = navpyLogViewerVisibility({ running: true }, false);
  assert.strictEqual(linkVisible, false, 'No log property → link hidden');
  assert.strictEqual(panelVisible, false, 'No log property → panel hidden');
})();

(function testWithLogsShowFalse() {
  const instance = { log: ['line 1', 'line 2'] };
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, false);
  assert.strictEqual(linkVisible, true, 'Has logs → link visible');
  assert.strictEqual(panelVisible, false, 'showLogs false → panel hidden');
})();

(function testWithLogsShowTrue() {
  const instance = { log: ['line 1', 'line 2'] };
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, true);
  assert.strictEqual(linkVisible, true, 'Has logs + show → link visible');
  assert.strictEqual(panelVisible, true, 'Has logs + show → panel visible');
})();

// ---- edge cases ----

(function testSingleLogLine() {
  const instance = { log: ['only one line'] };
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, true);
  assert.strictEqual(linkVisible, true, 'Single log line → link visible');
  assert.strictEqual(panelVisible, true, 'Single log line + show → panel visible');
})();

(function testRunningWithLogs() {
  const instance = { running: true, log: ['started', 'processing'] };
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, true);
  assert.strictEqual(linkVisible, true, 'Running with logs → link visible');
  assert.strictEqual(panelVisible, true, 'Running with logs + show → panel visible');
})();

(function testStoppedWithLogs() {
  const instance = { running: false, log: ['done'] };
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, false);
  assert.strictEqual(linkVisible, true, 'Stopped with logs → link visible');
  assert.strictEqual(panelVisible, false, 'Stopped + showLogs false → panel hidden');
})();

(function testShowTrueButNoLogs() {
  const instance = { log: [] };
  const { linkVisible, panelVisible } = navpyLogViewerVisibility(instance, true);
  assert.strictEqual(linkVisible, false, 'No logs + show true → link hidden');
  assert.strictEqual(panelVisible, false, 'No logs + show true → panel hidden');
})();

console.log('PASS');
