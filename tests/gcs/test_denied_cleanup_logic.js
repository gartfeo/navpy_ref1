/**
 * Tests for denied cleanup logic in useTaskAssignment.
 * Run via: node tests/gcs/test_denied_cleanup_logic.js
 */

const assert = require('assert');

/**
 * Pure updater extracted from handleDeniedCleanup in useTaskAssignment.js.
 * Removes all assignments where receiverId matches the denied sysId.
 */
function deniedCleanupUpdater(prev, sysId) {
  const toRemove = Object.keys(prev).filter(
    (tid) => prev[tid].receiverId === sysId,
  );
  if (toRemove.length === 0) return prev;
  const next = { ...prev };
  for (const tid of toRemove) delete next[tid];
  return next;
}

// ---- Tests ----

(function testRemovesSingleAssignment() {
  const prev = {
    't1': { taskId: 't1', receiverId: 5, senderId: 1, status: 'confirming' },
  };
  const result = deniedCleanupUpdater(prev, 5);
  assert.strictEqual(Object.keys(result).length, 0,
    'Should remove the assignment for denied sysId');
})();

(function testRemovesMultipleAssignmentsForSameSysId() {
  const prev = {
    't1': { taskId: 't1', receiverId: 5, senderId: 1, status: 'assigned' },
    't2': { taskId: 't2', receiverId: 5, senderId: 2, status: 'confirming' },
    't3': { taskId: 't3', receiverId: 7, senderId: 1, status: 'assigned' },
  };
  const result = deniedCleanupUpdater(prev, 5);
  assert.strictEqual(Object.keys(result).length, 1,
    'Should remove both assignments for sysId 5');
  assert.ok(result['t3'], 'Should keep assignment for sysId 7');
})();

(function testLeavesOtherAssignmentsUntouched() {
  const prev = {
    't1': { taskId: 't1', receiverId: 3, senderId: 1, status: 'assigned' },
    't2': { taskId: 't2', receiverId: 4, senderId: 2, status: 'confirming' },
  };
  const result = deniedCleanupUpdater(prev, 99);
  assert.strictEqual(result, prev,
    'Should return same reference when no assignments match');
})();

(function testEmptyAssignments() {
  const prev = {};
  const result = deniedCleanupUpdater(prev, 5);
  assert.strictEqual(result, prev,
    'Should return same reference for empty state');
})();

(function testDoesNotMatchBySenderId() {
  const prev = {
    't1': { taskId: 't1', receiverId: 3, senderId: 5, status: 'assigned' },
  };
  const result = deniedCleanupUpdater(prev, 5);
  // senderId=5 should NOT match — only receiverId matters
  assert.strictEqual(result, prev,
    'Should not remove assignment where only senderId matches');
})();

console.log('All denied cleanup logic tests passed.');
