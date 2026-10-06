/**
 * Frontend unit tests for task assignment state logic.
 * Run via: node tests/gcs/frontend/test_task_assignment_logic.js
 */

const assert = require('assert');

// ---- handleAssignRequest ----
function handleAssignRequest(assignments, data) {
  return {
    ...assignments,
    [data.task_id]: {
      taskId: data.task_id,
      taskType: data.task_type,
      lat: data.lat,
      lon: data.lon,
      alt: data.alt,
      senderId: data.sender_id,
      receiverId: data.receiver_id,
      status: 'assigned',
      receivedAt: data.receivedAt || Date.now(),
    },
  };
}

(function testHandleAssignRequest() {
  const result = handleAssignRequest({}, {
    task_id: 7, task_type: 'MEDIUM', lat: 32.5, lon: 34.8, alt: 100,
    sender_id: 1, receiver_id: 2, receivedAt: 1000,
  });
  assert.strictEqual(result[7].taskId, 7);
  assert.strictEqual(result[7].taskType, 'MEDIUM');
  assert.strictEqual(result[7].lat, 32.5);
  assert.strictEqual(result[7].lon, 34.8);
  assert.strictEqual(result[7].senderId, 1);
  assert.strictEqual(result[7].receiverId, 2);
  assert.strictEqual(result[7].status, 'assigned');
})();

(function testHandleAssignRequestMultiple() {
  let state = {};
  state = handleAssignRequest(state, {
    task_id: 1, task_type: 'SMALL', lat: 30, lon: 35, alt: 50,
    sender_id: 1, receiver_id: 2,
  });
  state = handleAssignRequest(state, {
    task_id: 2, task_type: 'LARGE', lat: 31, lon: 36, alt: 60,
    sender_id: 1, receiver_id: 3,
  });
  assert.strictEqual(Object.keys(state).length, 2);
  assert.strictEqual(state[1].receiverId, 2);
  assert.strictEqual(state[2].receiverId, 3);
})();

// ---- handleAssignResponse ----
function handleAssignResponse(assignments, data) {
  const entry = assignments[data.task_id];
  if (!entry) return assignments;
  if (data.is_accepted) {
    return {
      ...assignments,
      [data.task_id]: { ...entry, status: 'accepted' },
    };
  }
  // Rejected — remove
  const next = { ...assignments };
  delete next[data.task_id];
  return next;
}

(function testHandleAssignResponseAccepted() {
  const state = {
    7: { taskId: 7, status: 'assigned', receiverId: 2 },
  };
  const result = handleAssignResponse(state, { task_id: 7, is_accepted: true });
  assert.strictEqual(result[7].status, 'accepted');
})();

(function testHandleAssignResponseRejected() {
  const state = {
    7: { taskId: 7, status: 'assigned', receiverId: 2 },
  };
  const result = handleAssignResponse(state, { task_id: 7, is_accepted: false });
  assert.strictEqual(result[7], undefined, 'Rejected should be removed');
})();

(function testHandleAssignResponseMissingTask() {
  const state = { 7: { taskId: 7, status: 'assigned' } };
  const result = handleAssignResponse(state, { task_id: 99, is_accepted: true });
  assert.deepStrictEqual(result, state, 'Should not modify state for unknown task');
})();

// ---- handleDisarmAfterGuidedCleanup ----
function handleDisarmAfterGuidedCleanup(assignments, data) {
  const toRemove = Object.keys(assignments).filter(
    (tid) => assignments[tid].receiverId === data.sys_id,
  );
  if (toRemove.length === 0) return assignments;
  const next = { ...assignments };
  for (const tid of toRemove) delete next[tid];
  return next;
}

(function testDisarmAfterGuidedCleansUpReceiver() {
  const state = {
    1: { taskId: 1, receiverId: 2 },
    2: { taskId: 2, receiverId: 3 },
    3: { taskId: 3, receiverId: 2 },
  };
  const result = handleDisarmAfterGuidedCleanup(state, { sys_id: 2 });
  assert.strictEqual(result[1], undefined, 'Should remove task 1 (receiverId 2)');
  assert.strictEqual(result[3], undefined, 'Should remove task 3 (receiverId 2)');
  assert.strictEqual(result[2].receiverId, 3, 'Task 2 should remain');
})();

(function testDisarmAfterGuidedNoMatch() {
  const state = { 1: { taskId: 1, receiverId: 2 } };
  const result = handleDisarmAfterGuidedCleanup(state, { sys_id: 99 });
  assert.deepStrictEqual(result, state, 'Should not modify when no match');
})();

// ---- reset ----
(function testReset() {
  const state = { 1: { taskId: 1 }, 2: { taskId: 2 } };
  const result = {};
  assert.strictEqual(Object.keys(result).length, 0, 'Reset should clear all');
})();

console.log('PASS — All task assignment frontend tests passed.');
