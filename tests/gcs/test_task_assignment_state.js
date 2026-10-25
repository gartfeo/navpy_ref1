/**
 * Tests for the taskAssignmentState reducer (one case per action, copies,
 * same task id from two owners, recycled id, lost APPLIED, confirms carrying
 * the helper's own POI id).
 * Dynamically imports the REAL ESM module (frontend is type:module) so the
 * actual exports are exercised, not a copy.
 *
 * Run via: node tests/gcs/test_task_assignment_state.js
 */
const assert = require('assert');
const path = require('path');
const url = require('url');

const SRC = path.join(
  __dirname, '..', '..', 'src', 'gcs', 'frontend', 'src', 'utils',
  'taskAssignmentState.js',
);

const OWNER = 1;
const HELPER = 2;
const OTHER = 3;
const uid = (boot_id, msg_seq) => ({ boot_id, msg_seq });

// WS payloads as gcs/backend/task_assign_listener.py sends them.
const advert = (owner, taskIds, seq, boot = 7) => ({
  type: 'available_task_request',
  data: {
    sender_id: owner,
    uid: uid(boot, seq),
    tasks: taskIds.map((task_id) => ({
      task_id, task_type: 'DOCK', lat: 40 + task_id, lon: 44, alt: 0,
    })),
  },
});
const request = (owner, helper, task_id, seq, boot = 7) => ({
  type: 'task_assign_request',
  data: {
    sender_id: owner, receiver_id: helper, task_id, task_type: 'DOCK',
    lat: 40 + task_id, lon: 44, alt: 0, uid: uid(boot, seq),
  },
});
const doing = (helper, owner, task_id, seq, is_accepted = true, boot = 3) => ({
  type: 'task_assign_response',
  data: { sender_id: helper, receiver_id: owner, task_id, is_accepted, uid: uid(boot, seq) },
});
const applied = (owner, helper, task_id, refSeq, refBoot = 3) => ({
  type: 'task_assign_ack',
  data: {
    owner_id: owner, helper_id: helper, task_id, status: 'APPLIED',
    ref: uid(refBoot, refSeq), uid: uid(7, 90),
  },
});
const beat = (sys_id, state, seq, boot = 3) => ({
  type: 'swarm_heartbeat',
  data: { sys_id, swarm: { state, boot, seq, stale: false } },
});
const onTask = (type, sys_id, task_id, round_uid) => ({ type, data: { sys_id, task_id, round_uid } });
const onVehicle = (type, sys_id) => ({ type, data: { sys_id } });

(async () => {
  const m = await import(url.pathToFileURL(SRC).href);
  const {
    ASSIGNMENT_STATUS: S, INITIAL_ASSIGNMENT_STATE: INITIAL,
    reduceTaskAssignment: reduce, isAssignedOrLater, vehicleAssignment,
  } = m;
  for (const name of [
    'availableTaskRequest', 'assignRequest', 'assignResponse', 'assignAck',
    'swarmHeartbeat', 'taskConfirming', 'taskConfirmed', 'taskResolved',
    'disarmAfterGuided', 'vehicleReset', 'resetAll', 'reduceTaskAssignment',
    'isAssignedOrLater', 'vehicleAssignment', 'roundKey', 'helperKey',
  ]) {
    assert.strictEqual(typeof m[name], 'function', `missing export: ${name}`);
  }
  const run = (actions, state = INITIAL) => actions.reduce(reduce, state);
  const assigned = run([
    request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40), applied(OWNER, HELPER, 5, 40),
  ]);
  // The helper asks about the owner's task 5 under its own POI id 1.
  const confirming = reduce(assigned, onTask('task_confirming', HELPER, 1, '3:50'));
  let s;
  let r;

  // ---- available_task_request ----
  s = run([advert(OWNER, [5], 10), advert(OTHER, [5], 10)]);
  assert.deepStrictEqual(Object.keys(s.availableTasks).sort(), ['1:5', '3:5'],
    'task ids are per owner: two owners\' task 5 coexist');
  assert.deepStrictEqual(s.availableTasks['1:5'], {
    key: '1:5', taskId: 5, taskType: 'DOCK', lat: 45, lon: 44, alt: 0, senderId: OWNER,
  });
  assert.strictEqual(reduce(s, advert(OWNER, [5], 11)), s, 'a repeated advert changes nothing');

  // ---- task_assign_request: WAITING ----
  s = run([advert(OWNER, [5, 6], 10), request(OWNER, HELPER, 5, 20)]);
  assert.deepStrictEqual(s.assignments['1:5'], {
    key: '1:5', taskId: 5, taskType: 'DOCK', lat: 45, lon: 44, alt: 0,
    senderId: OWNER, receiverId: HELPER, status: S.WAITING,
    requestUid: uid(7, 20), doingUid: null,
  });
  assert.ok(!('1:5' in s.availableTasks), 'an offered task is no longer available');
  assert.ok('1:6' in s.availableTasks);
  assert.strictEqual(reduce(s, request(OWNER, HELPER, 5, 19)), s, 'an older copy is ignored');
  s = reduce(s, doing(HELPER, OWNER, 5, 40));
  r = reduce(s, request(OWNER, HELPER, 5, 22));
  assert.deepStrictEqual(r.assignments['1:5'].requestUid, uid(7, 22));
  assert.deepStrictEqual(r.assignments['1:5'].doingUid, uid(3, 40), 'a repeat keeps its round');
  r = reduce(s, request(OWNER, OTHER, 5, 30));
  assert.strictEqual(r.assignments['1:5'].receiverId, OTHER, 'a re-offer starts a new round');
  assert.strictEqual(r.assignments['1:5'].doingUid, null);
  r = reduce(s, request(OWNER, HELPER, 5, 1, 8));
  assert.deepStrictEqual(r.assignments['1:5'].requestUid, uid(8, 1), 'a restarted owner replaces');
  assert.strictEqual(r.assignments['1:5'].doingUid, null);

  // ---- task_assign_response: "doing" is not ASSIGNED ----
  s = run([request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40)]);
  assert.strictEqual(s.assignments['1:5'].status, S.WAITING);
  assert.deepStrictEqual(s.assignments['1:5'].doingUid, uid(3, 40));
  assert.strictEqual(reduce(s, doing(HELPER, OWNER, 5, 39)), s, 'an older doing is ignored');
  assert.strictEqual(reduce(s, doing(OTHER, OWNER, 5, 41)), s, 'not this round\'s helper');
  assert.strictEqual(reduce(s, doing(HELPER, OWNER, 9, 41)), s, 'unknown task');

  // ---- task_assign_response: a reject deletes only that receiver's round ----
  s = run([request(OWNER, HELPER, 5, 20), request(OTHER, 4, 5, 20)]);
  r = reduce(s, doing(HELPER, OWNER, 5, 40, false));
  assert.ok(!('1:5' in r.assignments));
  assert.ok('3:5' in r.assignments, 'another owner\'s task 5 stays');
  assert.strictEqual(reduce(s, doing(OTHER, OWNER, 5, 40, false)), s, 'another helper\'s reject');
  r = reduce(s, doing(HELPER, OWNER, 5, 45));
  assert.strictEqual(reduce(r, doing(HELPER, OWNER, 5, 44, false)), r,
    'a reject sent before the helper\'s latest doing answered an older offer');

  // ---- task_assign_ack: ASSIGNED, in the helper's slot ----
  assert.ok(!('1:5' in assigned.assignments));
  assert.deepStrictEqual(assigned.assignments['helper:2'], {
    key: 'helper:2', taskId: 5, taskType: 'DOCK', lat: 45, lon: 44, alt: 0,
    senderId: OWNER, receiverId: HELPER, status: S.ASSIGNED,
    requestUid: uid(7, 20), doingUid: uid(3, 40),
  });
  assert.strictEqual(reduce(assigned, applied(OWNER, HELPER, 5, 41)), assigned,
    'a repeated APPLIED changes nothing');
  s = run([request(OWNER, HELPER, 5, 20)]);
  assert.strictEqual(reduce(s, applied(OWNER, OTHER, 5, 40)), s, 'APPLIED to another helper');
  assert.strictEqual(reduce(INITIAL, applied(OWNER, HELPER, 5, 40)), INITIAL, 'no round');
  r = reduce(s, applied(OWNER, HELPER, 5, 40));
  assert.deepStrictEqual(r.assignments['helper:2'].doingUid, uid(3, 40),
    'APPLIED heard without its copy still orders later beats');

  // ---- copies never lower a rank ----
  assert.strictEqual(reduce(assigned, request(OWNER, HELPER, 5, 20)), assigned,
    'a late step-3 copy does not reopen an assigned task');
  r = reduce(confirming, onTask('task_confirmed', HELPER, 1));
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMED);
  assert.strictEqual(reduce(r, onTask('task_confirming', HELPER, 1)), r);
  s = run([request(OWNER, HELPER, 5, 22), applied(OWNER, HELPER, 5, 40)], confirming);
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMING,
    'a later step 3 and APPLIED keep the slot rank');
  assert.strictEqual(reduce(s, onTask('task_confirmed', HELPER, 1, '3:50'))
    .assignments['helper:2'].status, S.CONFIRMED, 'and the round it asked');

  // ---- an advert after the step 3 releases the pending round ----
  s = run([request(OWNER, HELPER, 5, 20)]);
  r = reduce(s, advert(OWNER, [5], 30));
  assert.ok(!('1:5' in r.assignments));
  assert.ok('1:5' in r.availableTasks);
  assert.ok('1:5' in reduce(s, advert(OWNER, [5], 15)).assignments, 'an older advert');
  assert.ok('1:5' in reduce(s, advert(OWNER, [6], 30)).assignments, 'without the task');
  assert.ok('1:5' in reduce(s, advert(OWNER, [5], 30, 8)).assignments, 'another owner boot');
  assert.ok('helper:2' in reduce(assigned, advert(OWNER, [5], 30)).assignments,
    'an advert never drops an assigned task');

  // ---- swarm_heartbeat: FREE after the helper's doing retires the round ----
  s = run([request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40)]);
  assert.strictEqual(vehicleAssignment(s.assignments, HELPER).status, S.WAITING,
    'lost APPLIED: the helper stays WAITING, never assigned');
  assert.strictEqual(reduce(s, beat(HELPER, 'FREE', 39)), s, 'a beat before its doing');
  assert.strictEqual(reduce(s, beat(HELPER, 'BUSY', 41)), s);
  assert.strictEqual(reduce(s, beat(OTHER, 'FREE', 99)), s);
  assert.ok(!('1:5' in reduce(s, beat(HELPER, 'FREE', 41)).assignments),
    'the stranded helper expired and reports FREE');
  assert.ok(!('1:5' in reduce(s, beat(HELPER, 'FREE', 1, 4)).assignments), 'helper restarted');
  s = run([request(OWNER, HELPER, 5, 20)]);
  assert.strictEqual(reduce(s, beat(HELPER, 'FREE', 99)), s, 'no doing: cannot be ordered');
  assert.ok(!('helper:2' in reduce(assigned, beat(HELPER, 'FREE', 41)).assignments),
    'nav ended the assigned task');
  r = reduce(confirming, onTask('task_confirmed', HELPER, 1));
  assert.strictEqual(reduce(r, beat(HELPER, 'FREE', 41)), r, 'confirmed stays');

  // ---- confirm events are matched by UAV, to its helper slot ----
  // Delivery demo 2026-10-25: owner 1 assigned its task 2 to UAV 2, which
  // confirmed it as its own POI 1.
  s = run([request(OWNER, HELPER, 2, 8), doing(HELPER, OWNER, 2, 7), applied(OWNER, HELPER, 2, 7)]);
  r = run([onTask('task_confirming', HELPER, 1), onTask('task_confirmed', HELPER, 1)], s);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMED, 'own POI id, the owner task');
  s = run([request(OTHER, 4, 5, 20), doing(4, OTHER, 5, 40), applied(OTHER, 4, 5, 40)], assigned);
  r = reduce(s, onTask('task_confirming', HELPER, 5));
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMING);
  assert.strictEqual(r.assignments['helper:4'].status, S.ASSIGNED, 'same task id, other helper');
  assert.strictEqual(reduce(assigned, onTask('task_confirming', OTHER, 5)), assigned, 'no slot');
  s = run([request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40)]);
  assert.strictEqual(reduce(s, onTask('task_confirming', HELPER, 5)), s,
    'before APPLIED the GCS cannot tell the task round from an own-POI round');
  assert.strictEqual(reduce(assigned, onTask('task_confirmed', HELPER, 7)), assigned,
    'an own-POI round asked before the assignment does not answer for it');
  // Fail closed: APPLIED heard only after the confirm request leaves the task
  // ASSIGNED rather than risk confirming it with an own-POI round (todo 88).
  r = run([
    onTask('task_confirming', HELPER, 1, '3:50'), applied(OWNER, HELPER, 5, 40),
    onTask('task_confirmed', HELPER, 1, '3:50'),
  ], s);
  assert.strictEqual(r.assignments['helper:2'].status, S.ASSIGNED, 'APPLIED heard late');

  // ---- only an answer for the asked round moves the slot ----
  assert.deepStrictEqual(confirming.assignments['helper:2'].confirm, { taskId: 1, roundUid: '3:50' });
  assert.strictEqual(reduce(confirming, onTask('task_confirmed', HELPER, 9, '3:60')), confirming,
    'an approve for another POI');
  assert.strictEqual(reduce(confirming, onTask('task_resolved', HELPER, 9, '3:60')).assignments,
    confirming.assignments, 'a deny for another POI');
  assert.strictEqual(reduce(confirming, onTask('task_confirmed', HELPER, 1, '3:45')), confirming,
    'an answer for an earlier round of the same POI');
  assert.strictEqual(reduce(confirming, onTask('task_confirmed', HELPER, 1))
    .assignments['helper:2'].status, S.CONFIRMED, 'an answer without a round uid: POI id alone');
  r = run([onTask('task_confirming', HELPER, 2, '3:55'), onTask('task_confirmed', HELPER, 1, '3:50')],
    confirming);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMING, 'a later round re-points the slot');
  assert.strictEqual(reduce(r, onTask('task_confirmed', HELPER, 2, '3:55'))
    .assignments['helper:2'].status, S.CONFIRMED);
  const confirmed = reduce(confirming, onTask('task_confirmed', HELPER, 1, '3:50'));
  r = run([onTask('task_confirming', HELPER, 3, '3:70'), onTask('task_resolved', HELPER, 3, '3:70')],
    confirmed);
  assert.strictEqual(r.assignments, confirmed.assignments,
    'a confirmed task stays when the UAV then asks and is denied its home dock');
  r = run([
    request(OWNER, HELPER, 8, 30), doing(HELPER, OWNER, 8, 60), applied(OWNER, HELPER, 8, 60),
    onTask('task_confirming', HELPER, 2, '3:70'), onTask('task_resolved', HELPER, 1, '3:50'),
  ], confirmed);
  assert.strictEqual(r.assignments['helper:2'].taskId, 8);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMING,
    'canceling the earlier approved POI keeps the next assignment');

  // ---- task_resolved: the denied POI's advert and confirming task go ----
  s = run([advert(HELPER, [7], 10), advert(OWNER, [5, 8], 30)], confirming);
  assert.ok('helper:2' in s.assignments);
  r = reduce(s, onTask('task_resolved', HELPER, 1, '3:50'));
  assert.ok(!('helper:2' in r.assignments));
  assert.deepStrictEqual(Object.keys(r.availableTasks).sort(), ['1:8', '2:7'],
    'the owner advert of the denied task goes too');
  s = run([advert(HELPER, [7], 10)], assigned);
  r = reduce(s, onTask('task_resolved', HELPER, 7));
  assert.ok(!('2:7' in r.availableTasks), 'its own advertised POI');
  assert.strictEqual(r.assignments, s.assignments, 'not confirming: the assigned task stays');
  assert.strictEqual(reduce(confirming, onTask('task_resolved', OTHER, 1)), confirming);

  // ---- disarm_after_guided: the UAV's rounds and its adverts go ----
  s = run([advert(HELPER, [7], 10), request(OWNER, HELPER, 5, 20), request(OWNER, OTHER, 6, 21)]);
  r = reduce(s, onVehicle('disarm_after_guided', HELPER));
  assert.deepStrictEqual(Object.keys(r.assignments), ['1:6']);
  assert.deepStrictEqual(r.availableTasks, {});
  assert.strictEqual(reduce(s, onVehicle('disarm_after_guided', 99)), s);

  // ---- vehicle_reset (task_confirm_reset): only that receiver's entries ----
  s = run([request(OWNER, HELPER, 5, 20), request(OTHER, HELPER, 6, 20), request(OWNER, 4, 7, 22)]);
  r = reduce(s, onVehicle('vehicle_reset', HELPER));
  assert.deepStrictEqual(Object.keys(r.assignments), ['1:7']);
  assert.strictEqual(reduce(s, onVehicle('vehicle_reset', OWNER)), s, 'never by sender');
  assert.strictEqual(reduce(s, onVehicle('vehicle_reset', 99)), s);
  assert.strictEqual(reduce(INITIAL, onVehicle('vehicle_reset', HELPER)), INITIAL);

  // ---- reset and unknown actions ----
  assert.strictEqual(reduce(assigned, { type: 'reset' }), INITIAL);
  assert.strictEqual(reduce(assigned, { type: 'nope', data: {} }), assigned);

  // ---- same task id from two owners ----
  s = run([
    request(OWNER, HELPER, 5, 20), request(OTHER, 4, 5, 20),
    doing(4, OTHER, 5, 40), applied(OTHER, 4, 5, 40),
  ]);
  assert.strictEqual(s.assignments['1:5'].status, S.WAITING);
  assert.strictEqual(s.assignments['helper:4'].senderId, OTHER);

  // ---- recycled task id: a restarted owner offers its new task 5 ----
  s = reduce(assigned, request(OWNER, OTHER, 5, 2, 8));
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(s.assignments['1:5'].receiverId, OTHER);
  r = reduce(s, onTask('task_confirmed', OTHER, 5));
  assert.strictEqual(r.assignments['helper:2'].status, S.ASSIGNED);
  r = run([doing(OTHER, OWNER, 5, 70, true, 5), applied(OWNER, OTHER, 5, 70, 5)], s);
  assert.strictEqual(r.assignments['helper:3'].taskId, 5);
  assert.strictEqual(r.assignments['helper:2'].taskId, 5);

  // ---- selectors ----
  assert.strictEqual(isAssignedOrLater(null), false);
  assert.strictEqual(isAssignedOrLater({ status: S.WAITING }), false);
  for (const status of [S.ASSIGNED, S.CONFIRMING, S.CONFIRMED]) {
    assert.ok(isAssignedOrLater({ status }), status);
  }
  s = run([request(OWNER, HELPER, 5, 20), request(OTHER, HELPER, 6, 20), doing(HELPER, OTHER, 6, 40)]);
  assert.strictEqual(vehicleAssignment(s.assignments, HELPER).key, '3:6', 'the round it answered');
  s = reduce(s, applied(OTHER, HELPER, 6, 40));
  s = reduce(s, request(OWNER, HELPER, 7, 30));
  assert.strictEqual(vehicleAssignment(s.assignments, HELPER).key, 'helper:2', 'assigned first');
  assert.strictEqual(vehicleAssignment(s.assignments, 99), null);

  console.log('PASS — taskAssignmentState reducer tests passed.');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
