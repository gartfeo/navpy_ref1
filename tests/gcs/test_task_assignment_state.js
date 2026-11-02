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
// The owner stamps a request's copies only until it applies an answer, then its APPLIED
// (task_auction_confirmation.py), so a request below seq 25 here is a copy and a later
// one is a new assignment.
const applied = (owner, helper, task_id, refSeq, refBoot = 3, seq = 25) => ({
  type: 'task_assign_ack',
  data: {
    owner_id: owner, helper_id: helper, task_id, status: 'APPLIED',
    ref: uid(refBoot, refSeq), uid: uid(7, seq),
  },
});
const beat = (sys_id, state, seq, boot = 3) => ({
  type: 'swarm_heartbeat',
  data: { sys_id, swarm: { state, boot, seq, stale: false } },
});
const onTask = (type, sys_id, task_id, round_uid) => ({ type, data: { sys_id, task_id, round_uid } });
const onVehicle = (type, sys_id) => ({ type, data: { sys_id } });
// A confirm request with the position NavPy fills in the sim (decision 3a).
const asked = (sys_id, task_id, round_uid, lat = 41, lon = 45) => ({
  type: 'task_confirming',
  data: { sys_id, task_id, round_uid, task_type: 'DOCK', lat, lon, alt: 0 },
});
// A beat with NavPy's display phase and latched outcome, as the snapshot sends it.
const phaseBeat = (sys_id, phase, seq, outcome = null, boot = 3, fail_reason = null) => ({
  type: 'swarm_heartbeat',
  data: { sys_id, swarm: { state: 'BUSY', phase, outcome, fail_reason, boot, seq, stale: false } },
});

(async () => {
  const m = await import(url.pathToFileURL(SRC).href);
  const {
    ASSIGNMENT_STATUS: S, INITIAL_ASSIGNMENT_STATE: INITIAL,
    reduceTaskAssignment: reduce, isAssignedOrLater, vehicleAssignment,
    DOCK_OUTCOME: O, dockMarkerEntries, uavOf,
  } = m;
  for (const name of [
    'availableTaskRequest', 'assignRequest', 'assignResponse', 'assignAck',
    'swarmHeartbeat', 'taskConfirming', 'taskConfirmed', 'taskResolved',
    'disarmAfterGuided', 'vehicleReset', 'resetAll', 'reduceTaskAssignment',
    'isAssignedOrLater', 'vehicleAssignment', 'roundKey', 'helperKey',
    'linkGap', 'dockMarkerEntries', 'uavOf',
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
    key: '1:5', taskId: 5, taskType: 'DOCK', lat: 45, lon: 44, alt: 0, senderId: OWNER, ownerBoot: 7,
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
    requestUid: uid(7, 20), doingUid: uid(3, 40), assignedUid: uid(3, 40), appliedUid: uid(7, 25),
  });
  assert.strictEqual(reduce(assigned, applied(OWNER, HELPER, 5, 41)), assigned,
    'a repeated APPLIED changes nothing');
  s = run([request(OWNER, HELPER, 5, 20)]);
  assert.strictEqual(reduce(s, applied(OWNER, OTHER, 5, 40)), s, 'APPLIED to another helper');
  assert.strictEqual(reduce(INITIAL, applied(OWNER, HELPER, 5, 40)), INITIAL, 'no round');
  r = reduce(s, applied(OWNER, HELPER, 5, 40));
  assert.deepStrictEqual(r.assignments['helper:2'].doingUid, uid(3, 40),
    'APPLIED heard without its copy still orders later beats');
  assert.deepStrictEqual(r.assignments['helper:2'].assignedUid, uid(3, 40), 'and later rounds');

  // ---- copies never lower a rank ----
  assert.strictEqual(reduce(assigned, request(OWNER, HELPER, 5, 20)), assigned,
    'a late step-3 copy does not reopen an assigned task');
  r = reduce(confirming, onTask('task_confirmed', HELPER, 1, '3:50'));
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMED);
  assert.strictEqual(reduce(r, onTask('task_confirming', HELPER, 1, '3:50')), r);
  assert.strictEqual(reduce(r, onTask('task_confirming', HELPER, 1, '3:45')), r);
  s = run([request(OWNER, HELPER, 5, 22), applied(OWNER, HELPER, 5, 40, 3, 95)], confirming);
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMING,
    'a delayed step 3 and a repeated APPLIED keep the slot rank');
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
  r = reduce(confirming, onTask('task_confirmed', HELPER, 1, '3:50'));
  assert.strictEqual(reduce(r, beat(HELPER, 'FREE', 41)), r, 'confirmed stays');

  // ---- confirm events are matched by UAV, to its helper slot ----
  // Delivery demo 2026-10-25: owner 1 assigned its task 2 to UAV 2, which
  // confirmed it as its own POI 1.
  s = run([request(OWNER, HELPER, 2, 8), doing(HELPER, OWNER, 2, 7), applied(OWNER, HELPER, 2, 7)]);
  r = run([onTask('task_confirming', HELPER, 1, '3:20'), onTask('task_confirmed', HELPER, 1, '3:20')], s);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMED, 'own POI id, the owner task');
  s = run([request(OTHER, 4, 5, 20), doing(4, OTHER, 5, 40), applied(OTHER, 4, 5, 40)], assigned);
  r = reduce(s, onTask('task_confirming', HELPER, 5, '3:50'));
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
  assert.strictEqual(reduce(confirming, onTask('task_confirmed', HELPER, 1)), confirming,
    'an answer without a round uid cannot name the round');
  assert.strictEqual(reduce(confirming, onTask('task_resolved', HELPER, 1)).assignments,
    confirming.assignments, 'nor can a deny without one');
  for (const [roundUid, why] of [
    ['4:20', 'a restarted helper asks about its own POI'],
    ['3:30', 'a round asked before its doing'],
    ['legacy', 'a round without metadata'],
    [undefined, 'a round without a uid'],
  ]) {
    assert.strictEqual(reduce(assigned, onTask('task_confirming', HELPER, 1, roundUid)), assigned, why);
    assert.strictEqual(reduce(confirming, onTask('task_confirming', HELPER, 2, roundUid)), confirming, why);
  }
  r = run([request(OWNER, HELPER, 5, 30), applied(OWNER, HELPER, 5, 10, 4, 95)], confirming);
  assert.strictEqual(r.assignments['helper:2'].status, S.ASSIGNED,
    'the task applied again to a restarted helper is confirmed afresh');
  assert.strictEqual(reduce(r, onTask('task_resolved', HELPER, 1, '3:50')).assignments, r.assignments,
    'the old boot round answers nothing');
  s = run([request(OWNER, HELPER, 5, 23), applied(OWNER, HELPER, 5, 40, 3, 90)], r);
  assert.deepStrictEqual(s.assignments['helper:2'].doingUid, uid(4, 10),
    'a late APPLIED copy does not bring the old helper boot back');
  assert.strictEqual(reduce(s, onTask('task_confirming', HELPER, 1, '3:50')), s);
  s = run([onTask('task_confirming', HELPER, 2, '3:55'), onTask('task_confirming', HELPER, 1, '3:50')],
    confirming);
  assert.deepStrictEqual(s.assignments['helper:2'].confirm, { taskId: 2, roundUid: '3:55' },
    'a late older request does not re-point the slot');
  assert.ok('helper:2' in reduce(s, onTask('task_resolved', HELPER, 1, '3:50')).assignments);
  s = run([request(OWNER, HELPER, 5, 24), doing(HELPER, OWNER, 5, 60), applied(OWNER, HELPER, 5, 60, 3, 96),
    onTask('task_confirming', HELPER, 1, '3:50'), onTask('task_confirmed', HELPER, 1, '3:50')], assigned);
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMED,
    'a request sent before a repeated doing still counts: the floor is the assigning doing');
  r = run([onTask('task_confirming', HELPER, 2, '3:55'), onTask('task_confirmed', HELPER, 1, '3:50')],
    confirming);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMING, 'a later round re-points the slot');
  assert.strictEqual(reduce(r, onTask('task_confirmed', HELPER, 2, '3:55'))
    .assignments['helper:2'].status, S.CONFIRMED);
  const confirmed = reduce(confirming, onTask('task_confirmed', HELPER, 1, '3:50'));
  r = run([onTask('task_confirming', HELPER, 3, '3:70'), onTask('task_resolved', HELPER, 3, '3:70')],
    confirmed);
  assert.ok(!('helper:2' in r.assignments), 'a later round ends the approved task, whatever its answer');
  assert.strictEqual(r.assignments['handled:2:3:50'].outcome, O.UNKNOWN);
  r = run([
    request(OWNER, HELPER, 8, 30), doing(HELPER, OWNER, 8, 60), applied(OWNER, HELPER, 8, 60, 3, 100),
    onTask('task_confirming', HELPER, 2, '3:70'), onTask('task_resolved', HELPER, 1, '3:50'),
  ], confirmed);
  assert.strictEqual(r.assignments['helper:2'].taskId, 8);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMING,
    'canceling the earlier approved POI keeps the next assignment');

  // ---- task_resolved: the denied POI's advert goes; its task is ASSIGNED ----
  s = run([advert(HELPER, [7], 10), advert(OWNER, [5, 8], 30)], confirming);
  assert.ok('helper:2' in s.assignments);
  r = reduce(s, onTask('task_resolved', HELPER, 1, '3:50'));
  assert.strictEqual(r.assignments['helper:2'].status, S.ASSIGNED,
    'a deny does not show the helper let the task go');
  assert.deepStrictEqual(Object.keys(r.availableTasks).sort(), ['1:8', '2:7'],
    'the owner advert of the denied task goes too');
  assert.strictEqual(reduce(r, onTask('task_resolved', HELPER, 1, '3:50')).assignments, r.assignments,
    'a repeated deny changes nothing');
  assert.ok(!('helper:2' in reduce(r, beat(HELPER, 'FREE', 41)).assignments),
    'an operator deny: the helper releases the task and its FREE beat retires the slot');
  // Peer re-ask (2026-10-25): the card's countdown deny of a round the helper
  // already timed out reaches the GCS before its next round's request.
  r = run([
    onTask('task_resolved', HELPER, 1, '3:50'), beat(HELPER, 'BUSY', 51),
    onTask('task_confirming', HELPER, 1, '3:52'), onTask('task_confirmed', HELPER, 1, '3:52'),
  ], confirming);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMED, 'the next round confirms the kept task');
  r = run([
    onTask('task_confirming', HELPER, 1, '3:52'), onTask('task_resolved', HELPER, 1, '3:50'),
    onTask('task_confirmed', HELPER, 1, '3:52'),
  ], confirming);
  assert.strictEqual(r.assignments['helper:2'].status, S.CONFIRMED, 'the deny after the next request');
  assert.ok(!('helper:2' in run([beat(HELPER, 'FREE', 51), onTask('task_resolved', HELPER, 1, '3:50')],
    confirming).assignments), 'FREE before the deny broadcast: the deny brings nothing back');
  r = run([onTask('task_resolved', HELPER, 1, '3:50'), onTask('task_confirmed', HELPER, 1, '3:50')],
    confirming);
  assert.strictEqual(r.assignments['helper:2'].status, S.ASSIGNED, 'a late approve of the denied round');
  r = run([onTask('task_resolved', HELPER, 1, '3:50'), onTask('task_confirming', HELPER, 1, '3:45')],
    confirming);
  assert.deepStrictEqual(r.assignments['helper:2'].confirm, { taskId: 1, roundUid: '3:50' },
    'a late copy of an older request does not re-point the denied slot');
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

  // ---- dock marker states (docs/design/gcs_dock_marker_states.md) ----
  const handled = (state, uav = HELPER, round = '3:50') => state.assignments[`handled:${uav}:${round}`];
  const ownEntry = (state, uav = HELPER, round = '3:60') => state.assignments[`own:${uav}:${round}`];
  // An approved helper slot, round 3:50.
  const approved = reduce(confirming, onTask('task_confirmed', HELPER, 1, '3:50'));
  assert.strictEqual(approved.assignments['helper:2'].status, S.CONFIRMED);

  // Rule 2 then rule 3: the outcome changes during this approach.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  assert.ok(!('helper:2' in s.assignments));
  assert.strictEqual(handled(s).status, S.HANDLED);
  assert.strictEqual(handled(s).outcome, O.DOCKED);
  assert.strictEqual(handled(s).handledBy, HELPER);
  assert.strictEqual(handled(s).receiverId, null);
  assert.strictEqual(handled(s).lat, 45, 'it keeps the dock position');
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'RECOVERING', 52, 'failed', 3, 'missed_dock')], approved);
  assert.strictEqual(handled(s).outcome, O.FAILED);
  // A failed reason replaced by another failed reason is a change too.
  s = run([phaseBeat(HELPER, 'APPROACH', 51, 'failed', 3, 'nav_fail'),
    phaseBeat(HELPER, 'HOLD', 52, 'failed', 3, 'missed_dock')], approved);
  assert.strictEqual(handled(s).outcome, O.FAILED);

  // The latch: an earlier approach's outcome never labels this dock.
  s = run([phaseBeat(HELPER, 'APPROACH', 51, 'failed', 3, 'nav_fail'),
    phaseBeat(HELPER, 'HOLD', 52, 'failed', 3, 'nav_fail')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'the outcome did not change');
  assert.strictEqual(handled(s).awaitingOutcome, false);
  // The first beat after the approach decides: a later outcome may be another approach's.
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'HOLD', 53, 'docked')), s);
  // Codex's sequence: no APPROACH beat of this round was heard.
  s = reduce(approved, phaseBeat(HELPER, 'HOLD', 52, 'docked'));
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'no approach heard: no word');
  assert.strictEqual(handled(s).awaitingOutcome, false);
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'HOLD', 53, 'failed', 3, 'nav_fail')), s,
    'nor can a later beat give it one');
  // Beats from before the round, and open phases, change nothing.
  assert.strictEqual(reduce(approved, phaseBeat(HELPER, 'HOLD', 49, 'docked')), approved);
  for (const phase of ['CONFIRM', 'TO_TASK']) {
    assert.strictEqual(reduce(approved, phaseBeat(HELPER, phase, 51)), approved, phase);
  }
  assert.strictEqual(reduce(approved, beat(HELPER, 'BUSY', 51)), approved, 'no phase: an old companion');
  assert.strictEqual(reduce(approved, phaseBeat(OTHER, 'HOLD', 51, 'docked')), approved, 'another UAV');
  // The outcome only changes as an approach ends: an APPROACH beat with a new one is a
  // later approach, whose end cannot label this dock (Codex's unheard-gap sequence).
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'APPROACH', 61, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(handled(reduce(s, phaseBeat(HELPER, 'HOLD', 70, 'failed', 3, 'nav_fail'))).outcome, O.UNKNOWN);
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
    phaseBeat(HELPER, 'APPROACH', 61, 'docked'), phaseBeat(HELPER, 'HOLD', 70, 'failed', 3, 'nav_fail')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(handled(s).awaitingOutcome, false);

  // Codex final diff, round 8: a next round's beat ends the heard approach, so a later
  // approach's end cannot label this dock.
  for (const phase of ['CONFIRM', 'TO_TASK']) {
    s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, phase, 70, 'failed', 3, 'nav_fail')], approved);
    assert.strictEqual(handled(s).outcome, O.FAILED, `${phase} carries the new outcome`);
    assert.strictEqual(handled(reduce(s, phaseBeat(HELPER, 'HOLD', 90, 'docked'))).outcome, O.FAILED, phase);
    s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, phase, 70)], approved);
    assert.strictEqual(handled(s).outcome, O.UNKNOWN, `${phase} with no new outcome`);
    assert.strictEqual(handled(reduce(s, phaseBeat(HELPER, 'HOLD', 90, 'docked'))).outcome, O.UNKNOWN, phase);
    s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
      phaseBeat(HELPER, phase, 70, 'failed', 3, 'nav_fail')], approved);
    assert.strictEqual(handled(s).outcome, O.FAILED, `${phase} after a disarm`);
    assert.strictEqual(handled(s).awaitingOutcome, false, phase);
    s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
      phaseBeat(HELPER, phase, 70), phaseBeat(HELPER, 'HOLD', 90, 'docked')], approved);
    assert.strictEqual(handled(s).outcome, O.UNKNOWN, `${phase} after a disarm, no new outcome`);
  }
  // Codex final diff, round 10: a gap takes the word away, not the approach it heard.
  for (const phase of ['CONFIRM', 'TO_TASK']) {
    s = run([phaseBeat(HELPER, 'APPROACH', 51), { type: 'link_gap', data: { sys_id: HELPER } },
      phaseBeat(HELPER, phase, 70, 'failed', 3, 'nav_fail')], approved);
    assert.strictEqual(handled(s).outcome, O.UNKNOWN, `${phase} after a gap`);
    s = run([{ type: 'link_gap', data: { sys_id: HELPER } }, phaseBeat(HELPER, phase, 70)], approved);
    assert.strictEqual(handled(s), undefined, `${phase}: no approach heard, the round is still open`);
  }
  // A beat of the same approach still changes nothing, before or after a disarm.
  s = reduce(approved, phaseBeat(HELPER, 'APPROACH', 51));
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'APPROACH', 52)), s);
  s = reduce(s, onVehicle('disarm_after_guided', HELPER));
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'APPROACH', 52)), s);

  // Rule 1: a beat from another boot.
  s = reduce(approved, phaseBeat(HELPER, 'SEARCH', 2, null, 4));
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'the helper restarted');
  s = run([asked(HELPER, 7, '3:60'), phaseBeat(HELPER, 'SEARCH', 2, null, 4)]);
  assert.strictEqual(ownEntry(s), undefined, 'a pending own round of a retired boot goes');
  s = run([phaseBeat(HELPER, 'APPROACH', 51, 'failed', 3, 'nav_fail'),
    phaseBeat(HELPER, 'HOLD', 52, 'failed', 3, 'nav_fail'), phaseBeat(HELPER, 'HOLD', 2, 'docked', 4)], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'another boot fills in nothing');
  assert.strictEqual(handled(s).awaitingOutcome, false);

  // A new round ends the fence: the later approach cannot label the earlier dock.
  s = run([
    phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52),
    asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'),
    phaseBeat(HELPER, 'APPROACH', 61), phaseBeat(HELPER, 'HOLD', 62, 'docked'),
  ], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'the earlier dock');
  assert.strictEqual(handled(s).awaitingOutcome, false);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.DOCKED, 'the later dock');
  s = run([asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'),
    asked(HELPER, 8, '3:70', 42, 46)]);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.UNKNOWN, 'an approved round is ended');
  assert.strictEqual(ownEntry(s, HELPER, '3:70').status, S.CONFIRMING);
  s = run([asked(HELPER, 7, '3:60'), asked(HELPER, 8, '3:70', 42, 46)]);
  assert.strictEqual(ownEntry(s), undefined, 'a superseded pending own round goes');

  // A WebSocket drop may hide a round: Codex's A, reconnect, B sequence.
  s = run([{ type: 'link_gap' }, phaseBeat(HELPER, 'APPROACH', 71, 'failed', 3, 'nav_fail'),
    phaseBeat(HELPER, 'HOLD', 72, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(handled(s).awaitingOutcome, false);
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER), { type: 'link_gap' },
    phaseBeat(HELPER, 'HOLD', 80, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'nor is a waiting entry filled in');
  assert.strictEqual(reduce(assigned, { type: 'link_gap' }), assigned, 'nothing approved');
  // Codex final diff, round 9: one UAV's beats outlived their TTL. Its word goes, another's stays.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), { type: 'link_gap', data: { sys_id: HELPER } },
    phaseBeat(HELPER, 'HOLD', 90, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  s = run([phaseBeat(HELPER, 'APPROACH', 51), { type: 'link_gap', data: { sys_id: OTHER } },
    phaseBeat(HELPER, 'HOLD', 90, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.DOCKED);

  // Handled stays: disarm, E-STOP, a new task on the slot, FREE beats.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  for (const action of [
    onVehicle('disarm_after_guided', HELPER), onVehicle('vehicle_reset', HELPER), beat(HELPER, 'FREE', 90),
    request(OTHER, HELPER, 8, 30), { type: 'link_gap' },
  ]) {
    assert.strictEqual(handled(reduce(s, action)).outcome, O.DOCKED, action.type);
  }
  r = run([request(OWNER, HELPER, 8, 30), doing(HELPER, OWNER, 8, 60), applied(OWNER, HELPER, 8, 60, 3, 100)], s);
  assert.strictEqual(r.assignments['helper:2'].taskId, 8);
  assert.strictEqual(handled(r).outcome, O.DOCKED, 'a new task on the slot does not erase it');
  // Cleared by a new mission only, and then the whole map (useWsHandlers).
  assert.strictEqual(reduce(s, { type: 'reset' }), INITIAL);

  // Disarm or E-STOP moves an approved dock; lower ranks go, as today.
  for (const type of ['disarm_after_guided', 'vehicle_reset']) {
    s = reduce(approved, onVehicle(type, HELPER));
    assert.strictEqual(handled(s).outcome, O.UNKNOWN, type);
    assert.strictEqual(handled(s).awaitingOutcome, false, 'no approach heard');
    assert.ok(!('helper:2' in s.assignments));
    assert.deepStrictEqual(Object.keys(reduce(confirming, onVehicle(type, HELPER)).assignments), [], type);
    s = run([asked(HELPER, 7, '3:60'), onVehicle(type, HELPER)]);
    assert.deepStrictEqual(s.assignments, {}, 'a pending own round goes');
    // Disarm before the outcome beat still gets its word.
    s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle(type, HELPER), phaseBeat(HELPER, 'HOLD', 52, 'docked')],
      approved);
    assert.strictEqual(handled(s).outcome, O.DOCKED, type);
  }

  // A new APPLIED that replaces an approved slot ends it.
  s = run([request(OWNER, HELPER, 8, 30), doing(HELPER, OWNER, 8, 60), applied(OWNER, HELPER, 8, 60, 3, 100)], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  // The same assignment applied again keeps its approval.
  s = run([request(OWNER, HELPER, 5, 24), applied(OWNER, HELPER, 5, 40, 3, 96)], approved);
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMED);
  assert.strictEqual(handled(s), undefined);

  // Cancel on an approved dock: handled, canceled; a pending deny still returns to assigned.
  s = reduce(approved, onTask('task_resolved', HELPER, 1, '3:50'));
  assert.strictEqual(handled(s).outcome, O.CANCELED);
  assert.ok(!('helper:2' in s.assignments));
  assert.strictEqual(reduce(s, onVehicle('disarm_after_guided', HELPER)).assignments['handled:2:3:50'].outcome, O.CANCELED);
  assert.strictEqual(reduce(confirming, onTask('task_resolved', HELPER, 1, '3:50')).assignments['helper:2'].status, S.ASSIGNED);

  // A UAV's own dock, including a homing diversion, through the same states.
  assert.strictEqual(reduce(INITIAL, onTask('task_confirming', HELPER, 7, '3:60')), INITIAL, 'no position, no marker');
  for (const [lat, lon] of [[null, null], [0, 0], [NaN, NaN]]) {
    assert.strictEqual(reduce(INITIAL, asked(HELPER, 7, '3:60', lat, lon)), INITIAL, `position ${lat},${lon}`);
  }
  assert.strictEqual(reduce(INITIAL, asked(HELPER, 7, 'legacy')), INITIAL, 'no round uid');
  s = reduce(INITIAL, phaseBeat(HELPER, 'HOMING', 59));
  assert.deepStrictEqual(s.assignments, {}, 'a homing UAV has no marker before its round');
  s = reduce(s, asked(HELPER, 7, '3:60'));
  assert.strictEqual(ownEntry(s).status, S.CONFIRMING);
  assert.strictEqual(ownEntry(s).receiverId, null);
  assert.strictEqual(ownEntry(s).askedBy, HELPER);
  assert.strictEqual(uavOf(ownEntry(s)), HELPER);
  assert.strictEqual(vehicleAssignment(s.assignments, HELPER), null, 'the UAV card does not read it');
  assert.strictEqual(reduce(s, asked(HELPER, 7, '3:60')), s, 'a repeat of the round');
  assert.strictEqual(reduce(s, asked(HELPER, 6, '3:55')), s, 'a late older round');
  assert.strictEqual(reduce(s, onTask('task_resolved', HELPER, 7, '3:60')).assignments['own:2:3:60'], undefined,
    'a deny drops it');
  assert.strictEqual(reduce(s, onTask('task_confirmed', HELPER, 7, '3:59')), s, 'another round\'s answer');
  r = reduce(s, onTask('task_confirmed', HELPER, 7, '3:60'));
  assert.strictEqual(ownEntry(r).status, S.CONFIRMED);
  assert.strictEqual(vehicleAssignment(r.assignments, HELPER), null);
  s = run([phaseBeat(HELPER, 'APPROACH', 61), phaseBeat(HELPER, 'HOMING', 62, 'failed', 3, 'missed_dock')], r);
  assert.strictEqual(ownEntry(s), undefined);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.FAILED);
  assert.strictEqual(handled(s, HELPER, '3:60').askedBy, undefined);
  assert.strictEqual(vehicleAssignment(s.assignments, HELPER), null);
  r = reduce(r, onTask('task_resolved', HELPER, 7, '3:60'));
  assert.strictEqual(handled(r, HELPER, '3:60').outcome, O.CANCELED, 'cancel on an approved own dock');

  // A helper's position-bearing round before APPLIED stays its own: never moved to the slot (todo 88).
  s = run([request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40), asked(HELPER, 1, '3:50'),
    applied(OWNER, HELPER, 5, 40), onTask('task_confirmed', HELPER, 1, '3:50')]);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(ownEntry(s, HELPER, '3:50').status, S.CONFIRMED);
  // A position-bearing round the slot takes makes no own entry.
  s = reduce(assigned, asked(HELPER, 1, '3:50', 45, 44));
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMING);
  assert.strictEqual(ownEntry(s, HELPER, '3:50'), undefined);
  assert.strictEqual(reduce(s, asked(HELPER, 1, '3:50', 45, 44)), s);
  assert.strictEqual(reduce(s, asked(HELPER, 1, '3:45', 45, 44)), s, 'a late older copy');

  // A delayed request copy after handled makes no slot; only an advert or another helper starts a new offer.
  const done = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  assert.strictEqual(reduce(done, request(OWNER, HELPER, 5, 22)), done);
  r = reduce(done, request(OWNER, OTHER, 5, 30));
  assert.deepStrictEqual(handled(r).reoffered, uid(7, 30), 'another helper');
  assert.strictEqual(r.assignments['1:5'].receiverId, OTHER);
  r = reduce(done, request(OWNER, HELPER, 5, 1, 8));
  assert.strictEqual(r.assignments['1:5'].status, S.WAITING, 'a restarted owner');
  r = reduce(done, advert(OWNER, [5], 30));
  assert.deepStrictEqual(handled(r).reoffered, uid(7, 30), 'a later advert');
  assert.ok('1:5' in reduce(r, request(OWNER, HELPER, 5, 31)).assignments, 'then the helper may take it again');
  assert.strictEqual(handled(reduce(done, advert(OWNER, [5], 15))).reoffered, undefined, 'an older advert');
  assert.strictEqual(handled(reduce(done, advert(OWNER, [6], 30))).reoffered, undefined, 'another task');

  // The cards never read handled or own entries.
  assert.strictEqual(vehicleAssignment(done.assignments, HELPER), null);
  assert.strictEqual(isAssignedOrLater(handled(done)), false);

  // ---- dockMarkerEntries: one marker per offer, by precedence ----
  const marks = (state) => dockMarkerEntries(state.assignments, state.availableTasks);
  const offered = run([advert(OWNER, [5], 10)]);
  assert.deepStrictEqual(Object.keys(marks(offered).availableTasks), ['1:5']);
  assert.deepStrictEqual(marks(offered).assignments, {});
  s = reduce(offered, request(OWNER, HELPER, 5, 20));
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['1:5'], 'waiting');
  // Codex's probe: a delayed retry beside the approved slot.
  assert.strictEqual(reduce(approved, request(OWNER, HELPER, 5, 22)), approved, 'a copy from before the APPLIED');
  // A stale available entry stays hidden behind a higher state.
  s = reduce(done, advert(OWNER, [5], 15));
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['handled:2:3:50']);
  assert.deepStrictEqual(marks(s).availableTasks, {});
  // History never hides a new offer.
  s = reduce(done, advert(OWNER, [5], 30));
  assert.deepStrictEqual(handled(s).reoffered, uid(7, 30));
  assert.deepStrictEqual(marks(s).assignments, {}, 'a live state hides the history at its spot');
  assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5'], 'a later advert');
  s = run([advert(OWNER, [5], 15), advert(OWNER, [5], 2, 8)], done);
  assert.strictEqual(s.availableTasks['1:5'].ownerBoot, 8);
  assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5'], 'a new owner boot');
  r = reduce(assigned, { type: 'task_assign_request', data: {
    sender_id: OWNER, receiver_id: OTHER, task_id: 5, task_type: 'DOCK', lat: 50, lon: 50, alt: 0, uid: uid(7, 30),
  } });
  assert.strictEqual(Object.keys(marks(r).assignments).length, 2, 'a recycled id at another position');

  // A repeat of the assignment keeps the outcome's evidence.
  s = run([{ type: 'link_gap' }, request(OWNER, HELPER, 5, 24), applied(OWNER, HELPER, 5, 40, 3, 96),
    phaseBeat(HELPER, 'APPROACH', 71, 'failed', 3, 'nav_fail'), phaseBeat(HELPER, 'HOLD', 72, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'a gap is not forgotten');
  s = run([phaseBeat(HELPER, 'APPROACH', 51), request(OWNER, HELPER, 5, 24), applied(OWNER, HELPER, 5, 40, 3, 96),
    phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.DOCKED, 'nor is a heard approach');

  // An approach after this one ended, with no round (auto-approve), gives no word,
  // heard or not.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOMING', 52), phaseBeat(HELPER, 'HOLD', 62, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  s = run([phaseBeat(HELPER, 'APPROACH', 51, 'failed', 3, 'nav_fail'), phaseBeat(HELPER, 'HOMING', 52, 'failed', 3, 'nav_fail'),
    phaseBeat(HELPER, 'APPROACH', 61, 'failed', 3, 'nav_fail'), phaseBeat(HELPER, 'HOLD', 62, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(handled(s).awaitingOutcome, false);
  // A disarm can come before the phase leaves, so the same APPROACH may still be beating.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
    phaseBeat(HELPER, 'APPROACH', 52), phaseBeat(HELPER, 'HOLD', 53, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.DOCKED);
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
    phaseBeat(HELPER, 'HOMING', 52), phaseBeat(HELPER, 'APPROACH', 61), phaseBeat(HELPER, 'HOLD', 62, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'left, then a new approach');
  assert.strictEqual(handled(s).awaitingOutcome, false);

  // The same task id at another position is another dock: it inherits no approval.
  const elsewhere = { type: 'task_assign_request', data: {
    sender_id: OWNER, receiver_id: HELPER, task_id: 5, task_type: 'DOCK', lat: 50, lon: 50, alt: 0, uid: uid(7, 30),
  } };
  s = run([phaseBeat(HELPER, 'APPROACH', 51), elsewhere, doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 100),
    phaseBeat(HELPER, 'HOLD', 81, 'docked')], approved);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(s.assignments['helper:2'].lat, 50);
  assert.strictEqual(s.assignments['helper:2'].confirm, undefined);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);

  // A repeat of the assignment does not hide the new offer again.
  s = run([advert(OWNER, [5], 97), request(OWNER, HELPER, 5, 24), applied(OWNER, HELPER, 5, 40, 3, 96),
    request(OWNER, OTHER, 5, 98), phaseBeat(HELPER, 'HOLD', 52)], approved);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['1:5']);
  assert.deepStrictEqual(handled(s).reoffered, uid(7, 97));
  r = reduce(s, doing(OTHER, OWNER, 5, 40, false));
  assert.deepStrictEqual(Object.keys(marks(r).assignments), ['handled:2:3:50'], 'it returns when the live entry ends');

  // A later advert frees the offer in either order with the approach's end.
  for (const order of [
    [advert(OWNER, [5], 30), phaseBeat(HELPER, 'HOLD', 52)],
    [phaseBeat(HELPER, 'HOLD', 52), advert(OWNER, [5], 30)],
  ]) {
    s = run(order, approved);
    assert.deepStrictEqual(handled(s).reoffered, uid(7, 30));
    assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5']);
    assert.strictEqual(reduce(s, request(OWNER, HELPER, 5, 31)).assignments['1:5'].status, S.WAITING);
  }

  // An older request to another helper is a delayed copy too.
  assert.strictEqual(reduce(done, request(OWNER, OTHER, 5, 1)), done);
  // The same task id at another position is another offer.
  r = reduce(done, { type: 'task_assign_request', data: {
    sender_id: OWNER, receiver_id: HELPER, task_id: 5, task_type: 'DOCK', lat: 50, lon: 50, alt: 0, uid: uid(7, 30),
  } });
  assert.strictEqual(r.assignments['1:5'].status, S.WAITING);
  assert.strictEqual(handled(r).reoffered, undefined);
  assert.strictEqual(Object.keys(marks(r).assignments).length, 2);

  // A new round on an own dock shows over its history; a deny brings the history back.
  s = run([asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'),
    phaseBeat(HELPER, 'HOLD', 61), asked(HELPER, 7, '3:70')]);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['own:2:3:70']);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.UNKNOWN);
  r = reduce(s, onTask('task_resolved', HELPER, 7, '3:70'));
  assert.deepStrictEqual(Object.keys(marks(r).assignments), ['handled:2:3:60']);
  // Approved and flown, its own end shows.
  r = run([onTask('task_confirmed', HELPER, 7, '3:70'), phaseBeat(HELPER, 'APPROACH', 71), phaseBeat(HELPER, 'HOLD', 72, 'docked')], s);
  assert.deepStrictEqual(Object.keys(marks(r).assignments), ['handled:2:3:70']);
  s = run([asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'),
    phaseBeat(HELPER, 'HOLD', 61), asked(HELPER, 7, '3:70', 42, 46)]);
  assert.strictEqual(Object.keys(marks(s).assignments).length, 2, 'the id at another position');

  // No marker before a detection or peer message.
  assert.deepStrictEqual(marks(INITIAL), { assignments: {}, availableTasks: {} });
  for (const action of [phaseBeat(OWNER, 'SEARCH', 5), phaseBeat(OWNER, 'HOMING', 6), { type: 'link_gap' }]) {
    assert.deepStrictEqual(marks(reduce(INITIAL, action)), { assignments: {}, availableTasks: {} });
  }

  // ---- final audit (Review section of gcs_dock_marker_states.md) ----

  // An own entry keeps what its round said: the map draws from these fields.
  s = reduce(INITIAL, { type: 'task_confirming', data: {
    sys_id: HELPER, task_id: 7, round_uid: '3:60', task_type: 'DOCK', lat: 41, lon: 45, alt: 12,
  } });
  assert.deepStrictEqual(ownEntry(s), {
    key: 'own:2:3:60', taskId: 7, taskType: 'DOCK', lat: 41, lon: 45, alt: 12,
    senderId: HELPER, receiverId: null, askedBy: HELPER, status: S.CONFIRMING,
    requestUid: uid(3, 60), confirm: { taskId: 7, roundUid: '3:60' },
  });

  // A new round ends an entry that still had a word to lose.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
    asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'),
    phaseBeat(HELPER, 'APPROACH', 61), phaseBeat(HELPER, 'HOLD', 62, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'the earlier dock, disarmed first');
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.DOCKED);
  s = run([phaseBeat(HELPER, 'APPROACH', 51), asked(HELPER, 7, '3:60'), phaseBeat(HELPER, 'HOLD', 61, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'the earlier dock, its approach still open');

  // The latch on the usual path, where the disarm comes first.
  s = run([phaseBeat(HELPER, 'APPROACH', 51, 'failed', 3, 'nav_fail'), onVehicle('disarm_after_guided', HELPER),
    phaseBeat(HELPER, 'HOLD', 52, 'failed', 3, 'nav_fail')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(handled(s).awaitingOutcome, false);

  // A word from another boot never labels this dock.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 2, 'docked', 4)], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);

  // The next beat does not overwrite the operator's cancel.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onTask('task_resolved', HELPER, 1, '3:50'),
    phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.CANCELED);

  // The UAV's teardown beat can overtake the broadcast of a cancel.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'SEARCH', 52),
    onTask('task_resolved', HELPER, 1, '3:50')], approved);
  assert.strictEqual(handled(s).outcome, O.CANCELED, 'a late cancel of a slot');
  s = run([asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'), phaseBeat(HELPER, 'APPROACH', 61),
    phaseBeat(HELPER, 'SEARCH', 62), onTask('task_resolved', HELPER, 7, '3:60')]);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.CANCELED, 'a late cancel of an own dock');
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52, 'docked'),
    onTask('task_resolved', HELPER, 1, '3:50')], approved);
  assert.strictEqual(handled(s).outcome, O.DOCKED, 'a real word stays');
  s = reduce(approved, phaseBeat(HELPER, 'HOLD', 52));
  assert.strictEqual(handled(reduce(s, onTask('task_resolved', HELPER, 1, '3:45'))).outcome, O.UNKNOWN, 'another round');
  assert.strictEqual(handled(reduce(s, onTask('task_resolved', HELPER, 9, '3:50'))).outcome, O.UNKNOWN, 'another POI');
  s = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER),
    onTask('task_resolved', HELPER, 1, '3:50'), phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.CANCELED, 'disarmed first');
  assert.strictEqual(handled(s).awaitingOutcome, false);

  // One UAV's disarm or E-STOP leaves another UAV's approved dock alone.
  const twoApproved = run([asked(OTHER, 7, '5:60', 42, 46), onTask('task_confirmed', OTHER, 7, '5:60')], approved);
  for (const action of [onVehicle('disarm_after_guided', HELPER), onVehicle('vehicle_reset', HELPER)]) {
    s = reduce(twoApproved, action);
    assert.strictEqual(ownEntry(s, OTHER, '5:60'), twoApproved.assignments['own:3:5:60'], action.type);
    assert.strictEqual(handled(s).outcome, O.UNKNOWN, action.type);
  }

  // Pending entries stay through ordinary beats; only an approved one has a history.
  const pendingOwn = reduce(INITIAL, asked(HELPER, 7, '3:60'));
  assert.strictEqual(reduce(pendingOwn, phaseBeat(HELPER, 'CONFIRM', 61)), pendingOwn);
  assert.strictEqual(reduce(pendingOwn, phaseBeat(HELPER, 'HOMING', 61)), pendingOwn, 'its answer can still come');
  s = reduce(confirming, onTask('task_resolved', HELPER, 1, '3:50'));
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'SEARCH', 51)), s, 'a denied slot');
  assert.strictEqual(reduce(confirming, phaseBeat(HELPER, 'HOLD', 51)), confirming, 'an unanswered slot');
  s = run([request(OWNER, HELPER, 8, 30), doing(HELPER, OWNER, 8, 60), applied(OWNER, HELPER, 8, 60, 3, 100)], confirming);
  assert.strictEqual(handled(s), undefined, 'a new task over a slot nobody approved');

  // A round pending across a socket gap: its answer or a later round may have been missed.
  s = reduce(pendingOwn, { type: 'link_gap' });
  assert.strictEqual(ownEntry(s).noWord, true);
  assert.strictEqual(reduce(s, { type: 'link_gap' }), s, 'a second gap');
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'CONFIRM', 61)), s, 'still asking');
  assert.strictEqual(reduce(s, phaseBeat(HELPER, 'HOLD', 59, 'docked')), s, 'a beat from before its round');
  assert.strictEqual(ownEntry(reduce(s, phaseBeat(HELPER, 'APPROACH', 61))), undefined, 'it has left CONFIRM');
  s = run([{ type: 'link_gap' }, phaseBeat(HELPER, 'CONFIRM', 61), onTask('task_confirmed', HELPER, 7, '3:60'),
    phaseBeat(HELPER, 'APPROACH', 91), phaseBeat(HELPER, 'HOLD', 92, 'docked')], pendingOwn);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.UNKNOWN, 'approved after the gap: no word');
  s = run([{ type: 'link_gap' }, onTask('task_confirmed', HELPER, 1, '3:50'),
    phaseBeat(HELPER, 'APPROACH', 91), phaseBeat(HELPER, 'HOLD', 92, 'docked')], confirming);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'a slot too');
  s = run([{ type: 'link_gap' }, onTask('task_confirming', HELPER, 2, '3:70'), onTask('task_confirmed', HELPER, 2, '3:70'),
    phaseBeat(HELPER, 'APPROACH', 71), phaseBeat(HELPER, 'HOLD', 72, 'docked')], confirming);
  assert.strictEqual(handled(s, HELPER, '3:70').outcome, O.DOCKED, 'a later round is one the gap did not hide');
  s = reduce(approved, { type: 'link_gap' });
  assert.strictEqual(reduce(s, { type: 'link_gap' }), s, 'a second gap on an approved entry');

  // A later advert releases a live slot too, so the new offer shows beside it.
  for (const slot of [assigned, confirming]) {
    s = reduce(slot, advert(OWNER, [5], 30));
    assert.deepStrictEqual(s.assignments['helper:2'].reoffered, uid(7, 30));
    assert.strictEqual(s.assignments['helper:2'].status, slot.assignments['helper:2'].status);
    assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5']);
    assert.deepStrictEqual(Object.keys(marks(s).assignments), ['helper:2']);
    assert.strictEqual(marks(s).availableTasks['1:5'].labelRow, 1, 'its word on the next row');
  }
  // A second UAV on the offer has its own marker, through to its own end.
  const twoOnOffer = run([advert(OWNER, [5], 30), request(OWNER, OTHER, 5, 31), doing(OTHER, OWNER, 5, 41, true, 5),
    applied(OWNER, OTHER, 5, 41, 5, 91)], confirming);
  assert.deepStrictEqual(Object.keys(marks(twoOnOffer).assignments).sort(), ['helper:2', 'helper:3']);
  s = run([onTask('task_confirmed', HELPER, 1, '3:50'), phaseBeat(HELPER, 'APPROACH', 51),
    phaseBeat(HELPER, 'HOLD', 52, 'docked')], twoOnOffer);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['helper:3'], 'a live state hides the history at its spot');
  s = run([asked(OTHER, 1, '5:50', 45, 44), onTask('task_confirmed', OTHER, 1, '5:50'), phaseBeat(OTHER, 'APPROACH', 51, null, 5),
    phaseBeat(OTHER, 'HOLD', 52, 'failed', 5, 'missed_dock')], s);
  const both = marks(s).assignments;
  assert.deepStrictEqual(Object.keys(both).sort(), ['handled:2:3:50', 'handled:3:5:50']);
  assert.deepStrictEqual(Object.values(both).map((entry) => entry.outcome).sort(), [O.DOCKED, O.FAILED]);
  assert.deepStrictEqual(Object.values(both).map((entry) => entry.labelRow ?? 0).sort(), [0, 1], 'one word per row');

  // An offer shows as detected or offered only until a UAV holds it.
  assert.deepStrictEqual(Object.keys(marks(reduce(assigned, request(OWNER, HELPER, 5, 22))).assignments), ['helper:2']);
  assert.deepStrictEqual(Object.keys(marks(reduce(confirming, request(OWNER, HELPER, 5, 22))).assignments), ['helper:2']);
  s = run([request(OWNER, HELPER, 5, 20), advert(OWNER, [5], 15)]);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['1:5'], 'offered over a delayed older advert');
  assert.deepStrictEqual(marks(s).availableTasks, {});
  // History stays after a late copy of the slot, and a restarted helper's new slot shows over it.
  s = run([request(OWNER, HELPER, 5, 22), phaseBeat(HELPER, 'HOLD', 52), applied(OWNER, HELPER, 5, 40, 3, 96)], approved);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['handled:2:3:50']);
  s = run([request(OWNER, HELPER, 5, 30), doing(HELPER, OWNER, 5, 10, true, 4), applied(OWNER, HELPER, 5, 10, 4, 95)], approved);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['helper:2']);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  // Another UAV's own round at a held dock has its own marker and hides that dock's history.
  s = run([request(HELPER, OTHER, 7, 20, 3), doing(OTHER, HELPER, 7, 40, true, 5), applied(HELPER, OTHER, 7, 40, 5),
    asked(OTHER, 1, '5:50', 47, 44), onTask('task_confirmed', OTHER, 1, '5:50'), asked(HELPER, 7, '3:60', 47, 44)]);
  assert.deepStrictEqual(Object.keys(marks(s).assignments).sort(), ['helper:3', 'own:2:3:60']);
  s = reduce(s, phaseBeat(OTHER, 'HOLD', 52, null, 5));
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['own:2:3:60']);

  // Another task at the same position is another offer, and inherits no approval.
  const sameSpotTask = (owner, helper, task_id, seq) => ({ type: 'task_assign_request', data: {
    sender_id: owner, receiver_id: helper, task_id, task_type: 'DOCK', lat: 45, lon: 44, alt: 0, uid: uid(7, seq),
  } });
  s = run([sameSpotTask(OWNER, HELPER, 6, 30), doing(HELPER, OWNER, 6, 80), applied(OWNER, HELPER, 6, 80, 3, 100)], approved);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(s.assignments['helper:2'].confirm, undefined);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(reduce(done, sameSpotTask(OWNER, HELPER, 6, 30)).assignments['1:6'].status, S.WAITING);
  s = reduce(INITIAL, { type: 'available_task_request', data: {
    sender_id: OWNER, uid: uid(7, 10),
    tasks: [5, 6].map((task_id) => ({ task_id, task_type: 'DOCK', lat: 45, lon: 44, alt: 0 })),
  } });
  assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5', '1:6']);
  assert.strictEqual(marks(s).availableTasks['1:6'].labelRow, 1);

  // One changed coordinate is another dock.
  for (const [lat, lon] of [[45, 50], [50, 44]]) {
    r = reduce(done, { type: 'task_assign_request', data: {
      sender_id: OWNER, receiver_id: HELPER, task_id: 5, task_type: 'DOCK', lat, lon, alt: 0, uid: uid(7, 30),
    } });
    assert.strictEqual(r.assignments['1:5'].status, S.WAITING, `a request at ${lat},${lon}`);
    r = reduce(done, { type: 'available_task_request', data: {
      sender_id: OWNER, uid: uid(7, 15), tasks: [{ task_id: 5, task_type: 'DOCK', lat, lon, alt: 0 }],
    } });
    assert.deepStrictEqual(Object.keys(marks(r).availableTasks), ['1:5'], `an advert at ${lat},${lon}`);
    assert.deepStrictEqual(Object.keys(marks(r).assignments), ['handled:2:3:50']);
  }

  // A UAV's advert and its own round of one dock are one marker, and its own advert
  // does not release its pending round.
  s = run([advert(HELPER, [1], 10, 3), asked(HELPER, 1, '3:60', 41, 44)]);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['own:2:3:60']);
  assert.deepStrictEqual(marks(s).availableTasks, {});
  assert.strictEqual(reduce(s, advert(HELPER, [1], 70, 3)), s);

  // Repeats change nothing, not even the state reference.
  assert.strictEqual(reduce(approved, onTask('task_confirmed', HELPER, 1, '3:50')), approved);
  s = reduce(done, advert(OWNER, [5], 30));
  assert.strictEqual(reduce(s, advert(OWNER, [5], 31)), s);

  // A re-advert with changed details updates the detected marker.
  for (const change of [{ lat: 46 }, { lon: 45 }, { alt: 12 }, { task_type: 'OTHER' }]) {
    const task = reduce(offered, { type: 'available_task_request', data: {
      sender_id: OWNER, uid: uid(7, 11), tasks: [{ task_id: 5, task_type: 'DOCK', lat: 45, lon: 44, alt: 0, ...change }],
    } }).availableTasks['1:5'];
    assert.deepStrictEqual(
      { lat: task.lat, lon: task.lon, alt: task.alt, task_type: task.taskType },
      { lat: 45, lon: 44, alt: 0, task_type: 'DOCK', ...change },
    );
  }

  // A restarted owner's task at the same position inherits no approval.
  s = run([request(OWNER, HELPER, 5, 2, 8), doing(HELPER, OWNER, 5, 80), { type: 'task_assign_ack', data: {
    owner_id: OWNER, helper_id: HELPER, task_id: 5, status: 'APPLIED', ref: uid(3, 80), uid: uid(8, 3),
  } }], approved);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(s.assignments['helper:2'].confirm, undefined);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);

  // A beat or an outcome the map cannot read changes nothing it says.
  assert.strictEqual(reduce(approved, { type: 'swarm_heartbeat', data: { sys_id: HELPER } }), approved, 'no swarm state');
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52, 'no_such_word')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);

  // A round needs both coordinates; the equator and the prime meridian are positions.
  for (const [lat, lon] of [[NaN, 44], [41, NaN], [null, 44], [41, null], [0, 0]]) {
    assert.strictEqual(reduce(INITIAL, asked(HELPER, 7, '3:60', lat, lon)), INITIAL, `no position: ${lat},${lon}`);
  }
  for (const [lat, lon] of [[0, 44], [41, 0]]) {
    assert.ok(ownEntry(reduce(INITIAL, asked(HELPER, 7, '3:60', lat, lon))), `a position: ${lat},${lon}`);
  }

  // An answer must name the round's POI too.
  assert.strictEqual(reduce(pendingOwn, onTask('task_confirmed', HELPER, 9, '3:60')), pendingOwn);
  assert.strictEqual(reduce(pendingOwn, onTask('task_resolved', HELPER, 9, '3:60')), pendingOwn);

  // Another UAV's or owner's messages leave these entries alone, even under the same boot id.
  r = reduce(approved, asked(OTHER, 7, '3:60'));
  assert.strictEqual(r.assignments['helper:2'], approved.assignments['helper:2'], 'a later round of another UAV');
  assert.ok(reduce(pendingOwn, asked(OTHER, 8, '3:40', 42, 46)).assignments['own:3:3:40'], 'an earlier round of another UAV');
  assert.strictEqual(handled(reduce(done, advert(OTHER, [5], 30))).reoffered, undefined, 'another owner\'s advert');
  assert.strictEqual(reduce(done, request(OTHER, HELPER, 5, 10)).assignments['3:5'].status, S.WAITING, 'another owner\'s request');
  s = run([request(OTHER, HELPER, 5, 30), doing(HELPER, OTHER, 5, 80), applied(OTHER, HELPER, 5, 80, 3, 100)], approved);
  assert.strictEqual(s.assignments['helper:2'].senderId, OTHER);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED, 'another owner\'s task inherits no approval');
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  s = run([advert(OWNER, [5], 10), advert(OTHER, [5], 10)]);
  assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5', '3:5'], 'two owners\' offers at one position');

  // A dock waiting for its word after a disarm.
  const awaiting = run([phaseBeat(HELPER, 'APPROACH', 51), onVehicle('disarm_after_guided', HELPER)], approved);
  assert.strictEqual(handled(awaiting).awaitingOutcome, true);
  assert.strictEqual(reduce(awaiting, phaseBeat(HELPER, 'HOLD', 49, 'docked')), awaiting, 'a beat from before its round');
  s = reduce(awaiting, phaseBeat(HELPER, 'HOLD', 2, 'docked', 4));
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(handled(s).awaitingOutcome, false, 'a new boot ends the wait');
  s = run([phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'SEARCH', 1, null, 4), phaseBeat(HELPER, 'HOLD', 52, 'docked')], approved);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN, 'a late beat of the old boot');

  // A request with no uid cannot be ordered: an advert does not release it, and it
  // is no copy of a handled dock's.
  const noUid = { type: 'task_assign_request', data: { ...request(OWNER, HELPER, 5, 20).data, uid: null } };
  assert.strictEqual(run([noUid, advert(OWNER, [5], 30)]).assignments['1:5'].status, S.WAITING);
  assert.strictEqual(reduce(done, noUid).assignments['1:5'].status, S.WAITING);

  // The same UAV on the same dock again after a re-offer: each decision keeps its word.
  s = run([advert(OWNER, [5], 30), request(OWNER, HELPER, 5, 31), doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 100),
    onTask('task_confirming', HELPER, 2, '3:90'), onTask('task_confirmed', HELPER, 2, '3:90'),
    phaseBeat(HELPER, 'APPROACH', 91), phaseBeat(HELPER, 'HOLD', 92, 'failed', 3, 'missed_dock')], done);
  assert.deepStrictEqual(Object.keys(marks(s).assignments).sort(), ['handled:2:3:50', 'handled:2:3:90']);

  // ---- Codex final diff, round 5 ----

  // A delayed APPLIED cannot bring back a slot whose dock ended; Codex's sequence: the
  // UAV's next dock keeps its own marker and word, and the finished dock keeps its own.
  s = run([phaseBeat(HELPER, 'APPROACH', 51), request(OWNER, HELPER, 5, 22), phaseBeat(HELPER, 'HOLD', 52, 'docked'),
    applied(OWNER, HELPER, 5, 40, 3, 96), asked(HELPER, 6, '3:70', 46, 44), onTask('task_confirmed', HELPER, 6, '3:70'),
    phaseBeat(HELPER, 'APPROACH', 71, 'docked'), phaseBeat(HELPER, 'HOLD', 72, 'failed', 3, 'missed_dock')], approved);
  assert.deepStrictEqual(Object.keys(marks(s).assignments).sort(), ['handled:2:3:50', 'handled:2:3:70']);
  assert.deepStrictEqual([handled(s).lat, handled(s).outcome], [45, O.DOCKED]);
  assert.deepStrictEqual([handled(s, HELPER, '3:70').lat, handled(s, HELPER, '3:70').outcome], [46, O.FAILED]);
  for (const [end, outcome] of [
    [phaseBeat(HELPER, 'HOLD', 52, 'docked'), O.DOCKED],
    [onTask('task_resolved', HELPER, 1, '3:50'), O.CANCELED],
    [onVehicle('disarm_after_guided', HELPER), O.UNKNOWN],
    [onVehicle('vehicle_reset', HELPER), O.UNKNOWN],
    [asked(HELPER, 6, '3:70', 46, 44), O.UNKNOWN],
  ]) {
    s = run([phaseBeat(HELPER, 'APPROACH', 51), request(OWNER, HELPER, 5, 22), end, applied(OWNER, HELPER, 5, 40, 3, 96)], approved);
    assert.ok(!('helper:2' in s.assignments), end.type);
    assert.ok(!('1:5' in s.assignments), end.type);
    assert.strictEqual(handled(s).outcome, outcome, end.type);
    assert.strictEqual(vehicleAssignment(s.assignments, HELPER), null, end.type);
  }

  // Only a copy of that offer goes with it: a new offer stays through the old one's end.
  for (const [offer, why] of [
    [[advert(OWNER, [5], 30), request(OWNER, HELPER, 5, 31)], 'made again'],
    [[request(OWNER, OTHER, 5, 30)], 'to another helper'],
    [[request(OWNER, HELPER, 5, 30)], 'to the same helper, its advert lost'],
    [[request(OWNER, HELPER, 5, 1, 8)], 'by a restarted owner'],
    [[elsewhere], 'at another position'],
    [[{ type: 'task_assign_request', data: { ...request(OWNER, HELPER, 5, 30).data, lat: 50 } }], 'at another latitude'],
    [[{ type: 'task_assign_request', data: { ...request(OWNER, HELPER, 5, 30).data, lon: 50 } }], 'at another longitude'],
  ]) {
    s = run([...offer, phaseBeat(HELPER, 'HOLD', 52)], approved);
    assert.strictEqual(s.assignments['1:5'].status, S.WAITING, why);
    assert.ok(handled(s), why);
  }
  // A copy from before the offer was made again is still a copy, in either order, to any helper.
  for (const helper of [HELPER, OTHER]) {
    for (const seq of [24, 27]) {
      s = run([advert(OWNER, [5], 30), phaseBeat(HELPER, 'HOLD', 52), request(OWNER, helper, 5, seq)], approved);
      assert.ok(!('1:5' in s.assignments), `after the end, to ${helper}, seq ${seq}`);
      s = run([advert(OWNER, [5], 30), request(OWNER, helper, 5, seq), phaseBeat(HELPER, 'HOLD', 52)], approved);
      assert.ok(!('1:5' in s.assignments), `before the end, to ${helper}, seq ${seq}`);
    }
  }
  // The release that counts is the first: a new offer does not move it.
  s = run([advert(OWNER, [5], 30), phaseBeat(HELPER, 'HOLD', 52), request(OWNER, OTHER, 5, 40), request(OWNER, HELPER, 5, 35)], approved);
  assert.deepStrictEqual(handled(s).reoffered, uid(7, 30));
  // The end of a UAV's own dock leaves the offer it made to another UAV alone, sent before
  // or after its round.
  for (const seq of [20, 61]) {
    s = run([asked(HELPER, 7, '3:60', 47, 44), onTask('task_confirmed', HELPER, 7, '3:60'), request(HELPER, OTHER, 7, seq, 3),
      phaseBeat(HELPER, 'HOLD', 62)]);
    assert.strictEqual(s.assignments['2:7'].status, S.WAITING, `seq ${seq}`);
  }

  // A round at another position is another dock: a slot does not take it, stale or not.
  for (const [lat, lon] of [[46, 44], [45, 50]]) {
    s = reduce(assigned, asked(HELPER, 6, '3:70', lat, lon));
    assert.strictEqual(s.assignments['helper:2'], assigned.assignments['helper:2'], `${lat},${lon}`);
    assert.deepStrictEqual([ownEntry(s, HELPER, '3:70').lat, ownEntry(s, HELPER, '3:70').lon], [lat, lon]);
  }
  // A round with no position is the slot's, as on a real build.
  for (const [lat, lon] of [[NaN, NaN], [null, null], [0, 0]]) {
    s = reduce(assigned, asked(HELPER, 1, '3:50', lat, lon));
    assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMING, `position ${lat},${lon}`);
    assert.strictEqual(ownEntry(s, HELPER, '3:50'), undefined, `position ${lat},${lon}`);
  }
  // The round the slot was asking about has been superseded: its answer no longer moves it.
  const elsewhereAsked = reduce(confirming, asked(HELPER, 6, '3:70', 46, 44));
  assert.strictEqual(elsewhereAsked.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(ownEntry(elsewhereAsked, HELPER, '3:70').status, S.CONFIRMING);
  assert.strictEqual(reduce(elsewhereAsked, asked(HELPER, 6, '3:70', 46, 44)), elsewhereAsked, 'a repeat');
  assert.strictEqual(reduce(elsewhereAsked, onTask('task_confirmed', HELPER, 1, '3:50')).assignments['helper:2'].status, S.ASSIGNED);
  // Each dock's word stays its own, and the slot goes at the UAV's FREE as any assigned slot does.
  s = run([onTask('task_confirmed', HELPER, 6, '3:70'), phaseBeat(HELPER, 'APPROACH', 71), phaseBeat(HELPER, 'HOLD', 72, 'docked')], elsewhereAsked);
  assert.deepStrictEqual([handled(s, HELPER, '3:70').lat, handled(s, HELPER, '3:70').outcome], [46, O.DOCKED]);
  assert.strictEqual(s.assignments['helper:2'].lat, 45);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.ok(!('helper:2' in reduce(s, beat(HELPER, 'FREE', 90)).assignments));
  // Back at the slot's dock, the slot takes the round and the other one is superseded.
  s = reduce(elsewhereAsked, asked(HELPER, 1, '3:80', 45, 44));
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMING);
  assert.strictEqual(ownEntry(s, HELPER, '3:70'), undefined);

  // A task assigned again after the owner released it is a new assignment, in either
  // order of the advert and the request: it needs a new approval.
  for (const order of [
    [advert(OWNER, [5], 100), request(OWNER, HELPER, 5, 101)],
    [request(OWNER, HELPER, 5, 101), advert(OWNER, [5], 100)],
  ]) {
    s = run([...order, doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 110)], approved);
    assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
    assert.strictEqual(s.assignments['helper:2'].confirm, undefined);
    assert.deepStrictEqual(s.assignments['helper:2'].assignedUid, uid(3, 80));
    assert.strictEqual(s.assignments['helper:2'].reoffered, undefined, 'it holds the new offer');
    assert.strictEqual(handled(s).outcome, O.UNKNOWN);
    assert.deepStrictEqual(Object.keys(marks(s).assignments), ['helper:2']);
  }
  s = run([advert(OWNER, [5], 100), request(OWNER, HELPER, 5, 101), doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 110)], confirming);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED, 'nobody approved it yet');
  assert.strictEqual(s.assignments['helper:2'].confirm, undefined);
  assert.strictEqual(handled(s), undefined);
  assert.strictEqual(reduce(s, onTask('task_confirmed', HELPER, 1, '3:50')), s, 'the old round answers nothing');
  r = run([onTask('task_confirming', HELPER, 2, '3:90'), onTask('task_confirmed', HELPER, 2, '3:90'),
    phaseBeat(HELPER, 'APPROACH', 91), phaseBeat(HELPER, 'HOLD', 92, 'docked')], s);
  assert.strictEqual(handled(r, HELPER, '3:90').outcome, O.DOCKED, 'the new round has its own word');
  s = run([advert(OWNER, [5], 100), request(OWNER, HELPER, 5, 101), doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 110)], assigned);
  assert.deepStrictEqual(s.assignments['helper:2'].assignedUid, uid(3, 80), 'a round before the new doing no longer counts');
  assert.strictEqual(reduce(s, onTask('task_confirming', HELPER, 1, '3:60')), s);

  // ---- One pad per spot (second live check: the older badge drew over the newer) ----
  const padsOf = (state) => {
    const { assignments, availableTasks } = marks(state);
    return Object.entries({ ...assignments, ...availableTasks })
      .filter(([, entry]) => !entry.hidePad).map(([key]) => key).sort();
  };
  const rowsOf = (state) => {
    const { assignments, availableTasks } = marks(state);
    return Object.fromEntries(Object.entries({ ...assignments, ...availableTasks })
      .map(([key, entry]) => [key, entry.labelRow ?? 0]));
  };
  // Alone at its spot, an entry is passed through as it is.
  assert.strictEqual(marks(approved).assignments['helper:2'], approved.assignments['helper:2']);
  assert.strictEqual(marks(offered).availableTasks['1:5'], offered.availableTasks['1:5']);
  // One UAV's two ended rounds at one dock (the task id changed): a word each, the newest's pad.
  s = run([asked(HELPER, 1, '3:60'), onTask('task_confirmed', HELPER, 1, '3:60'), onTask('task_resolved', HELPER, 1, '3:60'),
    asked(HELPER, 2, '3:70'), onTask('task_confirmed', HELPER, 2, '3:70'), phaseBeat(HELPER, 'APPROACH', 71),
    phaseBeat(HELPER, 'HOLD', 72, 'failed', 3, 'nav_fail')]);
  assert.strictEqual(handled(s, HELPER, '3:60').outcome, O.CANCELED);
  assert.strictEqual(handled(s, HELPER, '3:70').outcome, O.FAILED);
  assert.deepStrictEqual(rowsOf(s), { 'handled:2:3:60': 0, 'handled:2:3:70': 1 });
  assert.deepStrictEqual(padsOf(s), ['handled:2:3:70']);
  // Each spot has its own pad.
  r = run([asked(OTHER, 8, '5:61', 42, 46), onTask('task_confirmed', OTHER, 8, '5:61'), phaseBeat(OTHER, 'HOLD', 62, null, 5)], s);
  assert.deepStrictEqual(padsOf(r), ['handled:2:3:70', 'handled:3:5:61']);
  assert.deepStrictEqual(rowsOf(r), { 'handled:2:3:60': 0, 'handled:2:3:70': 1, 'handled:3:5:61': 0 }, 'rows count per spot');
  // Over any other entry, the round the operator must answer keeps its pad, then the furthest state.
  s = reduce(confirming, advert(OWNER, [5], 30));
  assert.deepStrictEqual(rowsOf(s), { 'helper:2': 0, '1:5': 1 });
  assert.deepStrictEqual(padsOf(s), ['helper:2']);
  s = reduce(assigned, advert(OWNER, [5], 30));
  assert.deepStrictEqual(rowsOf(s), { 'helper:2': 0, '1:5': 1 });
  assert.deepStrictEqual(padsOf(s), ['helper:2']);
  // A waiting offer beside the slot draws over neither an assigned slot nor an awaiting one.
  s = run([advert(OWNER, [5], 30), request(OWNER, OTHER, 5, 31)], assigned);
  assert.deepStrictEqual(padsOf(s), ['helper:2']);
  s = run([advert(OWNER, [5], 30), request(OWNER, OTHER, 5, 31)], confirming);
  assert.deepStrictEqual(padsOf(s), ['helper:2']);
  // Two rounds awaiting at one spot: the last.
  s = reduce(twoOnOffer, asked(OTHER, 1, '5:50', 45, 44));
  assert.strictEqual(s.assignments['helper:2'].status, S.CONFIRMING);
  assert.strictEqual(s.assignments['helper:3'].status, S.CONFIRMING);
  assert.deepStrictEqual(padsOf(s), ['helper:3']);
  // The earlier round awaiting, the later one not.
  assert.deepStrictEqual(padsOf(twoOnOffer), ['helper:2']);
  // Two adverts at one spot: the last.
  s = reduce(INITIAL, { type: 'available_task_request', data: {
    sender_id: OWNER, uid: uid(7, 10),
    tasks: [5, 6].map((task_id) => ({ task_id, task_type: 'DOCK', lat: 45, lon: 44, alt: 0 })),
  } });
  assert.deepStrictEqual(padsOf(s), ['1:6']);
  // A hidden history entry takes no row or pad from what is live at its spot.
  s = run([asked(HELPER, 7, '3:60'), onTask('task_confirmed', HELPER, 7, '3:60'),
    phaseBeat(HELPER, 'HOLD', 61), asked(HELPER, 7, '3:70')]);
  assert.deepStrictEqual(rowsOf(s), { 'own:2:3:70': 0 });
  assert.deepStrictEqual(padsOf(s), ['own:2:3:70']);
  // When the live entry ends, the history's pad draws again.
  r = reduce(s, onTask('task_resolved', HELPER, 7, '3:70'));
  assert.deepStrictEqual(padsOf(r), ['handled:2:3:60']);

  // ---- Codex final diff, round 6 ----

  // A delayed round cannot take the slot back and give its dock the outcome of the
  // approach to the dock asked about since. Codex's sequence.
  const askedElsewhere = run([asked(HELPER, 1, '3:50', 45, 44), asked(HELPER, 6, '3:70', 46, 44),
    onTask('task_confirmed', HELPER, 6, '3:70')], assigned);
  s = reduce(askedElsewhere, asked(HELPER, 1, '3:60', 45, 44));
  assert.strictEqual(s, askedElsewhere, 'older than a round the UAV asked since');
  assert.strictEqual(reduce(s, onTask('task_confirmed', HELPER, 1, '3:60')), s, 'and its answer moves nothing');
  s = run([phaseBeat(HELPER, 'APPROACH', 71), phaseBeat(HELPER, 'HOLD', 72, 'failed', 3, 'nav_fail')], s);
  assert.strictEqual(handled(s, HELPER, '3:70').outcome, O.FAILED);
  assert.strictEqual(handled(s, HELPER, '3:60'), undefined);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(s.assignments['helper:2'].lat, 45);
  // Nor can it end an earlier round, or take an empty slot, once it is known to be late.
  s = reduce(approved, asked(HELPER, 6, '3:45', 46, 44));
  assert.strictEqual(s, approved, 'older than the slot\'s own round');
  s = reduce(run([asked(HELPER, 6, '3:70', 46, 44)], assigned), onTask('task_confirming', HELPER, 1, '3:60'));
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED, 'a delayed round with no position');
  // Another boot's round is not ordered by this one's, and ends nothing of it: only a
  // beat from the new boot does, as before.
  s = run([asked(HELPER, 6, '3:70', 46, 44), asked(HELPER, 7, '4:10', 47, 44)]);
  assert.ok(ownEntry(s, HELPER, '4:10'));
  assert.ok(ownEntry(s, HELPER, '3:70'));
  assert.strictEqual(reduce(approved, asked(HELPER, 7, '4:10', 47, 44)).assignments['helper:2'].status, S.CONFIRMED);

  // A new request outlives the end of the dock before it, and its advert may come after
  // that end. Codex's sequence: the doing and the APPLIED still find it.
  s = run([request(OWNER, HELPER, 5, 101), phaseBeat(HELPER, 'APPROACH', 51), phaseBeat(HELPER, 'HOLD', 52, 'docked'),
    advert(OWNER, [5], 100), doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 110)], approved);
  assert.strictEqual(vehicleAssignment(s.assignments, HELPER).key, 'helper:2');
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(handled(s).outcome, O.DOCKED);
  assert.deepStrictEqual(Object.keys(marks(s).assignments), ['helper:2']);

  // An APPLIED from before a request answered an earlier assignment, so it does not use
  // that request up. Codex's sequence: the genuine doing and APPLIED still assign.
  const askedAgain = run([advert(OWNER, [5], 100), request(OWNER, HELPER, 5, 101)], approved);
  assert.strictEqual(reduce(askedAgain, applied(OWNER, HELPER, 5, 41, 3, 95)), askedAgain);
  s = run([doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 110)], askedAgain);
  assert.deepStrictEqual(s.assignments['helper:2'].assignedUid, uid(3, 80));
  assert.deepStrictEqual(s.assignments['helper:2'].appliedUid, uid(7, 110));
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED);
  assert.strictEqual(handled(s).outcome, O.UNKNOWN);
  assert.strictEqual(reduce(s, asked(HELPER, 2, '3:90', 45, 44)).assignments['helper:2'].status, S.CONFIRMING);
  // An older APPLIED of another task does not take the slot from a newer one.
  s = run([request(OWNER, HELPER, 6, 28), doing(HELPER, OWNER, 6, 80), applied(OWNER, HELPER, 6, 80, 3, 90), request(OWNER, HELPER, 5, 20)]);
  assert.strictEqual(reduce(s, applied(OWNER, HELPER, 5, 40, 3, 25)), s);
  // A slot's own late APPLIED does the same on a new request's round.
  r = run([request(OWNER, HELPER, 5, 30), doing(HELPER, OWNER, 5, 80)], assigned);
  assert.strictEqual(reduce(r, applied(OWNER, HELPER, 5, 40, 3, 28)), r);
  // Legacy meta reads as 0:0 (task_assign_listener.py), and an APPLIED from another owner
  // boot cannot be ordered against the request: neither is refused, as before this rule.
  s = run([request(OWNER, HELPER, 5, 0, 0), doing(HELPER, OWNER, 5, 40), { type: 'task_assign_ack', data: {
    ...applied(OWNER, HELPER, 5, 40).data, uid: uid(0, 0),
  } }]);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED, 'legacy');
  s = run([request(OWNER, HELPER, 5, 2, 8), doing(HELPER, OWNER, 5, 40), applied(OWNER, HELPER, 5, 40)]);
  assert.strictEqual(s.assignments['helper:2'].status, S.ASSIGNED, 'another boot');
  assert.strictEqual(reduce(s, request(OWNER, HELPER, 5, 3, 8)).assignments['1:5'].status, S.WAITING, 'and a later request is new');

  // A request older than the entry's APPLIED is a copy, to any helper, and a later one is
  // new and releases the offer as an advert does, so the new offer shows at once.
  assert.strictEqual(reduce(approved, request(OWNER, OTHER, 5, 24)), approved, 'a copy, to another helper');
  assert.strictEqual(reduce(done, request(OWNER, HELPER, 5, 24)), done, 'a copy, after the end');
  for (const [slot, helper] of [[assigned, HELPER], [confirming, HELPER], [approved, HELPER], [approved, OTHER]]) {
    s = reduce(slot, request(OWNER, helper, 5, 30));
    assert.deepStrictEqual(s.assignments['helper:2'].reoffered, uid(7, 30));
    assert.strictEqual(s.assignments['helper:2'].status, slot.assignments['helper:2'].status, 'until it is applied');
    assert.strictEqual(s.assignments['1:5'].status, S.WAITING);
    assert.deepStrictEqual(Object.keys(marks(s).assignments).sort(), ['1:5', 'helper:2']);
  }
  r = reduce(done, request(OWNER, HELPER, 5, 30));
  assert.deepStrictEqual(handled(r).reoffered, uid(7, 30), 'the same helper, after the end');
  assert.strictEqual(r.assignments['1:5'].status, S.WAITING);
  assert.deepStrictEqual(Object.keys(marks(r).assignments), ['1:5']);
  assert.strictEqual(reduce(approved, request(OTHER, HELPER, 5, 30)).assignments['helper:2'].reoffered, undefined, 'another owner');
  assert.strictEqual(reduce(approved, { type: 'task_assign_request', data: { ...request(OWNER, HELPER, 5, 30).data, lat: 50 } })
    .assignments['helper:2'].reoffered, undefined, 'another position');
  assert.strictEqual(reduce(approved, request(OWNER, HELPER, 6, 30)).assignments['helper:2'].reoffered, undefined, 'another task');
  assert.strictEqual(reduce(approved, sameSpotTask(OWNER, HELPER, 6, 30)).assignments['helper:2'].reoffered, undefined,
    'another task at the spot');
  assert.strictEqual(reduce(approved, request(OWNER, HELPER, 5, 1, 8)).assignments['helper:2'].reoffered, undefined, 'another owner boot');
  // A UAV's pending own round is not released by its own request, as by its own advert.
  s = run([asked(HELPER, 7, '3:60', 47, 44), request(HELPER, OTHER, 7, 61, 3)]);
  assert.strictEqual(ownEntry(s).reoffered, undefined);
  assert.strictEqual(s.assignments['2:7'].status, S.WAITING);
  // An approved own dock is, by a later request.
  s = run([asked(HELPER, 7, '3:60', 47, 44), onTask('task_confirmed', HELPER, 7, '3:60')]);
  assert.deepStrictEqual(ownEntry(reduce(s, request(HELPER, OTHER, 7, 61, 3))).reoffered, uid(3, 61));

  // A UAV's request to itself offers nothing (SelfAssignmentPublisher): it is outside the
  // owner's order, so it is neither a copy nor a release. It stays what it is on main.
  r = reduce(approved, request(OWNER, OWNER, 5, 30));
  assert.strictEqual(r.assignments['helper:2'].reoffered, undefined);
  assert.strictEqual(r.assignments['1:5'].receiverId, OWNER);
  assert.deepStrictEqual(Object.keys(marks(r).assignments), ['helper:2']);
  assert.deepStrictEqual(reduce(r, request(OWNER, HELPER, 5, 31)).assignments['helper:2'].reoffered, uid(7, 31));
  assert.strictEqual(reduce(approved, request(OWNER, OWNER, 5, 22)).assignments['1:5'].receiverId, OWNER);
  s = run([asked(HELPER, 7, '3:60', 47, 44), onTask('task_confirmed', HELPER, 7, '3:60'), request(HELPER, HELPER, 7, 61, 3)]);
  assert.strictEqual(ownEntry(s).reoffered, undefined, 'over its own approved dock');
  // One holder that makes it a copy is enough: a dock handled before this assignment does not.
  s = run([advert(OWNER, [5], 28), request(OWNER, HELPER, 5, 30), doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 35)], done);
  assert.strictEqual(reduce(s, request(OWNER, HELPER, 5, 33)), s);
  assert.strictEqual(reduce(s, request(OWNER, 4, 5, 33)), s, 'to a helper with no slot');
  // The same packet again, by another link: nothing moves.
  r = reduce(approved, request(OWNER, OTHER, 5, 30));
  assert.strictEqual(reduce(r, request(OWNER, OTHER, 5, 30)), r, 'a request');
  r = reduce(r, doing(OTHER, OWNER, 5, 41, true, 5));
  assert.strictEqual(reduce(r, doing(OTHER, OWNER, 5, 41, true, 5)), r, 'a doing');
  // A request its helper's slot refuses as a copy releases nothing either.
  s = run([elsewhere, doing(HELPER, OWNER, 5, 80), applied(OWNER, HELPER, 5, 80, 3, 100)], done);
  assert.strictEqual(reduce(s, request(OWNER, HELPER, 5, 60)), s);

  // An advert older than the entry's APPLIED came before a later reservation of this
  // same assignment: the entry still holds the offer.
  for (const [state, entry] of [[assigned, 'helper:2'], [done, 'handled:2:3:50']]) {
    s = reduce(state, advert(OWNER, [5], 22));
    assert.strictEqual(s.assignments[entry].reoffered, undefined, entry);
    assert.deepStrictEqual(marks(s).availableTasks, {}, entry);
  }

  // The state picks the pad, whatever order the entries are in. Codex's sequence: an
  // older detected ring of another owner at the dock must not hide the approved pad.
  const advertAt = (owner, task_id, seq) => ({ type: 'available_task_request', data: {
    sender_id: owner, uid: uid(7, seq), tasks: [{ task_id, task_type: 'DOCK', lat: 45, lon: 44, alt: 0 }],
  } });
  s = run([advertAt(OTHER, 9, 10), request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40), applied(OWNER, HELPER, 5, 40),
    onTask('task_confirming', HELPER, 1, '3:50'), onTask('task_confirmed', HELPER, 1, '3:50')]);
  assert.deepStrictEqual(padsOf(s), ['helper:2']);
  assert.deepStrictEqual(rowsOf(s), { 'helper:2': 0, '3:9': 1 });
  assert.strictEqual(marks(s).availableTasks['3:9'].hidePad, true);
  // Each step of the order, in both orders of the two entries where both can occur.
  assert.deepStrictEqual(padsOf(run([advertAt(OTHER, 9, 10), request(OWNER, HELPER, 5, 20)])), ['1:5'], 'offered over detected');
  assert.deepStrictEqual(padsOf(run([request(OWNER, HELPER, 5, 20), advertAt(OTHER, 9, 10)])), ['1:5'], 'in either order');
  const onTwo = (first) => run([advert(OWNER, [5], 30), request(OWNER, OTHER, 5, 31), doing(OTHER, OWNER, 5, 41, true, 5),
    applied(OWNER, OTHER, 5, 41, 5, 91)], first);
  const otherAsks = asked(OTHER, 1, '5:50', 45, 44);
  const otherApproved = [otherAsks, onTask('task_confirmed', OTHER, 1, '5:50')];
  assert.deepStrictEqual(padsOf(onTwo(approved)), ['helper:2'], 'approved over assigned, the approved first');
  assert.deepStrictEqual(padsOf(run(otherApproved, onTwo(assigned))), ['helper:3'], 'approved over assigned, the approved last');
  assert.deepStrictEqual(padsOf(reduce(onTwo(approved), otherAsks)), ['helper:3'], 'awaiting over approved, the awaiting last');
  assert.deepStrictEqual(padsOf(run(otherApproved, onTwo(confirming))), ['helper:2'], 'awaiting over approved, the awaiting first');
  assert.deepStrictEqual(padsOf(onTwo(assigned)), ['helper:3'], 'two assigned: the last');

  // ---- Codex final diff, round 7 ----

  // A UAV's own round is no step of an assignment it made before it. Codex's sequence: the
  // own round is heard first, and the helper's slot still comes, as in the other order.
  const peerHandshake = [request(HELPER, OTHER, 7, 20, 3), doing(OTHER, HELPER, 7, 41, true, 5), { type: 'task_assign_ack', data: {
    ...applied(HELPER, OTHER, 7, 41, 5).data, uid: uid(3, 25),
  } }];
  const ownApproved = [asked(HELPER, 7, '3:60', 47, 44), onTask('task_confirmed', HELPER, 7, '3:60')];
  s = run([...ownApproved, ...peerHandshake]);
  assert.strictEqual(vehicleAssignment(s.assignments, OTHER).key, 'helper:3');
  assert.strictEqual(ownEntry(s).status, S.CONFIRMED);
  assert.strictEqual(ownEntry(s).reoffered, undefined, 'it releases nothing');
  assert.deepStrictEqual(Object.keys(marks(s).assignments).sort(), ['helper:3', 'own:2:3:60']);
  assert.deepStrictEqual(Object.keys(marks(run([...peerHandshake, ...ownApproved])).assignments).sort(), ['helper:3', 'own:2:3:60'], 'the other order');
  // Over the dock's history too.
  s = run([...ownApproved, phaseBeat(HELPER, 'HOLD', 61), ...peerHandshake]);
  assert.strictEqual(vehicleAssignment(s.assignments, OTHER).key, 'helper:3');
  assert.strictEqual(handled(s, HELPER, '3:60').reoffered, undefined);
  // The slot still makes a delayed copy of its assignment a copy, with the own dock beside it.
  r = run([...ownApproved, ...peerHandshake]);
  assert.strictEqual(reduce(r, request(HELPER, OTHER, 7, 22, 3)), r);
  assert.strictEqual(reduce(r, request(HELPER, 4, 7, 22, 3)), r, 'to another helper');

  // A denied round leaves no entry, so a round older than it reads as new, as its card
  // does, and the card's answer ends it (Known limits).
  s = run([asked(HELPER, 6, '3:70', 46, 44), onTask('task_resolved', HELPER, 6, '3:70'), asked(HELPER, 7, '3:60', 47, 44)]);
  assert.strictEqual(ownEntry(s).status, S.CONFIRMING);
  assert.strictEqual(ownEntry(reduce(s, onTask('task_resolved', HELPER, 7, '3:60'))), undefined);

  // An APPLIED stamped after a RESET's advert can overtake it: the new offer shows at the
  // owner's next message (Known limits).
  s = run([request(OWNER, HELPER, 5, 20), doing(HELPER, OWNER, 5, 40), applied(OWNER, HELPER, 5, 40, 3, 35), advert(OWNER, [5], 30)]);
  assert.deepStrictEqual(marks(s).availableTasks, {});
  s = reduce(s, advert(OWNER, [5], 36));
  assert.deepStrictEqual(s.assignments['helper:2'].reoffered, uid(7, 36));
  assert.deepStrictEqual(Object.keys(marks(s).availableTasks), ['1:5']);

  console.log('PASS — taskAssignmentState reducer tests passed.');
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
