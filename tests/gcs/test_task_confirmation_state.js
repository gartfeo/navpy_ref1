/**
 * Tests for the taskConfirmationState reducer.
 * Dynamically imports the REAL ESM module (frontend is type:module) so the
 * actual exports are exercised, not a copy.
 *
 * Run via: node tests/gcs/test_task_confirmation_state.js
 */
const assert = require('assert');
const path = require('path');
const url = require('url');

const SRC = path.join(
  __dirname, '..', '..', 'src', 'gcs', 'frontend', 'src', 'hooks',
  'taskConfirmationState.js',
);

(async () => {
  const m = await import(url.pathToFileURL(SRC).href);
  const {
    CONFIRM_STATUS, confirmRequest, confirmImage, confirmResponse,
    disarmAfterGuided, confirmReset, removeCard, resetAll, expireDecided,
    isPending, isDecided, canCancel, hasPendingConfirm, getConfirm,
    DEFAULT_CONFIRM_WAIT_SEC, CONFIRM_PHASE, confirmWaitSec, confirmDeadlineMs, msLeft, secondsLeft,
    confirmRoundId, roundIdFromKey, confirmRoundKey, orderedConfirmEntries, pendingSummary,
    formatClock, confirmZeroLabel,
  } = m;

  for (const name of [
    'confirmRequest', 'confirmImage', 'confirmResponse',
    'disarmAfterGuided', 'confirmReset', 'removeCard', 'resetAll', 'expireDecided',
    'isPending', 'isDecided', 'canCancel', 'hasPendingConfirm', 'getConfirm',
    'confirmWaitSec', 'confirmDeadlineMs', 'msLeft', 'secondsLeft', 'confirmRoundId',
    'roundIdFromKey', 'orderedConfirmEntries', 'pendingSummary', 'formatClock', 'confirmZeroLabel',
  ]) {
    assert.strictEqual(typeof m[name], 'function', `missing export: ${name}`);
  }
  assert.strictEqual(m.vehicleAbort, undefined, 'vehicleAbort must be removed (abort is the E-STOP command)');

  const req = (sys_id, task_id, t = 1000) => ({
    sys_id, task_id, task_type: 'HEAVY', lat: 1, lon: 2, alt: 3, receivedAt: t,
  });

  // confirmRequest -> pending card
  const s = confirmRequest({}, req(1, 10));
  assert.strictEqual(s[1].status, CONFIRM_STATUS.PENDING);
  assert.strictEqual(s[1].taskId, 10);
  assert.strictEqual(s[1].action, null);
  assert.ok(hasPendingConfirm(s, 1));
  assert.ok(isPending(s[1]) && !isDecided(s[1]) && !canCancel(s[1]));

  // new request supersedes prior card
  const s2 = confirmRequest(s, req(1, 11, 2000));
  assert.strictEqual(s2[1].taskId, 11);

  // round uid is carried so the decision can be bound to the round the
  // operator saw, and so a re-ask of the SAME task is a distinguishable round
  const roundA = confirmRequest({}, { ...req(1, 10), round_uid: '424242:11' });
  assert.strictEqual(roundA[1].roundUid, '424242:11');
  const roundB = confirmRequest(roundA, { ...req(1, 10, 2000), round_uid: '424242:12' });
  assert.strictEqual(roundB[1].roundUid, '424242:12', 're-ask must carry its own round uid');
  assert.strictEqual(roundB[1].taskId, 10, 'a re-ask keeps the same task id');
  // absent uid (legacy backend) degrades to null rather than undefined
  assert.strictEqual(s[1].roundUid, null);

  // image task-id guard
  assert.strictEqual(confirmImage(s, { sys_id: 1, task_id: 99, image_b64: 'x' }), s);
  const si = confirmImage(s, { sys_id: 1, task_id: 10, image_b64: 'img' });
  assert.strictEqual(si[1].imageB64, 'img');

  // an image from a round that is over must not land on the current round's
  // card (same task id, different round)
  assert.strictEqual(
    confirmImage(roundB, { sys_id: 1, task_id: 10, image_b64: 'stale', round_uid: '424242:11' }),
    roundB,
    'an image from round A must not attach to round B',
  );
  const sib = confirmImage(roundB, {
    sys_id: 1, task_id: 10, image_b64: 'fresh', round_uid: '424242:12',
  });
  assert.strictEqual(sib[1].imageB64, 'fresh');
  // no uid on either side -> unchanged legacy behaviour
  assert.strictEqual(
    confirmImage(s, { sys_id: 1, task_id: 10, image_b64: 'img2' })[1].imageB64, 'img2',
  );

  // a decision for round A must not decide the live round B: the operator
  // answered a round that is already over (the response can complete after
  // the companion has moved on)
  const bAfterStaleResponse = confirmResponse(roundB, {
    sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve',
    round_uid: '424242:11', at: 3000,
  });
  assert.strictEqual(
    bAfterStaleResponse, roundB,
    "round A's response must leave round B pending",
  );
  assert.strictEqual(bAfterStaleResponse[1].status, CONFIRM_STATUS.PENDING);
  // ...while B's own response still decides B
  const bDecided = confirmResponse(roundB, {
    sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve',
    round_uid: '424242:12', at: 3000,
  });
  assert.strictEqual(bDecided[1].status, CONFIRM_STATUS.APPROVED);

  // approve: pending -> approved (cancelable), and NOT pending anymore
  const sa = confirmResponse(s, { sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve', at: 1100 });
  assert.strictEqual(sa[1].status, CONFIRM_STATUS.APPROVED);
  assert.strictEqual(sa[1].action, 'approve');
  assert.ok(canCancel(sa[1]) && isDecided(sa[1]) && !isPending(sa[1]));
  assert.ok(!hasPendingConfirm(sa, 1), 'decided card must not count as pending (mission status)');

  // deny: pending -> denied
  const sd = confirmResponse(s, { sys_id: 1, task_id: 10, is_confirmed: false, action: 'deny', at: 1100 });
  assert.strictEqual(sd[1].status, CONFIRM_STATUS.DENIED);

  // cancel of approved -> canceled
  const sc = confirmResponse(sa, { sys_id: 1, task_id: 10, is_confirmed: false, action: 'cancel', at: 1200 });
  assert.strictEqual(sc[1].status, CONFIRM_STATUS.CANCELED);
  assert.strictEqual(sc[1].action, 'cancel');

  // monotonic: denied + late approve -> unchanged (same ref)
  assert.strictEqual(
    confirmResponse(sd, { sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve', at: 1300 }),
    sd, 'denied must not be un-decided to approved',
  );
  // monotonic: canceled + approve -> unchanged
  assert.strictEqual(
    confirmResponse(sc, { sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve', at: 1300 }),
    sc,
  );

  // approved + any false action (deny) -> canceled
  const sad = confirmResponse(sa, { sys_id: 1, task_id: 10, is_confirmed: false, action: 'deny', at: 1250 });
  assert.strictEqual(sad[1].status, CONFIRM_STATUS.CANCELED);

  // confirmResponse task-id guard
  assert.strictEqual(
    confirmResponse(s, { sys_id: 1, task_id: 999, is_confirmed: true, action: 'approve', at: 1 }), s,
  );

  // fallback when action missing
  const sf = confirmResponse(s, { sys_id: 1, task_id: 10, is_confirmed: true, at: 1 });
  assert.strictEqual(sf[1].status, CONFIRM_STATUS.APPROVED);
  assert.strictEqual(sf[1].action, 'approve');

  // disarmAfterGuided removes the card
  assert.deepStrictEqual(disarmAfterGuided(sa, { sys_id: 1 }), {});

  // confirmReset removes listed ids; empty/missing -> no-op (same ref)
  const multi = confirmRequest(confirmRequest({}, req(1, 10)), req(2, 20, 1500));
  const sr = confirmReset(multi, { sys_ids: [1] });
  assert.ok(!(1 in sr) && (2 in sr));
  assert.strictEqual(confirmReset(multi, { sys_ids: [] }), multi);
  assert.strictEqual(confirmReset(multi, {}), multi);

  assert.deepStrictEqual(resetAll(), {});
  assert.strictEqual(removeCard(s, 99), s); // absent -> same ref

  // expireDecided: denied/canceled past ttl removed; approved + pending kept
  let mixed = confirmRequest({}, req(1, 10, 0));
  mixed = confirmResponse(mixed, { sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve', at: 0 }); // approved
  const pend = confirmRequest({}, req(2, 20, 0)); // pending
  const den = confirmResponse(confirmRequest({}, req(3, 30, 0)),
    { sys_id: 3, task_id: 30, is_confirmed: false, action: 'deny', at: 0 }); // denied
  const combined = { ...mixed, ...pend, ...den };
  const exp = expireDecided(combined, 9000, 8000);
  assert.ok(1 in exp, 'approved persists');
  assert.ok(2 in exp, 'pending kept');
  assert.ok(!(3 in exp), 'denied expired past ttl');
  assert.ok(3 in expireDecided(combined, 5000, 8000), 'denied within ttl kept');
  assert.strictEqual(expireDecided(mixed, 100000, 8000), mixed, 'approved-only -> same ref');

  assert.strictEqual(getConfirm(multi, 2).taskId, 20);
  assert.strictEqual(getConfirm(multi, 99), null);

  // ---- Todo 67/68: deadline helpers ----
  assert.strictEqual(DEFAULT_CONFIRM_WAIT_SEC, 30);
  assert.deepStrictEqual(confirmWaitSec(10), { sec: 10, estimated: false });
  assert.deepStrictEqual(confirmWaitSec(0), { sec: 0, estimated: false }, '0 = no wait, known');
  for (const bad of [undefined, null, NaN, -5, '10', Infinity]) {
    assert.deepStrictEqual(confirmWaitSec(bad), { sec: 30, estimated: true }, `wait ${bad}`);
  }
  assert.strictEqual(confirmDeadlineMs({ receivedAt: 1000 }, 10), 11000);
  assert.strictEqual(confirmDeadlineMs({ receivedAt: 1000 }, undefined), 31000);
  assert.strictEqual(msLeft(5000, 7000), 0, 'never negative');
  assert.strictEqual(secondsLeft(11000, 1001), 10, 'rounded up');
  assert.strictEqual(secondsLeft(11000, 1000), 10);
  assert.strictEqual(secondsLeft(11000, 10999), 1);
  assert.strictEqual(secondsLeft(11000, 11000), 0);
  assert.strictEqual(formatClock(30), '0:30');
  assert.strictEqual(formatClock(5), '0:05');
  assert.strictEqual(formatClock(125), '2:05');
  assert.strictEqual(formatClock(-3), '0:00');

  // round identity carries the UAV: the same round key on two UAVs differs
  const ra = { taskId: 10, roundUid: 'u1' };
  assert.strictEqual(confirmRoundId(1, ra), roundIdFromKey(1, confirmRoundKey(ra)));
  assert.notStrictEqual(confirmRoundId(1, ra), confirmRoundId(2, ra));

  // ---- Todo 67: order by time left, not by arrival ----
  // UAV 1 (NAV_CWT 30) arrives at t=0; UAV 3 (NAV_CWT 10) 5 s later. UAV 3's
  // deadline (15 s) is before UAV 1's (30 s), so it sorts first.
  const cwt = { 1: 30, 2: 30, 3: 10, 4: 30, 5: 30, 6: 30, 10: 30 };
  const waitFor = (sid) => cwt[sid];
  let fleet = confirmRequest({}, req(1, 10, 0));
  fleet = confirmRequest(fleet, req(3, 30, 5000));
  assert.deepStrictEqual(orderedConfirmEntries(fleet, waitFor).map(([sid]) => sid), [3, 1]);
  assert.strictEqual(typeof orderedConfirmEntries(fleet, waitFor)[0][0], 'number');
  // then approved, then denied or canceled; ties by numeric sys_id
  fleet = confirmResponse(confirmRequest(fleet, req(5, 50, 0)),
    { sys_id: 5, task_id: 50, is_confirmed: true, action: 'approve', at: 1 });
  fleet = confirmResponse(confirmRequest(fleet, req(4, 40, 0)),
    { sys_id: 4, task_id: 40, is_confirmed: true, action: 'approve', at: 1 });
  fleet = confirmResponse(confirmRequest(fleet, req(2, 20, 0)),
    { sys_id: 2, task_id: 20, is_confirmed: false, action: 'deny', at: 1 });
  fleet = confirmRequest(fleet, req(10, 100, 0)); // same deadline as UAV 1: numeric, not string, tie-break
  assert.deepStrictEqual(orderedConfirmEntries(fleet, waitFor).map(([sid]) => sid), [3, 1, 10, 4, 5, 2]);
  // an unknown wait orders by the 30 s default
  assert.deepStrictEqual(
    orderedConfirmEntries(confirmRequest(confirmRequest({}, req(1, 10, 0)), req(3, 30, 5000)), () => undefined)
      .map(([sid]) => sid), [1, 3],
  );
  // fired rounds come first, by fire time
  const fleetPending = confirmRequest(confirmRequest(confirmRequest({}, req(1, 10, 0)), req(3, 30, 5000)), req(6, 60, 0));
  const firedTwo = {
    [confirmRoundId(6, fleetPending[6])]: 40000,
    [confirmRoundId(1, fleetPending[1])]: 31000,
  };
  assert.deepStrictEqual(orderedConfirmEntries(fleetPending, waitFor, firedTwo).map(([sid]) => sid), [1, 6, 3]);
  assert.deepStrictEqual(orderedConfirmEntries({}, waitFor), []);
  assert.deepStrictEqual(orderedConfirmEntries(null, waitFor), []);

  // ---- Todo 67: the summary ----
  const two = confirmRequest(confirmRequest({}, req(1, 10, 0)), req(3, 30, 5000));
  assert.strictEqual(pendingSummary({}, waitFor, 0), null);
  assert.strictEqual(pendingSummary({ 4: fleet[4] }, waitFor, 0), null, 'decided only -> null');
  let sum = pendingSummary(two, waitFor, 6000);
  assert.strictEqual(sum.count, 2);
  assert.deepStrictEqual(sum.sysIds, [3, 1]);
  assert.strictEqual(sum.shownSysId, 3);
  assert.strictEqual(sum.shownEntry, two[3]);
  assert.strictEqual(sum.phase, CONFIRM_PHASE.LIVE);
  assert.strictEqual(sum.deadlineMs, 15000);
  assert.strictEqual(sum.secondsLeft, 9);
  assert.strictEqual(sum.liveCount, 2);
  // UAV 3 at zero, not fired yet: the soonest LIVE one (UAV 1) is shown
  sum = pendingSummary(two, waitFor, 15000);
  assert.strictEqual(sum.shownSysId, 1);
  assert.strictEqual(sum.secondsLeft, 15);
  assert.deepStrictEqual(sum.sysIds, [1, 3]);
  assert.strictEqual(sum.liveCount, 1);
  // all at zero: the earliest DUE one, 0 seconds
  sum = pendingSummary(two, waitFor, 31000);
  assert.strictEqual(sum.phase, CONFIRM_PHASE.DUE);
  assert.strictEqual(sum.shownSysId, 3);
  assert.strictEqual(sum.secondsLeft, 0);
  assert.strictEqual(sum.liveCount, 0);
  // one fired, one due: the due one is shown; all fired: the first fired one
  const f3 = { [confirmRoundId(3, two[3])]: 15000 };
  assert.strictEqual(pendingSummary(two, waitFor, 31000, f3).shownSysId, 1);
  assert.strictEqual(pendingSummary(two, waitFor, 31000, f3).phase, CONFIRM_PHASE.DUE);
  const fAll = { ...f3, [confirmRoundId(1, two[1])]: 30000 };
  sum = pendingSummary(two, waitFor, 31000, fAll);
  assert.strictEqual(sum.phase, CONFIRM_PHASE.FIRED);
  assert.strictEqual(sum.shownSysId, 3);
  // a fired latch for a round that is over (new round uid) does not count
  const reasked = confirmRequest(two, { ...req(3, 30, 19000), round_uid: 'new' }); // deadline 29 s
  assert.strictEqual(pendingSummary(reasked, waitFor, 21000, f3).shownSysId, 3);
  assert.strictEqual(pendingSummary(reasked, waitFor, 21000, f3).phase, CONFIRM_PHASE.LIVE);

  // ---- Todo 68: what the GCS sends at zero ----
  const live = (cmFl, cwtV, seconds = 21) => confirmZeroLabel({ phase: CONFIRM_PHASE.LIVE, cmFl, cwt: cwtV, seconds });
  assert.deepStrictEqual(live(false, 30), { cardKey: 'task.zero.autoDeny', chipKey: 'task.zeroChip.autoDeny', time: '0:21' });
  assert.deepStrictEqual(live(0, 30), live(false, 30), 'MAVLink 0 = off');
  assert.deepStrictEqual(live(true, 30), { cardKey: 'task.zero.autoApprove', chipKey: 'task.zeroChip.autoApprove', time: '0:21' });
  assert.deepStrictEqual(live(1, 30), live(true, 30), 'MAVLink 1 = on');
  assert.deepStrictEqual(live(undefined, 30), { cardKey: 'task.zero.autoDenyUnknown', chipKey: 'task.zeroChip.autoDenyUnknown', time: '0:21' });
  assert.strictEqual(live(false, undefined).time, '~0:21', 'wait not loaded -> estimated');
  assert.strictEqual(live(false, -1).time, '~0:21', 'invalid wait -> estimated');
  assert.deepStrictEqual(live(false, 0, 0), { cardKey: 'task.zero.noWaitDeny', chipKey: 'task.zero.noWaitDeny', time: '0:00' });
  assert.deepStrictEqual(live(true, 0, 0), { cardKey: 'task.zero.noWaitApprove', chipKey: 'task.zero.noWaitApprove', time: '0:00' });
  assert.strictEqual(live(undefined, 0, 0).cardKey, 'task.zero.noWaitDeny');
  // first match wins: fired, then a zero wait (due from the start), then due
  for (const cmFl of [true, false, undefined]) {
    for (const c of [0, 30, undefined]) {
      assert.strictEqual(confirmZeroLabel({ phase: CONFIRM_PHASE.FIRED, cmFl, cwt: c, seconds: 0 }).cardKey, 'task.zero.fired');
      assert.strictEqual(confirmZeroLabel({ phase: CONFIRM_PHASE.DUE, cmFl, cwt: c, seconds: 0 }).chipKey,
        c === 0 ? (cmFl ? 'task.zero.noWaitApprove' : 'task.zero.noWaitDeny') : 'task.zero.due');
    }
  }

  console.log('All taskConfirmationState reducer tests passed.');
})().catch((e) => { console.error(e); process.exit(1); });
