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
    visibleConfirmEntries,
  } = m;

  for (const name of [
    'confirmRequest', 'confirmImage', 'confirmResponse',
    'disarmAfterGuided', 'confirmReset', 'removeCard', 'resetAll', 'expireDecided',
    'isPending', 'isDecided', 'canCancel', 'hasPendingConfirm', 'getConfirm',
    'visibleConfirmEntries',
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

  // visibleConfirmEntries sorted by receivedAt, numeric sysId
  const v = visibleConfirmEntries(multi);
  assert.strictEqual(v[0][0], 1);
  assert.strictEqual(v[1][0], 2);
  assert.strictEqual(typeof v[0][0], 'number');
  assert.strictEqual(getConfirm(multi, 2).taskId, 20);
  assert.strictEqual(getConfirm(multi, 99), null);

  console.log('All taskConfirmationState reducer tests passed.');
})().catch((e) => { console.error(e); process.exit(1); });
