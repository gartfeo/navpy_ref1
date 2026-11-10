/**
 * Pure reducer + selectors for the task-assignment handshake shown in the GCS
 * (docs/design/swarm-task-assignment-ack.md).
 *
 * An owner offers a task with step 3 (`task_assign_request`); the helper is
 * WAITING until the owner applies its "doing" (`task_assign_ack`), which makes
 * it ASSIGNED. Owners and helpers repeat their messages with a fresh
 * uid {boot_id, msg_seq} per copy, so every transition is idempotent.
 *
 * Keys: a pending round is `${owner}:${task}`; once assigned it moves to the
 * helper's slot `helper:${id}` (one per helper), so a recycled task id never
 * collides with a task being flown. Ranks waiting < assigned < confirming <
 * confirmed are never lowered within a round, except that a deny returns a
 * confirming slot to assigned (`taskResolved`). Available tasks are keyed
 * `${owner}:${task}` too: task ids are per owner.
 *
 * Confirm events carry the confirming UAV's own POI id: each companion numbers
 * its POIs itself, so a helper confirms the owner's task 2 as its POI 1. They
 * are matched by UAV, to that UAV's helper slot, which records the round (POI
 * id, round uid) the GCS heard the UAV ask while it held the task
 * (CONFIRMING). The round must come from the helper's boot after the "doing"
 * it was assigned by; only an answer naming that round moves the slot.
 * Anything else (an own-POI round asked before the assignment, a restarted
 * helper's round, a later round at its home dock, a stale page's decision, a
 * round without a uid) does not. A round the GCS heard before it heard the
 * owner's APPLIED is never counted either: it may be an own-POI round.
 *
 * Two more entry kinds exist for the map (docs/design/gcs_dock_marker_states.md),
 * both with a null receiver, so no vehicle card or per-vehicle drop reads them:
 * `own:${uav}:${round}`, a round with a position that no helper slot took, and
 * `handled:${uav}:${round}`, an approved dock whose approach ended. A handled
 * entry names an outcome only with direct evidence from that approach: NavPy's
 * latch keeps the previous outcome through the next approach, so the outcome
 * must change after an APPROACH beat of this round.
 *
 * Every function returns the same state reference when nothing changes.
 */

export const ASSIGNMENT_STATUS = Object.freeze({
  WAITING: 'waiting',
  ASSIGNED: 'assigned',
  CONFIRMING: 'confirming',
  CONFIRMED: 'confirmed',
  HANDLED: 'handled',
});

export const DOCK_OUTCOME = Object.freeze({
  DOCKED: 'docked',
  FAILED: 'failed',
  CANCELED: 'canceled',
  UNKNOWN: 'unknown',
});

const RANK = Object.freeze({ waiting: 0, assigned: 1, confirming: 2, confirmed: 3 });

// NavPy phases during which an approved dock's approach has not ended.
const OPEN_ROUND_PHASES = new Set(['CONFIRM', 'TO_TASK']);
const OUTCOME_WORDS = Object.freeze({ docked: DOCK_OUTCOME.DOCKED, failed: DOCK_OUTCOME.FAILED });

export const INITIAL_ASSIGNMENT_STATE = Object.freeze({ assignments: {}, availableTasks: {} });

export function roundKey(ownerId, taskId) {
  return `${ownerId}:${taskId}`;
}

export function helperKey(helperId) {
  return `helper:${helperId}`;
}

function isRound(entry) {
  return entry.key === roundKey(entry.senderId, entry.taskId);
}

function higher(status, other) {
  return RANK[other] > RANK[status] ? other : status;
}

// One sender's uids are ordered only within a boot; a new boot replaces.
function isNewer(uid, than) {
  if (uid == null) return false;
  if (than == null) return true;
  return uid.boot_id !== than.boot_id || uid.msg_seq > than.msg_seq;
}

// True only when the same boot sent `uid` after `floor`.
function follows(uid, floor) {
  return uid != null && floor != null
    && uid.boot_id === floor.boot_id && uid.msg_seq > floor.msg_seq;
}

/** The UAV an entry belongs to, for every entry kind. */
export function uavOf(entry) {
  return entry.receiverId ?? entry.askedBy ?? entry.handledBy;
}

function isPendingOwn(entry) {
  return entry.askedBy != null && entry.status === ASSIGNMENT_STATUS.CONFIRMING;
}

function isApproved(entry) {
  return entry.status === ASSIGNMENT_STATUS.CONFIRMED;
}

function isHandled(entry) {
  return entry.status === ASSIGNMENT_STATUS.HANDLED;
}

// A dock its UAV asked about itself: no owner assigned it.
function isOwnDock(entry) {
  return entry.senderId === uavOf(entry);
}

function askedRound(entry) {
  return parseRoundUid(entry.confirm?.roundUid);
}

function ownKey(sysId, roundUid) {
  return `own:${sysId}:${roundUid}`;
}

function handledKey(sysId, roundUid) {
  return `handled:${sysId}:${roundUid}`;
}

// Whether the approach of this entry's round was heard and its outcome may
// still be tied to it.
function heardApproach(entry) {
  return entry.approachOutcome !== undefined && !entry.noWord;
}

function withAssignment(state, entry) {
  return { ...state, assignments: { ...state.assignments, [entry.key]: entry } };
}

function dropAssignments(state, matches) {
  const keys = Object.keys(state.assignments).filter((key) => matches(state.assignments[key]));
  if (keys.length === 0) return state;
  const assignments = { ...state.assignments };
  for (const key of keys) delete assignments[key];
  return { ...state, assignments };
}

function dropAvailable(state, keys) {
  const gone = keys.filter((key) => key in state.availableTasks);
  if (gone.length === 0) return state;
  const availableTasks = { ...state.availableTasks };
  for (const key of gone) delete availableTasks[key];
  return { ...state, availableTasks };
}

// A round uid as the backend publishes it, `${boot}:${seq}`
// (gcs/backend/task_confirm_uid.py); null for "legacy" or none.
function parseRoundUid(roundUid) {
  const match = /^(\d+):(\d+)$/.exec(roundUid ?? '');
  return match ? { boot_id: Number(match[1]), msg_seq: Number(match[2]) } : null;
}

function answersRound(asked, data) {
  return !!asked && asked.taskId === data.task_id && asked.roundUid === data.round_uid;
}

// The helper slot a confirm answer applies to, if any.
function confirmingSlot(state, data) {
  const slot = state.assignments[helperKey(data.sys_id)];
  return slot && RANK[slot.status] >= RANK.confirming
    && answersRound(slot.confirm, data) ? slot : null;
}

function raise(state, slot, status) {
  if (!slot || higher(slot.status, status) === slot.status) return state;
  return withAssignment(state, { ...slot, status });
}

function sameAvailable(current, entry) {
  return !!current && current.taskType === entry.taskType && current.ownerBoot === entry.ownerBoot
    && current.lat === entry.lat && current.lon === entry.lon && current.alt === entry.alt;
}

// An approved entry leaves for the map's history; its UAV is free of it.
function retire(state, entry, outcome, awaitingOutcome = false) {
  const { askedBy, ...kept } = entry;
  const uav = uavOf(entry);
  const key = handledKey(uav, entry.confirm.roundUid);
  const assignments = { ...state.assignments };
  delete assignments[entry.key];
  assignments[key] = {
    ...kept, key, status: ASSIGNMENT_STATUS.HANDLED, receiverId: null, handledBy: uav,
    outcome, awaitingOutcome,
  };
  return { ...state, assignments };
}

// `reoffered` holds the uid of the message that released the entry from its offer.
function markReoffered(state, entries, uid) {
  return entries.reduce((next, entry) => withAssignment(next, { ...entry, reoffered: uid }), state);
}

// The owner's last message that this entry's offer stands on. The owner stamps a
// task's adverts and requests under one lock, adverts only while it is free and
// request copies only until it applies an answer, before that APPLIED. So a request
// or an advert no newer than this is a delayed copy, and a later one is a new offer.
function offerFloor(entry) {
  return entry.reoffered ?? entry.appliedUid ?? entry.requestUid;
}

function outcomeCode(swarm) {
  return swarm.outcome == null ? null : `${swarm.outcome}:${swarm.fail_reason ?? ''}`;
}

// The outcome word for this entry, or null without direct evidence.
function outcomeWord(entry, swarm) {
  if (!heardApproach(entry) || outcomeCode(swarm) === entry.approachOutcome) return null;
  return OUTCOME_WORDS[swarm.outcome] ?? null;
}

// A new round of a UAV ends what it asked before, so a later approach cannot
// label an earlier dock.
function endEarlierRounds(state, sysId, uid) {
  let next = state;
  for (const entry of Object.values(state.assignments)) {
    if (uavOf(entry) !== sysId || !follows(uid, askedRound(entry))) continue;
    if (isApproved(entry)) next = retire(next, entry, DOCK_OUTCOME.UNKNOWN);
    else if (isPendingOwn(entry)) next = dropAssignments(next, (other) => other === entry);
    else if (isHandled(entry) && entry.awaitingOutcome) {
      next = withAssignment(next, { ...entry, awaitingOutcome: false });
    }
  }
  return next;
}

function sameSpot(entry, data) {
  return entry.lat === data.lat && entry.lon === data.lon;
}

function hasPosition(data) {
  return Number.isFinite(data.lat) && Number.isFinite(data.lon)
    && !(data.lat === 0 && data.lon === 0);
}

function withOwnRound(state, data, uid) {
  if (!uid || !hasPosition(data)) return state;
  return withAssignment(state, {
    key: ownKey(data.sys_id, data.round_uid),
    taskId: data.task_id,
    taskType: data.task_type,
    lat: data.lat,
    lon: data.lon,
    alt: data.alt,
    senderId: data.sys_id,
    receiverId: null,
    askedBy: data.sys_id,
    status: ASSIGNMENT_STATUS.CONFIRMING,
    requestUid: uid,
    confirm: { taskId: data.task_id, roundUid: data.round_uid },
  });
}

// Whether a UAV's round is one already heard, or older than a round it asked since.
function roundHeard(state, sysId, uid) {
  return Object.values(state.assignments).some((entry) => {
    const asked = askedRound(entry);
    return uavOf(entry) === sysId && asked?.boot_id === uid.boot_id && !follows(uid, asked);
  });
}

function ownRound(state, data) {
  const entry = state.assignments[ownKey(data.sys_id, data.round_uid)];
  return entry && answersRound(entry.confirm, data) ? entry : null;
}

// One phase beat for one entry of its UAV (the rules in gcs_dock_marker_states.md).
function readBeat(state, entry, beat, swarm) {
  const round = askedRound(entry);
  if (!round) return state;
  const sameBoot = round.boot_id === beat.boot_id;
  if (isHandled(entry)) {
    if (!entry.awaitingOutcome) return state;
    if (!sameBoot) return withAssignment(state, { ...entry, awaitingOutcome: false });
    if (!follows(beat, round)) return state;
    if (swarm.phase === 'APPROACH' && outcomeCode(swarm) !== entry.approachOutcome) {
      return withAssignment(state, { ...entry, awaitingOutcome: false });
    }
    if (swarm.phase === 'APPROACH') return state;
    // NavPy sets the outcome and leaves the phase on one thread, so the first
    // beat after the approach decides; anything later may be another approach's.
    return withAssignment(state, {
      ...entry, outcome: outcomeWord(entry, swarm) ?? entry.outcome, awaitingOutcome: false,
    });
  }
  if (isPendingOwn(entry)) {
    if (!sameBoot) return dropAssignments(state, (other) => other === entry);
    // After a socket gap its answer may have been missed; once the UAV has left
    // CONFIRM nothing ties it to a round.
    const left = entry.noWord && follows(beat, round) && swarm.phase !== 'CONFIRM';
    return left ? dropAssignments(state, (other) => other === entry) : state;
  }
  if (!isApproved(entry)) return state;
  if (!sameBoot) return retire(state, entry, DOCK_OUTCOME.UNKNOWN);
  if (!follows(beat, round)) return state;
  if (swarm.phase === 'APPROACH') {
    if (entry.approachOutcome === undefined) {
      return withAssignment(state, { ...entry, approachOutcome: outcomeCode(swarm) });
    }
    // The outcome only changes as an approach ends, so this beat is a later approach's.
    return outcomeCode(swarm) === entry.approachOutcome ? state : retire(state, entry, DOCK_OUTCOME.UNKNOWN);
  }
  // Before the approach these phases belong to this round; after it they are the next one's.
  if (OPEN_ROUND_PHASES.has(swarm.phase) && entry.approachOutcome === undefined) return state;
  const word = outcomeWord(entry, swarm);
  return retire(state, entry, word ?? DOCK_OUTCOME.UNKNOWN);
}

// Disarm or E-STOP: an approved dock goes to history, the rest of the UAV's goes.
function endVehicle(state, sysId) {
  let next = state;
  for (const entry of Object.values(state.assignments)) {
    if (isApproved(entry) && uavOf(entry) === sysId) {
      next = retire(next, entry, DOCK_OUTCOME.UNKNOWN, heardApproach(entry));
    }
  }
  return dropAssignments(next, (entry) => !isHandled(entry) && uavOf(entry) === sysId);
}

// ---- Transitions ----

export function availableTaskRequest(state, data) {
  const ownerId = data.sender_id;
  const listed = new Set();
  let availableTasks = state.availableTasks;
  for (const task of data.tasks || []) {
    listed.add(task.task_id);
    const entry = {
      key: roundKey(ownerId, task.task_id),
      taskId: task.task_id,
      taskType: task.task_type,
      lat: task.lat,
      lon: task.lon,
      alt: task.alt,
      senderId: ownerId,
      ownerBoot: data.uid?.boot_id,
    };
    if (sameAvailable(availableTasks[entry.key], entry)) continue;
    if (availableTasks === state.availableTasks) availableTasks = { ...availableTasks };
    availableTasks[entry.key] = entry;
  }
  const next = availableTasks === state.availableTasks ? state : { ...state, availableTasks };
  // An owner advertises a task again only after releasing it. Its WAITING helper
  // drops the task, and so does the pending round. A slot, an approved dock or a
  // handled dock of that task no longer holds the offer from now on, so it does
  // not hide the new one. A UAV's pending own round and its own advert are one dock.
  const released = (entry) => entry.senderId === ownerId && listed.has(entry.taskId)
    && follows(data.uid, offerFloor(entry));
  const kept = dropAssignments(next, (entry) => isRound(entry) && released(entry));
  return markReoffered(kept, Object.values(kept.assignments).filter(
    (entry) => !isPendingOwn(entry) && !entry.reoffered && released(entry),
  ), data.uid);
}

export function assignRequest(state, data) {
  const { sender_id: owner, receiver_id: helper, task_id: offeredTask, uid } = data;
  // What a UAV already holds of this offer: a slot, an approved or a handled dock. A
  // UAV's request to itself (SelfAssignmentPublisher) offers nothing to anyone.
  const holders = uid == null || owner === helper ? [] : Object.values(state.assignments).filter(
    (entry) => !isRound(entry) && !isPendingOwn(entry) && entry.senderId === owner
      && entry.taskId === offeredTask && sameSpot(entry, data)
      && entry.requestUid?.boot_id === uid.boot_id,
  );
  // A UAV's own round is no step of an assignment, so it makes no request a copy.
  if (holders.some((entry) => !isOwnDock(entry) && !isNewer(uid, offerFloor(entry)))) return state;
  const next = offerRound(state, data);
  // A later request releases them as an advert does, so the new offer shows.
  return next === state ? state : markReoffered(next, holders.filter(
    (entry) => !entry.reoffered && isNewer(uid, offerFloor(entry)),
  ), uid);
}

function offerRound(state, data) {
  const key = roundKey(data.sender_id, data.task_id);
  const current = state.assignments[key];
  if (current && !isNewer(data.uid, current.requestUid)) return state;
  // A copy of the assignment its helper already holds, wherever it says the task is.
  const slot = state.assignments[helperKey(data.receiver_id)];
  if (slot && slot.senderId === data.sender_id && slot.taskId === data.task_id
      && !isNewer(data.uid, offerFloor(slot))) return state;
  // A repeated copy keeps its round; another helper or owner boot starts one.
  const sameRound = !!current && current.receiverId === data.receiver_id
    && follows(data.uid, current.requestUid);
  const next = withAssignment(state, {
    key,
    taskId: data.task_id,
    taskType: data.task_type,
    lat: data.lat,
    lon: data.lon,
    alt: data.alt,
    senderId: data.sender_id,
    receiverId: data.receiver_id,
    status: sameRound ? current.status : ASSIGNMENT_STATUS.WAITING,
    requestUid: data.uid ?? null,
    doingUid: sameRound ? current.doingUid : null,
  });
  return dropAvailable(next, [key]);
}

export function assignResponse(state, data) {
  const round = state.assignments[roundKey(data.receiver_id, data.task_id)];
  if (!round || round.receiverId !== data.sender_id) return state;
  if (data.is_accepted) {
    // "doing" is not yet assigned: only the owner's APPLIED is. Its uid
    // orders the helper's later heartbeats against this round.
    if (!isNewer(data.uid, round.doingUid)) return state;
    return withAssignment(state, { ...round, doingUid: data.uid });
  }
  // A reject the helper sent before its latest "doing" answered an older offer.
  if (follows(round.doingUid, data.uid)) return state;
  return dropAssignments(state, (entry) => entry === round);
}

export function assignAck(state, data) {
  const round = state.assignments[roundKey(data.owner_id, data.task_id)];
  if (!round || round.receiverId !== data.helper_id) return state;
  // An APPLIED sent before this request answered an earlier assignment.
  if (follows(round.requestUid, data.uid)) return state;
  const key = helperKey(data.helper_id);
  const slot = state.assignments[key];
  // An APPLIED no newer than the one this owner last applied the slot by arrived
  // late: it must not bring back an older task.
  if (slot && slot.senderId === data.owner_id && slot.appliedUid != null
      && !isNewer(data.uid, slot.appliedUid)) return state;
  const doingUid = isNewer(data.ref, round.doingUid) ? data.ref : round.doingUid;
  // A request after the slot's APPLIED is a new assignment: it needs a new approval
  // and ends the approved dock the slot held.
  const start = slot && isApproved(slot) ? retire(state, slot, DOCK_OUTCOME.UNKNOWN) : state;
  const assignments = { ...start.assignments };
  delete assignments[round.key];
  assignments[key] = {
    ...round, key, status: ASSIGNMENT_STATUS.ASSIGNED, doingUid, assignedUid: doingUid, appliedUid: data.uid ?? null,
  };
  return { ...start, assignments };
}

export function swarmHeartbeat(state, data) {
  const swarm = data.swarm;
  if (!swarm) return state;
  const beat = { boot_id: swarm.boot, msg_seq: swarm.seq };
  // The helper stamps its state and its "doing" copies from one counter, so a
  // FREE beat after its latest "doing" means it no longer holds the task.
  let next = swarm.state !== 'FREE' ? state : dropAssignments(
    state,
    (entry) => entry.receiverId === data.sys_id && RANK[entry.status] < RANK.confirmed
      && entry.doingUid != null && isNewer(beat, entry.doingUid),
  );
  if (!swarm.phase) return next;
  for (const entry of Object.values(next.assignments)) {
    if (uavOf(entry) === data.sys_id) next = readBeat(next, entry, beat, swarm);
  }
  return next;
}

// The socket dropped, so rounds and their answers may have been missed: no open
// round, approved or still pending, can tie an outcome to its own approach any more.
// The socket dropped, or (with `data.sys_id`) one UAV's beats outlived their TTL.
export function linkGap(state, data) {
  let next = state;
  for (const entry of Object.values(state.assignments)) {
    if (data && uavOf(entry) !== data.sys_id) continue;
    const open = isApproved(entry) || entry.status === ASSIGNMENT_STATUS.CONFIRMING;
    if (open && !entry.noWord) next = withAssignment(next, { ...entry, noWord: true });
    else if (isHandled(entry) && entry.awaitingOutcome) {
      next = withAssignment(next, { ...entry, awaitingOutcome: false });
    }
  }
  return next;
}

// The latest round a UAV asks while it holds the task answers for it, until
// the task is confirmed. A round with a position that no slot takes is the
// UAV's own dock, and so is one at another position than the slot's: the round
// the slot was asking about has been superseded, so the slot is assigned again.
export function taskConfirming(state, data) {
  const uid = parseRoundUid(data.round_uid);
  if (uid && roundHeard(state, data.sys_id, uid)) return state;
  const next = endEarlierRounds(state, data.sys_id, uid);
  const slot = next.assignments[helperKey(data.sys_id)];
  if (!slot || !follows(uid, slot.assignedUid)) return withOwnRound(next, data, uid);
  if (hasPosition(data) && !sameSpot(slot, data)) {
    const asked = slot.status === ASSIGNMENT_STATUS.CONFIRMING;
    return withOwnRound(asked ? withAssignment(next, { ...slot, status: ASSIGNMENT_STATUS.ASSIGNED }) : next, data, uid);
  }
  // A later round is one the gap did not hide.
  const { noWord, ...held } = slot;
  return withAssignment(next, {
    ...held,
    status: ASSIGNMENT_STATUS.CONFIRMING,
    confirm: { taskId: data.task_id, roundUid: data.round_uid },
  });
}

export function taskConfirmed(state, data) {
  const own = ownRound(state, data);
  if (own) return raise(state, own, ASSIGNMENT_STATUS.CONFIRMED);
  return raise(state, confirmingSlot(state, data), ASSIGNMENT_STATUS.CONFIRMED);
}

// A denied POI: the UAV's own advert of it goes, and the task it was
// confirming for its owner goes back to ASSIGNED. A deny does not show that
// the UAV let the task go: after its own timeout it keeps the task and asks
// again (the card's countdown deny then comes late). Its FREE beat retires
// the slot once it lets the task go. Cancel on an approved dock ends it:
// it goes to history as canceled.
export function taskResolved(state, data) {
  const slot = confirmingSlot(state, data);
  const own = ownRound(state, data);
  let next = state;
  for (const entry of [slot, own]) {
    if (!entry) continue;
    if (isApproved(entry)) next = retire(next, entry, DOCK_OUTCOME.CANCELED);
    else if (isPendingOwn(entry)) next = dropAssignments(next, (other) => other === entry);
    else next = withAssignment(next, { ...entry, status: ASSIGNMENT_STATUS.ASSIGNED });
  }
  // The UAV's teardown beat can overtake the broadcast of a cancel (about 0.5 s
  // behind its first copy): the word is still the operator's. A real word stays.
  const late = state.assignments[handledKey(data.sys_id, data.round_uid)];
  if (late && answersRound(late.confirm, data) && late.outcome === DOCK_OUTCOME.UNKNOWN) {
    next = withAssignment(next, { ...late, outcome: DOCK_OUTCOME.CANCELED, awaitingOutcome: false });
  }
  return dropAvailable(next, [
    roundKey(data.sys_id, data.task_id),
    ...(slot ? [roundKey(slot.senderId, slot.taskId)] : []),
  ]);
}

export function disarmAfterGuided(state, data) {
  const next = endVehicle(state, data.sys_id);
  return dropAvailable(next, Object.keys(next.availableTasks).filter(
    (key) => next.availableTasks[key].senderId === data.sys_id,
  ));
}

// E-STOP keeps the map's history; only a new mission ends it (`reset`).
export function vehicleReset(state, data) {
  return endVehicle(state, data.sys_id);
}

export function resetAll() {
  return INITIAL_ASSIGNMENT_STATE;
}

const ACTIONS = Object.freeze({
  available_task_request: availableTaskRequest,
  task_assign_request: assignRequest,
  task_assign_response: assignResponse,
  task_assign_ack: assignAck,
  swarm_heartbeat: swarmHeartbeat,
  link_gap: linkGap,
  task_confirming: taskConfirming,
  task_confirmed: taskConfirmed,
  task_resolved: taskResolved,
  disarm_after_guided: disarmAfterGuided,
  vehicle_reset: vehicleReset,
  reset: resetAll,
});

export function reduceTaskAssignment(state, action) {
  const transition = ACTIONS[action.type];
  return transition ? transition(state, action.data) : state;
}

// ---- Selectors ----

export function isAssignedOrLater(entry) {
  return !!entry && RANK[entry.status] >= RANK.assigned;
}

/**
 * The dock a UAV works on: its furthest open round, whether the dock was given
 * to it (receiverId) or it asked for its own detected dock and loiters until
 * the operator confirms (askedBy, receiverId null; withOwnRound). An offer it
 * has not taken (waiting) and a handled round are not its dock.
 */
export function vehicleDock(assignments, sysId) {
  let best = null;
  for (const entry of Object.values(assignments || {})) {
    if (uavOf(entry) !== sysId || isHandled(entry) || !(RANK[entry.status] > RANK.waiting)) continue;
    if (!best || RANK[entry.status] > RANK[best.status]) best = entry;
  }
  return best;
}

/** The entry a vehicle card shows: highest rank, then the round it answered. */
export function vehicleAssignment(assignments, sysId) {
  const score = (entry) => RANK[entry.status] * 2 + (entry.doingUid ? 1 : 0);
  let best = null;
  for (const entry of Object.values(assignments)) {
    if (entry.receiverId === sysId && (!best || score(entry) > score(best))) best = entry;
  }
  return best;
}

/**
 * What the map draws. One offer is the same owner, task, position and owner
 * boot, until a later advert or request marks what holds it `reoffered`: such
 * an entry no longer holds its offer and has a marker of its own. Each UAV on
 * an offer has one marker: a live state is a new decision on its dock, so it
 * shows over that dock's history. An offer shows as detected
 * or offered only until a UAV holds it. History gives way to a live state at its
 * position until that ends, and what remains at one position gets a label row
 * each and one pad.
 */
export function dockMarkerEntries(assignments, availableTasks) {
  const offerOf = (entry, ownerBoot) => `${entry.senderId}:${entry.taskId}:${entry.lat}:${entry.lon}:${ownerBoot}`;
  const spotOf = (entry) => `${entry.lat}:${entry.lon}`;
  const isWaiting = (entry) => entry.status === ASSIGNMENT_STATUS.WAITING;
  const entries = Object.values(assignments);
  const top = new Map();
  const held = new Set();
  for (const entry of entries) {
    if (isWaiting(entry)) continue;
    const offer = offerOf(entry, entry.requestUid?.boot_id);
    if (!entry.reoffered) held.add(offer);
    const identity = entry.reoffered ? entry.key : `${offer}:${uavOf(entry)}`;
    const rank = isHandled(entry) ? 0 : 1;
    const current = top.get(identity);
    if (!current || rank >= current.rank) top.set(identity, { entry, rank, side: 'assignments' });
  }
  for (const entry of Object.values(availableTasks)) {
    const offer = offerOf(entry, entry.ownerBoot);
    if (!held.has(offer)) top.set(offer, { entry, side: 'availableTasks' });
  }
  for (const entry of entries.filter(isWaiting)) {
    const offer = offerOf(entry, entry.requestUid?.boot_id);
    if (!held.has(offer)) top.set(offer, { entry, side: 'assignments' });
  }
  const drawn = [...top.values()];
  const liveAt = new Set(drawn.filter(({ entry }) => !isHandled(entry)).map(({ entry }) => spotOf(entry)));
  const shown = drawn.filter(({ entry }) => !isHandled(entry) || !liveAt.has(spotOf(entry)));
  // Pads at one spot would cover each other in an order Cesium picks, so one draws:
  // the round the operator must answer, else the furthest state, and among equals the
  // last, which for history is the newest to end.
  const padRank = (entry) => (entry.status === ASSIGNMENT_STATUS.CONFIRMING
    ? RANK.confirmed + 1
    : RANK[entry.status] ?? -1);
  const padAt = new Map();
  for (const { entry } of shown) {
    const spot = spotOf(entry);
    const current = padAt.get(spot);
    if (!current || padRank(entry) >= padRank(current)) padAt.set(spot, entry);
  }
  const rows = new Map();
  const markers = { assignments: {}, availableTasks: {} };
  for (const { entry, side } of shown) {
    const spot = spotOf(entry);
    const row = rows.get(spot) ?? 0;
    rows.set(spot, row + 1);
    const hidePad = padAt.get(spot) !== entry;
    markers[side][entry.key] = row || hidePad ? { ...entry, labelRow: row, hidePad } : entry;
  }
  return markers;
}
