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
 * confirmed are never lowered within a round. Available tasks are keyed
 * `${owner}:${task}` too: task ids are per owner.
 *
 * Confirm events carry the confirming UAV's own POI id: each companion numbers
 * its POIs itself, so a helper confirms the owner's task 2 as its POI 1. They
 * are matched by UAV, to that UAV's helper slot, which records the round (POI
 * id, round uid) the GCS heard the UAV ask while it held the task
 * (CONFIRMING). The round must come from the helper's boot after its "doing";
 * only an answer naming that round moves the slot. Anything else (an own-POI
 * round asked before the assignment, a restarted helper's round, a later
 * round at its home dock, a stale page's decision, a round without a uid) does
 * not. A round the GCS heard before it heard the owner's APPLIED is never
 * counted either: it may be an own-POI round.
 *
 * Every function returns the same state reference when nothing changes.
 */

export const ASSIGNMENT_STATUS = Object.freeze({
  WAITING: 'waiting',
  ASSIGNED: 'assigned',
  CONFIRMING: 'confirming',
  CONFIRMED: 'confirmed',
});

const RANK = Object.freeze({ waiting: 0, assigned: 1, confirming: 2, confirmed: 3 });

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

function forVehicle(sysId) {
  return (entry) => entry.receiverId === sysId;
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
  return !!current && current.taskType === entry.taskType
    && current.lat === entry.lat && current.lon === entry.lon && current.alt === entry.alt;
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
    };
    if (sameAvailable(availableTasks[entry.key], entry)) continue;
    if (availableTasks === state.availableTasks) availableTasks = { ...availableTasks };
    availableTasks[entry.key] = entry;
  }
  const next = availableTasks === state.availableTasks ? state : { ...state, availableTasks };
  // An owner advertises a task again only after releasing it: its WAITING
  // helper drops the task, and so does the pending round.
  return dropAssignments(next, (entry) => isRound(entry) && entry.senderId === ownerId
    && listed.has(entry.taskId) && follows(data.uid, entry.requestUid));
}

export function assignRequest(state, data) {
  const key = roundKey(data.sender_id, data.task_id);
  const current = state.assignments[key];
  if (current && !isNewer(data.uid, current.requestUid)) return state;
  // A copy no newer than the step 3 its helper is already assigned by.
  const slot = state.assignments[helperKey(data.receiver_id)];
  if (slot && slot.senderId === data.sender_id && slot.taskId === data.task_id
      && !isNewer(data.uid, slot.requestUid)) return state;
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
  const key = helperKey(data.helper_id);
  const slot = state.assignments[key];
  const doingUid = isNewer(data.ref, round.doingUid) ? data.ref : round.doingUid;
  let status = higher(round.status, ASSIGNMENT_STATUS.ASSIGNED);
  let kept = {};
  // The same assignment applied again to the same helper boot keeps how far
  // its confirmation got; a restarted helper confirms afresh.
  if (slot && slot.senderId === round.senderId && slot.taskId === round.taskId
      && slot.requestUid?.boot_id === round.requestUid?.boot_id
      && slot.doingUid?.boot_id === doingUid?.boot_id) {
    status = higher(status, slot.status);
    if (slot.confirm) kept = { confirm: slot.confirm };
  }
  const assignments = { ...state.assignments };
  delete assignments[round.key];
  assignments[key] = { ...round, key, status, doingUid, ...kept };
  return { ...state, assignments };
}

export function swarmHeartbeat(state, data) {
  const swarm = data.swarm;
  if (!swarm || swarm.state !== 'FREE') return state;
  const beat = { boot_id: swarm.boot, msg_seq: swarm.seq };
  // The helper stamps its state and its "doing" copies from one counter, so a
  // FREE beat after its latest "doing" means it no longer holds the task.
  return dropAssignments(state, (entry) => entry.receiverId === data.sys_id
    && RANK[entry.status] < RANK.confirmed
    && entry.doingUid != null && isNewer(beat, entry.doingUid));
}

// The latest round a UAV asks while it holds the task answers for it, until
// the task is confirmed.
export function taskConfirming(state, data) {
  const slot = state.assignments[helperKey(data.sys_id)];
  if (!slot || slot.status === ASSIGNMENT_STATUS.CONFIRMED) return state;
  if (!follows(parseRoundUid(data.round_uid), slot.doingUid)) return state;
  if (answersRound(slot.confirm, data)) return state;
  return withAssignment(state, {
    ...slot,
    status: ASSIGNMENT_STATUS.CONFIRMING,
    confirm: { taskId: data.task_id, roundUid: data.round_uid },
  });
}

export function taskConfirmed(state, data) {
  return raise(state, confirmingSlot(state, data), ASSIGNMENT_STATUS.CONFIRMED);
}

// A denied POI: the UAV's own advert of it goes, and so does the task it was
// confirming for its owner.
export function taskResolved(state, data) {
  const slot = confirmingSlot(state, data);
  const next = slot ? dropAssignments(state, (entry) => entry === slot) : state;
  return dropAvailable(next, [
    roundKey(data.sys_id, data.task_id),
    ...(slot ? [roundKey(slot.senderId, slot.taskId)] : []),
  ]);
}

export function disarmAfterGuided(state, data) {
  const next = dropAssignments(state, forVehicle(data.sys_id));
  return dropAvailable(next, Object.keys(next.availableTasks).filter(
    (key) => next.availableTasks[key].senderId === data.sys_id,
  ));
}

export function vehicleReset(state, data) {
  return dropAssignments(state, forVehicle(data.sys_id));
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

/** The entry a vehicle card shows: highest rank, then the round it answered. */
export function vehicleAssignment(assignments, sysId) {
  const score = (entry) => RANK[entry.status] * 2 + (entry.doingUid ? 1 : 0);
  let best = null;
  for (const entry of Object.values(assignments)) {
    if (entry.receiverId === sysId && (!best || score(entry) > score(best))) best = entry;
  }
  return best;
}
