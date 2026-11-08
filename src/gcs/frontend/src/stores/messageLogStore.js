/**
 * Per-UAV Messages window history, outside React like the telemetry store.
 * The telemetry socket feeds new entries (`messages` in each vehicle's
 * snapshot); the window loads the backend backlog when it opens.
 */
import { appendEntries, mergeBacklog } from '../utils/messageLog';

let _histories = {};
const _listeners = new Set();

function _notify() {
  for (const fn of _listeners) fn();
}

function subscribe(listener) {
  _listeners.add(listener);
  return () => _listeners.delete(listener);
}

const EMPTY = [];

function getEntries(sysId) {
  return _histories[sysId] || EMPTY;
}

/** Live entries from the telemetry broadcast. */
function ingest(sysId, entries) {
  if (!entries || entries.length === 0) return;
  const next = appendEntries(getEntries(sysId), entries);
  if (next === _histories[sysId]) return;
  _histories = { ..._histories, [sysId]: next };
  _notify();
}

/** The backend backlog (`GET /api/vehicles/{sysId}/messages`): its entries and log epoch. */
function loadBacklog(sysId, backlogEntries, epoch) {
  _histories = { ..._histories, [sysId]: mergeBacklog(backlogEntries || [], epoch, getEntries(sysId)) };
  _notify();
}

/** Tests: forget every history. */
function reset() {
  _histories = {};
  _notify();
}

const messageLogStore = { subscribe, getEntries, ingest, loadBacklog, reset };
export default messageLogStore;
