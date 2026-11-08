/**
 * Messages window helpers: merge the per-UAV history, filter it, export it.
 * Entries come from the backend's VehicleMessageLog (vehicle_message_log.py):
 * {seq, kind: 'text'|'value', ts, source, severity, label, text, name?, value?}.
 */

// Entries kept per UAV, the backend's MESSAGE_LOG_CAPACITY.
export const MESSAGE_LOG_CAPACITY = 500;

// A named value not heard for longer than this shows as stale. NAVWD comes
// at 1 Hz; three missed reports on a lossy link is "stopped", not jitter.
export const NAMED_VALUE_STALE_S = 3;

// MAV_SEVERITY: 0..3 EMERG/ALERT/CRIT/ERR, 4 WARN, 5..7 NOTICE/INFO/DEBUG.
const ERROR_MAX_SEVERITY = 3;
const WARNING_SEVERITY = 4;

export const MESSAGE_FILTERS = ['all', 'warnings', 'texts', 'values'];

/**
 * Entries of `incoming` newer than `history`'s newest, appended. Every entry
 * carries its backend log's `epoch`; a different epoch means the log started
 * over (a reconnect or backend restart, seq from 1 again), so the history
 * restarts with the new log. Within one epoch, seq orders and deduplicates:
 * the backlog and the live feed may both carry an entry.
 */
export function appendEntries(history, incoming, capacity = MESSAGE_LOG_CAPACITY) {
  if (!incoming || incoming.length === 0) return history;
  const newest = history.length > 0 ? history[history.length - 1] : null;
  const epoch = incoming[incoming.length - 1].epoch;
  const sameLog = newest !== null && newest.epoch === epoch;
  const base = sameLog ? history : [];
  const lastSeq = sameLog ? newest.seq : 0;
  const added = incoming.filter((e) => e.epoch === epoch && e.seq > lastSeq);
  if (added.length === 0) return history;
  const merged = base.concat(added);
  return merged.length > capacity ? merged.slice(merged.length - capacity) : merged;
}

/**
 * The backend backlog (of log `epoch`) with live entries that arrived while
 * it loaded. The backlog is the authority: only live entries of its epoch
 * newer than it are kept; a previous log's history is dropped.
 */
export function mergeBacklog(backlog, epoch, live, capacity = MESSAGE_LOG_CAPACITY) {
  const sameLog = (live || []).filter((e) => e.epoch === epoch);
  if (backlog.length === 0) return sameLog.slice(-capacity);
  return appendEntries(backlog.slice(-capacity), sameLog, capacity);
}

/** Identity of the newest entry ('' when none): changes with every new row. */
export function newestKey(entries) {
  const e = entries[entries.length - 1];
  return e ? `${e.epoch}:${e.seq}` : '';
}

export function filterEntries(entries, filter) {
  switch (filter) {
    case 'warnings':
      return entries.filter((e) => e.kind === 'text' && e.severity <= WARNING_SEVERITY);
    case 'texts':
      return entries.filter((e) => e.kind === 'text');
    case 'values':
      return entries.filter((e) => e.kind === 'value');
    default:
      return entries;
  }
}

/** 'error' | 'warning' | 'info' for colouring a row. */
export function severityTone(entry) {
  if (entry.kind !== 'text') return 'info';
  if (entry.severity <= ERROR_MAX_SEVERITY) return 'error';
  if (entry.severity === WARNING_SEVERITY) return 'warning';
  return 'info';
}

function pad(n, width = 2) {
  return String(n).padStart(width, '0');
}

/** HH:MM:SS local time of a backend timestamp (seconds). */
export function formatTime(tsSeconds) {
  if (typeof tsSeconds !== 'number' || !Number.isFinite(tsSeconds)) return '--:--:--';
  const d = new Date(tsSeconds * 1000);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

/** Plain-text export, one line per entry, oldest first. */
export function formatExport(entries, uavName) {
  const lines = entries.map((e) => {
    const d = new Date(e.ts * 1000);
    const stamp = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ${formatTime(e.ts)}`;
    return `${stamp}\t${uavName}\t${e.source}\t${e.label}\t${e.text}`;
  });
  return `${lines.join('\n')}\n`;
}

/** Seconds since a named value last arrived, on the backend's clock. */
export function namedValueAgeS(named, nowMs) {
  if (!named || typeof named.ts !== 'number') return null;
  return Math.max(0, nowMs / 1000 - named.ts);
}

/** Named values sorted by name, each with its age and stale flag. */
export function namedValueRows(namedValues, nowMs) {
  return Object.entries(namedValues || {})
    .sort(([a], [b]) => a.localeCompare(b))
    .map(([name, nv]) => {
      const age = namedValueAgeS(nv, nowMs);
      return { name, ...nv, age, stale: age != null && age > NAMED_VALUE_STALE_S };
    });
}
