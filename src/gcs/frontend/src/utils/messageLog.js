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
 * Append live entries to a history. A seq at or below the newest kept seq
 * means the backend's log started over (backend restart or reconnect), so
 * the history restarts with the new entries.
 */
export function appendEntries(history, incoming, capacity = MESSAGE_LOG_CAPACITY) {
  if (!incoming || incoming.length === 0) return history;
  const lastSeq = history.length > 0 ? history[history.length - 1].seq : 0;
  const base = incoming[0].seq <= lastSeq ? [] : history;
  const merged = base.concat(incoming);
  return merged.length > capacity ? merged.slice(merged.length - capacity) : merged;
}

/**
 * Merge the backend backlog with entries that arrived live while it loaded.
 * The backlog is the authority; live entries newer than it are kept.
 */
export function mergeBacklog(backlog, live, capacity = MESSAGE_LOG_CAPACITY) {
  const lastSeq = backlog.length > 0 ? backlog[backlog.length - 1].seq : 0;
  const newer = (live || []).filter((e) => e.seq > lastSeq);
  const merged = backlog.concat(newer);
  return merged.length > capacity ? merged.slice(merged.length - capacity) : merged;
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
