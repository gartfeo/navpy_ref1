/**
 * The header that carries a request's time limit into a lockstep run: the
 * app relay turns it into the request's sim-time `deadline_s`
 * (scripts/lockstep_app_relay.py). The backend ignores it on hardware, where
 * the browser's own abort still frees the page; an abort sends no cancel.
 */
export const DEADLINE_HEADER = 'X-NavPy-Deadline-S';

export function deadlineHeaders(ms) {
  return { [DEADLINE_HEADER]: String(ms / 1000) };
}

/**
 * True when the page serves a lockstep run (scripts/lockstep_app_stack.py sets
 * `VITE_NAVPY_LOCKSTEP=1`). There no wall timer may decide whether a request is
 * sent: no retry after a wall delay, no diagnostic posted from a wall abort.
 */
export function isLockstepRun(env = import.meta.env) {
  return env?.VITE_NAVPY_LOCKSTEP === '1';
}

/**
 * A request's time limit. On hardware the page aborts the fetch after `ms`.
 * On a lockstep run it sets no wall timer: the run's `deadline_s` answers
 * instead (504, see isSimDeadline), so host speed never decides what the page
 * sends next. Spread `headers` into the fetch; call `clear` when it settles.
 */
export function requestDeadline(ms, lockstep = isLockstepRun()) {
  const ctrl = new AbortController();
  const timer = lockstep ? null : setTimeout(() => ctrl.abort(), ms);
  return { signal: ctrl.signal, headers: deadlineHeaders(ms), clear: () => clearTimeout(timer) };
}

/** True for the answer a lockstep run gives when a request's `deadline_s` passed. */
export function isSimDeadline(res, lockstep = isLockstepRun()) {
  return lockstep && res.status === 504;
}
