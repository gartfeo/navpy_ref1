/**
 * Smooth UAV motion on the map by playout (todo 267): every position fix is
 * drawn at its own time, a fixed delay behind the newest one, by
 * interpolating between the two fixes around a render clock. This is the
 * snapshot interpolation of networked games and flight trackers. The drawn
 * position moves only forward along the flown fixes, the trail can be drawn
 * up to it (useUavTrails), and late, early or bursty arrivals change nothing
 * while the delay covers them. Dead reckoning (todo 261) instead drew the
 * model past the newest fix and pulled it back when the next one disagreed.
 *
 * Timeline: the backend's `ts` of each telemetry message (backendClock, ms).
 * A message carries `path`, every fix since that UAV's last broadcast; its
 * fixes are spread evenly over the time since that broadcast (at most
 * MAX_SPAN_INTERVALS intervals), its newest at `ts`. The render clock runs
 * on the page's clock at the backend clock's measured rate and holds the
 * newest fix about DELAY_INTERVALS broadcast intervals ahead: each message
 * sets a rate a little above or below 1 that closes the difference, it slows
 * smoothly when the buffer runs short (late packets), and it never passes the
 * newest fix or moves backwards.
 *
 * Exceptions, handled explicitly:
 * - Telemetry gap (not fresh, or no position): the playout restarts at the
 *   first fresh sample; the model does not glide across the gap.
 * - A jump between two fixes faster than JUMP_SPEED_MPS (a SITL restart or a
 *   reposition): the clock moves to the new fix at once and the fix is marked
 *   `jump`, where the trail opens a new run.
 * - A main-thread or tab stall that leaves the clock over STALL_MS behind:
 *   the clock moves forward to the usual delay behind the newest fix.
 * - First connect: the model starts at the snapshot position; the first
 *   path may hold fixes behind it, so the clock starts on that path's newest
 *   fix (the model steps forward once, never back).
 * - A message older than the last one: its fixes are dropped (one WebSocket
 *   keeps the order, so this guards only against a clock reset).
 * A real reversal by the aircraft is drawn as flown: the fixes reverse.
 *
 * Heights are over home; the ground under the UAV is a separate height that
 * fades on its own (setGround), so a new terrain sample never reads as a
 * climb or a descent. Pure numbers, no Cesium: useUavMarkers turns a sample
 * into a position and an orientation.
 */

/** Assumed broadcast interval (ms) before two messages have arrived: the 5 Hz broadcast. */
const INITIAL_INTERVAL_MS = 200;
/** Bounds (ms) of the measured broadcast interval. */
const MIN_INTERVAL_MS = 50;
const MAX_INTERVAL_MS = 1000;
/**
 * Render delay in broadcast intervals: one for the next message to be due,
 * one for it to arrive late (backend loop or page stalls) without the model
 * stopping.
 */
export const DELAY_INTERVALS = 2;
/** Below this much buffered (intervals) the clock slows, to a stop at the newest fix. */
const LOW_INTERVALS = 0.5;
/**
 * Bounds of the clock rate a message sets: just after a message the newest
 * fix should be DELAY ahead; the rate closes the difference over about
 * 2 x DELAY, so it changes only at messages, not within an interval.
 */
const MIN_RATE = 0.5;
const MAX_RATE = 1.5;
/** Time constant (ms) of the clock rate's changes, so the speed never jumps. */
const RATE_TAU_MS = 250;
/** A clock this far behind the newest fix (ms of backend time) was stalled: catch up. */
export const STALL_MS = 3000;
/** One message's fixes span at most this many intervals (a UAV not broadcast while still). */
const MAX_SPAN_INTERVALS = 2;
/** Fixes kept behind the clock (ms), for the trail's live segment. */
const KEEP_MS = 2000;
/**
 * A step between two fixes over JUMP_MIN_M and faster than JUMP_SPEED_MPS
 * (m/s of backend time) is a jump, not flight: SITL speedup 10 at 40 m/s is
 * 400 m/s, 8 m per 20 ms fix and 80 m per 5 Hz snapshot without `path`.
 */
export const JUMP_SPEED_MPS = 2000;
export const JUMP_MIN_M = 300;
/** Window (ms of page time) over which the backend clock's rate is measured. */
const RATE_WINDOW_MS = 2000;

const M_PER_DEG = 111320;
// Fix ids never repeat, so a reader's watermark (useUavTrails) stays valid
// across a restart or a UAV that leaves and returns.
let nextFixId = 1;

/** The playouts of a map, by sys_id, kept on its entities ref. */
export function playoutsOf(ents) {
  if (!ents.uavPlayout) ents.uavPlayout = {};
  return ents.uavPlayout;
}

/** Drop the playouts of UAVs no longer in `ids`. */
export function prunePlayouts(ents, ids) {
  const all = playoutsOf(ents);
  for (const sid of Object.keys(all)) if (!ids.has(Number(sid))) delete all[sid];
}

function newPlayout() {
  return {
    fixes: [], oris: [], lastV: null, lastPath: undefined, lastTs: null, gap: false,
    interval: INITIAL_INTERVAL_MS, clock: null, rate: 1, target: 1, clockNow: null, arrivals: [],
  };
}

const finite = (x) => typeof x === 'number' && Number.isFinite(x);

function stepMetres(a, b) {
  const kx = Math.cos((a.lat * Math.PI) / 180) * M_PER_DEG;
  const dx = (b.lon - a.lon) * kx;
  const dy = (b.lat - a.lat) * M_PER_DEG;
  const dz = b.alt - a.alt;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * The positions a snapshot adds. With `path` only its fixes count, never the
 * snapshot position (it may be older than the newest fix, or already in the
 * next path); a path already used adds nothing. Without `path`, the current
 * position. A UAV with no fix yet starts from its snapshot position, for the
 * model only (`trail: false`).
 */
function snapshotFixes(pb, v) {
  // No altitude is no fix: a 0 fill would drop the UAV to the ground.
  const alt = (p) => p.alt_rel ?? p.alt;
  const ok = (p) => p && finite(p.lat) && finite(p.lon) && finite(alt(p));
  let out;
  if (!Array.isArray(v.path)) out = ok(v) ? [{ lat: v.lat, lon: v.lon, alt: alt(v), trail: true }] : [];
  else if (v.path === pb.lastPath) out = [];
  else out = v.path.filter(ok).map((p) => ({ lat: p.lat, lon: p.lon, alt: alt(p), trail: true }));
  if (out.length === 0 && pb.fixes.length === 0 && ok(v)) {
    out = [{ lat: v.lat, lon: v.lon, alt: alt(v), trail: false }];
  }
  return out;
}

/** Backend ms per page ms, measured over the last RATE_WINDOW_MS of arrivals. */
function clockSpeed(pb) {
  const a = pb.arrivals;
  if (a.length < 2) return 1;
  const wall = a[a.length - 1].now - a[0].now;
  if (wall < RATE_WINDOW_MS / 2) return 1;
  return Math.min(Math.max((a[a.length - 1].ts - a[0].ts) / wall, 0.1), 50);
}

/**
 * Take one UAV's snapshot into its playout, once per snapshot object (both
 * the model and the trail call it; the first call does the work). `fresh` is
 * the store's freshness of the UAV's telemetry; `tsMs` the backend time of
 * the message (backendNowMs). Returns the playout, or null while the UAV is
 * in a gap or has none.
 */
export function ingestVehicle(ents, v, fresh, tsMs, now = Date.now()) {
  const all = playoutsOf(ents);
  const sid = v.sys_id;
  if (!fresh || v.lat == null || v.lon == null) {
    if (all[sid]) all[sid].gap = true;
    return null;
  }
  let pb = all[sid];
  if (pb && pb.lastV === v) return pb;
  if (!pb || pb.gap) {
    const ground = pb?.ground;
    pb = newPlayout();
    if (ground) pb.ground = ground;
    all[sid] = pb;
  }
  pb.lastV = v;
  if (pb.lastTs !== null && tsMs < pb.lastTs) {
    pb.lastPath = v.path;
    return pb;
  }
  // Clock before the new fixes, so a burst cannot pull it forward.
  if (pb.clock !== null) playClock(pb, now);
  const prevTs = pb.lastTs;
  if (prevTs !== null && tsMs > prevTs && tsMs - prevTs <= MAX_INTERVAL_MS) {
    const sample = Math.max(tsMs - prevTs, MIN_INTERVAL_MS);
    pb.interval = Math.min(Math.max(pb.interval * 0.7 + sample * 0.3, MIN_INTERVAL_MS), MAX_INTERVAL_MS);
  }
  pb.arrivals.push({ now, ts: tsMs });
  while (pb.arrivals.length > 2 && now - pb.arrivals[1].now >= RATE_WINDOW_MS) pb.arrivals.shift();

  const fresh_ = snapshotFixes(pb, v);
  // The first fixes after a first-connect snapshot may lie behind it (the
  // backend sends the position before the path buffered up to it): drop the
  // snapshot and start the clock on the newest of them, which is past it.
  const restart = fresh_.some((f) => f.trail) && pb.fixes.length > 0 && pb.fixes.every((f) => !f.trail);
  if (restart) {
    pb.fixes = [];
    pb.clock = null;
  }
  pb.lastPath = v.path;
  pb.lastTs = tsMs;
  const span = prevTs === null ? pb.interval
    : Math.min(tsMs - prevTs, MAX_SPAN_INTERVALS * pb.interval);
  fresh_.forEach((f, i) => {
    const fix = { ...f, id: nextFixId++, t: tsMs - span * (1 - (i + 1) / fresh_.length) };
    const last = pb.fixes[pb.fixes.length - 1];
    if (last) {
      const metres = stepMetres(last, fix);
      const dt = Math.max(fix.t - last.t, 1) / 1000;
      if (metres > JUMP_MIN_M && metres / dt > JUMP_SPEED_MPS) {
        // The clock skips to it; the fixes before stay for the trail.
        fix.jump = true;
        pb.clock = Math.max(pb.clock ?? fix.t, fix.t);
        pb.clockNow ??= now;
      }
    }
    pb.fixes.push(fix);
  });
  if (finite(v.heading) && finite(v.roll) && finite(v.pitch)) {
    pb.oris.push({ t: tsMs, heading: v.heading, roll: v.roll, pitch: v.pitch });
  }
  if (pb.clock === null && pb.fixes.length) {
    pb.clock = restart ? newestT(pb) : Math.max(pb.fixes[0].t, newestT(pb) - delayMs(pb));
    pb.clockNow = now;
  }
  if (fresh_.length && pb.clock !== null) {
    const error = newestT(pb) - pb.clock - delayMs(pb);
    pb.target = Math.min(Math.max(1 + error / (2 * delayMs(pb)), MIN_RATE), MAX_RATE);
  }
  trim(pb);
  return pb;
}

/**
 * The newest buffered time, of a fix or an attitude: attitudes go on when
 * position fixes stop, and the clock follows them (the model holds its last
 * position and keeps turning).
 */
function newestT(pb) {
  const f = pb.fixes.length ? pb.fixes[pb.fixes.length - 1].t : -Infinity;
  const o = pb.oris.length ? pb.oris[pb.oris.length - 1].t : -Infinity;
  return Math.max(f, o);
}

function delayMs(pb) {
  return DELAY_INTERVALS * pb.interval;
}

function trim(pb) {
  const keepFrom = pb.clock - KEEP_MS;
  let i = 0;
  // Keep the fix the clock is past, for the interpolation.
  while (i < pb.fixes.length - 1 && pb.fixes[i + 1].t < keepFrom) i++;
  if (i) pb.fixes.splice(0, i);
  let j = 0;
  while (j < pb.oris.length - 1 && pb.oris[j + 1].t < keepFrom) j++;
  if (j) pb.oris.splice(0, j);
}

/**
 * The clock's target rate with `lag` ms buffered ahead of it: the rate the
 * last message set, slowed towards a stop when the buffer runs short.
 */
function targetRate(pb, lag) {
  const low = LOW_INTERVALS * pb.interval;
  return lag < low ? pb.target * Math.max(lag, 0) / low : pb.target;
}

/**
 * Advance the render clock to page time `now` and return it (backend ms).
 * The same `now` returns the same clock, so the model and the trail's live
 * segment read one position in a frame. Never moves backwards.
 */
export function playClock(pb, now = Date.now()) {
  if (pb.clock === null) return null;
  if (now === pb.clockNow) return pb.clock;
  const dt = Math.max(0, now - pb.clockNow);
  pb.clockNow = Math.max(now, pb.clockNow);
  const newest = newestT(pb);
  if (newest - pb.clock > STALL_MS) {
    pb.clock = Math.max(pb.clock, newest - delayMs(pb));
    return pb.clock;
  }
  const target = targetRate(pb, newest - pb.clock);
  pb.rate += (target - pb.rate) * (1 - Math.exp(-dt / RATE_TAU_MS));
  pb.clock = Math.min(pb.clock + dt * clockSpeed(pb) * pb.rate, Math.max(newest, pb.clock));
  // Close enough to the newest fix is on it: the model comes to rest exactly.
  if (newest - pb.clock < 0.5) pb.clock = Math.max(pb.clock, newest);
  return pb.clock;
}

/** Whether the model is still moving towards its newest fix. */
export function playoutMoving(pb, now = Date.now()) {
  const clock = playClock(pb, now);
  return clock !== null && clock < newestT(pb);
}

/** Interpolated { lat, lon, alt } (alt over home) at the clock, or null with no fix. */
export function positionSample(pb, clock) {
  const f = pb.fixes;
  if (!f.length) return null;
  let i = f.length - 1;
  while (i > 0 && f[i].t > clock) i--;
  const a = f[i];
  const b = f[i + 1];
  if (!b || clock <= a.t) return { lat: a.lat, lon: a.lon, alt: a.alt };
  const u = (clock - a.t) / (b.t - a.t);
  return { lat: a.lat + (b.lat - a.lat) * u, lon: a.lon + (b.lon - a.lon) * u, alt: a.alt + (b.alt - a.alt) * u };
}

/** Shortest-way blend of two angles in degrees. */
function lerpAngle(a, b, u) {
  const d = ((((b - a) % 360) + 540) % 360) - 180;
  return a + d * u;
}

/** Interpolated { heading, roll, pitch } (degrees) at the clock, or null with none. */
export function attitudeSample(pb, clock) {
  const o = pb.oris;
  if (!o.length) return null;
  let i = o.length - 1;
  while (i > 0 && o[i].t > clock) i--;
  const a = o[i];
  const b = o[i + 1];
  if (!b || clock <= a.t) return { heading: a.heading, roll: a.roll, pitch: a.pitch };
  const u = (clock - a.t) / (b.t - a.t);
  return {
    heading: lerpAngle(a.heading, b.heading, u),
    roll: lerpAngle(a.roll, b.roll, u),
    pitch: lerpAngle(a.pitch, b.pitch, u),
  };
}

/** The ground height under the UAV at `now`, on its own fade (setGround); 0 before any. */
export function groundAt(pb, now = Date.now()) {
  const g = pb.ground;
  if (!g) return 0;
  const t = Math.min(Math.max(0, now - g.start) / g.duration, 1.0);
  return g.from + (g.to - g.from) * t;
}

/** Whether the ground is still fading to a new height. */
export function groundFading(pb, now = Date.now()) {
  const g = pb.ground;
  return !!g && now - g.start < g.duration;
}

/**
 * A new ground height under the UAV: the model fades to it over one
 * broadcast interval, the first one applies at once.
 */
export function setGround(pb, height, now = Date.now()) {
  const from = pb.ground ? groundAt(pb, now) : height;
  pb.ground = { from, to: height, start: now, duration: pb.interval };
}
