import { describe, it, expect } from 'vitest';
import {
  attitudeSample, groundAt, ingestVehicle, JUMP_SPEED_MPS, playClock, playoutMoving, playoutsOf,
  positionSample, prunePlayouts, setGround, STALL_MS,
} from './uavPlayout';

const M_PER_DEG = 111320;
// SITL speedup 10 at 25 m/s: 250 m/s of backend (wall) time.
const SPEED = 250;
const FIX_MS = 20; // GLOBAL_POSITION_INT at 50 Hz of wall time
const FRAME_MS = 16;

/** Metres east of lon 0 at lat 0 at backend time t (ms), flying east from t = 0. */
const eastAt = (t) => (SPEED * Math.max(t, 0)) / 1000;

/** Seeded uniform 0..1, so a jitter test fails the same way every run. */
function rng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 2 ** 32;
  };
}

/**
 * Messages of one UAV flying east: the backend broadcasts every ~200 ms
 * (`ts`, with `tsJitter` of its own loop) every fix since the last one; each
 * arrives in the page at `arrival` (ts + a delay from `delayOf`, in order).
 */
function flight({ count, tsJitter = () => 0, delayOf = () => 30, speedAt = () => SPEED }) {
  const msgs = [];
  let lastFix = -FIX_MS;
  let ts = 1000;
  let x = 0;
  for (let k = 0; k < count; k++) {
    ts += 200 + tsJitter(k);
    const path = [];
    for (let t = lastFix + FIX_MS; t <= ts; t += FIX_MS) {
      x += (speedAt(t) * FIX_MS) / 1000;
      path.push({ lat: 0, lon: x / M_PER_DEG, alt_rel: 100 });
      lastFix = t;
    }
    const newest = path[path.length - 1];
    // One WebSocket (TCP) keeps the order: a held message holds the next ones.
    const arrival = Math.max(ts + delayOf(k), msgs.length ? msgs[msgs.length - 1].arrival : -Infinity);
    msgs.push({
      ts, arrival,
      v: { sys_id: 1, lat: newest.lat, lon: newest.lon, alt_rel: 100, heading: 90, roll: 0, pitch: 0, path },
    });
  }
  return msgs;
}

/**
 * Play messages in arrival order and draw frames every frameMs (a frame
 * time from `frameOf` when given). Returns per frame the drawn metres east
 * and the newest fix's metres east.
 */
function play(msgs, { endMs, frameOf } = {}) {
  const ents = {};
  const order = [...msgs].sort((a, b) => a.arrival - b.arrival);
  const end = endMs ?? order[order.length - 1].arrival + 1000;
  const frames = [];
  let next = 0;
  let newest = null;
  let now = order[0].arrival;
  for (let f = 0; now <= end; f++) {
    while (next < order.length && order[next].arrival <= now) {
      const m = order[next];
      ingestVehicle(ents, m.v, true, m.ts, m.arrival);
      newest = Math.max(newest ?? -Infinity, m.v.lon * M_PER_DEG);
      next++;
    }
    const pb = playoutsOf(ents)[1];
    const clock = playClock(pb, now);
    const s = positionSample(pb, clock);
    frames.push({ now, x: s.lon * M_PER_DEG, newest, clock });
    now = frameOf ? frameOf(f, now) : now + FRAME_MS;
  }
  return { frames, ents };
}

const steps = (frames) => frames.slice(1).map((f, i) => f.x - frames[i].x);

describe('UAV playout (todo 267)', () => {
  it('moves only forward, smoothly, on a steady 5 Hz broadcast', () => {
    const { frames } = play(flight({ count: 60 }));
    const cruise = frames.filter((f) => f.now > 3000 && f.now < 12000);
    const st = steps(cruise);
    expect(Math.min(...st)).toBeGreaterThanOrEqual(0);
    // Steady speed once settled: every frame within 10 % of 4 m (250 m/s x 16 ms).
    const perFrame = (SPEED * FRAME_MS) / 1000;
    for (const s of st) expect(Math.abs(s - perFrame)).toBeLessThan(0.1 * perFrame);
    // About DELAY (2 intervals, 100 m) behind the newest fix, not the old 150 m+.
    const behind = cruise.map((f) => f.newest - f.x);
    expect(Math.max(...behind)).toBeLessThan(160);
    expect(Math.min(...behind)).toBeGreaterThan(0);
  });

  it('never moves backwards with jittery and bursty arrivals', () => {
    const r = rng(267);
    // Page arrivals: 10-150 ms late, every 9th message held 450 ms by a
    // main-thread stall (so the next ones arrive in a burst), and the
    // backend loop itself 0-60 ms late at times.
    const msgs = flight({
      count: 150,
      tsJitter: () => (r() < 0.3 ? r() * 60 : 0),
      delayOf: (k) => 10 + r() * 140 + (k % 9 === 0 ? 450 : 0),
    });
    // Frames with gaps of up to 120 ms (a loaded page).
    const fr = rng(7);
    const { frames } = play(msgs, { frameOf: (f, now) => now + (fr() < 0.1 ? 40 + fr() * 80 : FRAME_MS) });
    const cruise = frames.filter((f) => f.now > 3000);
    const st = steps(cruise);
    expect(Math.min(...st)).toBeGreaterThanOrEqual(0);
    // No jump: under 2 x the flight of the longest frame gap (120 ms -> 30 m).
    expect(Math.max(...st)).toBeLessThan(60);
    // The drawn position never passes the newest fix, nor falls far behind it.
    for (const f of cruise) {
      expect(f.x).toBeLessThanOrEqual(f.newest + 1e-6);
      expect(f.newest - f.x).toBeLessThan(400);
    }
  });

  it('speeds and slows with the aircraft, as flown', () => {
    const { frames } = play(flight({ count: 60, speedAt: (t) => (t < 6000 ? SPEED : SPEED / 5) }));
    expect(Math.min(...steps(frames.filter((f) => f.now > 2000)))).toBeGreaterThanOrEqual(0);
  });

  it('draws a real reversal by the aircraft as a reversal', () => {
    const { frames } = play(flight({ count: 60, speedAt: (t) => (t < 6000 ? SPEED : -SPEED) }));
    const xs = frames.map((f) => f.x);
    const peak = Math.max(...xs);
    expect(xs[xs.length - 1]).toBeLessThan(peak - 500);
  });

  it('comes to rest exactly on the newest fix when messages stop', () => {
    const msgs = flight({ count: 30 });
    const { frames, ents } = play(msgs, { endMs: msgs[29].arrival + 2000 });
    const last = frames[frames.length - 1];
    expect(last.x).toBeCloseTo(msgs[29].v.lon * M_PER_DEG, 6);
    expect(playoutMoving(playoutsOf(ents)[1], last.now + FRAME_MS)).toBe(false);
    // And slows to it, no stop from full speed.
    const tail = steps(frames.filter((f) => f.now > msgs[29].arrival));
    expect(Math.min(...tail)).toBeGreaterThanOrEqual(0);
    for (let i = 1; i < tail.length; i++) expect(tail[i]).toBeLessThanOrEqual(tail[i - 1] + 0.5);
  });

  it('drops a message older than the last one', () => {
    const msgs = flight({ count: 3 });
    const ents = {};
    ingestVehicle(ents, msgs[0].v, true, msgs[0].ts, 0);
    ingestVehicle(ents, msgs[2].v, true, msgs[2].ts, 10);
    const pb = playoutsOf(ents)[1];
    const n = pb.fixes.length;
    ingestVehicle(ents, msgs[1].v, true, msgs[1].ts, 20);
    expect(pb.fixes.length).toBe(n);
  });

  it('takes a snapshot once, whoever calls first', () => {
    const [m] = flight({ count: 1 });
    const ents = {};
    ingestVehicle(ents, m.v, true, m.ts, 0);
    const pb = playoutsOf(ents)[1];
    const n = pb.fixes.length;
    expect(ingestVehicle(ents, m.v, true, m.ts, 0)).toBe(pb);
    expect(pb.fixes.length).toBe(n);
  });

  it('a path kept by the store adds nothing; no path adds the snapshot position', () => {
    const ents = {};
    const path = [{ lat: 0, lon: 0.001, alt_rel: 10 }];
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0.001, alt_rel: 10, path }, true, 1000, 0);
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0.002, alt_rel: 10, path }, true, 1200, 200);
    const pb = playoutsOf(ents)[1];
    expect(pb.fixes.map((f) => f.lon)).toEqual([0.001]);
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0.003, alt_rel: 10 }, true, 1400, 400);
    expect(pb.fixes.map((f) => f.lon)).toEqual([0.001, 0.003]);
  });

  it('starts a first-connect snapshot (empty path) for the model only', () => {
    const ents = {};
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0.001, alt_rel: 10, path: [] }, true, 1000, 0);
    const pb = playoutsOf(ents)[1];
    expect(pb.fixes).toHaveLength(1);
    expect(pb.fixes[0].trail).toBe(false);
    expect(positionSample(pb, playClock(pb, 0)).lon).toBe(0.001);
  });

  it('never moves back when the first path holds fixes behind the first-connect snapshot', () => {
    // send_initial_state sends the current position, then the next broadcast
    // carries the fixes buffered before it: 30 m and 40 m lie behind or on it.
    const m = (x) => ({ lat: 0, lon: x / M_PER_DEG, alt_rel: 100 });
    const ents = {};
    ingestVehicle(ents, { sys_id: 1, ...m(40), path: [] }, true, 1000, 0);
    const pb = playoutsOf(ents)[1];
    const xs = [positionSample(pb, playClock(pb, 0)).lon * M_PER_DEG];
    ingestVehicle(ents, { sys_id: 1, ...m(60), path: [m(30), m(40), m(50), m(60)] }, true, 1200, 200);
    for (let now = 200; now < 1500; now += FRAME_MS) xs.push(positionSample(pb, playClock(pb, now)).lon * M_PER_DEG);
    ingestVehicle(ents, { sys_id: 1, ...m(110), path: [m(70), m(80), m(90), m(100), m(110)] }, true, 1400, 1500);
    for (let now = 1500; now < 5000; now += FRAME_MS) xs.push(positionSample(pb, playClock(pb, now)).lon * M_PER_DEG);
    expect(Math.min(...steps(xs.map((x) => ({ x }))))).toBeGreaterThanOrEqual(0);
    expect(xs.at(-1)).toBeCloseTo(110, 6);
    expect(pb.fixes.some((f) => f.trail === false)).toBe(false);
  });

  it('keeps turning when position fixes stop but attitudes go on', () => {
    const ents = {};
    const p = { sys_id: 1, lat: 0, lon: 0, alt_rel: 100, roll: 0, pitch: 0 };
    ingestVehicle(ents, { ...p, heading: 0, path: [{ lat: 0, lon: 0, alt_rel: 100 }] }, true, 1000, 0);
    const pb = ingestVehicle(ents, { ...p, heading: 90, path: [] }, true, 1200, 200);
    expect(attitudeSample(pb, playClock(pb, 2000)).heading).toBeCloseTo(90, 6);
    expect(positionSample(pb, playClock(pb, 2000)).lon).toBe(0);
  });

  it('skips a fix without altitude, never drawing it at 0', () => {
    const ents = {};
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0, alt_rel: 100 }, true, 1000, 0);
    const pb = ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0.0001, alt_rel: null, alt: null }, true, 1200, 200);
    expect(pb.fixes.map((f) => f.alt)).toEqual([100]);
    expect(positionSample(pb, playClock(pb, 2000)).alt).toBe(100);
  });

  it('restarts at the first fresh sample after a gap, not gliding across it', () => {
    const msgs = flight({ count: 40 });
    const ents = {};
    for (const m of msgs.slice(0, 10)) ingestVehicle(ents, m.v, true, m.ts, m.arrival);
    ingestVehicle(ents, msgs[10].v, false, msgs[10].ts, msgs[10].arrival);
    expect(playoutsOf(ents)[1].gap).toBe(true);
    const m = msgs[30];
    const pb = ingestVehicle(ents, m.v, true, m.ts, m.arrival);
    expect(pb.gap).toBe(false);
    const x = positionSample(pb, playClock(pb, m.arrival)).lon * M_PER_DEG;
    // Within the new message's own fixes (one interval of flight), not back at message 10.
    expect(x).toBeGreaterThan(m.v.lon * M_PER_DEG - 60);
  });

  it('a jump faster than any flight (a SITL restart) moves the model at once and marks it', () => {
    const msgs = flight({ count: 20 });
    const ents = {};
    for (const m of msgs) ingestVehicle(ents, m.v, true, m.ts, m.arrival);
    const ts = msgs[19].ts + 200;
    // 900 m back home in one 20 ms fix.
    const home = { lat: 0, lon: 0, alt_rel: 0 };
    const pb = ingestVehicle(ents, { sys_id: 1, ...home, path: [home] }, true, ts, msgs[19].arrival + 200);
    expect(900 / 0.02).toBeGreaterThan(JUMP_SPEED_MPS);
    expect(pb.fixes.at(-1).jump).toBe(true);
    expect(pb.fixes.at(-2).jump).toBeUndefined(); // the fixes before stay, for the trail
    expect(positionSample(pb, playClock(pb, msgs[19].arrival + 216)).lon).toBe(0);
  });

  it('catches up forward after a stall, never backward', () => {
    const msgs = flight({ count: 40 });
    const ents = {};
    let pb;
    for (const m of msgs.slice(0, 10)) pb = ingestVehicle(ents, m.v, true, m.ts, m.arrival);
    const before = playClock(pb, msgs[9].arrival);
    // The page stalls for 5 s while messages pile up, then takes them all.
    const late = msgs[9].arrival + STALL_MS + 2000;
    for (const m of msgs.slice(10, 40)) pb = ingestVehicle(ents, m.v, true, m.ts, late);
    const after = playClock(pb, late + FRAME_MS);
    expect(after).toBeGreaterThan(before);
    expect(msgs[39].ts - after).toBeLessThanOrEqual(STALL_MS);
  });

  it('turns the attitude the short way across north', () => {
    const ents = {};
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0, alt_rel: 0, heading: 350, roll: 0, pitch: 0 }, true, 1000, 0);
    const pb = ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0.0001, alt_rel: 0, heading: 10, roll: 0, pitch: 0 }, true, 1200, 200);
    expect(attitudeSample(pb, 1100).heading % 360).toBeCloseTo(0, 6);
  });

  it('fades a new ground height on its own clock; the first applies at once', () => {
    const ents = {};
    const pb = ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0, alt_rel: 0 }, true, 1000, 0);
    setGround(pb, 50, 0);
    expect(groundAt(pb, 0)).toBe(50);
    setGround(pb, 150, 100);
    expect(groundAt(pb, 200)).toBeCloseTo(100, 6);
    expect(groundAt(pb, 400)).toBe(150);
  });

  it('prunes UAVs that left', () => {
    const ents = {};
    ingestVehicle(ents, { sys_id: 1, lat: 0, lon: 0, alt_rel: 0 }, true, 1000, 0);
    ingestVehicle(ents, { sys_id: 2, lat: 0, lon: 0, alt_rel: 0 }, true, 1000, 0);
    prunePlayouts(ents, new Set([2]));
    expect(Object.keys(playoutsOf(ents))).toEqual(['2']);
  });
});
