import { describe, it, expect } from 'vitest';
import * as Cesium from 'cesium';
import { advanceAnim, positionAt, setGround, SETTLE_WINDOWS } from './lerpAnim';

const { Cartesian3, Cartographic, Quaternion } = Cesium;
const ORI = Quaternion.IDENTITY;
const FRAME_MS = 16;
const HOME_ALT = 100;
// At lon 0, lat 0 east is +y: a sample `d` m east of the start.
const east = (d, alt = HOME_ALT) => {
  const p = Cartesian3.fromDegrees(0, 0, alt);
  p.y += d;
  return p;
};

/**
 * Fly a UAV east, one sample per arrival { t, d, ground? }, and draw a frame
 * every frameMs. Returns, per frame, whether a sample arrived in it, its distance from the newest sample (the
 * trail head; positive = model ahead), its step from the last frame and its
 * height over the newest sample's (the track is a straight line, so the
 * ellipsoid height alone would rise with distance).
 */
function fly(arrivals, endMs, frameMs = FRAME_MS) {
  let anim;
  let head = null;
  let next = 0;
  let prevY = null;
  const frames = [];
  const scratch = new Cartesian3();
  for (let now = 0; now <= endMs; now += frameMs) {
    let sampled = false;
    while (next < arrivals.length && arrivals[next].t <= now) {
      const a = arrivals[next];
      if (a.d !== undefined) {
        head = east(a.d);
        anim = advanceAnim(anim, head, ORI, Cesium, now);
        sampled = true;
      }
      if (a.ground !== undefined) anim = setGround(anim, a.ground, now);
      next += 1;
    }
    if (!anim) continue;
    const p = positionAt(anim, Cesium, scratch, now);
    frames.push({
      now,
      sampled,
      ahead: p.y - head.y,
      step: prevY == null ? 0 : p.y - prevY,
      height: Cartographic.fromCartesian(p).height - Cartographic.fromCartesian(head).height,
    });
    prevY = p.y;
  }
  return frames;
}

// Samples on the backend's regular 200 ms tick, optionally arriving off it.
function samples(count, speed, arrivalOf = (k) => k * 200) {
  return Array.from({ length: count }, (_, k) => ({ t: arrivalOf(k), d: speed * k * 0.2 }));
}

describe('UAV model motion (todo 261)', () => {
  // SITL speedup 10 at 25 m/s: 250 m/s of wall time, 50 m per 5 Hz sample.
  const SPEED = 250;
  const PER_SAMPLE = SPEED * 0.2;
  const PER_FRAME = SPEED * FRAME_MS / 1000;

  it('keeps the model at the trail head while flying, not intervals behind it', () => {
    const frames = fly(samples(60, SPEED), 59 * 200).filter((f) => f.now > 2000);
    const ahead = frames.map((f) => f.ahead);
    // The old 3-interval lerp drew it ~150 m (3 samples) behind. A few
    // metres behind come from arrivals rounded to the 16 ms frames.
    expect(Math.min(...ahead)).toBeGreaterThan(-0.1 * PER_SAMPLE);
    expect(Math.max(...ahead)).toBeLessThanOrEqual(PER_SAMPLE + 1);
  });

  it('moves smoothly forward with late and early packets', () => {
    // Arrival jitter of up to ±80 ms on a 200 ms cadence.
    const jitter = [0, 80, -60, 40, -80, 20, 70, -40];
    const arrivals = samples(60, SPEED, (k) => k * 200 + jitter[k % jitter.length] + 80);
    const frames = fly(arrivals, 59 * 200).filter((f) => f.now > 2000);
    const steps = frames.map((f) => f.step);
    // Forward, apart from a nudge back under a tenth of a frame's flight when
    // a late packet corrects the dead reckoning.
    expect(Math.min(...steps)).toBeGreaterThan(-0.1 * PER_FRAME);
    // No jump: a frame step stays within 2.5x the true per-frame distance.
    expect(Math.max(...steps)).toBeLessThan(2.5 * PER_FRAME);
    const ahead = frames.map((f) => Math.abs(f.ahead));
    expect(Math.max(...ahead)).toBeLessThan(2 * PER_SAMPLE);
  });

  it('slows smoothly when the samples stop, then eases back onto the last one', () => {
    const lastT = 19 * 200;
    const frames = fly(samples(20, SPEED), lastT + (2 + SETTLE_WINDOWS) * 200 + 400)
      .filter((f) => f.now > lastT);
    // No speed jump at any phase change: a frame's step changes by under
    // 3/8 of a frame's flight.
    for (let i = 1; i < frames.length; i++) {
      expect(Math.abs(frames[i].step - frames[i - 1].step)).toBeLessThan(1.5 * PER_FRAME / 4);
    }
    // At most 1.5 windows past the head, and back no faster than half speed.
    expect(Math.max(...frames.map((f) => f.ahead))).toBeLessThanOrEqual(1.5 * PER_SAMPLE + 1);
    expect(Math.min(...frames.map((f) => f.step))).toBeGreaterThan(-0.6 * PER_FRAME);
    const last = frames[frames.length - 1];
    expect(Math.abs(last.ahead)).toBeLessThan(1e-6);
    expect(last.step).toBe(0);
  });

  it('does not read one burst of two close packets as a jump in speed', () => {
    const arrivals = samples(30, SPEED, (k) => (k === 20 ? 20 * 200 + 190 : k * 200));
    const frames = fly(arrivals, 29 * 200).filter((f) => f.now > 2000);
    expect(Math.max(...frames.map((f) => f.ahead))).toBeLessThan(2 * PER_SAMPLE);
  });

  it('moves a stationary UAV to a new ground height without overshoot', () => {
    const frames = fly([
      { t: 0, d: 0, ground: 0 },
      { t: 200, d: 0 },
      { t: 400, ground: 100 },
    ], 2000).filter((f) => f.now >= 400);
    const heights = frames.map((f) => f.height);
    for (let i = 1; i < heights.length; i++) expect(heights[i]).toBeGreaterThanOrEqual(heights[i - 1] - 1e-6);
    expect(Math.max(...heights)).toBeLessThan(100 + 1e-3);
    expect(heights[heights.length - 1]).toBeCloseTo(100, 3);
    expect(Math.max(...frames.map((f) => Math.abs(f.ahead)))).toBeLessThan(1e-3);
  });

  it('keeps a new ground height out of the flight speed', () => {
    // Level flight; the ground under the UAV steps by 50 m at one sample.
    const arrivals = samples(30, SPEED).map((a, k) => (k === 15 ? { ...a, ground: 50 } : a));
    const frames = fly([...arrivals, { t: 0, ground: 0 }].sort((a, b) => a.t - b.t), 29 * 200)
      .filter((f) => f.now > 15 * 200);
    const heights = frames.map((f) => f.height);
    // Read as speed it would climb 50 m per sample; the centimetres left are
    // the straight track's rise over the ~50 m the model is ahead.
    expect(Math.max(...heights)).toBeLessThan(50.1);
    expect(heights[heights.length - 1]).toBeCloseTo(50, 1);
  });
  it('changes speed only when a sample arrives, from a standing start too', () => {
    // From rest into 250 m/s with jittered arrivals, drawn every 4 ms: between
    // samples the speed changes in steps that shrink with the frame time (a
    // linear fade ended in a jump of 1 m per 4 ms frame, 50 m / 200 ms).
    const frameMs = 4;
    const jitter = [0, 80, -60, 40, -80, 20, 70, -40];
    const arrivals = samples(30, SPEED, (k) => k * 200 + jitter[k % jitter.length] + 80);
    const frames = fly(arrivals, 29 * 200 + 400, frameMs);
    let worst = 0;
    for (let i = 2; i < frames.length; i++) {
      if (frames[i].sampled || frames[i - 1].sampled) continue;
      worst = Math.max(worst, Math.abs(frames[i].step - frames[i - 1].step));
    }
    expect(worst).toBeLessThan(0.3);
  });

  it('keeps a ground fade on its own clock when a sample arrives mid-fade', () => {
    const frames = fly([
      { t: 0, d: 0, ground: 0 },
      { t: 96, ground: 100 },
      { t: 192, d: 0 },
    ], 1000);
    // Window 200 ms: the fade from 96 ms is done by 296 ms despite the sample.
    const at = frames.find((f) => f.now >= 304);
    expect(at.height).toBeCloseTo(100, 6);
  });
});
