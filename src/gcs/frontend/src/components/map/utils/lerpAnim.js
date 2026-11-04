/**
 * Animation helpers for smooth entity motion.
 *
 * lerpFraction / LERP_DURATION — generic fixed-window lerp helper.
 *
 * advanceAnim / setGround / positionAt / orientationAt — used by
 *   useUavMarkers for smooth UAV model motion by dead reckoning: from each
 *   telemetry sample the model moves on at the speed of the last two
 *   samples, and the difference between where it was drawn and the new
 *   sample fades out (smoothstep, so the speed has no jump when the fade
 *   ends) over one telemetry interval (an EMA, α = 0.3, of the
 *   arrival intervals, so one late or early packet does not change it much).
 *   The model therefore stays at the newest sample, the trail head, while
 *   flying, ahead of it by about the distance flown in one interval, and
 *   never pauses between samples. Interpolating towards each sample instead
 *   keeps the model behind it: with a window of 3 intervals, about 3
 *   intervals of flight (todo 261: 70-155 m at SITL speedup 10).
 *
 *   Samples are heights over home; the ground under the UAV is a separate
 *   height that fades on its own (setGround), so a new terrain sample never
 *   reads as a climb or a descent.
 */

/** Animation duration (ms) for trail / coverage layers. */
export const LERP_DURATION = 200;

/** Assumed telemetry interval (ms) before two samples have arrived: the 5 Hz broadcast. */
const INITIAL_INTERVAL_MS = 200;
/** Bounds (ms) of the correction window, so a burst or a long stall cannot make it 0 or endless. */
const MIN_WINDOW_MS = 100;
const MAX_WINDOW_MS = 2000;
/**
 * Windows a UAV whose samples stop (it stopped and is no longer broadcast,
 * or its link was lost) takes to ease back from where it came to rest onto
 * its newest sample, where its trail ends. Long enough that the move back is
 * slow (at most about half the flight speed).
 */
export const SETTLE_WINDOWS = 4;


/**
 * Compute the interpolation fraction for the current frame.
 * @param {number} startTime  Animation start timestamp (Date.now())
 * @returns {number} 0..1
 */
export function lerpFraction(startTime) {
  return Math.min((Date.now() - startTime) / LERP_DURATION, 1.0);
}

/**
 * Time (ms) of flight the dead reckoning has added to the newest sample.
 * Full speed for one window (the next sample is due), then slowing evenly to
 * rest over a second (a late packet), at 1.5 windows; with still no sample,
 * a smooth ease back to the sample over SETTLE_WINDOWS. The speed never
 * jumps.
 */
function coastTime(anim, elapsed) {
  const w = anim.duration;
  if (elapsed <= w) return elapsed;
  if (elapsed <= 2 * w) {
    const s = elapsed - w;
    return w + s - (s * s) / (2 * w);
  }
  const u = Math.min((elapsed - 2 * w) / (SETTLE_WINDOWS * w), 1);
  return 1.5 * w * (1 - smoothstep(u));
}

/**
 * Whether the model is between the window its next sample was due in and
 * coming to rest: samples have stopped or are late, so it slows and eases
 * back on its own, with no telemetry to request a render (requestRenderMode).
 */
export function animSettling(anim, now = Date.now()) {
  const elapsed = now - anim.startTime;
  return elapsed > anim.duration && !animAtRest(anim, now);
}

/** Whether the model has come to rest on its last sample (coastTime is 0). */
export function animAtRest(anim, now = Date.now()) {
  return now - anim.startTime >= (2 + SETTLE_WINDOWS) * anim.duration;
}

/** 0..1 eased with zero slope at both ends. */
function smoothstep(u) {
  return u * u * (3 - 2 * u);
}

/** The ground height under the UAV at `now`, on its own fade (setGround). */
function groundAt(anim, now) {
  if (anim.groundTo === undefined) return 0;
  const t = Math.min(Math.max(0, now - anim.groundStart) / anim.groundDuration, 1.0);
  return anim.groundFrom + (anim.groundTo - anim.groundFrom) * t;
}

/**
 * The drawn position of an animation at `now`.
 * @param {object} anim      advanceAnim state
 * @param {object} Cesium    CesiumJS namespace
 * @param {Cartesian3} result Reusable Cartesian3 to avoid allocations
 * @param {number} [now]     Timestamp (Date.now())
 * @returns {Cartesian3}
 */
export function positionAt(anim, Cesium, result, now = Date.now()) {
  const C3 = Cesium.Cartesian3;
  const elapsed = Math.max(0, now - anim.startTime);
  const fade = 1 - smoothstep(Math.min(elapsed / anim.duration, 1));
  C3.multiplyByScalar(anim.vel, coastTime(anim, elapsed), result);
  C3.add(anim.toPos, result, result);
  // result += (fromPos - toPos) * fade + up * ground, without allocating.
  const e = anim.error;
  const g = groundAt(anim, now);
  const up = anim.up;
  result.x += e.x * fade + up.x * g;
  result.y += e.y * fade + up.y * g;
  result.z += e.z * fade + up.z * g;
  return result;
}

/**
 * The drawn orientation of an animation at `now`.
 * @param {object} anim      advanceAnim state
 * @param {object} Cesium    CesiumJS namespace
 * @param {Quaternion} result Reusable Quaternion to avoid allocations
 * @param {number} [now]     Timestamp (Date.now())
 * @returns {Quaternion}
 */
export function orientationAt(anim, Cesium, result, now = Date.now()) {
  const t = Math.min(Math.max(0, now - anim.startTime) / anim.duration, 1.0);
  return Cesium.Quaternion.slerp(anim.fromOri, anim.toOri, t, result);
}

/**
 * Update animation state for a new telemetry sample. The model starts from
 * where it is drawn now, so the motion has no jump.
 *
 * @param {object|undefined} prev     Previous animation state (may be undefined)
 * @param {Cartesian3}       toPos    New sample position, height over home (no ground)
 * @param {Quaternion}       toOri    New sample orientation
 * @param {object}           Cesium   CesiumJS namespace
 * @param {number}           [now]    Timestamp (Date.now())
 * @returns {object} animation state
 */
export function advanceAnim(prev, toPos, toOri, Cesium, now = Date.now()) {
  const C3 = Cesium.Cartesian3;
  // Up at the sample: adding h along it raises the height by exactly h.
  const up = Cesium.Ellipsoid.WGS84.geodeticSurfaceNormal(toPos, new C3());
  if (!prev) {
    return {
      toPos, up, vel: new C3(0, 0, 0), error: new C3(0, 0, 0), fromOri: toOri, toOri,
      startTime: now, duration: INITIAL_INTERVAL_MS, emaInterval: INITIAL_INTERVAL_MS,
    };
  }
  const interval = now - prev.startTime;
  const emaInterval = prev.emaInterval * 0.7 + interval * 0.3;
  const duration = Math.min(Math.max(emaInterval, MIN_WINDOW_MS), MAX_WINDOW_MS);
  const fromOri = orientationAt(prev, Cesium, new Cesium.Quaternion(), now);
  // The drawn position without the ground, which keeps its own fade.
  const fromPos = positionAt(prev, Cesium, new C3(), now);
  C3.subtract(fromPos, C3.multiplyByScalar(prev.up, groundAt(prev, now), new C3()), fromPos);
  // Speed over the longer of the measured and the usual interval: a sample
  // that arrives early in a burst would otherwise read as a burst of speed.
  const vel = C3.subtract(toPos, prev.toPos, new C3());
  C3.multiplyByScalar(vel, 1 / Math.max(interval, emaInterval, 1), vel);
  const error = C3.subtract(fromPos, toPos, new C3());
  return {
    toPos, up, vel, error, fromOri, toOri, startTime: now, duration, emaInterval,
    // The ground fade goes on untouched.
    groundFrom: prev.groundFrom, groundTo: prev.groundTo,
    groundStart: prev.groundStart, groundDuration: prev.groundDuration,
  };
}

/**
 * A new ground height under the UAV: the model fades to it over one window,
 * the first one applies at once. Motion timing and speed are not touched.
 */
export function setGround(anim, height, now = Date.now()) {
  const groundFrom = anim.groundTo === undefined ? height : groundAt(anim, now);
  return { ...anim, groundFrom, groundTo: height, groundStart: now, groundDuration: anim.duration };
}
