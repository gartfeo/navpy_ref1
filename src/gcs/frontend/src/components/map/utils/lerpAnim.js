/**
 * Animation helpers for smooth entity motion.
 *
 * lerpFraction / LERP_DURATION — generic fixed-window lerp helper.
 *
 * advanceAnim / lerpPosition / slerpOrientation — used by useUavMarkers
 *   for smooth UAV model interpolation.  Duration adapts to an EMA-smoothed
 *   telemetry interval (× 3) so the entity is always mid-animation when
 *   the next sample arrives — it never pauses.  The EMA (α = 0.3) filters
 *   out jitter from individual late/early packets.
 */

/** Animation duration (ms) for trail / coverage layers. */
export const LERP_DURATION = 200;


/**
 * Compute the interpolation fraction for the current frame.
 * @param {number} startTime  Animation start timestamp (Date.now())
 * @returns {number} 0..1
 */
export function lerpFraction(startTime) {
  return Math.min((Date.now() - startTime) / LERP_DURATION, 1.0);
}

/**
 * Compute the current interpolated position from an animation state.
 * @param {object} anim      { fromPos, toPos, startTime, duration }
 * @param {object} Cesium    CesiumJS namespace
 * @param {object} scratch   Reusable Cartesian3 to avoid allocations
 * @returns {Cartesian3}
 */
export function lerpPosition(anim, Cesium, scratch) {
  const t = Math.min((Date.now() - anim.startTime) / anim.duration, 1.0);
  return Cesium.Cartesian3.lerp(anim.fromPos, anim.toPos, t, scratch);
}

/**
 * Compute the current interpolated orientation from an animation state.
 * @param {object} anim      { fromOri, toOri, startTime, duration }
 * @param {object} Cesium    CesiumJS namespace
 * @param {object} scratch   Reusable Quaternion to avoid allocations
 * @returns {Quaternion}
 */
export function slerpOrientation(anim, Cesium, scratch) {
  const t = Math.min((Date.now() - anim.startTime) / anim.duration, 1.0);
  return Cesium.Quaternion.slerp(anim.fromOri, anim.toOri, t, scratch);
}

/**
 * Update animation state for a new telemetry sample.
 * Captures the current interpolated position as the new "from" so the
 * transition to the new target is seamless even if the previous animation
 * hadn't finished.
 *
 * @param {object|undefined} prev     Previous animation state (may be undefined)
 * @param {Cartesian3}       toPos    New target position
 * @param {Quaternion}       toOri    New target orientation
 * @param {object}           Cesium   CesiumJS namespace
 * @returns {{ fromPos, toPos, fromOri, toOri, startTime, duration, emaInterval }}
 */
export function advanceAnim(prev, toPos, toOri, Cesium) {
  const now = Date.now();
  if (!prev) {
    return { fromPos: toPos, toPos, fromOri: toOri, toOri, startTime: now, duration: 600, emaInterval: 200 };
  }
  const interval = now - prev.startTime;
  const emaInterval = prev.emaInterval * 0.7 + interval * 0.3;
  const duration = Math.min(Math.max(emaInterval * 3, 100), 2000);
  const t = Math.min((now - prev.startTime) / prev.duration, 1.0);
  const fromPos = Cesium.Cartesian3.lerp(prev.fromPos, prev.toPos, t, new Cesium.Cartesian3());
  const fromOri = Cesium.Quaternion.slerp(prev.fromOri, prev.toOri, t, new Cesium.Quaternion());
  return { fromPos, toPos, fromOri, toOri, startTime: now, duration, emaInterval };
}
