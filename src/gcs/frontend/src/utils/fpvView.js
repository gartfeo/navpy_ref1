/**
 * Pure rules of the synthetic first-person views (docs/design/
 * gcs_video_streams.md, section 5 "Synthetic view"): the vehicle window's
 * Synthetic source and RC FPV. Both are operator display only and never
 * confirm anything.
 *
 * A pose may be used when the telemetry socket is open and the vehicle's
 * snapshot was replaced since it opened, the backend's own `link_ok` flag (the
 * one the bottom bar and preflight use) is set, and the snapshot's position and
 * attitude are finite. No new freshness threshold.
 */

const finite = Number.isFinite;

/** Position and attitude a first-person camera needs are all finite numbers. */
export function poseFinite(v) {
  if (!v) return false;
  return finite(v.lat) && finite(v.lon) && finite(v.alt_rel ?? v.alt)
    && finite(v.roll) && finite(v.pitch) && finite(v.heading);
}

/**
 * Why a first-person view must hold its last valid pose for this vehicle, or
 * null when the pose may be used. Socket first, then link: a cached snapshot
 * is stale even when its numbers are finite.
 *
 * @param {object|undefined} v - the vehicle's telemetry snapshot.
 * @param {boolean} fresh - telemetryStore.isTelemetryFresh(sysId).
 */
export function fpvHoldReason(v, fresh) {
  if (!fresh) return 'telemetryDisconnected';
  if (!v || v.link_ok !== true) return 'linkLost';
  if (!poseFinite(v)) return 'poseUnavailable';
  return null;
}

/** The held reason for a fleet entry: its own (null = live), or link lost when the UAV left the fleet. */
export const fleetEntryHoldReason = (entry) => (entry ? entry.holdReason : 'linkLost');
