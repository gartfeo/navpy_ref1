/**
 * Pure rules of the FPV view mode (docs/design/gcs_video_streams.md, section 5
 * "FPV"). FPV is a synthetic Cesium nose view for orientation; it is operator
 * display only and never confirms anything.
 *
 * Eligibility is the backend's own `link_ok` flag (the one the bottom bar and
 * preflight use) plus a finite position and attitude in the snapshot. No new
 * freshness threshold.
 */

const finite = Number.isFinite;

/** Position and attitude the FPV camera needs are all finite numbers. */
export function poseFinite(v) {
  if (!v) return false;
  return finite(v.lat) && finite(v.lon) && finite(v.alt_rel ?? v.alt)
    && finite(v.roll) && finite(v.pitch) && finite(v.heading);
}

/**
 * Why the FPV camera must hold its last valid pose for this vehicle, or null
 * when the pose may be used. Link first: an unlinked vehicle's last pose is
 * stale even when its numbers are finite.
 */
export function fpvHoldReason(v) {
  if (!v || v.link_ok !== true) return 'linkLost';
  if (!poseFinite(v)) return 'poseUnavailable';
  return null;
}

export const fpvEligible = (v) => fpvHoldReason(v) === null;

/** The banner reason for a fleet entry: its own (null = live), or link lost when the UAV left the fleet. */
export const fleetEntryHoldReason = (entry) => (entry ? entry.holdReason : 'linkLost');

/**
 * Which UAV the FPV toggle puts on screen. `fleet` entries carry `sysId` and
 * `fpvEligible`.
 * - following a UAV: that UAV, or `{ ineligible }` naming it;
 * - Free: the camera panel's UAV if eligible, else the first eligible UAV;
 * - nothing eligible: `{ none: true }`.
 */
export function pickFpvTarget({ followSysId, panelSysId, fleet }) {
  const list = fleet || [];
  if (followSysId != null) {
    const followed = list.find((e) => e.sysId === followSysId);
    return followed?.fpvEligible ? { sysId: followSysId } : { ineligible: followSysId };
  }
  const panel = list.find((e) => e.sysId === panelSysId);
  if (panel?.fpvEligible) return { sysId: panelSysId };
  const first = list.find((e) => e.fpvEligible);
  return first ? { sysId: first.sysId } : { none: true };
}
