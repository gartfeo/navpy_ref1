/**
 * The first-person views' mission path (docs/design/gcs_video_streams.md,
 * section 5 "Synthetic view"): the viewed UAV's own route, from the same plan
 * and positions the main map draws (usePlanSync's track positions, useTrackLayer's
 * progress), and its next waypoint. Display only: never a navigation input.
 */
import { flatDist } from './geo';

/** Below this ground speed no time to the waypoint is shown (a hovering or parked UAV). */
export const MIN_ETE_SPEED_MPS = 1;

/**
 * The route of UAV `sysId`, or null without one.
 *
 * @param {number} sysId
 * @param {object} p
 * @param {object[]} p.vehicleList - the store's list; a UAV's index is its zone (as useTrackLayer).
 * @param {Array<object[]>} p.trackPositions - per zone, the track's Cartesian3 positions (usePlanSync).
 * @param {object|null} p.plan - zones[i].track: [{lat, lon, alt?}].
 * @param {object} p.progress - per zone, useTrackLayer's {trackIdx} (the next track
 *   waypoint), null before the track, or trackIdx past the end after it and
 *   outside AUTO. So outside AUTO there is no next waypoint: the UAV is not
 *   flying to one, and a time to it would mislead; AUTO again restores it.
 * @returns {{zone: number, positions: object[], track: object[], next: number|null}|null}
 */
export function routeFor(sysId, { vehicleList, trackPositions, plan, progress }) {
  const zone = (vehicleList || []).findIndex((v) => v?.sys_id === sysId);
  if (zone < 0) return null;
  const positions = trackPositions?.[zone];
  const track = plan?.zones?.[zone]?.track;
  if (!positions?.length || !track?.length || positions.length !== track.length) return null;
  const idx = progress?.[zone]?.trackIdx;
  const next = Number.isInteger(idx) && idx < track.length ? Math.max(0, idx) : null;
  return { zone, positions, track, next };
}

/**
 * The next waypoint readout: its number (1-based along the route), the
 * horizontal distance to it, and the time at the current ground speed.
 *
 * @returns {{number: number, distanceM: number, eteS: number|null}|null}
 */
export function nextWaypointInfo(route, vehicle) {
  if (!route || route.next == null || vehicle?.lat == null || vehicle?.lon == null) return null;
  const wp = route.track[route.next];
  const distanceM = flatDist({ lat: vehicle.lat, lon: vehicle.lon }, wp);
  if (!Number.isFinite(distanceM)) return null;
  const gs = vehicle.ground_speed;
  const eteS = Number.isFinite(gs) && gs >= MIN_ETE_SPEED_MPS ? distanceM / gs : null;
  return { number: route.next + 1, distanceM, eteS };
}

/** "420 m" below a kilometre, else "1.2 km". */
export function formatDistance(m) {
  return m < 1000 ? `${Math.round(m)} m` : `${(m / 1000).toFixed(1)} km`;
}

/** "00:21", "12:05", or "1:02:03" from an hour. */
export function formatEte(s) {
  const total = Math.max(0, Math.round(s));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const sec = total % 60;
  const two = (n) => String(n).padStart(2, '0');
  return h > 0 ? `${h}:${two(m)}:${two(sec)}` : `${two(m)}:${two(sec)}`;
}
