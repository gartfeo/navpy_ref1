/**
 * What the Synthetic view marks in its 3D scene besides the UAV's own route
 * (docs/design/gcs_video_streams.md, section 5 "Synthetic view"; research in
 * docs/design/synthetic_view_marks.md): the dock the UAV works on, its
 * default delivery hub (DDH), the rest of the fleet, and where it is going
 * (the flight path marker). Display only: never a navigation input.
 */
import { ASSIGNMENT_STATUS, vehicleAssignment } from './taskAssignmentState';
import { MIN_ETE_SPEED_MPS } from './fpvRoute';

/**
 * The dock the UAV is assigned to or approaching, as its card shows it
 * (vehicleAssignment), or null. A handled dock is no longer the UAV's POI.
 *
 * @returns {{lat: number, lon: number}|null}
 */
export function syntheticDock(assignments, sysId) {
  const entry = vehicleAssignment(assignments || {}, sysId);
  if (!entry || entry.status === ASSIGNMENT_STATUS.HANDLED) return null;
  if (!Number.isFinite(entry.lat) || !Number.isFinite(entry.lon)) return null;
  return { lat: entry.lat, lon: entry.lon };
}

/**
 * The default delivery hub assigned to the UAV's zone, or null. A UAV's zone
 * is its index in the fleet list, as on the main map (useTrackLayer, routeFor).
 *
 * @param {object[]} vehicleList
 * @param {object[]} deliveryHubs - settings.default_delivery_hubs.
 * @param {Array<number|null>} deliveryHubAssignments - per zone, an index into deliveryHubs.
 * @returns {{lat: number, lon: number, type?: string}|null}
 */
export function syntheticHub(vehicleList, deliveryHubs, deliveryHubAssignments, sysId) {
  const zone = (vehicleList || []).findIndex((v) => v?.sys_id === sysId);
  if (zone < 0) return null;
  const hub = (deliveryHubs || [])[(deliveryHubAssignments || [])[zone]];
  if (!hub || !Number.isFinite(hub.lat) || !Number.isFinite(hub.lon)) return null;
  return { lat: hub.lat, lon: hub.lon, ...(hub.type && { type: hub.type }) };
}

/**
 * Time constant of the flight path marker's smoothing. The research behind the
 * marker (FAA AC 23-26, sections 2d and o(1)) warns that an undamped marker
 * makes the eye chase it; at 5 Hz telemetry one second smooths gusts and GPS
 * noise and still follows a turn within a few seconds.
 */
export const FPM_SMOOTHING_S = 1;

/**
 * The UAV's measured ground velocity as east, north, up m/s, or null below
 * MIN_ETE_SPEED_MPS of ground speed (a parked or hovering UAV is going
 * nowhere) or without finite numbers.
 *
 * @param {number[]|null|undefined} velocityNed - the snapshot's velocity_ned.
 */
export function velocityEnu(velocityNed) {
  if (!Array.isArray(velocityNed) || velocityNed.length !== 3) return null;
  const [n, e, d] = velocityNed;
  if (![n, e, d].every(Number.isFinite)) return null;
  if (Math.hypot(n, e) < MIN_ETE_SPEED_MPS) return null;
  return [e, n, -d];
}

/**
 * First-order smoothing of a vector toward `next` over `dtS` seconds; `prev`
 * null (or a gap of no samples) starts at `next`.
 */
export function smoothVector(prev, next, dtS, tauS = FPM_SMOOTHING_S) {
  if (!next) return null;
  if (!prev || !(dtS > 0)) return next;
  const a = 1 - Math.exp(-dtS / tauS);
  return prev.map((p, i) => p + a * (next[i] - p));
}
