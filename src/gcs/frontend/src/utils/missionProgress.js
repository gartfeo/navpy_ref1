import { flatDist } from './geo';

/**
 * The track itself when every point carries `seq`, its item number in the
 * vehicle's mission (a downloaded mission: the backend's mission_sequence),
 * else null. A track the GCS planned, or one edited since the download, has
 * no `seq` on every point; its items follow the planner's layout
 * (missionWpOffset).
 */
function trackWithSeqs(track) {
  return track?.length && track.every((p) => Number.isInteger(p?.seq)) ? track : null;
}

/**
 * Mission item number of the zone's first track point: the vehicle's own
 * numbering when the track came from the vehicle, else the planner's offset.
 */
export function zoneWpOffset(track, planOffset) {
  return trackWithSeqs(track) ? track[0].seq : planOffset;
}

/**
 * Index in `track` of the waypoint the vehicle flies to, from its mission
 * progress (the current mission item): negative before the track,
 * track.length past it. With the vehicle's own numbering a mission made
 * elsewhere (no GCS metadata items) maps right; the planner's offset counts
 * metadata items that mission does not have.
 */
export function missionTrackIndex(missionProgress, track, planOffset) {
  const mp = missionProgress || 0;
  if (!trackWithSeqs(track)) return mp - planOffset;
  if (mp < track[0].seq) return mp - track[0].seq;
  const i = track.findIndex((p) => p.seq >= mp);
  return i < 0 ? track.length : i;
}

/** The track without the vehicle's item numbers, once the GCS has uploaded its own layout. */
export function withoutSeqs(track) {
  return (track || []).map(({ seq, ...p }) => p); // eslint-disable-line no-unused-vars
}

/**
 * Compute cumulative distances along each zone's track.
 * Returns an array of { cumulative: number[], total: number, track: LatLon[] }
 * for each zone, or [] if plan has no zones.
 */
export function computeZoneDistances(plan) {
  if (!plan?.zones) return [];
  return plan.zones.map((zone) => {
    if (!zone.track || zone.track.length < 2) return { cumulative: [], total: 0, track: [] };
    const cumulative = [0];
    for (let j = 1; j < zone.track.length; j++) {
      const a = zone.track[j - 1], b = zone.track[j];
      cumulative.push(cumulative[j - 1] + flatDist(a, b));
    }
    return { cumulative, total: cumulative[cumulative.length - 1], track: zone.track };
  });
}

/**
 * Interpolate distance along a zone track using vehicle position on current segment.
 * Returns distance in metres.
 */
export function interpolatedDistance(v, zd, wpOffset) {
  if (!zd || !zd.track || zd.track.length < 2) return 0;
  const trackIdx = missionTrackIndex(v.mission_progress, zd.track, wpOffset);
  if (trackIdx <= 0) return 0;
  const prevIdx = Math.max(0, Math.min(trackIdx - 1, zd.track.length - 1));
  const nextIdx = Math.min(trackIdx, zd.track.length - 1);
  const baseDist = zd.cumulative[prevIdx] || 0;
  if (prevIdx === nextIdx || v.lat == null || v.lon == null) return baseDist;
  const a = zd.track[prevIdx], b = zd.track[nextIdx];
  const dx = b.lon - a.lon, dy = b.lat - a.lat;
  const len2 = dx * dx + dy * dy;
  let t = len2 > 0 ? ((v.lon - a.lon) * dx + (v.lat - a.lat) * dy) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const segDist = (zd.cumulative[nextIdx] || 0) - baseDist;
  return baseDist + t * segDist;
}
