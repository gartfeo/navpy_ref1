import { flatDist } from './geo';

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
  const mp = v.mission_progress || 0;
  const trackIdx = mp - wpOffset;
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
