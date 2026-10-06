/**
 * Minimum-cost fallback location-to-zone assignment via bitmask DP.
 *
 * Finds the globally optimal assignment that minimises total distance,
 * which guarantees no crossing assignment lines (crossing edges always
 * have higher total cost than the uncrossed alternative).
 */

const DEG2RAD = Math.PI / 180;
const R = 6371000; // Earth radius in meters

/**
 * Haversine distance in meters between two {lat, lon} points.
 */
function haversine(a, b) {
  const dLat = (b.lat - a.lat) * DEG2RAD;
  const dLon = (b.lon - a.lon) * DEG2RAD;
  const sinLat = Math.sin(dLat / 2);
  const sinLon = Math.sin(dLon / 2);
  const h = sinLat * sinLat +
    Math.cos(a.lat * DEG2RAD) * Math.cos(b.lat * DEG2RAD) * sinLon * sinLon;
  return 2 * R * Math.asin(Math.sqrt(h));
}

/**
 * Get the last track point of a zone (the end-of-search position).
 * @param {object} zone - Zone with .track array of {lat, lon}
 * @returns {{lat: number, lon: number}|null}
 */
function zoneEndPoint(zone) {
  if (!zone?.track?.length) return null;
  const last = zone.track[zone.track.length - 1];
  return { lat: last.lat, lon: last.lon };
}

/** Count set bits in a 32-bit integer. */
function popcount(x) {
  x -= (x >> 1) & 0x55555555;
  x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
  return (((x + (x >> 4)) & 0x0f0f0f0f) * 0x01010101) >> 24;
}

/**
 * Minimum-cost bipartite matching via bitmask DP.
 *
 * Assigns k row-items to k distinct column-items chosen from m columns,
 * minimising total cost. Row i is assigned in step i (i = popcount of mask).
 *
 * @param {number[][]} costs - cost[row][col], dimensions k × m  (m >= k)
 * @param {number} k - number of rows to assign
 * @param {number} m - number of columns available
 * @returns {number[]} result[row] = column index, or null if no valid assignment
 */
function dpAssign(costs, k, m) {
  const size = 1 << m;
  const dp = new Float64Array(size);
  dp.fill(Infinity);
  const parent = new Int8Array(size);
  parent.fill(-1);
  dp[0] = 0;

  for (let mask = 0; mask < size; mask++) {
    const row = popcount(mask);
    if (row >= k || dp[mask] === Infinity) continue;
    for (let col = 0; col < m; col++) {
      if (mask & (1 << col)) continue;
      const next = mask | (1 << col);
      const c = dp[mask] + costs[row][col];
      if (c < dp[next]) {
        dp[next] = c;
        parent[next] = col;
      }
    }
  }

  // Find the best completed mask (exactly k bits set)
  let bestMask = -1;
  let bestCost = Infinity;
  for (let mask = 0; mask < size; mask++) {
    if (popcount(mask) === k && dp[mask] < bestCost) {
      bestCost = dp[mask];
      bestMask = mask;
    }
  }

  const result = new Array(k).fill(null);
  if (bestMask >= 0) {
    let mask = bestMask;
    for (let i = k - 1; i >= 0; i--) {
      result[i] = parent[mask];
      mask ^= (1 << parent[mask]);
    }
  }
  return result;
}

/**
 * Auto-assign fallback locations to zones — globally optimal unique assignments,
 * sharing only when there are more zones tha fallback locations.
 *
 * @param {Array<object>} zones - Plan zones with .track arrays
 * @param {Array<{name, type, lat, lon}>} fallbackLocations - Fallback delivery locations
 * @returns {Array<number|null>} assignments[zoneIdx] = fallbackLocationIndex or null
 */
export function autoAssignFallbackLocations(zones, fallbackLocations) {
  if (!zones?.length || !fallbackLocations?.length) return zones?.map(() => null) || [];

  const n = zones.length;
  const m = fallbackLocations.length;
  const assignments = new Array(n).fill(null);

  // Build cost matrix for valid zones only (zones with track endpoints)
  const validIdx = [];
  const costs = [];
  for (let zi = 0; zi < n; zi++) {
    const ep = zoneEndPoint(zones[zi]);
    if (!ep) continue;
    validIdx.push(zi);
    costs.push(fallbackLocations.map((location) => haversine(ep, location)));
  }
  const vn = validIdx.length;
  if (vn === 0) return assignments;

  if (vn <= m) {
    // Enough fallback locations for unique assignment — DP mask over fallback locations
    const result = dpAssign(costs, vn, m);
    for (let i = 0; i < vn; i++) assignments[validIdx[i]] = result[i];
    return assignments;
  }

  // More zones tha fallback locations — assign m fallback locations uniquely to m zones (optimal),
  // then remaining zones get nearest fallback location (shared).
  // Transpose: rows = fallback locations, cols = valid zones, DP mask over zones.
  const transposed = [];
  for (let oi = 0; oi < m; oi++) {
    transposed.push(validIdx.map((_, vi) => costs[vi][oi]));
  }
  const picked = dpAssign(transposed, m, vn);
  for (let oi = 0; oi < m; oi++) {
    if (picked[oi] !== null) assignments[validIdx[picked[oi]]] = oi;
  }

  // Remaining valid zones: nearest fallback location (shared)
  for (let i = 0; i < vn; i++) {
    const zi = validIdx[i];
    if (assignments[zi] !== null) continue;
    let bestFallbackLocation = -1;
    let bestDist = Infinity;
    for (let oi = 0; oi < m; oi++) {
      if (costs[i][oi] < bestDist) {
        bestDist = costs[i][oi];
        bestFallbackLocation = oi;
      }
    }
    if (bestFallbackLocation >= 0) assignments[zi] = bestFallbackLocation;
  }

  return assignments;
}

/**
 * Build fallback locations and assignments from downloaded fallback_delivery_location metadata.
 *
 * Deduplicates POIs within 1e-7 degrees (~0.01 m) so that zones sharing
 * the same POI get a single fallback location entry with multiple assignment references.
 *
 * When existingFallbackLocations is provided, downloaded POIs are matched against them
 * by coordinates — preserving name/type for matches and keeping unmatched
 * existing fallback locations intact. New POIs that don't match any existing fallback location are
 * appended with the downloaded type when present, else 'other'.
 *
 * @param {Array<{lat: number, lon: number, type?: string}|null>} missionFallbackLocations - one per zone
 * @param {Array<{name, type, lat, lon}>} [existingFallbackLocations=[]] - current settings fallback locations
 * @returns {{ fallbackLocations: Array<{name: string, type: string, lat: number, lon: number}>,
 *             assignments: Array<number|null> }}
 */
export function buildFallbackLocationsFromDownload(missionFallbackLocations, existingFallbackLocations = []) {
  if (!missionFallbackLocations?.length) return { fallbackLocations: existingFallbackLocations.map((o) => ({ ...o })), assignments: [] };

  const EPS = 1e-7;
  const fallbackLocations = existingFallbackLocations.map((o) => ({ ...o }));
  const assignments = missionFallbackLocations.map((t) => {
    if (!t || t.lat == null || t.lon == null) return null;
    const existing = fallbackLocations.findIndex(
      (o) => Math.abs(o.lat - t.lat) < EPS && Math.abs(o.lon - t.lon) < EPS,
    );
    if (existing >= 0) {
      // Plan type takes priority over settings type
      if (typeof t.type === 'string' && t.type && t.type !== 'other') {
        fallbackLocations[existing] = { ...fallbackLocations[existing], type: t.type };
      }
      return existing;
    }
    fallbackLocations.push({
      name: `Fallback delivery location ${fallbackLocations.length + 1}`,
      type: typeof t.type === 'string' && t.type ? t.type : 'other',
      lat: t.lat,
      lon: t.lon,
    });
    return fallbackLocations.length - 1;
  });

  return { fallbackLocations, assignments };
}

/**
 * Get distance from a zone's last track point to a fallback location.
 * @param {object} zone - Zone with .track array
 * @param {{lat, lon}} location - Target fallback location
 * @returns {number} Distance in meters, or Infinity if no track
 */
export function zoneToFallbackLocationDistance(zone, location) {
  const ep = zoneEndPoint(zone);
  if (!ep) return Infinity;
  return haversine(ep, location);
}
