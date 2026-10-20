/**
 * Plan/camera compatibility check (docs/design/plan_camera_check.md).
 *
 * The backend (POST /api/plan-camera-check) does the camera math with NavPy's
 * own numbers. This module only builds the request from what the vehicles fly
 * and words the result. It holds no camera or detector constant.
 */
import { decodeBitmaskArray } from './bitmask';
import { zoneMissionWaypoints } from '../components/map/utils/dockResolver';

/** Altitude the upload sends for zone i (home-relative): its own, else the zone stack. */
export function zoneUploadAltitude(plan, zone, i) {
  if (zone.altitude_m != null) return zone.altitude_m;
  const n = plan.zones.length;
  return plan.altitude_m + (n - 1 - i) * (plan.altitude_separation_m ?? 20);
}

/**
 * One zone of the request, indexed the way the vehicle indexes its mission:
 * targ_wps bit (n-1) is waypoint n = waypoints[n-1]; detection runs once the
 * next item is past nav_last_wp, i.e. from waypoints[nav_last_wp-1] on.
 * dockAlt / useTerrain are the vehicle's AAS_TARG_ALT / AAS_USE_TRN, or
 * undefined when the GCS has not read them.
 */
export function cameraCheckZone({ zoneIndex, waypoints, altitude, targWps, navLastWp, dockAlt, useTerrain }) {
  const docks = decodeBitmaskArray(targWps || 0)
    .filter((n) => n - 1 < waypoints.length)
    .map((n) => ({ lat: waypoints[n - 1].lat, lon: waypoints[n - 1].lon ?? waypoints[n - 1].lng, wp: n }));
  const start = Math.max((navLastWp || 0) - 1, 0);
  return {
    zone_index: zoneIndex,
    altitude_m: altitude,
    dock_alt_m: typeof dockAlt === 'number' && Number.isFinite(dockAlt) ? dockAlt : null,
    use_terrain: typeof useTerrain === 'boolean' ? useTerrain : null,
    route: waypoints.slice(start).map((p) => ({ lat: p.lat, lon: p.lon ?? p.lng })),
    docks,
  };
}

/**
 * Request zones for the plan the GCS holds (after an upload or a mission
 * download), with per-vehicle targ_wps / nav_last_wp maps and a confirmed
 * AAS-param getter. Zone i belongs to zone.sys_id, else vehicleList[i].
 */
export function cameraCheckZonesForPlan({
  plan, corridorPointsArr, searchPattern, vehicleList, targWpsBySys, navLastWpBySys, getAas,
}) {
  if (!plan?.zones?.length) return [];
  return plan.zones.map((zone, i) => {
    const sysId = zone.sys_id ?? vehicleList?.[i]?.sys_id;
    return cameraCheckZone({
      zoneIndex: i,
      waypoints: zoneMissionWaypoints(zone, corridorPointsArr, searchPattern),
      altitude: zoneUploadAltitude(plan, zone, i),
      targWps: sysId != null ? targWpsBySys?.[sysId] : 0,
      navLastWp: sysId != null ? navLastWpBySys?.[sysId] : 0,
      dockAlt: sysId != null ? getAas?.(sysId, 'targ_alt') : undefined,
      useTerrain: sysId != null ? getAas?.(sysId, 'use_trn') : undefined,
    });
  });
}

export function hasCheckableDocks(zones) {
  return zones.some((z) => z.docks.length > 0);
}

const m = (v) => Math.round(v);
// An "at most" limit is rounded down so following it still passes the check.
const atMost = (v) => Math.floor(v);

const isRecord = (v) => v != null && typeof v === 'object' && !Array.isArray(v);

/**
 * Why a check answer cannot be trusted, or null when it can. Trusted: "ok",
 * "problems" with at least one problem the GCS can word, and the intentional
 * ideal_360 skip, each with zone records the wording code reads without
 * error. Anything else is "could not run" (fail closed).
 */
export function cameraCheckUnavailable(result) {
  if (!isRecord(result) || !Array.isArray(result.zones) || !result.zones.every(isRecord)) return 'invalid response';
  const skipped = result.status === 'skipped';
  if (skipped && result.reason !== 'ideal_360') return result.reason || 'skipped';
  if (!skipped && result.status !== 'ok' && result.status !== 'problems') {
    return result.status ? String(result.status) : 'invalid response';
  }
  let worded;
  try {
    worded = describeCameraCheck(result, (k) => k);
  } catch {
    return 'invalid response';
  }
  if (result.status === 'problems' && worded.problems.length === 0) return 'invalid response';
  return null;
}

/**
 * Operator text for a result: one entry per problem with its cause and fixes,
 * plus grey notes for zones that could not be checked. Empty when compatible.
 */
export function describeCameraCheck(result, t, zoneName = (zi) => `${zi + 1}`) {
  const problems = [];
  const notes = [];
  for (const zone of result?.zones || []) {
    const zn = zoneName(zone.zone_index);
    if (!zone.checked) {
      if (zone.reason === 'dock_alt_unknown') notes.push(t('cameraCheck.dockAltUnknown', { zone: zn }));
      continue;
    }
    const estimate = zone.estimate ? ` ${t('cameraCheck.estimate')}` : '';
    for (const p of zone.problems || []) {
      if (p.kind === 'no_search_window') {
        problems.push({
          severity: p.severity,
          cause: t('cameraCheck.noWindow', {
            zone: zn, height: m(zone.height_m), near: m(p.near_m), range: m(p.range_m), max: atMost(p.max_height_m),
          }) + estimate,
          fixes: [
            t('cameraCheck.fixLowerAltitude', { max: atMost(p.max_height_m) }),
            t('cameraCheck.fixPitch'),
            t('cameraCheck.fixProfile'),
          ],
        });
      } else if (p.kind === 'dock_never_in_view') {
        const win = zone.windows?.[0];
        problems.push({
          severity: p.severity,
          cause: t('cameraCheck.dockNeverInView', {
            zone: zn, wp: p.wp, height: m(zone.height_m),
            near: win ? m(win.near_m) : '-', range: win ? m(win.range_edge_m) : '-',
          }) + estimate,
          fixes: [
            t('cameraCheck.fixMoveDock'),
            t('cameraCheck.fixDetectAfter'),
            t('cameraCheck.fixLowerAltitude', { max: atMost(zone.max_height_m) }),
            t('cameraCheck.fixProfile'),
          ],
        });
      }
    }
  }
  return { problems, notes };
}

/** The confirm-dialog text for a result, or null when there is nothing to ask. */
export function cameraCheckConfirmText(result, t, zoneName) {
  const { problems } = describeCameraCheck(result, t, zoneName);
  if (!problems.length) return null;
  const body = problems.map((p) => `• ${p.cause}\n  ${t('cameraCheck.whatToDo')}: ${p.fixes.join('; ')}`);
  return `${t('cameraCheck.confirmTitle')}\n\n${body.join('\n\n')}\n\n${t('cameraCheck.confirmProceed')}`;
}

/**
 * Gate an action (upload, launch, start) on the check. Resolves true to go
 * ahead. Nothing to check → true. A failed check asks too; it never passes
 * silently.
 */
export async function confirmCameraCheck({ zones, check, t, zoneName, confirm = window.confirm }) {
  if (!hasCheckableDocks(zones)) return true;
  let result;
  try {
    result = await check(zones);
  } catch (error) {
    return confirm(t('cameraCheck.couldNotRun', { error: error?.message || String(error) }));
  }
  const unavailable = cameraCheckUnavailable(result);
  if (unavailable) return confirm(t('cameraCheck.couldNotRun', { error: unavailable }));
  const text = cameraCheckConfirmText(result, t, zoneName);
  return text == null ? true : confirm(text);
}
