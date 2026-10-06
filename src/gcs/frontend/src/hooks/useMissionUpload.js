import { useCallback, useRef, useState } from 'react';
import { buildTargWpsUpdates, buildNavLastWpUpdates } from '../utils/simDockReindex';
import { resolveUploadAssignments } from '../utils/uploadAssignments';
import { activateUploadedSimPlan } from '../utils/missionUploadActivation';
import { fenceSignature, fenceUploadFailures } from '../utils/fenceIntent';

/**
 * Handles mission upload: zone sorting, vehicle assignment, corridor handling.
 *
 * `fence` is the request's fence block, or null to leave every vehicle's fence
 * untouched. `fenceInvalid` marks an enable request whose geometry cannot be
 * enforced: the upload is refused rather than quietly downgraded to a disable.
 * `fenceIntentGeneration` identifies the pending operator request so
 * `onFenceIntentResolved` can consume exactly the one this upload answered.
 */
export default function useMissionUpload({
  mission,
  effectiveUavCount,
  localGenerate, approachPoint, corridorPath,
  vehicleList, api, onPlanSynced,
  onOperatorAction,
  setVehicleTargWps, setVehicleNavLastWp,
  settings,
  fence,
  fenceInvalid = false,
  fenceIntentGeneration = null,
  onFenceIntentResolved,
}) {
  const {
    polygon, searchPattern,
    launchPoint, corridorPoints,
    plan, setPlan,
    setLaunchPoints, setCorridorPointsArr,
    goToMonitor,
    fallbackLocationAssignments, manualFallbackLocationEdit,
    simDockWps, detectAfterWps,
    setUploadProgress,
  } = mission;
  const [uploading, setUploading] = useState(false);

  // Live mirror of fallbackLocationAssignments so handleUpload reads the latest committed
  // assignments (not a render-stale closure) when in manual-edit mode.
  const fallbackLocationAssignmentsRef = useRef(fallbackLocationAssignments);
  fallbackLocationAssignmentsRef.current = fallbackLocationAssignments;

  const handleUpload = useCallback(async () => {
    const fallbackLocations = settings?.fallback_delivery_locations || [];
    if (fallbackLocations.length === 0) {
      window.alert('Please add at least one fallback delivery location before uploading.');
      return;
    }
    // An enabled fence without an enforceable boundary is rejected here, before
    // anything is sent. Uploading it as "disabled" instead would switch off a
    // fence the operator explicitly switched on.
    if (fenceInvalid) {
      window.alert('The geofence is enabled but has no valid boundary — draw or reset the fence ring, or switch the fence off, then upload again.');
      return;
    }
    // Upload is an explicit ownership boundary. Cancel any incremental startup
    // reconciliation before generation or plan mutation so a late download
    // cannot overwrite the exact plan sent to vehicles.
    onOperatorAction?.();
    // Corridor mode uses corridor path as dummy polygon
    const polyArg = searchPattern === 'corridor' ? (polygon.length >= 3 ? polygon : corridorPath || polygon) : polygon;
    const generated = await localGenerate(polyArg, searchPattern, effectiveUavCount, approachPoint, corridorPath);
    const activePlan = generated || plan;
    // Plan not ready (planner still loading, or area too small) — surface it
    // instead of silently leaving planning.
    if (!activePlan?.zones?.length) {
      window.alert('Plan is not ready yet — adjust the search area and try again.');
      return;
    }
    // No vehicles to upload to: keep the plan and just switch to monitor.
    if (vehicleList.length === 0) {
      goToMonitor();
      return;
    }
    // Need at least one vehicle per zone; extra vehicles are ignored.
    if (vehicleList.length < activePlan.zones.length) {
      window.alert(`Not enough connected vehicles: ${activePlan.zones.length} zone(s) but ${vehicleList.length} vehicle(s) connected.`);
      return;
    }
    // Resolve fallback location assignments synchronously from the plan being uploaded, never
    // from render-stale state — this is the first-click upload fix. Pass whether
    // the plan was just regenerated: if so the displayed assignments may not match
    // the new zones and are recomputed; otherwise they are preserved (e.g. POIs
    // restored from a downloaded plan).
    const { assignments: effectiveAssignments, missingLabels } = resolveUploadAssignments(
      activePlan.zones, fallbackLocations, fallbackLocationAssignmentsRef.current, manualFallbackLocationEdit, generated != null,
    );
    if (missingLabels.length > 0) {
      window.alert(`All zones must have a fallback delivery location assigned. Missing: ${missingLabels.join(', ')}`);
      return;
    }

    // Use plan zone order as-is — what the user sees in the plan is what gets uploaded.
    const sortedZones = activePlan.zones;
    const isCorridor = searchPattern === 'corridor';
    const baseAlt = activePlan.altitude_m;
    const sep = activePlan.altitude_separation_m ?? 20;
    const assignments = sortedZones.map((zone, i) => {
      const v = vehicleList[i];
      if (!v) return null;
      // Per-set launch point and corridor
      const setIdx = zone.set_index ?? 0;
      const setLp = setLaunchPoints[setIdx] ?? launchPoint;
      const setCp = (setCorridorPointsArr ?? [[]])[setIdx] ?? corridorPoints;
      // Waypoints: corridor points + track (launch point sent separately)
      const wps = isCorridor
        ? zone.track
        : [...setCp, ...zone.track];
      const cc = isCorridor ? 0 : setCp.length;
      // Polygon vertices for round-trip
      const polyVerts = polygon.length >= 3
        ? polygon.map((p) => ({ lat: p.lat, lon: p.lon }))
        : [];
      // Corridor backbone for round-trip (corridor search pattern only)
      const corrBackbone = isCorridor && setCp.length > 0
        ? setCp.map((p) => ({ lat: p.lat, lon: p.lon }))
        : [];
      const zoneAlt = zone.altitude_m ?? (baseAlt + (sortedZones.length - 1 - i) * sep);
      // Look up assigned fallback location for this zone (resolved from the active plan above)
      const fallbackLocations = settings?.fallback_delivery_locations || [];
      const assignedFallbackLocationIdx = effectiveAssignments[i];
      const assignedFallbackLocation = assignedFallbackLocationIdx != null ? fallbackLocations[assignedFallbackLocationIdx] : null;
      return {
        sys_id: v.sys_id,
        zone_index: zone.zone_index,
        waypoints: wps,
        altitude_m: zoneAlt,
        corridor_count: cc,
        corridor_altitude_m: cc > 0 ? zoneAlt : null,
        search_pattern: searchPattern,
        polygon: polyVerts,
        corridor_backbone: corrBackbone,
        launch_point: setLp ? { lat: setLp.lat, lon: setLp.lon } : null,
        fallback_delivery_location: assignedFallbackLocation ? { lat: assignedFallbackLocation.lat, lon: assignedFallbackLocation.lon, type: assignedFallbackLocation.type } : null,
      };
    }).filter(Boolean);

    const zonesWithAlt = sortedZones.map((zone, i) => ({
      ...zone,
      altitude_m: baseAlt + (sortedZones.length - 1 - i) * sep,
      sys_id: vehicleList[i]?.sys_id ?? zone.sys_id,
    }));
    setPlan((prev) => prev ? { ...prev, zones: zonesWithAlt } : prev);

    if (setUploadProgress) setUploadProgress({});
    setUploading(true);
    try {
      const result = await api.uploadMissions(assignments, fence);
      // Fence truth is separate from mission truth. A mission can upload
      // cleanly while a fence parameter write is rejected, and the operator
      // must not be moved to monitoring on a half-applied fence.
      const fenceFailures = fenceUploadFailures(result);
      const fenceOk = result.fence_ok !== false && fenceFailures.length === 0;
      if (fence != null) {
        // Record the acknowledgement even when no explicit request was pending
        // (a geometry edit is a request too), so an unchanged re-upload does
        // not rewrite the fence.
        onFenceIntentResolved?.(
          fenceIntentGeneration, result.status === 'complete' && fenceOk,
          fenceSignature(fence),
        );
      }
      const missionFailures = (result.results || []).filter((r) => r.error);
      if (result.status === 'complete' && fenceOk) {
        // Write sim POI params to vehicles
        const simMode = settings?.simulation?.sim_mode;
        if (simMode) {
          const corridorLens = sortedZones.map((z) => {
            const setIdx = z.set_index ?? 0;
            const setCp = isCorridor ? [] : ((setCorridorPointsArr ?? [[]])[setIdx] ?? corridorPoints);
            return isCorridor ? 0 : setCp.length;
          });
          // targ_wps
          const tw = buildTargWpsUpdates(simDockWps, sortedZones, vehicleList, isCorridor, corridorLens);
          // nav_last_wp
          const nl = buildNavLastWpUpdates(detectAfterWps, sortedZones, vehicleList, isCorridor, corridorLens);
          const activated = await activateUploadedSimPlan({
            targWpsCalls: tw.calls,
            navLastWpCalls: nl.calls,
            writeAasParams: api.writeAasParams,
            restartReady: api.restartCompanionsReady,
          });
          if (setVehicleTargWps) {
            setVehicleTargWps((prev) => ({ ...prev, ...activated.targWpsUpdates }));
          }
          if (setVehicleNavLastWp) {
            setVehicleNavLastWp((prev) => ({ ...prev, ...activated.navLastWpUpdates }));
          }
        }
        onPlanSynced?.();
        goToMonitor();
      } else if (missionFailures.length || fenceFailures.length) {
        // Mission and fence are independent outcomes on the same vehicle.
        // Listing only the mission failures hid a vehicle whose mission
        // uploaded but whose fence did not — and calling a fence failure a
        // failed mission would be just as wrong.
        const sections = [];
        if (missionFailures.length) {
          sections.push(`Mission upload failed:\n${missionFailures
            .map((r) => `  Vehicle ${r.sys_id}: ${r.error}`).join('\n')}`);
        }
        if (fenceFailures.length) {
          sections.push(`Geofence not applied (mission upload unaffected):\n${fenceFailures
            .map((f) => `  Vehicle ${f.sysId}: ${f.error || `${f.action} failed`}${
              f.failedParams.length ? ` (${f.failedParams.join(', ')})` : ''}`).join('\n')}`);
        }
        window.alert(sections.join('\n\n'));
      } else if (!fenceOk) {
        window.alert('Missions uploaded, but the vehicles did not confirm the geofence change.');
      } else {
        window.alert(`Upload failed: ${result.error || 'Unknown error'}`);
      }
    } catch (error) {
      console.error('Mission activation error:', error);
      window.alert(`Mission uploaded, but NavPy activation failed: ${error?.message || error}`);
    } finally {
      setUploading(false);
    }
  }, [polygon, searchPattern, effectiveUavCount, launchPoint, corridorPoints, approachPoint, corridorPath, plan, vehicleList, localGenerate, api.uploadMissions, api.writeAasParams, api.restartCompanionsReady, goToMonitor, setPlan, setLaunchPoints, setCorridorPointsArr, manualFallbackLocationEdit, settings, onPlanSynced, onOperatorAction, simDockWps, setVehicleTargWps, detectAfterWps, setVehicleNavLastWp, setUploadProgress, fence, fenceInvalid, fenceIntentGeneration, onFenceIntentResolved]);

  return { uploading, handleUpload };
}
