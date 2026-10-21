import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { buildNavLastWpUpdates, buildTargWpsUpdates } from '../utils/simDockReindex';
import {
  cameraCheckUnavailable, cameraCheckZonesForPlan, confirmCameraCheck, hasCheckableDocks,
} from '../utils/planCameraCheck';
import { zoneLabel } from '../components/sidebar/DeliveryHubAssignmentSection';

// Typing pause before the planning panel re-checks (one request per edit burst).
const RECHECK_DEBOUNCE_MS = 400;

/**
 * Planning panel: the camera check of the plan being edited, with the
 * targ_wps / nav_last_wp upload would write. Latest request wins.
 * Returns null while there is nothing to check. `cameraKey` changes when the
 * camera selection changes, so the answer is re-checked against the new camera.
 */
export function usePlanCameraCheckResult({
  enabled, plan, corridorPointsArr, searchPattern, vehicleList, simDockWps, detectAfterWps, getAas, check,
  cameraKey,
}) {
  const zones = useMemo(() => {
    if (!enabled || !plan?.zones?.length) return [];
    const isCorridor = searchPattern === 'corridor';
    const corridorLens = plan.zones.map((z) => (isCorridor ? 0 : ((corridorPointsArr || [])[z.set_index ?? 0] || []).length));
    const tw = buildTargWpsUpdates(simDockWps, plan.zones, vehicleList, isCorridor, corridorLens);
    const nl = buildNavLastWpUpdates(detectAfterWps, plan.zones, vehicleList, isCorridor, corridorLens);
    // Planning zones belong to vehicleList[i] (upload assigns them that way).
    const ordered = { ...plan, zones: plan.zones.map((z, i) => ({ ...z, sys_id: vehicleList?.[i]?.sys_id })) };
    return cameraCheckZonesForPlan({
      plan: ordered, corridorPointsArr, searchPattern, vehicleList,
      targWpsBySys: tw.updates, navLastWpBySys: nl.updates, getAas,
    });
  }, [enabled, plan, corridorPointsArr, searchPattern, vehicleList, simDockWps, detectAfterWps, getAas]);
  // Its inputs are rebuilt on telemetry and param updates; re-check only when
  // the request itself changes, not on every new object.
  const zonesKey = JSON.stringify(zones);

  const [result, setResult] = useState(null);
  const seqRef = useRef(0);
  useEffect(() => {
    const seq = ++seqRef.current;
    const zones = JSON.parse(zonesKey);
    if (!hasCheckableDocks(zones)) {
      setResult(null);
      return undefined;
    }
    const timer = setTimeout(() => {
      check(zones)
        .then((r) => {
          if (seq !== seqRef.current) return;
          const unavailable = cameraCheckUnavailable(r);
          setResult(unavailable ? { status: 'error', error: unavailable, zones: [] } : r);
        })
        .catch((error) => {
          if (seq === seqRef.current) setResult({ status: 'error', error: error?.message || String(error), zones: [] });
        });
    }, RECHECK_DEBOUNCE_MS);
    return () => clearTimeout(timer);
  }, [zonesKey, check, cameraKey]);
  return result;
}

/**
 * Launch / Start gate: re-checks the mission the GCS holds now (no snapshot)
 * against the current camera, and asks before going ahead on a problem.
 */
export function usePlanCameraGate({
  enabled, plan, corridorPointsArr, searchPattern, vehicleList, vehicleTargWps, vehicleNavLastWp, getAas, check,
}) {
  const { t } = useTranslation();
  const stateRef = useRef(null);
  stateRef.current = {
    enabled, plan, corridorPointsArr, searchPattern, vehicleList, vehicleTargWps, vehicleNavLastWp, getAas, check,
  };
  return useCallback(async () => {
    const s = stateRef.current;
    if (!s.enabled) return true;
    const zones = cameraCheckZonesForPlan({
      plan: s.plan, corridorPointsArr: s.corridorPointsArr, searchPattern: s.searchPattern,
      vehicleList: s.vehicleList, targWpsBySys: s.vehicleTargWps, navLastWpBySys: s.vehicleNavLastWp,
      getAas: s.getAas,
    });
    return confirmCameraCheck({
      zones, check: s.check, t, zoneName: (i) => zoneLabel(s.plan.zones, i),
    });
  }, [t]);
}
