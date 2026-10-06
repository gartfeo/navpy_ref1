import { useState, useCallback, useRef } from 'react';
import { DEFAULT_FENCE_OFFSET_M, DEFAULT_TAKEOFF_ROUND_M } from '../utils/geo';
import {
  consumeFenceIntent,
  emptyFenceObservations,
  fenceObservationToken,
  invalidateFenceObservations,
  recordFenceObservation as recordObservation,
  reserveFenceObservations as reserveObservations,
  resetFenceObservations as resetObservations,
} from '../utils/fenceIntent';

/**
 * Mission state: MONITOR (default) ↔ PLANNING
 *
 * Also tracks polygon, analysis results, generated plan, etc.
 *
 * Per-set state: each "container set" has its own launch point and corridor.
 * When sets=1, arrays have length 1 and behavior is identical to the old single values.
 */

export const PHASES = {
  MONITOR: 'MONITOR',
  PLANNING: 'PLANNING',
};

export default function useMissionState() {
  const [phase, setPhase] = useState(PHASES.MONITOR);

  // Planning state
  const [polygon, setPolygon] = useState([]);
  const [searchPattern, setSearchPattern] = useState('distributed');
  const [analysis, setAnalysis] = useState(null);
  const [plan, setPlan] = useState(null);
  const [uavCount, setUavCount] = useState(null); // null = use analysis default
  const [uavCountLocked, setUavCountLocked] = useState(true);
  const [partitionAngleDeg, setPartitionAngleDeg] = useState(null); // null = auto (longest edge)
  const [routeOffsetM, setRouteOffsetM] = useState(null); // demo-mode route offset (m); null = auto

  // Per-set state
  const [setLaunchPoints, setSetLaunchPoints] = useState([null]);       // [{lat,lon}|null, ...]
  const [setCorridorPointsArr, setSetCorridorPoints] = useState([[]]);     // [[ {lat,lon}, ... ], ...]
  const [activeSetIndex, setActiveSetIndex] = useState(0);

  // Convenience: current set's launch point and corridor points
  const launchPoint = setLaunchPoints[activeSetIndex] ?? null;
  const corridorPoints = setCorridorPointsArr[activeSetIndex] ?? [];

  // Wrapped setters that update the active set's slot
  const setLaunchPoint = useCallback((val) => {
    setSetLaunchPoints((prev) => {
      const next = [...prev];
      next[activeSetIndex] = typeof val === 'function' ? val(prev[activeSetIndex]) : val;
      return next;
    });
  }, [activeSetIndex]);

  const setCorridorPoints = useCallback((val) => {
    setSetCorridorPoints((prev) => {
      const next = [...prev];
      next[activeSetIndex] = typeof val === 'function' ? val(prev[activeSetIndex] ?? []) : val;
      return next;
    });
  }, [activeSetIndex]);

  // fallback location assignments: fallbackLocationAssignments[zoneIdx] = fallbackLocationIndex | null
  const [fallbackLocationAssignments, setFallbackLocationAssignments] = useState([]);
  const [manualFallbackLocationEdit, setManualFallbackLocationEdit] = useState(false);

  // Sim POI waypoints (plan-local, snapshotted)
  // simDockWps:   { zoneIndex → [wpIndex, ...] } — 0-based track indices
  // detectAfterWps: { zoneIndex → wpIndex }        — single per zone
  const [simDockWps, setSimDockWps] = useState({});
  const [detectAfterWps, setDetectAfterWps] = useState({});

  // Geofence: an optional inclusion polygon offset outward from the search
  // polygon, uploaded to every UAV (breach action RTL). `fenceTouched` records
  // that the operator changed the toggle, so demo auto-enable can't override a
  // deliberate off. It is authored state only — a vehicle download must never
  // set it, or an untouched round trip would re-send fence intent.
  const [fenceEnabled, setFenceEnabled] = useState(false);
  const [fenceOffsetM, setFenceOffsetM] = useState(DEFAULT_FENCE_OFFSET_M);
  const [fenceTouched, setFenceTouched] = useState(false);
  // Pending explicit fence request: { generation, enabled }. Cleared only by an
  // upload that acknowledged THIS generation, so a failed upload keeps it and
  // an edit made mid-upload is never dropped. `fenceTouched` outlives it — the
  // operator's decision still has to suppress demo auto-enable afterwards.
  const [fenceIntent, setFenceIntent] = useState(null);
  const fenceIntentCounterRef = useRef(0);
  // Mirror so the adoption check reads the committed request, not a closure.
  const fenceIntentRef = useRef(fenceIntent);
  fenceIntentRef.current = fenceIntent;
  // Signature of the fence request the vehicles last confirmed. An identical
  // request is not re-sent; a different one is a new request.
  const [fenceAcknowledged, setFenceAcknowledged] = useState(null);
  // Bumped when the planner geometry is REPLACED by something the operator did
  // not author — a downloaded plan, or a roster shrink. The derived fence ring
  // moves with it, and that movement must not be mistaken for an operator edit.
  const [fenceGeometryRevision, setFenceGeometryRevision] = useState(0);
  const fenceGeometryRevisionRef = useRef(0);
  const fenceGeometryAdoptedRef = useRef(0);
  // What the vehicles report about their own fences. Display truth only; this
  // never becomes upload geometry. `epoch` scopes a download lifecycle so a
  // late result cannot land in a newer plan.
  const [fenceObservations, setFenceObservations] = useState(emptyFenceObservations);
  const fenceObservationsRef = useRef(fenceObservations);
  fenceObservationsRef.current = fenceObservations;

  // Monotonic per-request counter: two reads of the same vehicle can resolve
  // out of order, and the older one must not overwrite the newer.
  const fenceRequestSeqRef = useRef(0);

  /** Token for one read, taken synchronously before the request is issued. */
  const beginFenceObservation = useCallback(() => {
    fenceRequestSeqRef.current += 1;
    return fenceObservationToken(fenceObservationsRef.current, fenceRequestSeqRef.current);
  }, []);

  const applyObservationState = useCallback((update) => {
    setFenceObservations((prev) => {
      const next = update(prev);
      fenceObservationsRef.current = next;
      return next;
    });
  }, []);

  const recordFenceObservation = useCallback((sysId, observation, token) => {
    applyObservationState((prev) => recordObservation(prev, sysId, observation, token));
  }, [applyObservationState]);

  /** Mark a roster as asked-but-unanswered before the reads are issued. */
  const reserveFenceObservations = useCallback((sysIds, seq = null) => {
    applyObservationState((prev) => reserveObservations(prev, sysIds, seq));
  }, [applyObservationState]);

  const resetFenceObservations = useCallback(() => {
    applyObservationState(resetObservations);
  }, [applyObservationState]);

  // Mirror so a toggle reads the committed value without a state updater.
  const fenceEnabledRef = useRef(fenceEnabled);
  fenceEnabledRef.current = fenceEnabled;

  /**
   * Record a fence decision at a fresh generation.
   *
   * `touched` marks it as an OPERATOR decision, which suppresses the demo
   * default from then on; the demo default itself authors versioned intent
   * without claiming to be one.
   */
  const authorFenceIntent = useCallback((enabled, { touched = true } = {}) => {
    fenceIntentCounterRef.current += 1;
    if (touched) setFenceTouched(true);
    setFenceEnabled(enabled);
    setFenceIntent({ generation: fenceIntentCounterRef.current, enabled });
    // Reads issued before this decision answer the previous question.
    applyObservationState(invalidateFenceObservations);
  }, [applyObservationState]);

  /** Drop any pending request without acknowledging it (new / cleared plan). */
  const clearFenceIntent = useCallback(() => {
    setFenceIntent(null);
    setFenceAcknowledged(null);
  }, []);

  const resolveFenceIntent = useCallback((ackGeneration, ok, signature) => {
    setFenceIntent((prev) => consumeFenceIntent(prev, ackGeneration, ok));
    if (ok && signature != null) setFenceAcknowledged({ signature });
  }, []);

  /** A download/replacement moved the planner geometry under the fence. */
  const notifyFenceGeometryReplaced = useCallback(() => {
    fenceGeometryRevisionRef.current += 1;
    setFenceGeometryRevision(fenceGeometryRevisionRef.current);
  }, []);

  /**
   * Re-base the acknowledged request onto geometry a replacement just derived.
   *
   * The fence the vehicles hold has not changed, and neither has the operator's
   * decision: only the ring the planner would derive has. Without this the
   * changed content signature reads as an operator edit, and the next untouched
   * upload re-sends the fence — exactly what a round trip must never do. A
   * request still pending is NOT re-based: the operator asked for it and it has
   * to go. Nothing is invented either — a fence never acknowledged stays
   * unacknowledged.
   */
  const adoptDownloadedFenceGeometry = useCallback((signature) => {
    if (fenceGeometryAdoptedRef.current === fenceGeometryRevisionRef.current) return;
    // The replacement is handled once, here, whether or not it can be re-based.
    // Leaving it outstanding was the bug: a pending request blocked the re-base,
    // and after that request was acknowledged the still-open replacement re-based
    // onto the operator's NEXT edit — dropping geometry they had just authored.
    fenceGeometryAdoptedRef.current = fenceGeometryRevisionRef.current;
    // A pending request is sent regardless, and its acknowledgement records what
    // actually went to the vehicles; there is nothing to re-base onto yet. With
    // no signature (fence off, or no enforceable ring) there is likewise nothing
    // that could be misread as an edit.
    if (fenceIntentRef.current != null || signature == null) return;
    setFenceAcknowledged((prev) => (prev == null ? prev : { signature }));
  }, []);
  // Operator-edited fence ring. null = auto-derived from zone/corridor/fallback locations;
  // set on the first vertex/midpoint edit gesture, cleared by "Reset to auto".
  const [fenceCustomVertices, setFenceCustomVertices] = useState(null);
  // Takeoff-round radius: the fence rings each launch by this much so the UAV's
  // unknown-direction climb-out stays inside before it turns to the corridor.
  const [takeoffRoundM, setTakeoffRoundM] = useState(DEFAULT_TAKEOFF_ROUND_M);
  // Exclusion keep-outs: polygons the UAVs must stay OUT of (uploaded as
  // ArduPilot exclusion fences). [[{lat,lon}, ...], ...]
  const [exclusionPolygons, setExclusionPolygons] = useState([]);

  const toggleFence = useCallback(() => {
    authorFenceIntent(!fenceEnabledRef.current);
  }, [authorFenceIntent]);

  // Upload state
  const [uploadProgress, setUploadProgress] = useState({});

  const goToPlanning = useCallback(() => setPhase(PHASES.PLANNING), []);
  const goToMonitor = useCallback(() => setPhase(PHASES.MONITOR), []);

  return {
    phase,
    setPhase,
    goToPlanning,
    goToMonitor,

    // Planning
    polygon, setPolygon,
    searchPattern, setSearchPattern,
    analysis, setAnalysis,
    plan, setPlan,
    uavCount, setUavCount,
    uavCountLocked, setUavCountLocked,
    partitionAngleDeg, setPartitionAngleDeg,
    routeOffsetM, setRouteOffsetM,

    // Per-set state
    setLaunchPoints, setSetLaunchPoints,
    setCorridorPointsArr, setSetCorridorPoints,
    activeSetIndex, setActiveSetIndex,

    // Convenience (active set)
    launchPoint, setLaunchPoint,
    corridorPoints, setCorridorPoints,

    // fallback location assignments
    fallbackLocationAssignments, setFallbackLocationAssignments,
    manualFallbackLocationEdit, setManualFallbackLocationEdit,

    // Sim POIs
    simDockWps, setSimDockWps,
    detectAfterWps, setDetectAfterWps,

    // Geofence
    fenceEnabled, setFenceEnabled,
    fenceOffsetM, setFenceOffsetM,
    fenceTouched, setFenceTouched,
    toggleFence,
    fenceIntent, authorFenceIntent, clearFenceIntent, resolveFenceIntent,
    fenceAcknowledged,
    fenceGeometryRevision, notifyFenceGeometryReplaced, adoptDownloadedFenceGeometry,
    fenceObservations, beginFenceObservation, recordFenceObservation,
    reserveFenceObservations, resetFenceObservations,
    fenceCustomVertices, setFenceCustomVertices,
    takeoffRoundM, setTakeoffRoundM,
    exclusionPolygons, setExclusionPolygons,

    // Upload
    uploadProgress, setUploadProgress,

  };
}
