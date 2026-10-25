import React, { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, topBarHeight, bottomBarHeight, sidebarWidth, setColors } from './styles';
import useTelemetry from './hooks/useTelemetry';
import useMissionState, { PHASES } from './hooks/useMissionState';
import useDisplaySettings from './hooks/useDisplaySettings';
import usePlanningApi from './hooks/usePlanningApi';
import useSettings from './hooks/useSettings';
import useAasParams from './hooks/useAasParams';
import { usePlanCameraCheckResult, usePlanCameraGate } from './hooks/usePlanCameraCheck';
import useFullParams from './hooks/useFullParams';
import useCompassCal from './hooks/useCompassCal';
import useAccelCal from './hooks/useAccelCal';
import { AAS_DEFAULTS } from './utils/aasParams';
import usePlanningOrchestrator from './hooks/usePlanningOrchestrator';
import useMissionUpload from './hooks/useMissionUpload';
import useVehicleConnection from './hooks/useVehicleConnection';
import useTaskConfirmation from './hooks/useTaskConfirmation';
import useTaskAssignment from './hooks/useTaskAssignment';
import useWsHandlers from './hooks/useWsHandlers';
import { emitDiagnostic, planDiagnosticFields } from './utils/diagnostics';
import useVehicleCommands from './hooks/useVehicleCommands';
import useManualControlWiring from './hooks/useManualControlWiring';
import TopBar from './components/TopBar';
import BottomBar from './components/BottomBar';
import CesiumMap from './components/map/CesiumMap';
import PlanningSidebar from './components/sidebar/PlanningSidebar';
import MonitoringSidebar from './components/sidebar/MonitoringSidebar';
import SettingsModal from './components/SettingsModal';
import PreflightReadiness from './components/PreflightReadiness';
import MapIconBtn from './components/MapIconBtn';
import MapDisplayToggles from './components/map/MapDisplayToggles';
import ManualControlOverlay from './components/ManualControlOverlay';
import { rcPwmToStickY } from './utils/joystickMath';
import { APP_BRANCH } from './utils/appBranch';
import { fenceInclusion, fenceCoversPlan, isSimpleRing, analyzeExclusionConflicts, DEFAULT_ORBIT_RADIUS_M } from './utils/geo';
import { fenceMapDisplay, fenceUploadPayload, shouldAutoEnableDemoFence, summarizeFenceObservations } from './utils/fenceIntent';
import useFenceEditing from './hooks/useFenceEditing';
import useVehicleFence from './hooks/useVehicleFence';
import TelemetryHud from './components/TelemetryHud';
import FlightModeColumn from './components/FlightModeColumn';
import { computePrearmWarnings, launchGatesFromSettings } from './utils/prearmChecks';
import VehicleSelector from './components/VehicleSelector';
import TaskConfirmCard from './components/TaskConfirmCard';
import PlanningToolbar from './components/map/PlanningToolbar';
import MonitorMapOverlay from './components/map/MonitorMapOverlay';
import { computePlanEntry } from './constants/monitorPhases';
import NotificationBanner from './components/NotificationBanner';
import LoadingScreen from './components/LoadingScreen';
import VehicleWindow, { MAP_SWAPPED_Z_INDEX } from './components/VehicleWindow';
import FpvViewOverlay from './components/map/FpvViewOverlay';
import useVideoFleet from './hooks/useVideoFleet';
import useVehicleWindow from './hooks/useVehicleWindow';
import useFpvView from './hooks/useFpvView';
import useElementSize from './hooks/useElementSize';
import useSimAutoConnect from './hooks/useSimAutoConnect';
import { DEFAULT_TOP_CONTROLS_BOTTOM_PX, vehicleWindowLayout } from './utils/vehicleWindowLayout';
import { vehicleWindowVisible } from './utils/videoStreams';
import { fleetEntryHoldReason } from './utils/fpvView';
import { prunePendingVehicles, upsertPendingVehicles } from './utils/pendingVehicles';

// RC FPV: the SYNTHETIC tag sits below the HUD heading tape (top 12, 36 px tall).
const RC_FPV_TAG_TOP_PX = 12 + 36 + 6;
/** Desktop map pane, used until the real pane is measured. */
const FALLBACK_PANE = { width: 1280, height: 800 };

export default function App() {
  const { t } = useTranslation();
  const { connected, vehicleList, seenIds, removeVehicle, messageHandlersRef, sendWsMessage, storeRef } = useTelemetry();

  const mission = useMissionState();
  const {
    phase,
    polygon, searchPattern,
    analysis, setAnalysis,
    plan, setPlan,
    launchPoint, corridorPoints,
    setLaunchPoints, setCorridorPointsArr,
    activeSetIndex, setActiveSetIndex,
    uavCount, setUavCount,
    uavCountLocked, setUavCountLocked,
    partitionAngleDeg,
    routeOffsetM, setRouteOffsetM,
    deliveryHubAssignments, setDeliveryHubAssignments,
    setManualDeliveryHubEdit,
    simDockWps, detectAfterWps, setDetectAfterWps,
    fenceEnabled, setFenceEnabled, fenceOffsetM, setFenceOffsetM,
    fenceTouched,
    fenceIntent, authorFenceIntent, resolveFenceIntent, fenceAcknowledged,
    fenceGeometryRevision, adoptDownloadedFenceGeometry,
    fenceObservations,
    fenceCustomVertices, setFenceCustomVertices,
    takeoffRoundM, setTakeoffRoundM,
    uploadProgress, setUploadProgress,
  } = mission;
  const lastPlanDiagnosticRef = useRef(null);
  useEffect(() => {
    const fields = planDiagnosticFields(plan?.zones || []);
    const signature = JSON.stringify(fields);
    if (signature === lastPlanDiagnosticRef.current) return;
    lastPlanDiagnosticRef.current = signature;
    emitDiagnostic('plan_rendered', fields);
  }, [plan]);

  const {
    showZones, setShowZones,
    showTracks, setShowTracks,
    showLaunchZone, setShowLaunchZone,
    showCoverage, setShowCoverage,
    showTrails, setShowTrails,
    showVision, setShowVision,
    hiddenSysIds, toggleSysIdHidden,
    historyPausedAt, toggleHistoryPaused, resumeHistory,
  } = useDisplaySettings();

  // Keep phase accessible to effects without making it a dependency
  const phaseRef = useRef(phase);
  phaseRef.current = phase;

  const resetViewRef = useRef(null);
  const flyToRef = useRef(null);
  const mapCameraRef = useRef(null);
  // The map's UAV pose source for the vehicle window's synthetic view.
  const poseSourceRef = useRef(null);
  const mapPaneRef = useRef(null);
  const mapPane = useElementSize(mapPaneRef, FALLBACK_PANE);
  const [mapCreditsPx, setMapCreditsPx] = useState(0);
  const [windowCreditsPx, setWindowCreditsPx] = useState(0);
  // Null until the open window measures its bar: no picture mounts before that.
  const [windowChromePx, setWindowChromePx] = useState(null);
  const [ionTerrain, setIonTerrain] = useState(false);
  const [topControlsBottom, setTopControlsBottom] = useState(DEFAULT_TOP_CONTROLS_BOTTOM_PX);
  const [mapReady, setMapReady] = useState(false);
  const [vehicleTargWps, setVehicleTargWps] = useState({});
  const [vehicleNavLastWp, setVehicleNavLastWp] = useState({});
  const [showConnPanel, setShowConnPanel] = useState(false);
  const connFromSidebarRef = useRef(false);
  const [availableVehicles, setAvailableVehicles] = useState(() => []);

  // ---- Notification banner ----
  const [notification, setNotification] = useState(null);
  useEffect(() => {
    if (!notification) return;
    const t = setTimeout(() => setNotification(null), 3000);
    return () => clearTimeout(t);
  }, [notification]);

  // ---- Auto-close connection panel when opened from sidebar and vehicles connect ----
  useEffect(() => {
    if (connFromSidebarRef.current && vehicleList.length > 0) {
      connFromSidebarRef.current = false;
      setShowConnPanel(false);
    }
  }, [vehicleList]);


  // ---- Settings ----
  const {
    settings, showSettings, setShowSettings,
    settingsTab, setSettingsTab, reopenSettingsRef,
    plannerReady,
    settingsVersion,
    handleSaveSettings, handleResetSettings,
  } = useSettings(vehicleList);

  // Dev mode gates dev-only affordances (e.g. the branch badge / tab-title
  // suffix that distinguish parallel instances). Operators run non-dev.
  const devMode = settings?.simulation?.dev_mode ?? false;

  // ---- Geofence (search polygon + takeoff corridors, offset outward) ----
  const simMode = settings?.simulation?.sim_mode ?? false;
  // One [launch, ...corridor] path per set — the fence must also enclose the
  // takeoff corridor, else the UAV starts/transits outside its own inclusion
  // fence (un-armable / immediate RTL). See fenceWithTransit.
  const transitPaths = React.useMemo(() => {
    const launches = setLaunchPoints || [];
    const corridors = setCorridorPointsArr || [];
    const count = Math.max(launches.length, corridors.length);
    const paths = [];
    for (let i = 0; i < count; i++) {
      const path = [];
      const lp = launches[i];
      if (lp) path.push({ lat: lp.lat, lon: lp.lon });
      for (const c of (corridors[i] || [])) path.push({ lat: c.lat, lon: c.lon });
      if (path.length > 0) paths.push(path);
    }
    return paths;
  }, [setLaunchPoints, setCorridorPointsArr]);
  // Default delivery hubs — the fence must also clear their confirmation
  // orbit; conflict analysis checks the orbit against exclusion keep-outs.
  const deliveryHubs = React.useMemo(
    () => (settings?.default_delivery_hubs || []).filter((o) => o && o.lat != null && o.lon != null),
    [settings],
  );
  // Only POIs the UAVs will actually orbit shape the fence: delivery hubs ASSIGNED
  // to a zone (unassigned delivery hubs are display-only) plus the plan's simulation
  // POIs (track waypoints the demo approves and approaches).
  const fencePois = React.useMemo(() => {
    const out = [];
    const seenDeliveryHub = new Set();
    for (const deliveryHubIdx of (deliveryHubAssignments || [])) {
      if (deliveryHubIdx == null || seenDeliveryHub.has(deliveryHubIdx)) continue;
      seenDeliveryHub.add(deliveryHubIdx);
      const o = deliveryHubs[deliveryHubIdx];
      if (o) out.push({ lat: o.lat, lon: o.lon });
    }
    for (const [zi, wps] of Object.entries(simDockWps || {})) {
      const track = plan?.zones?.[zi]?.track || [];
      for (const wi of (wps || [])) {
        const p = track[wi];
        if (p) out.push({ lat: p.lat, lon: p.lon });
      }
    }
    return out;
  }, [deliveryHubs, deliveryHubAssignments, simDockWps, plan]);
  const fenceAuto = React.useMemo(
    () => (fenceEnabled && polygon.length >= 3
      ? fenceInclusion(polygon, transitPaths, fencePois, {
          marginM: fenceOffsetM,
          takeoffRadiusM: takeoffRoundM,
          orbitRadiusM: DEFAULT_ORBIT_RADIUS_M,
        })
      : null),
    [fenceEnabled, polygon, transitPaths, fencePois, fenceOffsetM, takeoffRoundM],
  );
  // Operator-edited ring wins over the auto-derivation until "Reset to auto".
  const fencePolygon = fenceEnabled ? (fenceCustomVertices ?? fenceAuto) : null;
  // Safety net: if the search zone disappears (undo below 3 vertices, clear,
  // Search pattern switch), the geometry the custom ring was shaped against is gone —
  // drop it so a stale ring can never render or upload over a later plan.
  useEffect(() => {
    if (polygon.length < 3 && fenceCustomVertices) setFenceCustomVertices(null);
  }, [polygon.length, fenceCustomVertices, setFenceCustomVertices]);
  // A custom-shaped fence can be dragged too tight — warn (never block) when it
  // no longer contains the full flight path (zone + takeoff rounds + corridor +
  // confirmation orbits): outside the inclusion fence means breach → RTL.
  const fenceCoverageOk = React.useMemo(() => {
    if (!fenceEnabled || !fenceCustomVertices) return true;
    return fenceCoversPlan(fenceCustomVertices, polygon, transitPaths, fencePois, {
      takeoffRadiusM: takeoffRoundM,
      orbitRadiusM: DEFAULT_ORBIT_RADIUS_M,
    });
  }, [fenceEnabled, fenceCustomVertices, polygon, transitPaths, fencePois, takeoffRoundM]);
  // A vertex dragged across the ring makes it self-intersecting — ArduPilot's
  // even-odd test flips the crossover pockets to "outside", so warn loudly.
  const fenceSelfIntersecting = React.useMemo(
    () => !!fenceCustomVertices && !isSimpleRing(fenceCustomVertices),
    [fenceCustomVertices],
  );
  // What the connected vehicles report about their OWN fences. Display truth
  // only — it never feeds the upload payload.
  const observedFence = React.useMemo(
    () => summarizeFenceObservations(fenceObservations, vehicleList.map((v) => v.sys_id)),
    [fenceObservations, vehicleList],
  );
  // A fleet that already holds a fence reads ON on an undecided plan; see the hook.
  const vehicleFenceState = useVehicleFence({ mission, observedFence, polygonLength: polygon.length });
  const { vehicleFence, displayExclusionPolygons } = vehicleFenceState;
  // The keep-outs the operator sees and edits (the fleet's merged in while its
  // fence shows); the list, the checks and the upload all index this one list.
  const { editableExclusions } = vehicleFenceState;
  // Nearest-edge insertion can self-intersect a keep-out too (even-odd shrinks
  // the enforced no-fly area) — flag the affected rings in the sidebar list.
  const selfIntersectingExclusions = React.useMemo(
    () => editableExclusions
      .map((r, i) => (Array.isArray(r) && r.length >= 3 && !isSimpleRing(r) ? i : -1))
      .filter((i) => i >= 0),
    [editableExclusions],
  );
  // Map-side hazard cue: the amber fence turns red while it is unsafe.
  const fenceWarning = fenceCustomVertices != null && (!fenceCoverageOk || fenceSelfIntersecting);
  // Sanitized exclusion keep-outs (>= 3 vertices) shared by the layer, the
  // conflict check, and the upload payload.
  const exclusions = React.useMemo(
    () => editableExclusions.filter((e) => Array.isArray(e) && e.length >= 3),
    [editableExclusions],
  );
  // Does the planned flight (corridor legs, scan tracks, confirmation orbits)
  // enter any keep-out? Detect-and-warn — the operator moves things or replans.
  const exclusionConflicts = React.useMemo(() => {
    if (exclusions.length === 0) return [];
    return analyzeExclusionConflicts({
      corridorPaths: transitPaths,
      tracks: (plan?.zones || []).map((z) => z.track || []),
      orbitPois: fencePois, // Assigned delivery hubs and simulation POIs.
      orbitRadiusM: DEFAULT_ORBIT_RADIUS_M,
      exclusions,
    });
  }, [exclusions, transitPaths, plan, fencePois]);
  // In demo mode, auto-enable the fence once a search zone exists — unless the
  // operator has deliberately toggled it (fenceTouched), or we have observed
  // the vehicles' real fences. Auto-enabling on top of an observation would
  // turn "this is what the fleet has" into "upload this", which is exactly the
  // confusion this whole path exists to avoid. A new/cleared plan resets the
  // observations, so the demo default still applies to genuinely new plans.
  useEffect(() => {
    if (shouldAutoEnableDemoFence({
      simMode, polygonLength: polygon.length, fenceTouched, fenceEnabled,
      observations: observedFence,
    })) {
      // Versioned like any other request, so it is sent once and then consumed
      // — but not marked "touched", because the operator did not touch it.
      authorFenceIntent(true, { touched: false });
    }
  }, [simMode, polygon.length, fenceTouched, fenceEnabled, authorFenceIntent,
      observedFence]);
  const fenceMap = fenceMapDisplay({ enabled: fenceEnabled, polygon: fencePolygon, vehicleFence });
  // Fence edit gestures (vertex drag / midpoint insert / vertex delete) — the
  // first gesture snapshots the rendered ring into fenceCustomVertices.
  const fenceEditing = useFenceEditing({
    fencePolygon: fenceMap.polygon, setFenceCustomVertices: vehicleFenceState.editFenceVertices,
  });
  // Upload payload. Absent (null) = leave every vehicle's fence exactly as it
  // is, which is what an untouched download/upload round trip must produce.
  // Only operator-authored geometry can be uploaded.
  const { payload: fenceUpload, invalid: fenceInvalid, signature: fenceRequestSignature } = React.useMemo(
    () => fenceUploadPayload({
      enabled: fenceEnabled, intent: fenceIntent, acknowledged: fenceAcknowledged,
      vertices: fencePolygon, exclusions,
    }),
    [fenceEnabled, fenceIntent, fenceAcknowledged, fencePolygon, exclusions],
  );
  // A downloaded plan (or a roster shrink) republishes the planner geometry, so
  // the derived ring moves without anyone editing it. Adopt that ring as the
  // acknowledged request's geometry — otherwise the changed content signature
  // reads as an operator edit and the next untouched upload re-sends the fence.
  // Runs after the derivation above, so it sees the new ring; a still-pending
  // operator request is left alone and still goes out.
  useEffect(() => {
    adoptDownloadedFenceGeometry(fenceRequestSignature);
  }, [fenceGeometryRevision, adoptDownloadedFenceGeometry, fenceRequestSignature]);
  // Display-only labels for the fence, derived from the same facts the map and
  // the upload already use. They name which ring is on screen and what the
  // next upload will actually request; neither one feeds the payload.
  const fenceRequestStatus = fenceInvalid
    ? 'invalid'
    : (fenceUpload == null ? 'none' : (fenceUpload.enabled ? 'enable' : 'disable'));

  // Surface the running branch in the tab title only in dev mode, matching the
  // topbar branch badge; keep a clean title for operators in non-dev mode.
  useEffect(() => {
    document.title = devMode && APP_BRANCH ? `AAS GCS — ${APP_BRANCH}` : 'AAS GCS';
  }, [devMode]);

  // One-click preflight readiness overlay (opened from the top bar).
  const [showReadiness, setShowReadiness] = useState(false);

  // ---- AAS params (vehicle-side source of truth) ----
  const aasParams = useAasParams(vehicleList);
  const fullParams = useFullParams(vehicleList);
  const compassCal = useCompassCal(vehicleList);

  // One-shot AAS + full param download for newly connected vehicles.
  //  - AAS: without it, ConfirmSection / TaskConfirmCard would see `loading`
  //    or defaults forever if the operator never opens the AasTab.
  //  - Full params: pre-warm the cache so the Parameters tab opens instantly
  //    instead of paying the MAVFTP setup wait on first open. The Parameters
  //    tab's own fetch dedups against this in-flight pre-warm (refreshVehicles
  //    claims a pending slot per sys_id), so it never double-fetches.
  // Tracked by sys_id; reconnects after disconnect re-trigger.
  const knownConnectedSysIdsRef = useRef(new Set());
  useEffect(() => {
    const live = new Set((vehicleList || []).map((v) => v.sys_id));
    const newIds = [];
    for (const sid of live) {
      if (!knownConnectedSysIdsRef.current.has(sid)) newIds.push(sid);
    }
    knownConnectedSysIdsRef.current = live;
    if (newIds.length > 0) {
      aasParams.refreshVehicles(newIds);
      fullParams.refreshVehicles(newIds);
    }
  }, [vehicleList, aasParams.refreshVehicles, fullParams.refreshVehicles]);

  // ---- Planning orchestrator ----
  const planning = usePlanningOrchestrator({
    mission,
    settings,
    handleSaveSettings,
    settingsVersion,
    plannerReady,
    exclusionEditing: {
      exclusionPolygons: vehicleFenceState.editableExclusions,
      setExclusionPolygons: vehicleFenceState.editExclusionPolygons,
      drawable: vehicleFenceState.keepOutDrawable,
    },
  });

  // ---- Task / WS handler wiring ----
  const taskConfirm = useTaskConfirmation();
  const taskAssign = useTaskAssignment();
  const ws = useWsHandlers({
    messageHandlersRef,
    taskConfirm,
    taskAssign,
    setUploadProgress,
    fullParams,
    compassCal,
  });

  // ---- Planning API ----
  const api = usePlanningApi({ setAnalysis, setPlan });

  // ---- Accelerometer / level calibration ----
  // Registers directly on messageHandlersRef (not through useWsHandlers),
  // matching the source branch's own wiring (INTEG-04, D-04).
  const accelCal = useAccelCal({ messageHandlersRef, sendCommand: api.sendCommand });

  // ---- Persist last-used connection device to settings ----
  const handleDeviceUsed = useCallback((device) => {
    handleSaveSettings({ connection: { default_device: device } });
  }, [handleSaveSettings]);

  const handleAvailableVehiclesChange = useCallback((vehicles) => {
    setAvailableVehicles((prev) => {
      const next = vehicles || [];
      const same = prev.length === next.length
        && prev.every((v, i) => (
          v.sys_id === next[i].sys_id
          && v.name === next[i].name
          && v.device === next[i].device
        ));
      return same ? prev : next;
    });
  }, []);

  useEffect(() => {
    setAvailableVehicles((prev) => prunePendingVehicles(prev, vehicleList));
  }, [vehicleList]);

  // ---- Vehicle connection ----
  const {
    handleConnect,
    handleDisconnect,
    handleScan,
    handleDownloadPlan,
    cancelStartupMissionReconciliation,
    downloadingSysIds,
    pendingVehicles,
  } = useVehicleConnection({
    mission,
    vehicleList, removeVehicle, api, telemetryStoreRef: storeRef,
    derivePlanPolygon: planning.derivePlanPolygon,
    onPlanSynced: planning.onPlanSynced,
    settings,
    setVehicleTargWps, setVehicleNavLastWp, setNotification,
  });

  const statusPendingVehicles = React.useMemo(
    () => upsertPendingVehicles(pendingVehicles, availableVehicles),
    [pendingVehicles, availableVehicles],
  );

  // ---- Auto-connect on startup (sim mode) ----
  useSimAutoConnect({
    enabled: settings?.connection?.auto_connect,
    simMode: settings?.simulation?.sim_mode,
    device: settings?.connection?.default_device,
    scan: handleScan,
    vehicleCount: vehicleList.length,
  });

  // ---- Mission upload ----
  const { uploading, handleUpload } = useMissionUpload({
    mission,
    effectiveUavCount: planning.effectiveUavCount,
    localGenerate: planning.localGenerate,
    approachPoint: planning.approachPoint,
    corridorPath: planning.corridorPath,
    vehicleList, api,
    onPlanSynced: planning.onPlanSynced,
    onOperatorAction: cancelStartupMissionReconciliation,
    setVehicleTargWps, setVehicleNavLastWp,
    settings,
    fence: fenceUpload,
    fenceInvalid,
    fenceIntentGeneration: fenceIntent?.generation ?? null,
    onFenceIntentResolved: resolveFenceIntent,
    getAas: aasParams.getConfirmedValue,
  });

  // ---- Plan/camera check: live in planning, re-run before Launch / Start ----
  const planCameraCheck = usePlanCameraCheckResult({
    enabled: simMode && phase === PHASES.PLANNING,
    plan, corridorPointsArr: setCorridorPointsArr, searchPattern, vehicleList,
    simDockWps, detectAfterWps, getAas: aasParams.getConfirmedValue, check: api.checkPlanCamera,
    cameraKey: JSON.stringify(settings?.camera ?? null),
  });
  const confirmBeforeStart = usePlanCameraGate({
    enabled: simMode,
    plan, corridorPointsArr: setCorridorPointsArr, searchPattern, vehicleList,
    vehicleTargWps, vehicleNavLastWp, getAas: aasParams.getConfirmedValue, check: api.checkPlanCamera,
  });

  // ---- Vehicle commands ----
  const commands = useVehicleCommands({
    api, vehicleList,
    taskConfirmReset: taskConfirm.reset,
    taskAssignReset: taskAssign.reset,
    setLaunchStates: ws.setLaunchStates,
    resumeHistory,
    confirmBeforeStart,
  });
  // ---- Manual control wiring (must be after useVehicleCommands for handleSetMode) ----
  const mc = useManualControlWiring({
    sendWsMessage, storeRef,
    phase, vehicleList,
    handleSetMode: commands.handleSetMode,
    setNotification,
  });

  // ---- Center view (frame the plan + GPS-fixed UAVs) ----
  // Keep the latest fit inputs in a ref so the reset button's callback stays
  // stable without going stale.
  const centerArgsRef = useRef(null);
  centerArgsRef.current = { polygon, vehicleList, plan, launchPoint, corridorPoints };
  const centerView = useCallback(() => {
    const a = centerArgsRef.current;
    resetViewRef.current?.(a.polygon, a.vehicleList, a.plan, a.launchPoint, a.corridorPoints);
  }, []);

  // ---- Vehicle window + RC FPV (docs/design/gcs_video_streams.md §5) ----
  // The video icon in the action bar opens the window; its picker, its UAV
  // list and View camera pick the window's UAV; the map never follows a UAV.
  // useFpvView owns RC FPV and the saved map camera.
  const videoFleet = useVideoFleet(storeRef);
  const monitor = phase === PHASES.MONITOR;
  const win = useVehicleWindow({ fleet: videoFleet, active: monitor && !mc.manualControlEnabled });
  const fpv = useFpvView({
    rcEnabled: mc.manualControlEnabled, rcTarget: mc.manualControlTarget, mapCameraRef,
  });
  const showWindow = vehicleWindowVisible({
    monitor, rcEnabled: mc.manualControlEnabled, fleetSize: videoFleet.length, open: win.open,
  });
  const windowLayout = vehicleWindowLayout({
    paneWidth: mapPane.width,
    paneHeight: mapPane.height,
    topControlsBottom,
    swapped: showWindow && win.swapped,
    mapCreditsPx,
    windowChromePx,
    windowCreditsPx,
  });
  const swapped = showWindow && win.swapped && windowLayout.canSwap;
  const smallMap = windowLayout.smallMap;
  const { unswap } = win;
  // The pane became too short for a whole small map: the swap ends, before paint.
  useLayoutEffect(() => {
    if (win.swapped && !windowLayout.canSwap) unswap();
  }, [win.swapped, windowLayout.canSwap, unswap]);
  // Sidebar coordinate click: swap back first, so the map flies large
  // (the map's fly-to waits for the swap back while the map is small).
  const flyToLocation = useCallback((lat, lon) => {
    unswap();
    flyToRef.current?.(lat, lon);
  }, [unswap]);
  const { viewing, viewCamera } = win;
  // "Viewing" only while the picture is drawn: not while closed, nor with too
  // little room for the picture.
  const pictureShown = showWindow && windowLayout.pictureFits;
  const videoActionFor = useCallback((sysId) => {
    if (!videoFleet.find((e) => e.sysId === sysId)?.streams.length) return null;
    return {
      viewing: pictureShown && viewing(sysId),
      onView: () => viewCamera(sysId),
    };
  }, [videoFleet, pictureShown, viewing, viewCamera]);
  const fpvEntry = videoFleet.find((e) => e.sysId === fpv.fpvSysId);

  const mapHeight = `calc(100dvh - ${topBarHeight}px - ${bottomBarHeight}px)`;

  return (
    <div
      style={{
        width: '100vw',
        height: '100dvh',
        background: colors.bg,
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
      }}
    >
      <TopBar
        connected={connected}
        phase={phase}
        vehicleList={vehicleList}
        seenIds={seenIds}
        onScan={handleScan}
        onConnect={handleConnect}
        onDisconnect={handleDisconnect}
        onDownloadPlan={handleDownloadPlan}
        downloadingSysIds={downloadingSysIds}
        showConn={showConnPanel}
        setShowConn={setShowConnPanel}
        onOpenSettings={() => setShowSettings(true)}
        defaultDevice={settings?.connection?.default_device}
        onDeviceUsed={handleDeviceUsed}
        onAvailableVehiclesChange={handleAvailableVehiclesChange}
        autoScan={connFromSidebarRef.current}
        devMode={devMode}
        onEstop={commands.handleEstop}
        manualControlEnabled={mc.manualControlEnabled}
      />

      <div style={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
        {/* Map */}
        <div ref={mapPaneRef} style={{ flex: 1, position: 'relative', height: mapHeight }}>
          {/* The map box. Swapped with the maximized vehicle window it shrinks
              into a small box at the top right of the window's picture: CSS
              only, never a remount, never an animation, never size 0. */}
          <div
            data-testid="map-box"
            style={smallMap ? {
              position: 'absolute',
              top: smallMap.top,
              right: smallMap.right,
              width: smallMap.width,
              height: smallMap.height,
              zIndex: MAP_SWAPPED_Z_INDEX,
              borderRadius: 6,
              overflow: 'hidden',
              border: `1px solid ${colors.border}`,
            } : { position: 'absolute', inset: 0 }}
          >
          <CesiumMap
            polygon={polygon}
            plan={plan}
            analysis={analysis}
            searchPattern={searchPattern}
            fencePolygon={fenceMap.polygon}
            showFence={fenceMap.show}
            fenceEditable={vehicleFenceState.editable}
            fenceWarning={fenceWarning}
            onFenceVertexDrag={fenceEditing.handleFenceVertexDrag}
            onFenceMidpointInsert={fenceEditing.handleFenceMidpointInsert}
            onFenceVertexDelete={fenceEditing.handleFenceVertexDelete}
            exclusionPolygons={displayExclusionPolygons}
            exclusionDraft={planning.draftExclusion}
            exclusionConflicts={exclusionConflicts}
            placingExclusion={planning.placingExclusion}
            onPlaceExclusionVertex={planning.handlePlaceExclusionVertex}
            onFinishExclusion={planning.handleFinishExclusion}
            onExclusionVertexDrag={planning.handleExclusionVertexDrag}
            onExclusionMidpointInsert={planning.handleExclusionMidpointInsert}
            onExclusionVertexDelete={planning.handleExclusionVertexDelete}
            showZones={showZones}
            showTracks={showTracks}
            showLaunchZone={showLaunchZone}
            showCoverage={showCoverage}
            showVision={showVision}
            showTrails={showTrails}
            historyResetKey={commands.historyResetKey}
            hiddenSysIds={hiddenSysIds}
            historyPaused={historyPausedAt != null}
            simClosestApproaches={ws.simClosestApproaches}
            vehicleList={vehicleList}
            storeRef={storeRef}
            trackResetKey={commands.trackResetKey}
            phase={phase}
            isDrawing={planning.drawing.isDrawing}
            onMapClick={planning.handleMapClick}
            onVertexDrag={planning.handleVertexDrag}
            onVertexDelete={planning.drawing.onVertexDelete}
            onMidpointInsert={planning.drawing.onMidpointInsert}
            onPolygonMove={planning.handlePolygonMove}
            onFinishDraw={planning.drawing.stopDraw}
            onRemoveLaunchPoint={planning.handleRemoveLaunchPoint}
            placingLaunchPoint={planning.placingLaunchPoint}
            onPlaceLaunchPoint={planning.handlePlaceLaunchPoint}
            placingCorridor={planning.placingCorridor}
            onPlaceCorridorPoint={planning.handlePlaceCorridorPoint}
            onCorridorMidpointInsert={planning.handleCorridorMidpointInsert}
            onCorridorPointDelete={planning.handleCorridorPointDelete}
            onDragStart={planning.handleDragStart}
            onDragEnd={planning.handleDragEnd}
            onSuppressRegen={planning.handleSuppressRegen}
            setLaunchPoints={setLaunchPoints}
            setCorridorPoints={setCorridorPointsArr}
            activeSetIndex={activeSetIndex}
            setActiveSetIndex={setActiveSetIndex}
            onSetCorridorPointDrag={planning.handleSetCorridorPointDrag}
            onCorridorDragRecord={planning.handleCorridorDragRecord}
            onPolygonDragRecord={planning.handlePolygonDragRecord}
            partitionAngleDeg={partitionAngleDeg}
            onPartitionAngleDrag={planning.handlePartitionAngleDrag}
            effectiveSets={planning.effectiveSets}
            resetViewRef={resetViewRef}
            flyToRef={flyToRef}
            pendingConfirms={taskConfirm.pendingConfirms}
            assignments={taskAssign.assignments}
            availableTasks={taskAssign.availableTasks}
            mapDefaults={settings?.map_display}
            simMode={settings?.simulation?.sim_mode ?? false}
            simDockWps={simDockWps}
            onToggleSimDock={planning.handleToggleSimDock}
            vehicleTargWps={vehicleTargWps}
            vehicleNavLastWp={vehicleNavLastWp}
            detectAfterWps={detectAfterWps}
            deliveryHubs={settings?.default_delivery_hubs}
            deliveryHubAssignments={deliveryHubAssignments}
            placingDeliveryHub={planning.placingDeliveryHub}
            onPlaceDeliveryHub={planning.handlePlaceDeliveryHub}
            onRemoveDeliveryHub={planning.handleRemoveDeliveryHub}
            onMoveDeliveryHub={planning.handleMoveDeliveryHub}
            fpvSysId={fpv.fpvSysId}
            cameraPersistSuspended={fpv.cameraOwned}
            mapCameraRef={mapCameraRef}
            swapped={swapped}
            poseSourceRef={poseSourceRef}
            onCreditsHeight={setMapCreditsPx}
            onTerrainReady={setIonTerrain}
            onViewerReady={setMapReady}
            plannerReady={plannerReady}
          />
          {smallMap && (
            <button
              type="button"
              aria-label={t('vehicleWindow.showMapLarge')}
              title={t('vehicleWindow.showMapLarge')}
              onClick={win.unswap}
              style={{
                position: 'absolute', inset: 0, padding: 0, border: 'none',
                background: 'transparent', cursor: 'pointer', touchAction: 'manipulation',
              }}
            />
          )}
          </div>
          {/* Map toolbar — right side, vertical */}
          {phase === PHASES.PLANNING && (
            <PlanningToolbar
              isDrawing={planning.drawing.isDrawing}
              onStartDraw={planning.startDrawExclusive}
              onStopDraw={planning.drawing.stopDraw}
              searchPattern={searchPattern}
              polygon={polygon}
              launchPoint={launchPoint}
              corridorPoints={corridorPoints}
              placingCorridor={planning.placingCorridor}
              onToggleCorridor={planning.toggleCorridorPlacement}
              effectiveSets={planning.effectiveSets}
              activeSetIndex={activeSetIndex}
              setActiveSetIndex={setActiveSetIndex}
              placingDeliveryHub={planning.placingDeliveryHub}
              onToggleDeliveryHub={planning.toggleDeliveryHubPlacement}
              placingDeliveryHubType={planning.placingDeliveryHubType}
              setPlacingDeliveryHubType={planning.setPlacingDeliveryHubType}
              fenceEnabled={vehicleFenceState.switchOn}
              onToggleFence={vehicleFenceState.handleToggleFence}
              observedFenceMode={observedFence.mode}
              observedFenceAutoenableMode={observedFence.autoenableMode}
              fenceMapSource={fenceMap.source}
              placingExclusion={planning.placingExclusion}
              onToggleExclusion={planning.toggleExclusionPlacement}
              keepOutDrawable={vehicleFenceState.keepOutDrawable}
              onUndo={planning.handleUndo}
              canUndo={planning.canUndo}
              onClearAll={planning.handleClearAll}
              onLoadPolygon={planning.handleLoadPolygon}
              onSavePolygon={planning.handleSavePolygon}
            />
          )}
          {phase === PHASES.MONITOR && vehicleList.length > 0 && !mc.manualControlEnabled && (
            <MonitorMapOverlay
              onTopControlsBottom={setTopControlsBottom}
              onToggleManualControl={mc.handleToggleManualControl}
              showVision={showVision}
              onToggleVision={() => setShowVision(!showVision)}
              trails={{
                vehicleList,
                showTrails,
                onToggleTrails: () => setShowTrails(!showTrails),
                showMarks: showCoverage,
                onToggleMarks: () => setShowCoverage(!showCoverage),
                hiddenSysIds,
                onToggleUav: toggleSysIdHidden,
                pausedAt: historyPausedAt,
                onTogglePause: toggleHistoryPaused,
                onClearAll: commands.handleClearHistory,
              }}
              onEditPlan={planning.handleGoToPlanning}
              planEntry={computePlanEntry({
                hasPlan: (plan?.zones?.length || 0) > 0,
                vehiclesHaveMission: vehicleList.some((v) => (v.mission_total || 0) > 0),
                isBusyConnecting: (downloadingSysIds?.size || 0) > 0,
                allDisarmed: vehicleList.length > 0 && vehicleList.every((v) => !v.armed),
              })}
              videoOpen={win.open}
              onToggleVideo={win.toggleOpen}
            />
          )}
          {/* Vehicle window — while the video icon is pressed, monitor view
              only, never during manual control, so it cannot sit over the
              joysticks, the flight-mode column or the RC FPV HUD. Top-right
              under the action bar at z-index 9, under the map overlay buttons
              and the confirmation cards. */}
          {showWindow && (
            <VehicleWindow
              fleet={videoFleet}
              win={win}
              layout={windowLayout}
              poseSourceRef={poseSourceRef}
              storeRef={storeRef}
              ionTerrain={ionTerrain}
              onSyntheticCreditsHeight={setWindowCreditsPx}
              onChromeHeight={setWindowChromePx}
            />
          )}
          {fpv.fpvSysId != null && (
            <FpvViewOverlay
              label={fpvEntry?.label ?? `UAV ${fpv.fpvSysId}`}
              holdReason={fleetEntryHoldReason(fpvEntry)}
              top={RC_FPV_TAG_TOP_PX}
            />
          )}
          {mc.manualControlEnabled && (
            <>
              <ManualControlOverlay
                key={mc.manualControlTarget}
                onLeftMove={mc.handleLeftMove}
                onRightMove={mc.handleRightMove}
                initialThrottleY={rcPwmToStickY(mc.mcVehicle?.rc3)}
              />
              <VehicleSelector
                vehicleList={vehicleList}
                selectedSysId={mc.manualControlTarget}
                onSelect={mc.handleVehicleSelect}
              />
              {(() => {
                const pa = computePrearmWarnings(mc.mcVehicle, launchGatesFromSettings(settings));
                return (
                  <FlightModeColumn
                    currentMode={mc.mcVehicle?.mode}
                    armed={mc.mcVehicle?.armed}
                    prearmOk={mc.mcVehicle?.prearm_ok}
                    prearmSeverity={pa.severity}
                    prearmWarnings={pa.warnings}
                    altRel={mc.mcVehicle?.alt_rel}
                    sysId={mc.manualControlTarget}
                    onSetMode={commands.handleSetMode}
                    onArmDisarm={commands.handleArmDisarm}
                    onToggleManualControl={mc.handleToggleManualControl}
                  />
                );
              })()}
            </>
          )}
          {mc.manualControlEnabled && mc.manualControlTarget != null && (
              <TelemetryHud key={mc.manualControlTarget} sysId={mc.manualControlTarget} />
          )}
          <NotificationBanner text={notification} />
          {/* Confirm cards — visible even during manual control */}
          {mc.manualControlEnabled && Object.entries(taskConfirm.pendingConfirms || {}).length > 0 && (
            <div style={{
              position: 'absolute',
              top: 60,
              right: 12,
              width: 260,
              zIndex: 20,
              pointerEvents: 'auto',
            }}>
              {Object.entries(taskConfirm.pendingConfirms).map(([sysId, entry]) => {
                const sid = Number(sysId);
                const vi = vehicleList.findIndex((v) => v.sys_id === sid);
                const vehicle = vi >= 0 ? vehicleList[vi] : null;
                return (
                  <TaskConfirmCard
                    key={`${sid}-${entry.taskId}-${entry.roundUid ?? ''}`}
                    sysId={sid}
                    entry={entry}
                    vehicleName={vehicle?.name || `UAV ${sysId}`}
                    vehicleIndex={vi >= 0 ? vi : 0}
                    onApprove={ws.handleTaskApprove}
                    onDeny={ws.handleTaskDeny}
                    onCancel={ws.handleTaskCancel}
                    autoApprove={aasParams.getConfirmedValue(sid, 'nav_cm_fl', AAS_DEFAULTS.nav_cm_fl)}
                    timeoutSec={aasParams.getConfirmedValue(sid, 'nav_cwt', AAS_DEFAULTS.nav_cwt)}
                    forced={taskConfirm.forcedConfirms[sid] === entry.taskId}
                  />
                );
              })}
            </div>
          )}
          {!mc.manualControlEnabled && <MapDisplayToggles
            phase={phase}
            searchPattern={searchPattern}
            showZones={showZones}
            setShowZones={setShowZones}
            showTracks={showTracks}
            setShowTracks={setShowTracks}
            showLaunchZone={showLaunchZone}
            setShowLaunchZone={setShowLaunchZone}
          />}
          {/* Reset zoom — hidden during manual control (the RC FPV camera
              would override it every frame) and while the map is small (a fit
              there would use the small box's aspect ratio) */}
          {!mc.manualControlEnabled && !swapped && <div style={{
            position: 'absolute',
            bottom: 12,
            right: 12,
            zIndex: 10,
          }}>
            <MapIconBtn
              title={t('monitorOverlay.resetView')}
              onClick={centerView}
            >
              <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
                <circle cx="7" cy="7" r="5.5" stroke="currentColor" strokeWidth="1.4"/>
                <circle cx="7" cy="7" r="1.5" fill="currentColor"/>
              </svg>
            </MapIconBtn>
          </div>}
        </div>

        {/* Sidebar — hidden during manual control for full-width map */}
        {!mc.manualControlEnabled && <div
          style={{
            width: sidebarWidth,
            height: mapHeight,
            background: colors.bgLight,
            borderLeft: `1px solid ${colors.border}`,
            overflowY: 'auto',
            overflowX: 'hidden', // sidebar must never scroll horizontally — rows shrink to fit
            flexShrink: 0,
          }}
        >
          {phase === PHASES.PLANNING && (
            <PlanningSidebar
              cameraCheck={planCameraCheck}
              searchPattern={searchPattern}
              setSearchPattern={planning.handleSearchPatternChange}
              analysis={analysis}
              uavCount={planning.effectiveUavCount}
              setUavCount={setUavCount}
              launchPoint={launchPoint}
              corridorPoints={corridorPoints}
              plan={plan}
              uavCountLocked={uavCountLocked}
              setUavCountLocked={setUavCountLocked}
              settings={settings}
              vehicleList={vehicleList}
              aasParams={aasParams}
              setLaunchPoints={setLaunchPoints}
              deliveryHubAssignments={deliveryHubAssignments}
              setDeliveryHubAssignments={setDeliveryHubAssignments}
              setManualDeliveryHubEdit={setManualDeliveryHubEdit}
              simDockWps={simDockWps}
              simMode={settings?.simulation?.sim_mode ?? false}
              detectAfterWps={detectAfterWps}
              setDetectAfterWps={setDetectAfterWps}
              onToggleSimDock={planning.handleToggleSimDock}
              routeOffsetM={routeOffsetM}
              setRouteOffsetM={setRouteOffsetM}
              hasSearchPolygon={polygon.length >= 3}
              fenceEnabled={vehicleFenceState.switchOn}
              fencePlanned={fenceEnabled}
              onToggleFence={vehicleFenceState.handleToggleFence}
              observedFence={observedFence}
              fenceRequestStatus={fenceRequestStatus}
              fenceOffsetM={fenceOffsetM}
              setFenceOffsetM={setFenceOffsetM}
              takeoffRoundM={takeoffRoundM}
              setTakeoffRoundM={setTakeoffRoundM}
              fenceCustomized={fenceCustomVertices != null || vehicleFence != null}
              onFenceReset={vehicleFenceState.editable ? fenceEditing.handleFenceReset : null}
              fenceCoverageOk={fenceCoverageOk}
              fenceSelfIntersecting={fenceSelfIntersecting}
              selfIntersectingExclusions={selfIntersectingExclusions}
              exclusionPolygons={editableExclusions}
              exclusionConflicts={exclusionConflicts}
              onRemoveExclusion={planning.handleRemoveExclusion}
              onClearExclusions={planning.handleClearExclusions}
            />
          )}
          {phase === PHASES.MONITOR && (
            <MonitoringSidebar
              vehicleList={vehicleList}
              plan={plan}
              analysis={analysis}
              polygon={polygon}
              launchPoint={launchPoint}
              corridorPoints={corridorPoints}
              searchPattern={searchPattern}
              onEditPlan={planning.handleGoToPlanning}
              onLaunch={commands.handleLaunch}
              onOpenConnect={() => { connFromSidebarRef.current = true; setShowConnPanel(true); }}
              onRestart={commands.handleRestart}
              onDownloadPlan={handleDownloadPlan}
              downloadingSysIds={downloadingSysIds}
              pendingVehicles={statusPendingVehicles}
              pendingConfirms={taskConfirm.pendingConfirms}
              onApprove={ws.handleTaskApprove}
              onDeny={ws.handleTaskDeny}
              onCancel={ws.handleTaskCancel}
              forcedConfirms={taskConfirm.forcedConfirms}
              onForceConfirm={taskConfirm.forceConfirm}
              assignments={taskAssign.assignments}
              settings={settings}
              launchStates={ws.launchStates}
              onAbortLaunch={commands.handleAbortLaunch}
              onTriggerVehicle={commands.handleTriggerVehicle}
              navpyStatus={ws.navpyStatus}
              onFlyToLocation={flyToLocation}
              aasParams={aasParams}
              onEstop={commands.handleEstop}
              onOpenReadiness={() => setShowReadiness(true)}
              videoActionFor={videoActionFor}
            />
          )}
        </div>}
      </div>

      <BottomBar
        phase={phase}
        analysis={analysis}
        polygon={polygon}
        onUpload={handleUpload}
        onExitPlanning={planning.handleExitPlanning}
        onOpenConnect={() => setShowConnPanel(true)}
        vehicleList={vehicleList}
        plan={plan}
        launchPoint={launchPoint}
        corridorPoints={corridorPoints}
        searchPattern={searchPattern}
        uploading={uploading}
        uploadProgress={uploadProgress}
        effectiveUavCount={planning.effectiveUavCount}
      />

      <LoadingScreen visible={!mapReady} />

      {showSettings && (
        <SettingsModal
          settings={settings}
          onSave={handleSaveSettings}
          onReset={handleResetSettings}
          onClose={() => { setShowSettings(false); setSettingsTab(null); }}
          onOpenConnect={() => { reopenSettingsRef.current = true; setShowConnPanel(true); }}
          vehicleList={vehicleList}
          initialTab={settingsTab}
          onVehicleTargWpsChange={setVehicleTargWps}
          onVehicleNavLastWpChange={setVehicleNavLastWp}
          aasParams={aasParams}
          fullParams={fullParams}
          compassCal={compassCal}
          accelCal={accelCal}
          sendCommand={api.sendCommand}
          onStartPlacingDeliveryHub={() => {
            setShowSettings(false);
            setSettingsTab(null);
            planning.startPlacingDeliveryHubFromSettings();
          }}
        />
      )}

      {showReadiness && (
        <PreflightReadiness
          settings={settings}
          onClose={() => setShowReadiness(false)}
          onReboot={commands.handleReboot}
          fullParams={fullParams}
          onOpenParamSync={() => {
            setShowReadiness(false);
            setSettingsTab('Parameters');
            setShowSettings(true);
          }}
        />
      )}
    </div>
  );
}
