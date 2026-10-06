import React, { useState, useCallback } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import EstopDropdown from '../EstopDropdown';
import { missionWpOffset } from '../../utils/geo';
import { computeZoneDistances } from '../../utils/missionProgress';
import LaunchPanel from './LaunchPanel';
import TaskConfirmCard from '../TaskConfirmCard';
import ConfirmModal from '../ConfirmModal';
import { useTelemetryStore } from '../../hooks/useTelemetryStore';
import { computeLaunchReadiness, launchGatesFromSettings } from '../../utils/prearmChecks';
import { computeFleetStatus, computeVehicleReadiness } from '../../utils/preflightReadiness';
import useContainerLauncher from '../../hooks/useContainerLauncher';
import VehicleStatusCard from './VehicleStatusCard';
import useLoadStages from '../../hooks/useLoadStages';
import { ForceStartButton, LauncherStatus, EditButton, EditPlanLink } from './LaunchControls';
import { MONITOR_PHASES, computeMonitorPhase } from '../../constants/monitorPhases';
import { SectionTitle, Divider } from './SidebarPrimitives';
import { AAS_DEFAULTS } from '../../utils/aasParams';
import { groupVehiclesByContainer } from '../../utils/containers';
import { mergePendingVehicles } from '../../utils/pendingVehicles';
import { selectLaunchVehicles } from '../../utils/launchSequence';

// Fleet-wide GO/CAUTION/NO-GO color, shared by the readiness trigger below.
const FLEET_STATUS_COLOR = {
  go: colors.success,
  warn: colors.warning,
  nogo: colors.error,
};

// Re-export for existing consumers
export { MONITOR_PHASES, computeMonitorPhase };

export default function MonitoringSidebar({
  vehicleList,
  plan,
  analysis,
  polygon,
  launchPoint,
  corridorPoints,
  searchPattern,
  onEditPlan,
  onLaunch,
  onOpenConnect,
  onRestart,
  onDownloadPlan,
  downloadingSysIds,
  pendingVehicles,
  pendingConfirms,
  onApprove,
  onDeny,
  onCancel,
  forcedConfirms,
  onForceConfirm,
  assignments,
  settings,
  launchStates,
  onAbortLaunch,
  onTriggerVehicle,
  navpyStatus,
  onFlyToLocation,
  aasParams,
  onEstop,
  onOpenReadiness,
}) {
  // Subscribe to telemetry store for fresh vehicle data (battery, speed, etc.)
  // The prop vehicleList only updates structurally; this gives us live data.
  useTelemetryStore(s => s.getSnapshot());
  const liveVehicleList = useTelemetryStore(s => s.getVehicleList());
  vehicleList = liveVehicleList.length > 0 ? liveVehicleList : vehicleList;

  const confirmEntries = pendingConfirms ? Object.entries(pendingConfirms) : [];

  return (
    <div style={{ padding: 16 }}>
      {confirmEntries.map(([sysId, entry]) => {
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
            onApprove={onApprove}
            onDeny={onDeny}
            onCancel={onCancel}
            autoApprove={aasParams.getConfirmedValue(sid, 'nav_cm_fl', AAS_DEFAULTS.nav_cm_fl)}
            timeoutSec={aasParams.getConfirmedValue(sid, 'nav_cwt', AAS_DEFAULTS.nav_cwt)}
            forced={forcedConfirms?.[sid] === entry.taskId}
          />
        );
      })}
      <ActiveMonitoring
        vehicleList={vehicleList}
        plan={plan}
        analysis={analysis}
        polygon={polygon}
        launchPoint={launchPoint}
        corridorPoints={corridorPoints}
        searchPattern={searchPattern}
        onEditPlan={onEditPlan}
        onLaunch={onLaunch}
        onOpenConnect={onOpenConnect}
        onRestart={onRestart}
        onDownloadPlan={onDownloadPlan}
        downloadingSysIds={downloadingSysIds}
        pendingVehicles={pendingVehicles}
        launchType={settings?.launch?.launch_type || 'bungee'}
        launchStates={launchStates}
        onAbortLaunch={onAbortLaunch}
        onTriggerVehicle={onTriggerVehicle}
        settings={settings}
        navpyStatus={navpyStatus}
        assignments={assignments}
        pendingConfirms={pendingConfirms}
        onForceConfirm={onForceConfirm}
        onFlyToLocation={onFlyToLocation}
        onEstop={onEstop}
        onOpenReadiness={onOpenReadiness}
      />
    </div>
  );
}

function ActiveMonitoring({ vehicleList, plan, analysis, polygon, launchPoint, corridorPoints, searchPattern, onEditPlan, onLaunch, onOpenConnect, onRestart, onDownloadPlan, downloadingSysIds, pendingVehicles, launchType, launchStates, onAbortLaunch, onTriggerVehicle, settings, navpyStatus, assignments, pendingConfirms, onForceConfirm, onFlyToLocation, onEstop, onOpenReadiness }) {
  const { t } = useTranslation();
  const [busy, setBusy] = useState(false);
  // Per-launch operator ack that pitots are covered / air is calm, allowing
  // auto preflight cal to re-zero airspeed on pitot vehicles at START.
  const [pitotCovered, setPitotCovered] = useState(false);
  // Container launch confirmation. START in container mode initiates launch of
  // every connected UAV, so the button is gated behind a hold-to-arm; the hold
  // opens this modal and only a confirm here actually triggers the launch.
  const [showLaunchConfirm, setShowLaunchConfirm] = useState(false);

  // Per-vehicle load stage (connecting → downloading → ready), latched at ready.
  const statusVehicleList = React.useMemo(
    () => mergePendingVehicles(vehicleList, pendingVehicles),
    [vehicleList, pendingVehicles],
  );
  const loadStages = useLoadStages(statusVehicleList, downloadingSysIds);

  const isContainer = launchType === 'container';
  const plannedUavCount = plan?.zones?.length || 0;
  const containers = analysis?.sets || 1;
  const simMode = settings?.simulation?.sim_mode ?? false;
  const devMode = settings?.simulation?.dev_mode ?? false;

  const [navpyBusy, setNavpyBusy] = useState({});

  // Collapsible container groups in the UAV Status list — expanded by default.
  // Holds the container numbers the operator has collapsed (empty = all open).
  const [collapsedContainers, setCollapsedContainers] = useState(() => new Set());
  const toggleContainer = useCallback((num) => {
    setCollapsedContainers((prev) => {
      const next = new Set(prev);
      if (next.has(num)) next.delete(num); else next.add(num);
      return next;
    });
  }, []);

  const toggleNavpy = useCallback(async (sysId, isRunning) => {
    setNavpyBusy((prev) => ({ ...prev, [sysId]: true }));
    try {
      if (isRunning) {
        await fetch(`/api/control/navpy-sim/stop/${sysId}`, { method: 'POST' });
      } else {
        // Connection is derived server-side from the chat (matches auto-start).
        await fetch('/api/control/navpy-sim/start', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ sys_id: sysId }),
        });
      }
    } catch { /* ignore */ }
    setNavpyBusy((prev) => ({ ...prev, [sysId]: false }));
  }, []);

  // Launch readiness: container mode covers every connected vehicle; other
  // launch modes retain the plan-sized roster. Gates come from settings so the
  // UI never disagrees with the backend gate.
  const launchGates = launchGatesFromSettings(settings);
  const launchVehicles = selectLaunchVehicles(vehicleList, isContainer, plannedUavCount);

  const launcher = useContainerLauncher({
    isContainer, vehicleList: launchVehicles, launchStates,
    onLaunch, onTriggerVehicle, settings,
  });

  const phase = computeMonitorPhase({ vehicleList, containerLaunching: launcher.containerLaunching });

  // Plan awareness for PRE_LAUNCH
  const hasGcsPlan = (plan?.zones?.length || 0) > 0;
  const vehiclesHaveMission = vehicleList.some(v => (v.mission_total || 0) > 0);

  const hasPendingStatusVehicles = statusVehicleList.some((v) => v.pending);
  const launchRosterComplete = launchVehicles.length > 0
    && (isContainer || launchVehicles.length === plannedUavCount)
    && !hasPendingStatusVehicles;
  const allLaunchReady = launchRosterComplete && launchVehicles.every(v => computeLaunchReadiness(v, launchGates).ready);

  // Show the pitot-covered ack only when auto-cal is on AND a launch-target
  // vehicle has a pitot (otherwise START's auto-cal never touches airspeed).
  const showPitotAck = settings?.launch?.auto_preflight_cal === true
    && launchVehicles.some(v => v.airspeed_present === true);
  const pitotAckEl = showPitotAck ? (
    <label style={{ display: 'flex', alignItems: 'flex-start', gap: 8, fontSize: 11, color: colors.text, margin: '8px 0', cursor: 'pointer' }}>
      <input
        type="checkbox"
        checked={pitotCovered}
        onChange={(e) => setPitotCovered(e.target.checked)}
        style={{ accentColor: colors.accent, marginTop: 1, cursor: 'pointer' }}
      />
      <span>
        {t('monitor.pitotCovered')}
        <span style={{ display: 'block', color: colors.textDim, fontSize: 10 }}>{t('monitor.pitotCoveredHint')}</span>
      </span>
    </label>
  ) : null;

  // Action-scoped acknowledgement: only forward the ack when its checkbox is
  // actually on screen (never a stale value from a since-hidden checkbox), and
  // reset it after each launch so a later launch/relaunch can't silently reuse a
  // prior "pitot covered" to re-zero airspeed without a fresh, visible ack.
  const pitotCoveredAck = showPitotAck && pitotCovered;

  const handleBungeeLaunch = useCallback(async (sysId, opts) => {
    await onLaunch([sysId], { ...opts, pitotCovered: pitotCoveredAck });
    setPitotCovered(false);
  }, [onLaunch, pitotCoveredAck]);

  const handleRestart = async () => {
    setBusy(true);
    const sysIds = isContainer
      ? vehicleList.map(v => v.sys_id)
      : vehicleList.filter(v => v.armed).map(v => v.sys_id);
    await onRestart(sysIds);
    setBusy(false);
  };

  const handleStart = async (opts) => {
    setBusy(true);
    const sysIds = vehicleList.map((v) => v.sys_id);
    await onRestart(sysIds, { ...opts, pitotCovered: pitotCoveredAck });
    setPitotCovered(false);
    setBusy(false);
  };

  const zoneDistances = React.useMemo(() => computeZoneDistances(plan), [plan]);

  const wpOffset = missionWpOffset(
    searchPattern, polygon?.length || 0, corridorPoints?.length || 0, !!launchPoint,
  );

  const renderVehicleCards = (vehicles) => {
    const containerGroups = groupVehiclesByContainer(vehicles);
    const showContainers = containerGroups.length > 1;
    return containerGroups.map((grp) => {
      const isExpanded = !collapsedContainers.has(grp.container);
      return (
        <React.Fragment key={grp.container}>
          {showContainers && (
            <button
              type="button"
              onClick={() => toggleContainer(grp.container)}
              aria-expanded={isExpanded}
              style={{
                display: 'flex', alignItems: 'center', gap: 6, width: '100%',
                margin: '10px 0 6px', padding: '3px 2px',
                background: 'none', border: 'none', cursor: 'pointer', textAlign: 'left',
              }}
            >
              <span style={{ color: colors.textDim, fontSize: 9, flex: '0 0 auto', width: 9, transition: 'transform 0.15s ease', transform: isExpanded ? 'rotate(90deg)' : 'none' }}>â–¶</span>
              <span style={{ color: colors.accent, fontSize: 11, fontWeight: 600, flex: '0 0 auto' }}>
                {t('monitor.containerLabel', { num: grp.container })}
              </span>
              <span style={{ marginLeft: 'auto', flex: '0 0 auto', color: colors.textDim, fontSize: 10 }}>
                {t('monitor.uavCount', { count: grp.items.length })}
              </span>
            </button>
          )}
          {(!showContainers || isExpanded) && grp.items.map(({ v, index }) => (
            <VehicleStatusCard
              key={v.sys_id}
              vehicle={v} index={index}
              zoneDistance={zoneDistances[index]}
              wpOffset={wpOffset}
              assignments={assignments}
              pendingConfirms={pendingConfirms}
              onForceConfirm={onForceConfirm}
              isContainer={isContainer}
              phase={phase}
              launchStates={launchStates}
              onLaunch={handleBungeeLaunch}
              gates={launchGates}
              simMode={simMode}
              devMode={devMode}
              navpyStatus={navpyStatus}
              navpyBusy={navpyBusy}
              navpyConn={navpyStatus[v.sys_id]?.connection}
              onToggleNavpy={() => toggleNavpy(v.sys_id, navpyStatus[v.sys_id]?.running)}
              onFlyToLocation={onFlyToLocation}
              loadStage={loadStages.get(v.sys_id)}
            />
          ))}
        </React.Fragment>
      );
    });
  };


  // NO_VEHICLES phase. A connect attempt is in flight (downloadingSysIds is
  // populated the moment handleConnect/handleScan fires, before the vehicle
  // has any telemetry and so before it exists in vehicleList) — show that
  // instead of the blunt "No vehicles connected", which used to blank the
  // whole panel and hide any indication that something was happening.
  if (phase === MONITOR_PHASES.NO_VEHICLES) {
    const busyConnecting = downloadingSysIds?.size > 0;
    const hasPendingCards = statusVehicleList.length > 0;
    return (
      <>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <SectionTitle>{t('monitor.uavStatus')}</SectionTitle>
          {plan && <EditPlanLink label={t('monitor.editPlan')} onClick={onEditPlan} />}
        </div>
        <Divider />
        {hasPendingCards ? (
          renderVehicleCards(statusVehicleList)
        ) : busyConnecting ? (
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 16 }}>
            <style>{`
              @keyframes loadStagePulse { 0%,100% { opacity: 1; } 50% { opacity: 0.25; } }
              @media (prefers-reduced-motion: reduce) { .loadStageDot { animation: none !important; } }
            `}</style>
            <span
              className="loadStageDot"
              style={{ width: 7, height: 7, borderRadius: '50%', background: colors.accent, flexShrink: 0, animation: 'loadStagePulse 1s ease-in-out infinite' }}
            />
            <span style={{ color: colors.textDim, fontSize: 13 }}>
              {t('monitor.connectingUavs', { count: downloadingSysIds.size })}
            </span>
          </div>
        ) : (
          <>
            <div style={{ color: colors.textDim, fontSize: 13, marginBottom: 16 }}>
              {t('monitor.noVehiclesConnected')}
            </div>
            <button
              onClick={onOpenConnect}
              style={{
                width: '100%',
                padding: '10px 0',
                background: 'transparent',
                color: colors.accent,
                border: `1px solid ${colors.accent}`,
                borderRadius: 6,
                fontWeight: 700,
                fontSize: 13,
                cursor: 'pointer',
                marginBottom: 8,
              }}
            >
              {t('monitor.connect')}
            </button>
          </>
        )}
        {!plan && <EditButton label={t('monitor.startPlanning')} onClick={onEditPlan} />}
      </>
    );
  }

  const canContainerLaunch = launchVehicles.length > 0;

  // Pre-launch action button based on plan awareness
  const isBusyConnecting = downloadingSysIds?.size > 0;
  function preLaunchActionButton() {
    if (hasGcsPlan) return null; // caller renders START or START MISSION
    if (vehiclesHaveMission || isBusyConnecting) {
      const label = isBusyConnecting
        ? (vehiclesHaveMission ? t('monitor.downloadingPlan') : t('monitor.connectingPlan'))
        : t('monitor.downloadPlan');
      return (
        <button
          onClick={onDownloadPlan}
          disabled={isBusyConnecting}
          style={{
            width: '100%',
            padding: '10px 0',
            background: isBusyConnecting ? colors.surfaceLight : colors.accent,
            color: isBusyConnecting ? colors.textDim : '#000',
            border: 'none',
            borderRadius: 6,
            fontWeight: 700,
            fontSize: 13,
            cursor: isBusyConnecting ? 'wait' : 'pointer',
            opacity: isBusyConnecting ? 0.7 : 1,
            marginTop: 8,
            marginBottom: 8,
          }}
        >
          {label}
        </button>
      );
    }
    return (
      <EditButton label={t('monitor.startPlanning')} onClick={onEditPlan} />
    );
  }

  const containerGroups = groupVehiclesByContainer(statusVehicleList);
  const showContainers = containerGroups.length > 1;

  // The ubiquitous planning entry point ("Edit Plan" / "Start Planning") now
  // lives on the monitor map overlay (top-right), not in this header row — a
  // long translated label there could crowd or clip E-STOP. See
  // computePlanEntry + MonitorMapOverlay. This header keeps only the section
  // title and E-STOP, so it stays clean in every language.
  return (
    <>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <SectionTitle>{t('monitor.uavStatus')}</SectionTitle>
        <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
          {/* E-STOP lives here while the monitor sidebar is mounted; TopBar
              renders the same dropdown as a fallback for the states where this
              one is gone (planning phase, manual control). */}
          {onEstop && <EstopDropdown vehicleList={vehicleList} onEstop={onEstop} />}
        </div>
      </div>
      <Divider />

      {containerGroups.map((grp) => {
        const isExpanded = !collapsedContainers.has(grp.container);
        return (
          <React.Fragment key={grp.container}>
            {showContainers && (
              <button
                type="button"
                onClick={() => toggleContainer(grp.container)}
                aria-expanded={isExpanded}
                style={{
                  display: 'flex', alignItems: 'center', gap: 6, width: '100%',
                  margin: '10px 0 6px', padding: '3px 2px',
                  background: 'none', border: 'none', cursor: 'pointer', textAlign: 'left',
                }}
              >
                <span style={{ color: colors.textDim, fontSize: 9, flex: '0 0 auto', width: 9, transition: 'transform 0.15s ease', transform: isExpanded ? 'rotate(90deg)' : 'none' }}>▶</span>
                <span style={{ color: colors.accent, fontSize: 11, fontWeight: 600, flex: '0 0 auto' }}>
                  {t('monitor.containerLabel', { num: grp.container })}
                </span>
                <span style={{ marginLeft: 'auto', flex: '0 0 auto', color: colors.textDim, fontSize: 10 }}>
                  {t('monitor.uavCount', { count: grp.items.length })}
                </span>
              </button>
            )}
            {(!showContainers || isExpanded) && grp.items.map(({ v, index }) => (
              <VehicleStatusCard
                key={v.sys_id}
                vehicle={v} index={index}
                zoneDistance={zoneDistances[index]}
                wpOffset={wpOffset}
                assignments={assignments}
                pendingConfirms={pendingConfirms}
                onForceConfirm={onForceConfirm}
                isContainer={isContainer}
                phase={phase}
                launchStates={launchStates}
                onLaunch={handleBungeeLaunch}
                gates={launchGates}
                simMode={simMode}
                devMode={devMode}
                navpyStatus={navpyStatus}
                navpyBusy={navpyBusy}
                navpyConn={navpyStatus[v.sys_id]?.connection}
                onToggleNavpy={() => toggleNavpy(v.sys_id, navpyStatus[v.sys_id]?.running)}
                onFlyToLocation={onFlyToLocation}
                loadStage={loadStages.get(v.sys_id)}
              />
            ))}
          </React.Fragment>
        );
      })}

      {plannedUavCount > 0 && (
        <div style={{ color: colors.textDim, fontSize: 11, marginTop: 4, marginBottom: 4 }}>
          {t('monitor.sets', { count: containers })}, {t('monitor.uavCount', { count: plannedUavCount })}
        </div>
      )}

      {/* Readiness lives here now, not the top bar — it's only meaningful
          pre-launch, so it shows up exactly when relevant and is gone once
          airborne. Covers both the container and bungee paths below. Reads as
          a verb-first action rather than a bare status word: calm/informational
          when everything's ready, an unmistakable "fix" action when it's not —
          but always clickable, so "just checking" stays available either way. */}
      {phase === MONITOR_PHASES.PRE_LAUNCH && onOpenReadiness && (() => {
        const overalls = vehicleList.map((v) => computeVehicleReadiness(v, launchGates).overall);
        const fleetStatus = computeFleetStatus(vehicleList, launchGates);
        const nogoCount = overalls.filter((o) => o === 'nogo').length;
        const warnCount = overalls.filter((o) => o === 'warn').length;
        const urgent = fleetStatus === 'nogo' || fleetStatus === 'warn';
        const iconColor = FLEET_STATUS_COLOR[fleetStatus] || colors.textDim;
        const tint = urgent ? iconColor : null;
        const message = fleetStatus === 'nogo' ? t('preflight.notReady', { count: nogoCount })
          : fleetStatus === 'warn' ? t('preflight.needsAttention', { count: warnCount })
          : fleetStatus === 'na' ? t('preflight.noDataYet')
          : t('preflight.checkReady');
        return (
          <button
            type="button"
            onClick={onOpenReadiness}
            style={{
              display: 'flex', alignItems: 'center', gap: 8, width: '100%',
              padding: '8px 10px', marginBottom: 8, borderRadius: 6, cursor: 'pointer',
              background: tint ? `${tint}1f` : colors.surface,
              border: `1px solid ${tint || colors.border}`,
            }}
          >
            {urgent ? (
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none" style={{ flexShrink: 0 }}>
                <path d="M8 1.8L14.8 13.4a1 1 0 01-.86 1.5H2.06a1 1 0 01-.86-1.5L8 1.8z" stroke={iconColor} strokeWidth="1.3" strokeLinejoin="round"/>
                <path d="M8 6v3.2" stroke={iconColor} strokeWidth="1.3" strokeLinecap="round"/>
                <circle cx="8" cy="11.6" r="0.7" fill={iconColor}/>
              </svg>
            ) : (
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none" style={{ flexShrink: 0 }}>
                <circle cx="8" cy="8" r="6.5" stroke={iconColor} strokeWidth="1.3"/>
                <path d="M5.3 8.3l1.8 1.8 3.6-3.8" stroke={iconColor} strokeWidth="1.3" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            )}
            <span style={{ flex: 1, textAlign: 'left', fontSize: 12, fontWeight: urgent ? 700 : 600, color: urgent ? iconColor : colors.text }}>
              {message}
            </span>
            <svg width="14" height="14" viewBox="0 0 16 16" fill="none" style={{ flexShrink: 0 }}>
              <path d="M6 3l5 5-5 5" stroke={urgent ? iconColor : colors.textDim} strokeWidth="1.3" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </button>
        );
      })()}

      {/* PRE_LAUNCH + container */}
      {phase === MONITOR_PHASES.PRE_LAUNCH && isContainer && (
        hasGcsPlan ? (
          <>
            <LaunchPanel
              vehicleList={launchVehicles}
              launchStates={launchStates}
              launchOrder={settings?.launch?.launch_order}
              gates={launchGates}
              launcherStatus={(
                <LauncherStatus
                  inline
                  connected={launcher.launcherConnected}
                  checking={launcher.checkingLauncher}
                  onRetry={launcher.checkLauncher}
                  simulatorRunning={launcher.simulatorRunning}
                  host={launcher.esp32Host}
                  port={launcher.esp32Port}
                />
              )}
            />
            {pitotAckEl}
            <ForceStartButton
              label={t('monitor.start')}
              armLabel={t('monitor.holdToLaunch')}
              armReady
              ready={allLaunchReady}
              disabled={!canContainerLaunch || !launcher.launcherConnected || !launchRosterComplete}
              onArm={() => setShowLaunchConfirm(true)}
              onStart={async (opts) => {
                // Only reached via the not-ready force path (opts.force);
                // the ready path arms and confirms through the modal below.
                await launcher.doContainerLaunch({ ...opts, pitotCovered: pitotCoveredAck });
                setPitotCovered(false);
              }}
              large
            />
          </>
        ) : (
          <>
            <LauncherStatus
              connected={launcher.launcherConnected}
              checking={launcher.checkingLauncher}
              onRetry={launcher.checkLauncher}
              simulatorRunning={launcher.simulatorRunning}
              host={launcher.esp32Host}
              port={launcher.esp32Port}
            />
            {preLaunchActionButton()}
          </>
        )
      )}

      {/* PRE_LAUNCH + bungee */}
      {phase === MONITOR_PHASES.PRE_LAUNCH && !isContainer && (
        hasGcsPlan ? (
          <>
            {pitotAckEl}
            <ForceStartButton
              label={t('monitor.startMission')}
              busyLabel={t('monitor.starting')}
              ready={allLaunchReady}
              disabled={busy || !launchRosterComplete}
              onStart={handleStart}
            />
          </>
        ) : preLaunchActionButton()
      )}

      {/* CONTAINER_LAUNCHING */}
      {phase === MONITOR_PHASES.CONTAINER_LAUNCHING && (
        <LaunchPanel
          vehicleList={launchVehicles}
          launchStates={launchStates}
          onAbort={onAbortLaunch}
          onTriggerVehicle={onTriggerVehicle}
          launchOrder={settings?.launch?.launch_order}
          gates={launchGates}
          launcherStatus={(
            <LauncherStatus
              inline
              connected={launcher.launcherConnected}
              checking={launcher.checkingLauncher}
              onRetry={launcher.checkLauncher}
              simulatorRunning={launcher.simulatorRunning}
              host={launcher.esp32Host}
              port={launcher.esp32Port}
            />
          )}
        />
      )}

      {/* IN_FLIGHT */}
      {phase === MONITOR_PHASES.IN_FLIGHT && (
        <button
          onClick={handleRestart}
          disabled={busy}
          style={{
            width: '100%',
            padding: '10px 0',
            background: colors.warning,
            color: '#000',
            border: 'none',
            borderRadius: 6,
            fontWeight: 700,
            fontSize: 13,
            cursor: busy ? 'wait' : 'pointer',
            opacity: busy ? 0.5 : 1,
            marginTop: 8,
            marginBottom: 8,
          }}
        >
          {busy ? t('monitor.restarting') : t('monitor.restartMission')}
        </button>
      )}

      {showLaunchConfirm && (
        <ConfirmModal
          title={t('launch.containerLaunchTitle')}
          message={t('launch.containerLaunchConfirm', { count: launchVehicles.length })}
          confirmLabel={t('launch.containerLaunchGo')}
          tone="caution"
          onConfirm={async () => {
            setShowLaunchConfirm(false);
            await launcher.doContainerLaunch({ pitotCovered: pitotCoveredAck });
            setPitotCovered(false);
          }}
          onCancel={() => setShowLaunchConfirm(false)}
        />
      )}
    </>
  );
}


// Re-export for tests that import from this file
export { navpyLogViewerVisibility } from './NavpySimButton';


