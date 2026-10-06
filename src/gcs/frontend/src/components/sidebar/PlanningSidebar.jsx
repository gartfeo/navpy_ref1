import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsLabel } from '../../styles';
import { computeSets, getUavsPerSet } from '../../utils/planner';
import { SectionTitle, Divider, Label, InfoRow, StepBtn } from './SidebarPrimitives';
import DeliveryHubAssignmentSection, { zoneLabel } from './DeliveryHubAssignmentSection';
import MultiSelectWps from './MultiSelectWps';
import ConfirmSection from './ConfirmSection';
import ExclusionSection from './ExclusionSection';

const SEARCH_PATTERN_OPTIONS = [
  { value: 'distributed' },
  { value: 'corridor' },
];

export default function PlanningSidebar({
  searchPattern,
  setSearchPattern,
  analysis,
  uavCount,
  setUavCount,
  plan,
  launchPoint,
  corridorPoints,
  uavCountLocked,
  setUavCountLocked,
  settings,
  vehicleList,
  aasParams,
  setLaunchPoints,
  deliveryHubAssignments,
  setDeliveryHubAssignments,
  setManualDeliveryHubEdit,
  simDockWps,
  simMode,
  detectAfterWps,
  setDetectAfterWps,
  onToggleSimDock,
  routeOffsetM,
  setRouteOffsetM,
  hasSearchPolygon,
  fenceEnabled,
  onToggleFence,
  observedFence,
  fenceRequestStatus,
  fenceOffsetM,
  setFenceOffsetM,
  takeoffRoundM,
  setTakeoffRoundM,
  fenceCustomized,
  onFenceReset,
  fenceCoverageOk,
  fenceSelfIntersecting,
  selfIntersectingExclusions,
  exclusionPolygons,
  exclusionConflicts,
  onRemoveExclusion,
  onClearExclusions,
}) {
  const { t } = useTranslation();

  return (
    <div style={{ padding: 16 }}>
      <SectionTitle>{t('planningSidebar.missionPlanning')}</SectionTitle>
      <Divider />

      {/* Search pattern */}
      <Label>{t('planningSidebar.searchPattern')}</Label>
      <select
        value={searchPattern}
        onChange={(e) => setSearchPattern(e.target.value)}
        style={{
          width: '100%',
          padding: '8px 10px',
          background: colors.surface,
          color: colors.textBright,
          border: `1px solid ${colors.border}`,
          borderRadius: 4,
          fontSize: 13,
          marginBottom: 16,
          cursor: 'pointer',
        }}
      >
        {SEARCH_PATTERN_OPTIONS.map((opt) => (
          <option key={opt.value} value={opt.value}>
            {t('planningSidebar.searchPatterns.' + opt.value)}
          </option>
        ))}
      </select>

      {/* UAV Count */}
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6 }}>
        <Label style={{ marginBottom: 0 }}>{t('planningSidebar.uavCount')}</Label>
        {searchPattern !== 'corridor' && (
          <button
            onClick={() => setUavCountLocked(v => !v)}
            title={uavCountLocked ? 'Locked: polygon constrained to launch zone' : 'Unlocked: polygon moves freely'}
            style={{
              background: 'transparent',
              border: 'none',
              cursor: 'pointer',
              fontSize: 14,
              padding: '0 2px',
              color: uavCountLocked ? '#ff9800' : colors.textDim,
            }}
          >
            {uavCountLocked ? '\u{1F512}' : '\u{1F513}'}
          </button>
        )}
      </div>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 8 }}>
        <StepBtn
          disabled={uavCount <= 1}
          onClick={() => setUavCount(Math.max(1, (uavCount || 1) - 1))}
        >-</StepBtn>
        <span style={{ color: colors.textBright, fontSize: 16, fontWeight: 700, minWidth: 24, textAlign: 'center' }}>
          {uavCount || '?'}
        </span>
        <StepBtn
          disabled={analysis && uavCount >= analysis.max_uavs}
          onClick={() => setUavCount(Math.min(analysis?.max_uavs || 99, (uavCount || 1) + 1))}
        >+</StepBtn>
      </div>
      {analysis && (
        <div style={{ marginBottom: 16 }}>
          <InfoRow
            label={t('planningSidebar.containers')}
            value={`${computeSets(uavCount)} x ${getUavsPerSet()}`}
          />
          {uavCount !== analysis.required_uavs && (
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: 4 }}>
              <span style={{ color: colors.textDim, fontSize: 13 }}>
                {t('planningSidebar.recommended', { count: analysis.required_uavs })}
              </span>
              <button
                onClick={() => setUavCount(analysis.required_uavs)}
                style={{
                  background: 'transparent',
                  color: colors.accent,
                  border: `1px solid ${colors.accent}`,
                  borderRadius: 4,
                  padding: '2px 8px',
                  fontSize: 11,
                  cursor: 'pointer',
                }}
              >
                {t('settings.reset')}
              </button>
            </div>
          )}
        </div>
      )}

      {/* Route offset (demo mode): manual override of the camera-derived track spacing */}
      {simMode && (
        <div style={{ marginBottom: 16 }}>
          <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6 }}>
            <Label style={{ marginBottom: 0 }}>{t('planningSidebar.routeOffset')}</Label>
            <span style={{ color: colors.textBright, fontSize: 12, fontFamily: 'monospace', whiteSpace: 'nowrap', flexShrink: 0, marginLeft: 8 }}>
              {routeOffsetM != null
                ? `${routeOffsetM} m`
                : t('planningSidebar.routeOffsetAuto', { m: analysis?.track_spacing_m ?? '—' })}
            </span>
          </div>
          <input
            type="range"
            min={20}
            max={500}
            step={5}
            value={routeOffsetM ?? analysis?.track_spacing_m ?? 150}
            onChange={(e) => setRouteOffsetM(Number(e.target.value))}
            style={{ width: '100%', accentColor: colors.accent, cursor: 'pointer' }}
          />
          {routeOffsetM != null && (
            <div style={{ display: 'flex', justifyContent: 'flex-end', marginTop: 4 }}>
              <button
                onClick={() => setRouteOffsetM(null)}
                style={{
                  background: 'transparent',
                  color: colors.accent,
                  border: `1px solid ${colors.accent}`,
                  borderRadius: 4,
                  padding: '2px 8px',
                  fontSize: 11,
                  cursor: 'pointer',
                }}
              >
                {t('settings.reset')}
              </button>
            </div>
          )}
        </div>
      )}

      {/* Corridor info */}
      {(searchPattern === 'corridor' || launchPoint) && (
        <div style={{ marginBottom: 16 }}>
          <Label>{t('planningSidebar.corridor')}</Label>
          {launchPoint ? (
            <>
              <div style={{ color: colors.textBright, fontSize: 12, fontFamily: 'monospace', marginBottom: 4 }}>
                {t('planningSidebar.start')}: {launchPoint.lat.toFixed(6)}, {launchPoint.lon.toFixed(6)}
              </div>
              {corridorPoints.length > 0 && (
                <div style={{ color: colors.textBright, fontSize: 12, marginBottom: 4 }}>
                  {t('planningSidebar.waypoints', { count: corridorPoints.length })}
                </div>
              )}
              <div style={{ color: colors.textDim, fontSize: 11, marginBottom: 6 }}>
                {t('planningSidebar.dragToMove')}
              </div>
            </>
          ) : (
            <div style={{ color: colors.textDim, fontSize: 12, marginBottom: 6 }}>
              {t('planningSidebar.clickMapToSetStart')}
            </div>
          )}
        </div>
      )}

      <Divider />

      {/* Geofence */}
      <div style={{
        border: `1px solid ${fenceEnabled ? '#ff9800' : colors.border}`,
        borderRadius: 8,
        padding: 10,
        marginBottom: 16,
        background: fenceEnabled ? 'rgba(255, 152, 0, 0.05)' : 'transparent',
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: fenceEnabled ? 10 : 0 }}>
          <svg width="15" height="15" viewBox="0 0 14 14" fill="none" style={{ flexShrink: 0 }}>
            <path d="M7 1l5 2v4c0 3-2.2 5-5 6-2.8-1-5-3-5-6V3l5-2z" stroke="#ff9800" strokeWidth="1.2" strokeLinejoin="round" fill={fenceEnabled ? '#ff9800' : 'none'} fillOpacity={fenceEnabled ? 0.25 : 0}/>
          </svg>
          <span style={{ color: colors.textBright, fontSize: 13, fontWeight: 700 }}>{t('planningSidebar.geofence')}</span>
          <span style={{ flex: 1 }} />
          <button
            onClick={onToggleFence}
            disabled={!hasSearchPolygon}
            title={hasSearchPolygon ? '' : t('planningSidebar.fenceNeedsZone')}
            role="switch"
            aria-checked={fenceEnabled}
            style={{
              width: 34, height: 18, borderRadius: 10, border: 'none', padding: 0,
              position: 'relative', flexShrink: 0,
              cursor: hasSearchPolygon ? 'pointer' : 'not-allowed',
              background: fenceEnabled ? '#ff9800' : colors.border,
              opacity: hasSearchPolygon ? 1 : 0.5,
            }}
          >
            <span style={{
              position: 'absolute', top: 2, left: fenceEnabled ? 18 : 2,
              width: 14, height: 14, borderRadius: '50%', background: colors.bg,
              transition: 'left 0.12s',
            }} />
          </button>
        </div>

        {/* What the connected vehicles report about their OWN fences. Shown
            whether or not the planner fence is on, because the planner fence is
            a request and this is the current state. Per-vehicle values are kept
            distinct: unknown and mixed never collapse into a single answer, and
            FENCE_ENABLE=0 with FENCE_AUTOENABLE>0 reads as auto-enable, not
            off. Nothing here is uploaded — only the operator's own fence is. */}
        {observedFence?.hasObservations && (
          <div style={{ margin: '8px 0 2px' }}>
            <InfoRow
              label={t('planningSidebar.fenceObservedLabel')}
              value={t(`planningSidebar.fenceObserved.${observedFence.mode}`)}
            />
            {/* The configured auto-enable value itself, not just "auto".
                The trigger differs per value (AC_Fence: 3 = ONLY_WHEN_ARMED,
                2 = ENABLE_DISABLE_FLOOR_ONLY), so the value is shown under
                its parameter name instead of being described — it matches
                what the vehicle holds and asserts no trigger we did not read. */}
            {observedFence.autoenableMode != null && (
              <InfoRow
                label={t('planningSidebar.fenceAutoenableLabel')}
                value={`FENCE_AUTOENABLE=${observedFence.autoenableMode}`}
              />
            )}
            {(observedFence.mode === 'mixed' || observedFence.autoenableMode == null) && (
              <div style={{ color: colors.textDim, fontSize: 11, marginTop: 2 }}>
                {Object.entries(observedFence.modes)
                  .map(([sysId, mode]) => {
                    const autoenable = observedFence.autoenableModes?.[sysId];
                    const detail = autoenable != null ? ` FENCE_AUTOENABLE=${autoenable}` : '';
                    return `${sysId}: ${t(`planningSidebar.fenceObserved.${mode}`)}${detail}`;
                  })
                  .join(' · ')}
              </div>
            )}
            <div style={{ color: colors.textDim, fontSize: 11, marginTop: 2 }}>
              {t('planningSidebar.fenceObservedHint')}
            </div>
          </div>
        )}

        {fenceEnabled && (
          <>
            <InfoRow label={t('planningSidebar.fenceType')} value={t('planningSidebar.fenceTypePolygon')} />
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', margin: '4px 0' }}>
              <span style={{ color: colors.textDim, fontSize: 13 }}>{t('planningSidebar.fenceBreachAction')}</span>
              <span style={{
                background: 'rgba(255,152,0,0.16)', color: '#ffb84d',
                border: '1px solid #ff9800', borderRadius: 4, padding: '1px 7px', fontSize: 11,
              }}>{t('planningSidebar.fenceActionRtl')}</span>
            </div>

            {/* Custom shape: operator dragged the fence — sliders drive only the
                auto derivation, so they pause until "Reset to auto". */}
            {fenceCustomized && (
              <div style={{ display: 'flex', alignItems: 'center', gap: 6, margin: '8px 0 2px' }}>
                <span style={{
                  background: 'rgba(255,152,0,0.16)', color: '#ffb84d',
                  border: '1px solid #ff9800', borderRadius: 4, padding: '1px 7px', fontSize: 11,
                }}>{t('planningSidebar.fenceCustom')}</span>
                <span style={{ flex: 1 }} />
                <button
                  onClick={onFenceReset}
                  style={{
                    background: 'transparent', border: `1px solid ${colors.border}`,
                    color: colors.textBright, borderRadius: 4, padding: '2px 8px',
                    fontSize: 11, cursor: 'pointer',
                  }}
                >
                  {t('planningSidebar.fenceResetAuto')}
                </button>
              </div>
            )}

            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', margin: '10px 0 4px', opacity: fenceCustomized ? 0.5 : 1 }}>
              <Label style={{ marginBottom: 0 }}>{t('planningSidebar.fenceOffset')}</Label>
              <span style={{ color: colors.textBright, fontSize: 12, fontFamily: 'monospace' }}>{fenceOffsetM} m</span>
            </div>
            <input
              type="range"
              min={50}
              max={1500}
              step={50}
              value={fenceOffsetM}
              disabled={fenceCustomized}
              title={fenceCustomized ? t('planningSidebar.fenceSlidersPaused') : ''}
              onChange={(e) => setFenceOffsetM(Number(e.target.value))}
              style={{ width: '100%', accentColor: '#ff9800', cursor: fenceCustomized ? 'not-allowed' : 'pointer', opacity: fenceCustomized ? 0.5 : 1 }}
            />

            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', margin: '10px 0 4px', opacity: fenceCustomized ? 0.5 : 1 }}>
              <Label style={{ marginBottom: 0 }}>{t('planningSidebar.takeoffRound')}</Label>
              <span style={{ color: colors.textBright, fontSize: 12, fontFamily: 'monospace' }}>{takeoffRoundM} m</span>
            </div>
            <input
              type="range"
              min={0}
              max={300}
              step={10}
              value={takeoffRoundM}
              disabled={fenceCustomized}
              title={fenceCustomized ? t('planningSidebar.fenceSlidersPaused') : ''}
              onChange={(e) => setTakeoffRoundM(Number(e.target.value))}
              style={{ width: '100%', accentColor: '#ff9800', cursor: fenceCustomized ? 'not-allowed' : 'pointer', opacity: fenceCustomized ? 0.5 : 1 }}
            />

            {/* A hand-shaped fence can be dragged too tight — non-blocking hazard note. */}
            {fenceCustomized && fenceCoverageOk === false && (
              <div style={{
                border: '1px solid #ff1744', borderRadius: 6, padding: 8, marginTop: 8,
                background: 'rgba(255, 23, 68, 0.08)', color: '#ff5252', fontSize: 11, lineHeight: 1.5,
              }}>
                {'⚠ '}{t('planningSidebar.fenceCoverageWarning')}
              </div>
            )}
            {/* A vertex dragged across the ring self-intersects it — ArduPilot's
                even-odd test flips crossover pockets to "outside the fence". */}
            {fenceCustomized && fenceSelfIntersecting && (
              <div style={{
                border: '1px solid #ff1744', borderRadius: 6, padding: 8, marginTop: 8,
                background: 'rgba(255, 23, 68, 0.08)', color: '#ff5252', fontSize: 11, lineHeight: 1.5,
              }}>
                {'⚠ '}{t('planningSidebar.fenceSelfIntersect')}
              </div>
            )}

            <div style={{ color: colors.textDim, fontSize: 11, lineHeight: 1.5, marginTop: 8 }}>
              {t('planningSidebar.fenceHint')}
            </div>
          </>
        )}

        {/* What the next upload will actually do to the fence. Driven by the
            real request, not by the toggle: an enabled fence the vehicles have
            already acknowledged sends nothing, and a planner fence switched
            off still carries a pending disable. Outside the fenceEnabled block
            so that pending disable stays visible. */}
        <div style={{
          fontSize: 11, marginTop: 8,
          color: fenceRequestStatus === 'invalid'
            ? colors.error
            : (!fenceRequestStatus || fenceRequestStatus === 'none' ? colors.textDim : colors.success),
        }}>
          {fenceRequestStatus === 'invalid' && t('planningSidebar.fenceRequestInvalid')}
          {fenceRequestStatus === 'enable' && t('planningSidebar.fenceRequestEnable')}
          {fenceRequestStatus === 'disable' && t('planningSidebar.fenceRequestDisable')}
          {(!fenceRequestStatus || fenceRequestStatus === 'none') && t('planningSidebar.fenceRequestNone')}
        </div>
        {/* Always visible, including on a fresh plan with no readback: the
            operator has to know before touching the toggle that Off is an
            explicit disable request, not "leave the vehicles alone". */}
        <div style={{ color: colors.textDim, fontSize: 11, lineHeight: 1.5, marginTop: 6 }}>
          {t('planningSidebar.fenceUploadHelp')}
        </div>
      </div>

      {/* Keep-out zones + non-blocking conflict warnings */}
      <ExclusionSection
        exclusionPolygons={exclusionPolygons}
        exclusionConflicts={exclusionConflicts}
        selfIntersectingExclusions={selfIntersectingExclusions}
        onRemoveExclusion={onRemoveExclusion}
        onClearExclusions={onClearExclusions}
      />

      <Divider />

      {/* Confirmation */}
      <Label>{t('planningSidebar.confirmation')}</Label>
      <ConfirmSection vehicleList={vehicleList} aasParams={aasParams} />

      <Divider />

      {/* Default delivery hub assignments */}
      <DeliveryHubAssignmentSection
        plan={plan}
        settings={settings}
        deliveryHubAssignments={deliveryHubAssignments}
        setDeliveryHubAssignments={setDeliveryHubAssignments}
        setManualDeliveryHubEdit={setManualDeliveryHubEdit}
      />

      {/* Sim POIs grid */}
      {simMode && plan?.zones?.length > 0 && (
        <>
          <Divider />
          <Label>{t('planningSidebar.simDocks')}</Label>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 4, marginBottom: 4 }}>
            {/* Header row */}
            <div style={{ display: 'grid', gridTemplateColumns: '48px minmax(0, 1fr) minmax(0, 1fr)', gap: 4, fontSize: 11, color: colors.textDim, paddingBottom: 2 }}>
              <span>{t('planningSidebar.uavHeader')}</span>
              <span>{t('planningSidebar.dockWps')}</span>
              <span>{t('planningSidebar.detectWp')}</span>
            </div>
            {/* Data rows */}
            {plan.zones.map((zone, zi) => {
              const label = zoneLabel(plan.zones, zi);
              const zoneColor = zoneColorsLabel[zi % zoneColorsLabel.length];
              const trackLen = zone.track?.length || 0;
              const selected = simDockWps?.[zi] || [];
              const detectWp = detectAfterWps?.[zi];
              return (
                <div key={zi} style={{ display: 'grid', gridTemplateColumns: '48px minmax(0, 1fr) minmax(0, 1fr)', gap: 4, alignItems: 'center', fontSize: 12 }}>
                  <span style={{ display: 'flex', alignItems: 'center', gap: 3 }}>
                    <span style={{ width: 8, height: 8, borderRadius: '50%', background: zoneColor, flexShrink: 0 }} />
                    <span style={{ color: colors.textDim }}>{label}</span>
                  </span>
                  <MultiSelectWps
                    trackLen={trackLen}
                    selected={selected}
                    onToggle={(wi) => onToggleSimDock(zi, wi)}
                  />
                  <select
                    value={detectWp ?? ''}
                    onChange={(e) => {
                      const val = e.target.value;
                      setDetectAfterWps((prev) => {
                        const next = { ...prev };
                        if (val === '') {
                          delete next[zi];
                        } else {
                          next[zi] = parseInt(val, 10);
                        }
                        return next;
                      });
                    }}
                    style={{
                      background: colors.surface,
                      color: detectWp != null ? colors.textBright : colors.textDim,
                      border: `1px solid ${colors.border}`,
                      borderRadius: 4,
                      padding: '3px 18px 3px 6px', // right padding reserves the native arrow area
                      fontSize: 12,
                      cursor: 'pointer',
                      textOverflow: 'ellipsis',
                      whiteSpace: 'nowrap',
                      overflow: 'hidden',
                    }}
                  >
                    <option value="">{'\u2014'}</option>
                    {Array.from({ length: trackLen }, (_, wi) => (
                      <option key={wi} value={wi}>WP {wi + 1}</option>
                    ))}
                  </select>
                </div>
              );
            })}
          </div>
          <div style={{ color: colors.textDim, fontSize: 11 }}>
            {t('planningSidebar.clickTrackWps')}
          </div>
        </>
      )}

    </div>
  );
}
