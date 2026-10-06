/**
 * The observed-fence block in the planning sidebar.
 *
 * The approved scope requires the operator to see the EXACT fence mode each
 * vehicle is configured with. "auto" is not that: FENCE_AUTOENABLE=1 arms at
 * takeoff, 2 arms at arming, 3 arms after takeoff only — an operator deciding
 * whether to leave the fleet alone needs the number, not the category.
 */
import { render } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import PlanningSidebar from './PlanningSidebar';
import en from '../../locales/en.json';
import {
  emptyFenceObservations,
  fenceObservationFromDownload,
  recordFenceObservation,
  summarizeFenceObservations,
} from '../../utils/fenceIntent';

const ring = () => [
  { lat: 32.0, lon: 34.0 },
  { lat: 32.01, lon: 34.0 },
  { lat: 32.0, lon: 34.01 },
];

const observed = (autoenable, enable = 0) => fenceObservationFromDownload({
  vertices: ring(), exclusions: [], total_items: 3,
  params: { enable, autoenable, type: 4, action: 1 },
  params_readback: 'ok',
});

const summarize = (perVehicle) => {
  let state = emptyFenceObservations();
  let seq = 0;
  for (const [sysId, obs] of perVehicle) {
    seq += 1;
    state = recordFenceObservation(state, sysId, obs, { epoch: state.epoch, edit: state.edit, seq });
  }
  return summarizeFenceObservations(state, perVehicle.map(([sysId]) => sysId));
};

const renderSidebar = (observedFence, over = {}) => render(
  <PlanningSidebar
    searchPattern="distributed"
    setSearchPattern={vi.fn()}
    dockClasses={['small']}
    setDockClasses={vi.fn()}
    perUavDockClasses={{}}
    setPerUavDockClasses={vi.fn()}
    analysis={null}
    uavCount={2}
    setUavCount={vi.fn()}
    onTargetChange={vi.fn()}
    plan={null}
    launchPoint={null}
    corridorPoints={[]}
    uavCountLocked
    setUavCountLocked={vi.fn()}
    settings={{ fallback_delivery_locations: [] }}
    vehicleList={[{ sys_id: 1 }, { sys_id: 2 }]}
    aasParams={{
      sessionDraft: {},
      setSessionDraft: vi.fn(),
      uploadPatch: vi.fn(),
      getConfirmedConsensus: () => ({ state: 'unknown', value: null }),
    }}
    setLaunchPoints={vi.fn()}
    fallbackLocationAssignments={[]}
    setFallbackLocationAssignments={vi.fn()}
    setManualFallbackLocationEdit={vi.fn()}
    simDockWps={{}}
    simMode
    detectAfterWps={{}}
    setDetectAfterWps={vi.fn()}
    onToggleSimDock={vi.fn()}
    routeOffsetM={null}
    setRouteOffsetM={vi.fn()}
    hasSearchPolygon
    fenceEnabled={false}
    onToggleFence={vi.fn()}
    observedFence={observedFence}
    fenceOffsetM={50}
    setFenceOffsetM={vi.fn()}
    takeoffRoundM={60}
    setTakeoffRoundM={vi.fn()}
    fenceCustomized={false}
    onFenceReset={vi.fn()}
    fenceCoverageOk
    fenceSelfIntersecting={false}
    selfIntersectingExclusions={[]}
    exclusionPolygons={[]}
    exclusionConflicts={[]}
    onRemoveExclusion={vi.fn()}
    onClearExclusions={vi.fn()}
    {...over}
  />,
);

describe('PlanningSidebar observed fence — configured autoenable mode', () => {
  it('shows the fleet-wide FENCE_AUTOENABLE value the vehicles actually hold', () => {
    const { container } = renderSidebar(summarize([[1, observed(2)], [2, observed(2)]]));
    expect(container.textContent).toContain('FENCE_AUTOENABLE=2');
  });

  it('names each vehicle\'s own value when the fleet disagrees', () => {
    const { container } = renderSidebar(summarize([[1, observed(1)], [2, observed(3)]]));
    // Per vehicle, and paired with that vehicle: the ' · ' separator bounds
    // each entry, so a value cannot be read off the wrong UAV.
    expect(container.textContent).toMatch(/1:[^·]*FENCE_AUTOENABLE=1/);
    expect(container.textContent).toMatch(/2:[^·]*FENCE_AUTOENABLE=3/);
  });

  it('shows no autoenable value when the parameters could not be read', () => {
    const { container } = renderSidebar(
      summarize([[1, fenceObservationFromDownload(null)]]),
    );
    expect(container.textContent).not.toContain('FENCE_AUTOENABLE=');
  });
});

/**
 * What the next upload will actually do to the fence, and the help that has to
 * be readable before the operator touches anything.
 *
 * The old line claimed every upload sends the fence to all UAVs. That is wrong
 * twice over: with the fence enabled and already acknowledged the next
 * untouched upload omits the fence entirely, and even when a fence IS sent it
 * reaches only the vehicles whose mission upload succeeds — not every
 * connected UAV. The replacement states the request and its carrier (the
 * mission) and claims no count.
 */
describe('PlanningSidebar fence request status', () => {
  const noObservations = summarize([]);

  it('says no fence update when the upload carries no fence request', () => {
    const { container } = renderSidebar(noObservations, {
      fenceEnabled: true, fenceRequestStatus: 'none',
    });
    expect(container.textContent).toContain('planningSidebar.fenceRequestNone');
    expect(container.textContent).not.toContain('planningSidebar.fenceRequestEnable');
    // The unconditional "uploads to all UAVs" claim is gone.
    expect(container.textContent).not.toContain('planningSidebar.fenceUploads');
  });

  it('says an enable/update is pending when the request carries geometry', () => {
    const { container } = renderSidebar(noObservations, {
      fenceEnabled: true, fenceRequestStatus: 'enable',
    });
    expect(container.textContent).toContain('planningSidebar.fenceRequestEnable');
    expect(container.textContent).not.toContain('planningSidebar.fenceRequestNone');
  });

  it('says a disable is pending even though the planner fence reads off', () => {
    const { container } = renderSidebar(noObservations, {
      fenceEnabled: false, fenceRequestStatus: 'disable',
    });
    expect(container.textContent).toContain('planningSidebar.fenceRequestDisable');
    expect(container.textContent).not.toContain('planningSidebar.fenceRequestNone');
  });

  it('flags an unenforceable enable request instead of a pending update', () => {
    const { container } = renderSidebar(noObservations, {
      fenceEnabled: true, fenceRequestStatus: 'invalid',
    });
    expect(container.textContent).toContain('planningSidebar.fenceRequestInvalid');
    expect(container.textContent).not.toContain('planningSidebar.fenceRequestEnable');
  });

  it('states the request without claiming how many UAVs it reaches', () => {
    // The i18n stub renders keys, so the count claim has to be checked where it
    // would live: the strings themselves, plus the absence of a plural variant
    // that would reintroduce a {{count}} call site.
    for (const key of ['fenceRequestEnable', 'fenceRequestDisable']) {
      expect(en.planningSidebar[key]).toBeTruthy();
      expect(en.planningSidebar[key]).not.toMatch(/\{\{count\}\}|\bUAVs?\b|\ball\b/i);
      expect(Object.keys(en.planningSidebar)).not.toContain(`${key}_one`);
      expect(Object.keys(en.planningSidebar)).not.toContain(`${key}_other`);
    }
  });

  it.each([
    ['a fresh plan with no readback', summarize([]), false],
    ['a plan whose fence is on', summarize([[1, observed(0, 1)]]), true],
  ])('always shows the preserve / Off-requests-disable help on %s', (_label, obs, fenceEnabled) => {
    const { container } = renderSidebar(obs, { fenceEnabled, fenceRequestStatus: 'none' });
    expect(container.textContent).toContain('planningSidebar.fenceUploadHelp');
  });
});
