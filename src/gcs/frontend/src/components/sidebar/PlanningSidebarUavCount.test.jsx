/**
 * The UAV count the sidebar lets the operator author stays within the limit a
 * plan file is loaded with, so a file the GCS saves always loads back.
 */
import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import PlanningSidebar from './PlanningSidebar';
import { MAX_UAV_COUNT } from '../../utils/planLimits';

// An area with more usable strips (max_uavs) and a larger requirement than
// the limit — the large-area fixture at 20 m route offset has max_uavs 111.
const bigAnalysis = { max_uavs: 111, required_uavs: 120, track_spacing_m: 20 };

const renderSidebar = (over) => {
  const setUavCount = vi.fn();
  render(
    <PlanningSidebar
      searchPattern="distributed" setSearchPattern={vi.fn()}
      analysis={bigAnalysis} uavCount={MAX_UAV_COUNT} setUavCount={setUavCount}
      plan={null} launchPoint={null} corridorPoints={[]}
      uavCountLocked setUavCountLocked={vi.fn()}
      settings={{ default_delivery_hubs: [] }} vehicleList={[]}
      aasParams={{
        sessionDraft: {}, setSessionDraft: vi.fn(), uploadPatch: vi.fn(),
        getConfirmedConsensus: () => ({ state: 'unknown', value: null }),
      }}
      setLaunchPoints={vi.fn()} deliveryHubAssignments={[]}
      setDeliveryHubAssignments={vi.fn()} setManualDeliveryHubEdit={vi.fn()}
      simDockWps={{}} simMode={false} detectAfterWps={{}} setDetectAfterWps={vi.fn()}
      onToggleSimDock={vi.fn()} routeOffsetM={null} setRouteOffsetM={vi.fn()}
      hasSearchPolygon fenceEnabled={false} onToggleFence={vi.fn()} observedFence={null}
      fenceOffsetM={50} setFenceOffsetM={vi.fn()} takeoffRoundM={60} setTakeoffRoundM={vi.fn()}
      fenceCustomized={false} onFenceReset={vi.fn()} fenceCoverageOk
      fenceSelfIntersecting={false} selfIntersectingExclusions={[]}
      exclusionPolygons={[]} exclusionConflicts={[]}
      onRemoveExclusion={vi.fn()} onClearExclusions={vi.fn()}
      {...over}
    />,
  );
  return setUavCount;
};

describe('PlanningSidebar UAV count limit', () => {
  it('cannot step past MAX_UAV_COUNT even when the area has more strips', () => {
    renderSidebar();
    expect(screen.getByRole('button', { name: '+' })).toBeDisabled();
  });

  it('steps up to, not beyond, MAX_UAV_COUNT', () => {
    const setUavCount = renderSidebar({ uavCount: MAX_UAV_COUNT - 1 });
    fireEvent.click(screen.getByRole('button', { name: '+' }));
    expect(setUavCount).toHaveBeenCalledWith(MAX_UAV_COUNT);
  });

  it('resets to the recommended count within MAX_UAV_COUNT', () => {
    const setUavCount = renderSidebar({ uavCount: 50 });
    fireEvent.click(screen.getByRole('button', { name: 'settings.reset' }));
    expect(setUavCount).toHaveBeenCalledWith(MAX_UAV_COUNT);
  });
});
