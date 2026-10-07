/**
 * Fence-related labels on the planning toolbar.
 *
 * Two live findings: the map draws the same amber ring whether it came from the
 * planner or from the vehicles' own readback, with nothing saying which; and the
 * shield tooltip named one auto-enable trigger ("at takeoff") for every
 * FENCE_AUTOENABLE value, including 3 (ONLY_WHEN_ARMED) and 2
 * (ENABLE_DISABLE_FLOOR_ONLY).
 */
import { render } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import PlanningToolbar from './PlanningToolbar';

const ring = () => [
  { lat: 32.0, lon: 34.0 },
  { lat: 32.01, lon: 34.0 },
  { lat: 32.0, lon: 34.01 },
];

const renderToolbar = (over = {}) => render(
  <PlanningToolbar
    isDrawing={false}
    onStartDraw={vi.fn()}
    onStopDraw={vi.fn()}
    searchPattern="distributed"
    polygon={ring()}
    launchPoint={null}
    corridorPoints={[]}
    placingCorridor={false}
    onToggleCorridor={vi.fn()}
    effectiveSets={1}
    activeSetIndex={0}
    setActiveSetIndex={vi.fn()}
    placingDeliveryHub={false}
    onToggleDeliveryHub={vi.fn()}
    placingDeliveryHubType="other"
    setPlacingDeliveryHubType={vi.fn()}
    fenceEnabled={false}
    onToggleFence={vi.fn()}
    observedFenceMode="none"
    placingExclusion={false}
    onToggleExclusion={vi.fn()}
    onUndo={vi.fn()}
    canUndo={false}
    onClearAll={vi.fn()}
    onLoadPolygon={vi.fn()}
    onSavePolygon={vi.fn()}
    {...over}
  />,
);

describe('PlanningToolbar — which fence ring the map is showing', () => {
  it('labels a planner-derived ring as the planner preview', () => {
    const { container } = renderToolbar({ fenceEnabled: true, fenceMapSource: 'planner' });
    expect(container.textContent).toContain('planningToolbar.fenceSource.planner');
    expect(container.textContent).not.toContain('planningToolbar.fenceSource.vehicles');
  });

  it('labels a ring read back from the vehicles as theirs, not the planner\'s', () => {
    const { container } = renderToolbar({ fenceEnabled: false, fenceMapSource: 'vehicles' });
    expect(container.textContent).toContain('planningToolbar.fenceSource.vehicles');
    expect(container.textContent).not.toContain('planningToolbar.fenceSource.planner');
  });

  it('says no ring is drawn when there is none', () => {
    const { container } = renderToolbar({ fenceEnabled: false, fenceMapSource: 'none' });
    expect(container.textContent).toContain('planningToolbar.fenceSource.none');
  });
});

describe('PlanningToolbar — auto-enable tooltip', () => {
  it('does not name a trigger the vehicle may not be configured for', () => {
    const { getByTitle } = renderToolbar({
      fenceEnabled: false, observedFenceMode: 'auto', observedFenceAutoenableMode: 3,
    });
    // Mode-neutral wording plus the raw value, which is the only trigger fact
    // we actually have.
    const btn = getByTitle(/planningToolbar\.fenceObserved\.auto/);
    expect(btn.getAttribute('title')).toContain('FENCE_AUTOENABLE=3');
  });

  it('omits the numeric mode when the fleet does not agree on one', () => {
    const { getByTitle } = renderToolbar({
      fenceEnabled: false, observedFenceMode: 'mixed', observedFenceAutoenableMode: null,
    });
    const btn = getByTitle(/planningToolbar\.fenceObserved\.mixed/);
    expect(btn.getAttribute('title')).not.toContain('FENCE_AUTOENABLE=');
  });
});
