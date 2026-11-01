// CesiumMap's map-stopped wiring (docs/design/gcs_map_stopped_notice.md):
// the notice over the map box, and a stopped map dismisses the loading screen.
// Every map hook is mocked; only useCesiumViewer's state matters here.
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen } from '@testing-library/react';
import CesiumMap from './CesiumMap';

const { viewerState } = vi.hoisted(() => ({ viewerState: {} }));
vi.mock('./hooks/useCesiumViewer', () => ({ default: () => viewerState }));
vi.mock('./hooks/usePlanSync', () => ({ default: () => ({}) }));
vi.mock('./hooks/useMapInteraction');
vi.mock('./hooks/usePolygonLayer');
vi.mock('./hooks/useZoneLayer');
vi.mock('./hooks/useTrackLayer');
vi.mock('./hooks/useLaunchZoneLayer');
vi.mock('./hooks/useCorridorLayer');
vi.mock('./hooks/useFenceLayer');
vi.mock('./hooks/useExclusionLayer');
vi.mock('./hooks/useCorridorWaypoints');
vi.mock('./hooks/usePartitionHandle');
vi.mock('./hooks/useUavMarkers');
vi.mock('./hooks/useUavTrails');
vi.mock('./hooks/useClosestApproachMarks');
vi.mock('./hooks/useCoverageLayer');
vi.mock('./hooks/useAssignmentMarkers');
vi.mock('./hooks/useAvailableTaskMarkers');
vi.mock('./hooks/useDockMarkers');
vi.mock('./hooks/useDetectionMarkers');
vi.mock('./hooks/useDeliveryHubLayer');
vi.mock('./hooks/useFollowUav');
vi.mock('./hooks/useMapSwap');
vi.mock('./hooks/useUavPoseSource');
vi.mock('./hooks/useTrackWpMarkers');

function setViewer({ viewerReady = false, tilesReady = false, mapStopped = false } = {}) {
  Object.assign(viewerState, {
    containerRef: React.createRef(),
    viewerRef: { current: null },
    cesiumRef: { current: null },
    readyRef: { current: false },
    fitFlyingRef: { current: false },
    viewerReady,
    terrainReady: false,
    tilesReady,
    mapStopped,
  });
}

describe('CesiumMap map stop', () => {
  beforeEach(() => setViewer());

  it('a running map shows no notice and keeps the loading screen until tiles load', () => {
    const onViewerReady = vi.fn();
    setViewer({ viewerReady: true });
    render(<CesiumMap onViewerReady={onViewerReady} />);
    expect(screen.queryByRole('alert')).toBeNull();
    expect(onViewerReady).not.toHaveBeenCalled();
  });

  it('a stopped map shows the notice and dismisses the loading screen at once', () => {
    const onViewerReady = vi.fn();
    setViewer({ viewerReady: true, mapStopped: true });
    render(<CesiumMap onViewerReady={onViewerReady} />);
    expect(screen.getByRole('alert')).toHaveTextContent('mapStopped.title');
    expect(onViewerReady).toHaveBeenCalledWith(true);
  });

  it('keeps the notice beside the Cesium container, which the viewer hook clears', () => {
    setViewer({ viewerReady: true, mapStopped: true });
    render(<CesiumMap onViewerReady={() => {}} />);
    const container = viewerState.containerRef.current;
    expect(container).not.toBeNull();
    expect(container.contains(screen.getByRole('alert'))).toBe(false);
  });
});
