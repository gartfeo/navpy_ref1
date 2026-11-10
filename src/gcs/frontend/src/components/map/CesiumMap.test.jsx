// CesiumMap's map-stopped wiring (docs/design/gcs_map_stopped_notice.md):
// the notice over the map box, and a stopped map dismisses the loading screen.
// Every map hook is mocked; only useCesiumViewer's state matters here.
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen } from '@testing-library/react';
import CesiumMap from './CesiumMap';
import useAssignmentMarkers from './hooks/useAssignmentMarkers';
import useAvailableTaskMarkers from './hooks/useAvailableTaskMarkers';
import useDockMarkers from './hooks/useDockMarkers';

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

describe('CesiumMap dock markers', () => {
  beforeEach(() => setViewer({ viewerReady: true }));

  it('shows the planned docks while planning only: in monitoring a dock needs a detection or peer message', () => {
    const planned = {
      onViewerReady: () => {},
      simMode: true,
      plan: { zones: [{ sys_id: 1, zone_index: 0, set_index: 0, track: [{ lat: 40, lon: 44 }] }] },
      vehicleList: [{ sys_id: 1 }],
      simDockWps: { 0: [0] },
    };
    const { rerender } = render(<CesiumMap {...planned} phase="MONITOR" />);
    expect(useDockMarkers.mock.lastCall[3]).toEqual([]);
    rerender(<CesiumMap {...planned} phase="PLANNING" />);
    expect(useDockMarkers.mock.lastCall[3]).toHaveLength(1);
  });

  // The stray "D" markers: a new plan picks no dock, but the UAVs' stored
  // TARG_WPS (the demo's static parameters, or a previous mission) was drawn
  // onto its waypoints. A UAV's parameter describes the mission on that UAV.
  it('a plan with no dock picks shows no dock, whatever the UAVs store', () => {
    render(
      <CesiumMap
        onViewerReady={() => {}}
        simMode
        phase="PLANNING"
        plan={{ zones: [{ sys_id: 1, zone_index: 0, set_index: 0, track: [{ lat: 40, lon: 44 }, { lat: 40.1, lon: 44 }] }] }}
        vehicleList={[{ sys_id: 1 }]}
        simDockWps={{}}
      />,
    );
    expect(useDockMarkers.mock.lastCall[3]).toEqual([]);
  });

  it('hands the marker hooks what the selector picks, each its own side', () => {
    const onOffer = { taskId: 5, senderId: 1, lat: 45, lon: 44, requestUid: { boot_id: 7, msg_seq: 20 } };
    render(
      <CesiumMap
        onViewerReady={() => {}}
        assignments={{
          'helper:2': { ...onOffer, key: 'helper:2', status: 'assigned', receiverId: 2 },
          '1:5': { ...onOffer, key: '1:5', status: 'waiting', receiverId: 2 },
        }}
        availableTasks={{ '1:6': { key: '1:6', taskId: 6, senderId: 1, lat: 46, lon: 44, ownerBoot: 7 } }}
      />,
    );
    expect(Object.keys(useAssignmentMarkers.mock.lastCall[2])).toEqual(['helper:2']);
    expect(Object.keys(useAvailableTaskMarkers.mock.lastCall[2])).toEqual(['1:6']);
  });
});
