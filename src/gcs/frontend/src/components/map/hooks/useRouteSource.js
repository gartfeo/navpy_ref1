import { useEffect } from 'react';
import { routeFor } from '../../../utils/fpvRoute';
import { createRouteOverlay } from '../utils/routeOverlay';

/**
 * Publishes the first-person views' route source, wired like the pose source:
 * `route(sysId)` is the UAV's own route and next waypoint from the main map's
 * plan, track positions and progress (utils/fpvRoute.js routeFor).
 */
export function useRouteSource(routeSourceRef, storeRef, planRef, trackPosRef, vehiclePosRef) {
  useEffect(() => {
    if (!routeSourceRef) return undefined;
    routeSourceRef.current = {
      route: (sysId) => routeFor(sysId, {
        vehicleList: storeRef.current?.getVehicleList(),
        trackPositions: trackPosRef.current,
        plan: planRef.current,
        progress: vehiclePosRef.current,
      }),
    };
    return () => { routeSourceRef.current = null; };
  }, [routeSourceRef, storeRef, planRef, trackPosRef, vehiclePosRef]);
}

/**
 * RC control's first-person view on the main map: the RC UAV's waypoints,
 * numbered, with the next one marked, per telemetry update while RC owns the
 * camera. With tracks shown the main map draws the line itself
 * (useTrackLayer); with tracks hidden the overlay draws it, so the path stays
 * visible in RC.
 */
export function useFpvRouteLabels(cesiumRef, viewerRef, routeSourceRef, fpvSysId, viewerReady, storeRef, showTracks) {
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer || !viewerReady || fpvSysId == null || !routeSourceRef) return undefined;
    const overlay = createRouteOverlay(Cesium, viewer.scene);
    const update = () => {
      if (viewer.isDestroyed()) return;
      if (overlay.update(routeSourceRef.current?.route(fpvSysId) ?? null, { line: !showTracks, labels: 'all' })) {
        viewer.scene.requestRender();
      }
    };
    update();
    const unsubscribe = storeRef.current.subscribe(update);
    return () => {
      unsubscribe();
      if (!viewer.isDestroyed()) {
        overlay.destroy();
        viewer.scene.requestRender();
      }
    };
  }, [fpvSysId, viewerReady, showTracks]);
}
