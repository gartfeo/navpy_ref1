import { pickCartographic, pickEntity } from '../../utils/entityPicking';

/**
 * LEFT_CLICK: place launch point / add corridor point / add vertex during drawing.
 */
export function handleClick(Cesium, viewer, click, callbacksRef) {
  const cb = callbacksRef.current;

  // Launch point placement mode
  if (cb.placingLaunchPoint && cb.onPlaceLaunchPoint && cb.editable) {
    const latlon = pickCartographic(Cesium, viewer, click.position);
    if (latlon) cb.onPlaceLaunchPoint(latlon);
    return;
  }

  // Corridor placement mode — append corridor point (skip if clicking existing point)
  if (cb.placingCorridor && cb.onPlaceCorridorPoint && cb.editable) {
    const hit = pickEntity(Cesium, viewer, click.position);
    if (hit && hit.type === 'setCorridorPoint') return;
    const latlon = pickCartographic(Cesium, viewer, click.position);
    if (latlon) cb.onPlaceCorridorPoint(latlon);
    return;
  }

  // Sim target toggle — click track waypoint dots in sim mode
  if (cb.simMode && cb.editable && cb.onToggleSimDock) {
    const hit = pickEntity(Cesium, viewer, click.position);
    if (hit && hit.type === 'trackWp') {
      cb.onToggleSimDock(hit.zoneIndex, hit.wpIndex);
      return;
    }
  }

  // fallback location placement mode — skip if clicking an existing fallback location (let double-click remove it)
  if (cb.placingFallbackLocation && cb.onPlaceFallbackLocation && cb.editable) {
    const hit = pickEntity(Cesium, viewer, click.position);
    if (hit && hit.type === 'fallbackLocation') return;
    const latlon = pickCartographic(Cesium, viewer, click.position);
    if (latlon) cb.onPlaceFallbackLocation(latlon);
    return;
  }

  // Keep-out (exclusion) drawing mode — append a vertex to the in-progress ring
  if (cb.placingExclusion && cb.onPlaceExclusionVertex && cb.editable) {
    const latlon = pickCartographic(Cesium, viewer, click.position);
    if (latlon) cb.onPlaceExclusionVertex(latlon);
    return;
  }

  if (!cb.editable || !cb.isDrawing) return;

  const latlon = pickCartographic(Cesium, viewer, click.position);
  if (latlon) {
    cb.onMapClick(latlon);
  }
}
