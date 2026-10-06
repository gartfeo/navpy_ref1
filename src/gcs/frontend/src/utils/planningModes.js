/**
 * Pure state-transition helpers for planning placement modes.
 * No React dependency — consumed by usePlanningOrchestrator,
 * tested directly via Node.js subprocess.
 */

/**
 * Compute next placement flags given current state and an action.
 *
 * isDrawing is read-only input (owned by useDrawing's imperative API).
 * The caller is responsible for calling drawing.startDraw()/stopDraw()
 * based on the returned shouldStartDraw/shouldStopDraw flags.
 *
 * @param {{ placingLaunchPoint: boolean, placingCorridor: boolean, placingDeliveryHub: boolean, placingExclusion: boolean, isDrawing: boolean }} current
 * @param {'startDraw' | 'toggleCorridor' | 'toggleDeliveryHub' | 'toggleExclusion' | 'startDeliveryHubFromSettings'} action
 * @returns {{ placingLaunchPoint: boolean, placingCorridor: boolean, placingDeliveryHub: boolean, placingExclusion: boolean, shouldStartDraw: boolean, shouldStopDraw: boolean }}
 */
export function nextPlacementState(current, action) {
  const base = {
    placingLaunchPoint: false,
    placingCorridor: false,
    placingDeliveryHub: false,
    placingExclusion: false,
    shouldStartDraw: false,
    shouldStopDraw: false,
  };

  switch (action) {
    case 'startDraw':
      return { ...base, shouldStartDraw: true };

    case 'toggleCorridor':
      if (current.placingCorridor) {
        // Toggling off — only clear corridor, leave everything else alone
        return {
          placingLaunchPoint: current.placingLaunchPoint,
          placingCorridor: false,
          placingDeliveryHub: current.placingDeliveryHub,
          placingExclusion: current.placingExclusion,
          shouldStartDraw: false,
          shouldStopDraw: false,
        };
      }
      return {
        ...base,
        placingCorridor: true,
        shouldStopDraw: current.isDrawing,
      };

    case 'toggleDeliveryHub':
      if (current.placingDeliveryHub) {
        // Toggling off — only clear delivery hub, leave everything else alone
        return {
          placingLaunchPoint: current.placingLaunchPoint,
          placingCorridor: current.placingCorridor,
          placingDeliveryHub: false,
          placingExclusion: current.placingExclusion,
          shouldStartDraw: false,
          shouldStopDraw: false,
        };
      }
      return {
        ...base,
        placingDeliveryHub: true,
        shouldStopDraw: current.isDrawing,
      };

    case 'toggleExclusion':
      if (current.placingExclusion) {
        // Toggling off — only clear keep-out drawing, leave everything else alone
        return {
          placingLaunchPoint: current.placingLaunchPoint,
          placingCorridor: current.placingCorridor,
          placingDeliveryHub: current.placingDeliveryHub,
          placingExclusion: false,
          shouldStartDraw: false,
          shouldStopDraw: false,
        };
      }
      return {
        ...base,
        placingExclusion: true,
        shouldStopDraw: current.isDrawing,
      };

    case 'startDeliveryHubFromSettings':
      return {
        ...base,
        placingDeliveryHub: true,
        shouldStopDraw: current.isDrawing,
      };

    default:
      return base;
  }
}

/**
 * Toggle a waypoint index in the sim POI map.
 * Zone key stays with empty array on toggle-off (preserves serialization semantics).
 *
 * @param {Object} prev - { [zoneIndex]: [wpIndex, ...] }
 * @param {number} zoneIndex
 * @param {number} wpIndex
 * @returns {Object} updated simDockWps
 */
export function toggleSimDock(prev, zoneIndex, wpIndex) {
  const arr = prev[zoneIndex] || [];
  const idx = arr.indexOf(wpIndex);
  const next = idx >= 0 ? arr.filter((_, i) => i !== idx) : [...arr, wpIndex];
  return { ...prev, [zoneIndex]: next };
}
