import { useCallback, useEffect, useState } from 'react';
import { nextPlacementState } from '../utils/planningModes';

/**
 * Manages mutually-exclusive placement modes (draw, corridor, delivery hub)
 * and auto-cancels placement when the plan becomes invalid.
 */
export default function usePlacementModes({
  drawing, placingDeliveryHub, setPlacingDeliveryHub,
  searchPattern, analysis, polygon,
}) {
  const [placingLaunchPoint, setPlacingLaunchPoint] = useState(false);
  const [placingCorridor, setPlacingCorridor] = useState(false);
  // Keep-out (exclusion) drawing mode. Independent of the search polygon /
  // analysis, so it is NOT auto-cancelled below — an operator can draw a
  // keep-out before (or without) any search zone.
  const [placingExclusion, setPlacingExclusion] = useState(false);

  // Cancel placement modes when polygon cleared or analysis lost
  useEffect(() => {
    if (searchPattern === 'corridor') return;
    if (!analysis || polygon.length < 3) {
      setPlacingLaunchPoint(false);
      setPlacingCorridor(false);
      setPlacingDeliveryHub(false);
    }
  }, [analysis, polygon, searchPattern]);

  const startDrawExclusive = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'startDraw');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingDeliveryHub(result.placingDeliveryHub);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStartDraw) drawing.startDraw();
  }, [placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, drawing]);

  const toggleCorridorPlacement = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'toggleCorridor');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingDeliveryHub(result.placingDeliveryHub);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, drawing]);

  const toggleDeliveryHubPlacement = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'toggleDeliveryHub');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingDeliveryHub(result.placingDeliveryHub);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, drawing]);

  const toggleExclusionPlacement = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'toggleExclusion');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingDeliveryHub(result.placingDeliveryHub);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, drawing]);

  const startPlacingDeliveryHubFromSettings = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'startDeliveryHubFromSettings');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingDeliveryHub(result.placingDeliveryHub);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingDeliveryHub, placingExclusion, drawing]);

  return {
    placingLaunchPoint, setPlacingLaunchPoint,
    placingCorridor, setPlacingCorridor,
    placingExclusion, setPlacingExclusion,
    startDrawExclusive,
    toggleCorridorPlacement,
    toggleDeliveryHubPlacement,
    toggleExclusionPlacement,
    startPlacingDeliveryHubFromSettings,
  };
}
