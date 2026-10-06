import { useCallback, useEffect, useState } from 'react';
import { nextPlacementState } from '../utils/planningModes';

/**
 * Manages mutually-exclusive placement modes (draw, corridor, fallback location)
 * and auto-cancels placement when the plan becomes invalid.
 */
export default function usePlacementModes({
  drawing, placingFallbackLocation, setPlacingFallbackLocation,
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
      setPlacingFallbackLocation(false);
    }
  }, [analysis, polygon, searchPattern]);

  const startDrawExclusive = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'startDraw');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingFallbackLocation(result.placingFallbackLocation);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStartDraw) drawing.startDraw();
  }, [placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, drawing]);

  const toggleCorridorPlacement = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'toggleCorridor');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingFallbackLocation(result.placingFallbackLocation);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, drawing]);

  const toggleFallbackLocationPlacement = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'toggleFallbackLocation');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingFallbackLocation(result.placingFallbackLocation);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, drawing]);

  const toggleExclusionPlacement = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'toggleExclusion');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingFallbackLocation(result.placingFallbackLocation);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, drawing]);

  const startPlacingFallbackLocationFromSettings = useCallback(() => {
    const result = nextPlacementState({
      placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, isDrawing: drawing.isDrawing,
    }, 'startFallbackLocationFromSettings');
    setPlacingLaunchPoint(result.placingLaunchPoint);
    setPlacingCorridor(result.placingCorridor);
    setPlacingFallbackLocation(result.placingFallbackLocation);
    setPlacingExclusion(result.placingExclusion);
    if (result.shouldStopDraw) drawing.stopDraw();
  }, [placingLaunchPoint, placingCorridor, placingFallbackLocation, placingExclusion, drawing]);

  return {
    placingLaunchPoint, setPlacingLaunchPoint,
    placingCorridor, setPlacingCorridor,
    placingExclusion, setPlacingExclusion,
    startDrawExclusive,
    toggleCorridorPlacement,
    toggleFallbackLocationPlacement,
    toggleExclusionPlacement,
    startPlacingFallbackLocationFromSettings,
  };
}
