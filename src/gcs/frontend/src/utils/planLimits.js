// Operator-settable ranges of the track-shaping plan inputs. Every producer of
// a UAV count (sidebar stepper, recommended reset, automatic count) and the
// route offset slider stay inside them, and a plan file outside them is
// rejected — so a file the GCS saves always loads, and track generation never
// runs on values the operator could not have set.
export const ROUTE_OFFSET_MIN_M = 20;
export const ROUTE_OFFSET_MAX_M = 500;
export const ROUTE_OFFSET_STEP_M = 5;
export const MAX_UAV_COUNT = 99;

/** Automatic UAV count for an analysis: its requirement, within MAX_UAV_COUNT (null without one). */
export function autoUavCount(analysis) {
  return analysis?.required_uavs == null ? null : Math.min(analysis.required_uavs, MAX_UAV_COUNT);
}

/** Highest UAV count the operator can step to: the area's usable maximum, within MAX_UAV_COUNT. */
export function maxUavCountFor(analysis) {
  return Math.min(analysis?.max_uavs ?? MAX_UAV_COUNT, MAX_UAV_COUNT);
}
