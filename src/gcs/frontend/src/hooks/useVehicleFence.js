import { useCallback, useMemo } from 'react';
import { mergeExclusionRings, vehicleFenceInEffect } from '../utils/fenceIntent';

/**
 * The fleet's own fence on a plan with no fence decision.
 *
 * The switch shows the fence the vehicles will fly with, so a fleet that
 * already holds an active fence reads ON. It is derived from the newest fleet
 * answer and kept out of plan state, so nothing is sent for it and no older
 * answer can later overwrite what the operator decided. Switching it off is an
 * explicit disable.
 *
 * Keep-outs are part of the fence. While the fleet's fence is shown, its
 * keep-outs are drawn and edited together with any the plan already has, and
 * the first edit of the ring or a keep-out makes the whole fence the plan's:
 * an enable request with that ring and every keep-out, so none is dropped.
 * A gesture the edit helpers refuse (they return null) takes nothing over.
 * Editing is not offered when the fleet's keep-outs disagree (any request
 * would overwrite some of them) or without a search zone (a hand-shaped ring
 * is dropped with the zone). Switched off, no keep-out is drawn or drawable.
 */
export default function useVehicleFence({ mission, observedFence, polygonLength }) {
  const {
    fenceEnabled, fenceTouched, fenceIntent, toggleFence, authorFenceIntent,
    fenceCustomVertices, setFenceCustomVertices, exclusionPolygons, setExclusionPolygons,
  } = mission;
  const vehicleFence = useMemo(() => vehicleFenceInEffect({
    observed: observedFence, touched: fenceTouched, intent: fenceIntent, enabled: fenceEnabled,
  }), [observedFence, fenceTouched, fenceIntent, fenceEnabled]);
  const switchOn = fenceEnabled || vehicleFence != null;
  const editable = !vehicleFence || (!vehicleFence.exclusionsMixed && polygonLength >= 3);

  const handleToggleFence = useCallback(() => toggleFence(switchOn), [toggleFence, switchOn]);

  // The keep-outs the operator sees and edits.
  const editableExclusions = useMemo(() => (vehicleFence
    ? mergeExclusionRings(vehicleFence.exclusions, exclusionPolygons)
    : (exclusionPolygons || [])), [vehicleFence, exclusionPolygons]);

  // `next` is a ring, null ("Reset to auto"), or a gesture's updater, which
  // returns null when the gesture is refused (e.g. a delete below 3 vertices).
  const editFenceVertices = useCallback((next) => {
    if (!vehicleFence) {
      setFenceCustomVertices(next);
      return;
    }
    if (!editable) return;
    const ring = typeof next === 'function' ? next(fenceCustomVertices) : next;
    if (typeof next === 'function' && ring == null) return;
    setExclusionPolygons(editableExclusions);
    authorFenceIntent(true);
    setFenceCustomVertices(ring);
  }, [vehicleFence, editable, fenceCustomVertices, editableExclusions, setExclusionPolygons,
      authorFenceIntent, setFenceCustomVertices]);

  const editExclusionPolygons = useCallback((next) => {
    if (!vehicleFence) {
      setExclusionPolygons(next);
      return;
    }
    if (!editable) return;
    setFenceCustomVertices(vehicleFence.ring);
    authorFenceIntent(true);
    setExclusionPolygons(typeof next === 'function' ? next(editableExclusions) : next);
  }, [vehicleFence, editable, editableExclusions, setExclusionPolygons, setFenceCustomVertices,
      authorFenceIntent]);

  const displayExclusionPolygons = useMemo(
    () => (switchOn ? editableExclusions : []), [switchOn, editableExclusions],
  );

  return {
    vehicleFence, switchOn, editable, handleToggleFence,
    editFenceVertices, editExclusionPolygons, editableExclusions,
    displayExclusionPolygons,
    keepOutDrawable: switchOn && editable,
  };
}
