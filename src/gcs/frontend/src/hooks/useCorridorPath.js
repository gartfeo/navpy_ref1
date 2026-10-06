import { useCallback } from 'react';

/**
 * Corridor point placement, drag, insert, delete, and clear handlers.
 */
export default function useCorridorPath({
  launchPoint,
  setLaunchPoint,
  corridorPoints,
  setCorridorPoints,
  setSetLaunchPoints,
  setSetCorridorPoints,
  undoRef,
}) {
  // First click sets launch/start point, subsequent clicks add waypoints
  const handlePlaceCorridorPoint = useCallback((latlon) => {
    if (!launchPoint) {
      setLaunchPoint({ lat: latlon.lat, lon: latlon.lon });
      undoRef.current = [...undoRef.current, { type: 'launch' }];
    } else {
      setCorridorPoints((prev) => [...prev, { lat: latlon.lat, lon: latlon.lon }]);
      undoRef.current = [...undoRef.current, { type: 'corridor' }];
    }
  }, [launchPoint]);

  // Midpoint-insert — optional setIdx targets a specific set
  const handleCorridorMidpointInsert = useCallback((index, latlon, setIdx) => {
    if (setIdx != null) {
      setSetCorridorPoints((prev) => {
        const next = [...prev];
        const arr = next[setIdx] ?? [];
        next[setIdx] = [...arr.slice(0, index), { lat: latlon.lat, lon: latlon.lon }, ...arr.slice(index)];
        return next;
      });
    } else {
      // Structural edit (not a plain append): snapshot the corridor so undo can
      // restore it exactly. A count-based 'corridor' entry can't represent an
      // insert, so undo would otherwise pop the wrong (last) point.
      undoRef.current = [...undoRef.current, { type: 'corridorReplace', prev: corridorPoints }];
      setCorridorPoints((prev) => [
        ...prev.slice(0, index),
        { lat: latlon.lat, lon: latlon.lon },
        ...prev.slice(index),
      ]);
    }
  }, [setCorridorPoints, setSetCorridorPoints, corridorPoints, undoRef]);

  // Corridor point drag handler
  const handleCorridorPointDrag = useCallback((index, newLatLon) => {
    setCorridorPoints((prev) => prev.map((p, i) =>
      i === index ? { lat: newLatLon.lat, lon: newLatLon.lon } : p
    ));
  }, []);

  // Delete corridor point — optional setIdx targets a specific set
  const handleCorridorPointDelete = useCallback((index, setIdx) => {
    if (setIdx != null) {
      setSetCorridorPoints((prev) => {
        const next = [...prev];
        const arr = next[setIdx] ?? [];
        next[setIdx] = arr.filter((_, i) => i !== index);
        return next;
      });
    } else {
      // Structural edit — snapshot for undo (see handleCorridorMidpointInsert).
      undoRef.current = [...undoRef.current, { type: 'corridorReplace', prev: corridorPoints }];
      setCorridorPoints((prev) => prev.filter((_, i) => i !== index));
    }
  }, [setCorridorPoints, setSetCorridorPoints, corridorPoints, undoRef]);

  const handleClearCorridor = useCallback(() => {
    setCorridorPoints([]);
    undoRef.current = undoRef.current.filter((e) => e.type === 'vertex');
  }, []);

  // Drag handler for any set's corridor point (by set index + corridor index)
  // corridorIndex -1 = launch point for that set
  const handleSetCorridorPointDrag = useCallback((setIdx, corridorIndex, newLatLon) => {
    if (corridorIndex === -1) {
      setSetLaunchPoints((prev) => {
        const next = [...prev];
        next[setIdx] = { lat: newLatLon.lat, lon: newLatLon.lon };
        return next;
      });
    } else {
      setSetCorridorPoints((prev) => {
        const next = [...prev];
        next[setIdx] = (next[setIdx] || []).map((p, i) =>
          i === corridorIndex ? { lat: newLatLon.lat, lon: newLatLon.lon } : p
        );
        return next;
      });
    }
  }, []);

  // Record pre-drag position for undo
  const handleCorridorDragRecord = useCallback((setIdx, corridorIndex, oldLatLon) => {
    undoRef.current = [...undoRef.current, {
      type: 'corridorDrag',
      setIdx,
      corridorIndex,
      oldLatLon: { lat: oldLatLon.lat, lon: oldLatLon.lon },
    }];
  }, []);

  return {
    handlePlaceCorridorPoint,
    handleCorridorMidpointInsert,
    handleCorridorPointDrag,
    handleCorridorPointDelete,
    handleClearCorridor,
    handleSetCorridorPointDrag,
    handleCorridorDragRecord,
  };
}
