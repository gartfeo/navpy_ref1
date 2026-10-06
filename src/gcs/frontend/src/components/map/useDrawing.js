import { useState, useCallback, useRef } from 'react';
import { closestEdgeIndex } from '../../utils/geo';

/**
 * Polygon drawing/editing state management.
 * - Click to add vertices, double-click to finish
 * - Drag vertices/midpoints to reshape
 * - Double-click vertex to delete
 * - Drag inside polygon to move all vertices
 * - Undo removes last vertex, Clear resets everything
 *
 * Uses refs for vertices/undoStack so callbacks can read current
 * values synchronously without nesting state setters inside updaters.
 */
export default function useDrawing({ setPolygon, analyze, dockClasses }) {
  const [isDrawing, setIsDrawing] = useState(false);
  const [vertices, setVertices] = useState([]);
  const verticesRef = useRef([]);
  const undoStackRef = useRef([]);

  // Helpers to update both ref and state in sync
  function updateVertices(next) {
    verticesRef.current = next;
    setVertices(next);
  }
  function pushUndo(entry) {
    undoStackRef.current = [...undoStackRef.current, entry];
  }

  const startDraw = useCallback(() => {
    setIsDrawing(true);
    if (verticesRef.current.length === 0) {
      undoStackRef.current = [];
      updateVertices([]);
      setPolygon([]);
    }
  }, [setPolygon]);

  const onMapClick = useCallback(
    (latlon) => {
      if (!latlon) return;

      const prev = verticesRef.current;
      let next, insertAt;

      if (prev.length >= 3) {
        const idx = closestEdgeIndex(prev, latlon);
        insertAt = idx + 1;
        next = [...prev];
        next.splice(insertAt, 0, latlon);
      } else {
        insertAt = prev.length;
        next = [...prev, latlon];
      }

      pushUndo({ index: insertAt });
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next, dockClasses);
      }
    },
    [setPolygon, analyze, dockClasses]
  );

  const undo = useCallback(() => {
    const stack = undoStackRef.current;
    if (stack.length === 0) return;
    const last = stack[stack.length - 1];

    const prev = verticesRef.current;
    if (prev.length === 0) return;

    const next = [...prev];
    next.splice(last.index, 1);

    undoStackRef.current = stack.slice(0, -1);
    updateVertices(next);
    setPolygon(next);
    if (next.length >= 3) {
      analyze(next, dockClasses);
    }
  }, [setPolygon, analyze, dockClasses]);

  const clear = useCallback(() => {
    undoStackRef.current = [];
    updateVertices([]);
    setPolygon([]);
    setIsDrawing(false);
  }, [setPolygon]);

  const stopDraw = useCallback(() => {
    setIsDrawing(false);
  }, []);

  const onVertexDrag = useCallback(
    (index, newLatLon) => {
      const prev = verticesRef.current;
      const next = [...prev];
      next[index] = newLatLon;
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next, dockClasses);
      }
    },
    [setPolygon, analyze, dockClasses]
  );

  const onVertexDelete = useCallback(
    (index) => {
      const prev = verticesRef.current;
      if (prev.length <= 3) return; // keep at least 3 vertices
      const next = [...prev];
      next.splice(index, 1);
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next, dockClasses);
      }
    },
    [setPolygon, analyze, dockClasses]
  );

  const onMidpointInsert = useCallback(
    (afterIndex, latlon) => {
      const prev = verticesRef.current;
      const next = [...prev];
      next.splice(afterIndex + 1, 0, latlon);
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next, dockClasses);
      }
    },
    [setPolygon, analyze, dockClasses]
  );

  const onPolygonMove = useCallback(
    (dlat, dlon) => {
      const prev = verticesRef.current;
      const next = prev.map((v) => ({
        lat: v.lat + dlat,
        lon: v.lon + dlon,
      }));
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next, dockClasses);
      }
    },
    [setPolygon, analyze, dockClasses]
  );

  // Sync internal vertices when polygon is loaded externally (file load)
  const loadVertices = useCallback((poly) => {
    undoStackRef.current = [];
    updateVertices(poly);
  }, []);

  // Restore vertices from a snapshot (drag undo) — preserves drawing undo stack
  const restoreVertices = useCallback((poly) => {
    updateVertices(poly);
    setPolygon(poly);
    if (poly.length >= 3) {
      analyze(poly, dockClasses);
    }
  }, [setPolygon, analyze, dockClasses]);

  return {
    isDrawing,
    vertices,
    canUndo: undoStackRef.current.length > 0,
    startDraw,
    stopDraw,
    onMapClick,
    undo,
    clear,
    loadVertices,
    restoreVertices,
    onVertexDrag,
    onVertexDelete,
    onMidpointInsert,
    onPolygonMove,
  };
}
